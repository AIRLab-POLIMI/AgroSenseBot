#!/usr/bin/python3
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from asb_msgs.msg import CanopyRegionOfInterest, CanopyData, CanopyDataArray
from asb_msgs.srv import InitializeCanopyRegion

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter

np.set_printoptions(precision=2)


class SprayingRegulator(Node):

    def __init__(self):
        super().__init__('spraying_regulator')

        plt.ion()
        self.fig = dict()
        self.ax = dict()
        self.all_plot = dict()
        self.all_scatter = dict()
        self.roi_plot = dict()
        self.roi_scatter = dict()
        for canopy_id in ['hedge', 'row_1']:
            self.fig[canopy_id] = plt.figure()
            self.ax[canopy_id] = self.fig[canopy_id].add_subplot(111)
            self.all_plot[canopy_id], = self.ax[canopy_id].plot([], [], '-', linewidth=1, color='green')
            self.all_scatter[canopy_id], = self.ax[canopy_id].plot([], [], 's', markersize=3, color='green')
            self.roi_plot[canopy_id], = self.ax[canopy_id].plot([], [], '-', linewidth=3, color='blue')
            self.roi_scatter[canopy_id], = self.ax[canopy_id].plot([], [], 's', markersize=5, color='blue')
            self.ax[canopy_id].set_xlim(-2, 22)
            self.ax[canopy_id].set_ylim(0, 5)
            self.ax[canopy_id].set_title(canopy_id)

        self.all_canopy_volume = defaultdict(dict)

        self.canopy_data_sub_ = self.create_subscription(CanopyDataArray, 'canopy_data', self.canopy_data_callback, 10)
        self.init_canopy_region_client = self.create_client(InitializeCanopyRegion, 'initialize_canopy_region')

        qos_reliable_transient_local = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.canopy_region_of_interest_pub_ = self.create_publisher(CanopyRegionOfInterest, '/canopy_region_of_interest', qos_profile=qos_reliable_transient_local)

        while not self.init_canopy_region_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('service not available, waiting...')

        result_future = self.init_canopy_region_client.call_async(InitializeCanopyRegion.Request(
            canopy_id='hedge',
            canopy_frame_id='hedge',
            min_x=0.5,
            max_x=22.0,
            min_y=-2.0,
            max_y=1.5,
            min_z=0.3,
            max_z=2.5,
            roi=CanopyRegionOfInterest(
                frame_id='base_link',
                x_1=-1.0,
                x_2=1.0,
            ),
        ))
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f"initialize_canopy_region result: {result_future.result().result}")

        result_future = self.init_canopy_region_client.call_async(InitializeCanopyRegion.Request(
            canopy_id='row_1',
            canopy_frame_id='row_1',
            min_x=-1.0,
            max_x=16.0,
            min_y=-1.0,
            max_y=1.0,
            min_z=0.3,
            max_z=2.5,
            roi=CanopyRegionOfInterest(
                frame_id='base_link',
                x_1=-1.0,
                x_2=1.0,
            ),
        ))
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f"initialize_canopy_region result: {result_future.result().result}")

    def canopy_data_callback(self, canopy_data_array_msg: CanopyDataArray):

        for canopy_data_msg in canopy_data_array_msg.canopy_data_array:

            roi_canopy_volume_ = dict()
            for x, y in zip(canopy_data_msg.volume_x_array, canopy_data_msg.volume_y_array):
                self.all_canopy_volume[canopy_data_msg.canopy_id][x] = y
                roi_canopy_volume_[x] = y

            if not len(self.all_canopy_volume[canopy_data_msg.canopy_id]):
                return

            all_canopy_volume_x, all_canopy_volume_y = list(zip(*sorted(self.all_canopy_volume[canopy_data_msg.canopy_id].items())))
            all_canopy_volume_y_filtered = savgol_filter(all_canopy_volume_y, 10, 2, mode='nearest')

            roi_canopy_volume_x = list()
            roi_canopy_volume_y = list()
            roi_canopy_volume_y_filtered = list()
            for x, y, y_f in zip(all_canopy_volume_x, all_canopy_volume_y, all_canopy_volume_y_filtered):
                if x in canopy_data_msg.volume_x_array:
                    roi_canopy_volume_x.append(x)
                    roi_canopy_volume_y.append(y)
                    roi_canopy_volume_y_filtered.append(y_f)

            self.all_plot[canopy_data_msg.canopy_id].set_xdata(all_canopy_volume_x)
            self.all_plot[canopy_data_msg.canopy_id].set_ydata(np.array(all_canopy_volume_y_filtered) / canopy_data_msg.resolution)
            self.all_scatter[canopy_data_msg.canopy_id].set_xdata(all_canopy_volume_x)
            self.all_scatter[canopy_data_msg.canopy_id].set_ydata(np.array(all_canopy_volume_y) / canopy_data_msg.resolution)

            self.roi_plot[canopy_data_msg.canopy_id].set_xdata(roi_canopy_volume_x)
            self.roi_plot[canopy_data_msg.canopy_id].set_ydata(np.array(roi_canopy_volume_y_filtered) / canopy_data_msg.resolution)
            self.roi_scatter[canopy_data_msg.canopy_id].set_xdata(roi_canopy_volume_x)
            self.roi_scatter[canopy_data_msg.canopy_id].set_ydata(np.array(roi_canopy_volume_y) / canopy_data_msg.resolution)

            self.fig[canopy_data_msg.canopy_id].canvas.draw()
            self.fig[canopy_data_msg.canopy_id].canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = SprayingRegulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
