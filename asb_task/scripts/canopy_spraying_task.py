#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from asb_msgs.msg import CanopyRegionOfInterest, CanopyData

import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import savgol_filter

np.set_printoptions(precision=2)


class CanopySprayingTask(Node):

    def __init__(self):
        super().__init__('canopy_spraying_task')

        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.all_plot, = self.ax.plot([], [], '-', linewidth=1, color='green')
        self.all_scatter, = self.ax.plot([], [], 's', markersize=3, color='green')
        self.roi_plot, = self.ax.plot([], [], '-', linewidth=3, color='blue')
        self.roi_scatter, = self.ax.plot([], [], 's', markersize=5, color='blue')
        self.ax.set_xlim(-2, 22)
        self.ax.set_ylim(0, 5)

        self.all_canopy_volume_ = dict()

        self.canopy_data_sub_ = self.create_subscription(CanopyData, '/canopy_data', self.canopy_data_callback, 10)

        canopy_region_of_interest_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.canopy_region_of_interest_pub_ = self.create_publisher(CanopyRegionOfInterest, '/canopy_region_of_interest', qos_profile=canopy_region_of_interest_qos)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        roi = CanopyRegionOfInterest()
        roi.header.frame_id = "base_link"
        roi.header.stamp = self.get_clock().now().to_msg()
        roi.x_1 = -1.0
        roi.x_2 = 1.0
        self.canopy_region_of_interest_pub_.publish(roi)

    def canopy_data_callback(self, canopy_data_msg: CanopyData):

        roi_canopy_volume_ = dict()
        for x, y in zip(canopy_data_msg.volume_x_array, canopy_data_msg.volume_y_array):
            self.all_canopy_volume_[x] = y
            roi_canopy_volume_[x] = y

        if not len(self.all_canopy_volume_):
            return

        all_canopy_volume_x, all_canopy_volume_y = list(zip(*sorted(self.all_canopy_volume_.items())))
        all_canopy_volume_y_filtered = savgol_filter(all_canopy_volume_y, 10, 2, mode='nearest')

        roi_canopy_volume_x = list()
        roi_canopy_volume_y = list()
        roi_canopy_volume_y_filtered = list()
        for x, y, y_f in zip(all_canopy_volume_x, all_canopy_volume_y, all_canopy_volume_y_filtered):
            if x in canopy_data_msg.volume_x_array:
                roi_canopy_volume_x.append(x)
                roi_canopy_volume_y.append(y)
                roi_canopy_volume_y_filtered.append(y_f)

        self.all_plot.set_xdata(all_canopy_volume_x)
        self.all_plot.set_ydata(np.array(all_canopy_volume_y_filtered) / canopy_data_msg.resolution)
        self.all_scatter.set_xdata(all_canopy_volume_x)
        self.all_scatter.set_ydata(np.array(all_canopy_volume_y) / canopy_data_msg.resolution)

        self.roi_plot.set_xdata(roi_canopy_volume_x)
        self.roi_plot.set_ydata(np.array(roi_canopy_volume_y_filtered) / canopy_data_msg.resolution)
        self.roi_scatter.set_xdata(roi_canopy_volume_x)
        self.roi_scatter.set_ydata(np.array(roi_canopy_volume_y) / canopy_data_msg.resolution)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = CanopySprayingTask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
