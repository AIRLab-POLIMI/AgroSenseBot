#!/usr/bin/python3

import rclpy
from rclpy import Future
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Header

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, PointStamped
from asb_msgs.msg import CanopyRegionOfInterest, CanopyDataArray, SprayRegulatorStatus, CanopyData
from asb_msgs.srv import InitializeCanopyRegion, StartRowSpraying, StopRowSpraying, StopRowSpraying_Response, \
    StartRowSpraying_Request, StartRowSpraying_Response, StopRowSpraying_Request

import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np
from scipy.signal import savgol_filter
np.set_printoptions(precision=2)


class SprayingRegulator(Node):

    def __init__(self):
        super().__init__('spraying_regulator')

        self.declare_parameter('max_canopy_width', float(2.0))
        self.max_canopy_width = self.get_parameter('max_canopy_width').get_parameter_value().double_value

        self.declare_parameter('canopy_layers', [0.2, 0.6, 1.0, 1.4, 1.8, 2.2])
        self.canopy_layers = self.get_parameter('canopy_layers').get_parameter_value().double_array_value

        # plotting stuff
        plt.ion()
        self.fig = dict()
        self.ax = dict()
        self.all_plot = dict()
        self.all_scatter = dict()
        self.roi_plot = dict()
        self.roi_scatter = dict()
        for canopy_id in ['row_1', 'row_2']:
            self.fig[canopy_id] = plt.figure()
            self.ax[canopy_id] = self.fig[canopy_id].add_subplot(111)
            self.all_plot[canopy_id], = self.ax[canopy_id].plot([], [], '-', linewidth=1, color='green')
            self.all_scatter[canopy_id], = self.ax[canopy_id].plot([], [], 's', markersize=3, color='green')
            self.roi_plot[canopy_id], = self.ax[canopy_id].plot([], [], '-', linewidth=3, color='blue')
            self.roi_scatter[canopy_id], = self.ax[canopy_id].plot([], [], 's', markersize=5, color='blue')
            self.ax[canopy_id].set_xlim(-2, 22)
            self.ax[canopy_id].set_ylim(0, 5)
            self.ax[canopy_id].set_title(canopy_id)

        # volume estimate for each canopy
        self.all_canopy_volume = defaultdict(dict)

        # publishers, subscribers and services
        qos_reliable_transient_local = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.canopy_data_sub = self.create_subscription(CanopyDataArray, 'canopy_data', self.canopy_data_callback, 10)
        self.init_canopy_region_client = self.create_client(InitializeCanopyRegion, 'initialize_canopy_region')
        self.canopy_region_of_interest_pub = self.create_publisher(CanopyRegionOfInterest, 'canopy_region_of_interest', qos_profile=qos_reliable_transient_local)
        self.spray_regulator_status_pub = self.create_publisher(SprayRegulatorStatus, 'spray_regulator_status', qos_profile=qos_reliable_transient_local)

        # services that depend on other services
        while not self.init_canopy_region_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('service not available, waiting...')

        self.start_row_spraying_service = self.create_service(StartRowSpraying, 'start_row_spraying', self.start_row_spraying_callback)
        self.stop_row_spraying_service = self.create_service(StopRowSpraying, 'stop_row_spraying', self.stop_row_spraying_callback)

    def start_row_spraying_callback(self, request: StartRowSpraying_Request, response: StartRowSpraying_Response):
        self.get_logger().info(f"start_row_spraying_callback: {request.row_id}")

        p_1: PointStamped = request.start
        p_2: PointStamped = request.end

        canopy_radius = self.max_canopy_width/2
        canopy_length = np.linalg.norm(np.array([p_2.point.x, p_2.point.y]) - np.array([p_1.point.x, p_1.point.y]))
        canopy_yaw = np.arctan2(p_2.point.y - p_1.point.y, p_2.point.x - p_1.point.x)
        canopy_q = quaternion_from_euler(0, 0, canopy_yaw)

        self.broadcast_static_transform(child_frame_id=request.row_id, p=p_1, q=canopy_q)

        init_canopy_region_result_future = self.init_canopy_region_client.call_async(InitializeCanopyRegion.Request(
            canopy_id=request.row_id,
            canopy_frame_id=request.row_id,
            min_x=-canopy_radius,
            max_x=canopy_length + canopy_radius,
            min_y=-canopy_radius,
            max_y=canopy_radius,
            min_z=self.canopy_layers[0],
            max_z=self.canopy_layers[-1],
            roi=CanopyRegionOfInterest(
                frame_id='base_link',
                x_1=-1.0,
                x_2=1.0,
            ),
        ))

        init_canopy_region_result_future.add_done_callback(lambda f: self.start_row_spraying_response_callback(f, request.row_id))
        response.result = True
        return response

    def start_row_spraying_response_callback(self, future: Future, row_id: str):
        result = future.result().result
        if not result:
            self.get_logger().error(f"initialize_canopy_region row_id: {row_id} result: {result}")
            self.spray_regulator_status_pub.publish(SprayRegulatorStatus(header=Header(stamp=self.get_clock().now().to_msg()), status=SprayRegulatorStatus.STATUS_FAILED))
            # TODO stop all regulation in progress
        else:
            self.get_logger().info(f"initialize_canopy_region row_id: {row_id} result: {result}")

    @staticmethod
    def stop_row_spraying_callback(request: StopRowSpraying_Request, response: StopRowSpraying_Response):
        # TODO
        response.result = True  # TODO
        return response

    def canopy_data_callback(self, canopy_data_array_msg: CanopyDataArray):
        canopy_data_msg: CanopyData
        for canopy_data_msg in canopy_data_array_msg.canopy_data_array:
            self.spray_regulator_status_pub.publish(SprayRegulatorStatus(header=canopy_data_msg.header, row_id=canopy_data_msg.canopy_id, status=SprayRegulatorStatus.STATUS_OK))

            roi_canopy_volume_ = dict()
            for x, y in zip(canopy_data_msg.volume_x_array, canopy_data_msg.volume_y_array):
                self.all_canopy_volume[canopy_data_msg.canopy_id][x] = y
                roi_canopy_volume_[x] = y

            if len(self.all_canopy_volume[canopy_data_msg.canopy_id]):
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

    def broadcast_static_transform(self, child_frame_id: str, p: PointStamped, q: list[float]):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = p.header.frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = p.point.x
        t.transform.translation.y = p.point.y
        t.transform.translation.z = p.point.z
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_static_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SprayingRegulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
