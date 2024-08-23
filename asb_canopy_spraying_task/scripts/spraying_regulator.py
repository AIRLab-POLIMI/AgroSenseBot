#!/usr/bin/python3
import os
from enum import Enum

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy import Future
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Header

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, PointStamped
from asb_msgs.msg import (CanopyRegionOfInterest,
                          CanopyDataArray, CanopyData,
                          CanopyLayerDepthArray, CanopyLayerDepth,
                          SprayRegulatorStatus, PlatformState,
                          FanCmd, NozzleCommandArray, NozzleCommand)
from asb_msgs.srv import (InitializeCanopyRegion, InitializeCanopyRegion_Request,
                          SuspendCanopyRegion, SuspendCanopyRegion_Request,
                          StartRowSpraying, StartRowSpraying_Request, StartRowSpraying_Response,
                          StopRowSpraying, StopRowSpraying_Request, StopRowSpraying_Response)

from collections import defaultdict
import numpy as np
from typing_extensions import Self

np.set_printoptions(precision=2)


class SprayingSide(Enum):
    LEFT = 0
    RIGHT = 1
    UNKNOWN = 2

    @classmethod
    def from_msg(cls, req: StartRowSpraying_Request) -> Self:
        if req.side == StartRowSpraying_Request.SIDE_LEFT:
            return SprayingSide.LEFT
        elif req.side == StartRowSpraying_Request.SIDE_RIGHT:
            return SprayingSide.RIGHT
        else:
            return SprayingSide.UNKNOWN

    @classmethod
    def from_str(cls, s: str) -> Self:
        if s == "left":
            return SprayingSide.LEFT
        elif s == "right":
            return SprayingSide.RIGHT
        else:
            return SprayingSide.UNKNOWN


class SprayingRequest:
    def __init__(self, side: SprayingSide, init_time: Time):
        self.side: SprayingSide = side
        self.init_time: Time = init_time
        self.last_canopy_data_msg: CanopyData | None = None
        self.mean_depth: defaultdict[float, float] = defaultdict(float)


class SprayingRegulator(Node):

    def __init__(self):
        super().__init__('spraying_regulator')

        self.declare_parameter('canopy_data_timeout', rclpy.Parameter.Type.DOUBLE)
        self.canopy_data_timeout = Duration(seconds=self.get_parameter('canopy_data_timeout').get_parameter_value().double_value)

        self.declare_parameter('velocity_timeout', rclpy.Parameter.Type.DOUBLE)
        self.velocity_timeout = Duration(seconds=self.get_parameter('velocity_timeout').get_parameter_value().double_value)

        self.declare_parameter('max_canopy_width', rclpy.Parameter.Type.DOUBLE)
        self.max_canopy_width = self.get_parameter('max_canopy_width').get_parameter_value().double_value

        self.declare_parameter('normalizing_velocity', rclpy.Parameter.Type.DOUBLE)
        self.normalizing_velocity = self.get_parameter('normalizing_velocity').get_parameter_value().double_value

        self.declare_parameter('canopy_layer_bounds', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.canopy_layer_bounds = self.get_parameter('canopy_layer_bounds').get_parameter_value().double_array_value

        self.declare_parameter('nozzles_configuration_file_path', rclpy.Parameter.Type.STRING)
        nozzles_configuration_file_path = os.path.expanduser(self.get_parameter('nozzles_configuration_file_path').get_parameter_value().string_value)

        self.declare_parameter('nozzle_rate_lookup_table_file_path', rclpy.Parameter.Type.STRING)
        nozzle_rate_lookup_table_file_path = os.path.expanduser(self.get_parameter('nozzle_rate_lookup_table_file_path').get_parameter_value().string_value)

        self.declare_parameter('fan_velocity_target_rpm', rclpy.Parameter.Type.INTEGER)
        self.fan_velocity_target_rpm = self.get_parameter('fan_velocity_target_rpm').get_parameter_value().integer_value

        self.declare_parameter('fan_velocity_threshold_rpm', rclpy.Parameter.Type.INTEGER)
        self.fan_velocity_threshold_rpm = self.get_parameter('fan_velocity_threshold_rpm').get_parameter_value().integer_value

        self.declare_parameter('velocity_threshold', rclpy.Parameter.Type.DOUBLE)
        self.velocity_threshold = self.get_parameter('velocity_threshold').get_parameter_value().integer_value

        # canopy layer bound configuration variables
        if len(self.canopy_layer_bounds) < 2:
            self.get_logger().fatal(f"canopy_layer_bounds should have at least two values, but it has {len(self.canopy_layer_bounds)}")
            raise ValueError("one or more parameters are not correct")
        if not np.all(np.diff(np.array(self.canopy_layer_bounds)) > 0):
            self.get_logger().fatal(f"height values in canopy_layer_bounds are not monotonically increasing, but they should be")
            raise ValueError("one or more parameters are not correct")

        self.canopy_layer_bound_pairs = list(zip(self.canopy_layer_bounds[0:-1], self.canopy_layer_bounds[1:]))

        # nozzles configuration variables
        self.nozzles_by_side_layer: defaultdict[(SprayingSide, float), list] = defaultdict(list)
        with open(nozzles_configuration_file_path, 'r') as f:
            nozzles_configuration = yaml.safe_load(f)
        if not isinstance(nozzles_configuration, list):
            self.get_logger().fatal(f"nozzles_configuration is not of type list in file {nozzles_configuration_file_path}")
            raise TypeError("one or more parameters have the wrong type")
        for nozzle_configuration in nozzles_configuration:
            if 'id' not in nozzle_configuration or 'side' not in nozzle_configuration or 'spray_height' not in nozzle_configuration:
                self.get_logger().fatal(f"nozzle configuration does not have one or more of the required fields in file {nozzles_configuration_file_path}")
                raise ValueError("one or more parameters are not correct")
            if not isinstance(nozzle_configuration['id'], str):
                self.get_logger().fatal(f"nozzle id is not of type str [nozzle_id={nozzle_configuration['id']}] in file {nozzles_configuration_file_path}")
                raise TypeError("one or more parameters have the wrong type")
            if not nozzle_configuration['side'] in ['left', 'right']:
                self.get_logger().fatal(f"nozzle side is neither left nor right [nozzle_id={nozzle_configuration['id']}] in nozzles configuration file [{nozzles_configuration_file_path}]")
                raise ValueError("one or more parameters are not correct")
            if not isinstance(nozzle_configuration['spray_height'], (float, int)):
                self.get_logger().fatal(f"nozzle spray_height is not of type float or int [nozzle_id={nozzle_configuration['id']}] in file {nozzles_configuration_file_path}")
                raise TypeError("one or more parameters have the wrong type")

            if not self.canopy_layer_bounds[0] < nozzle_configuration['spray_height'] < self.canopy_layer_bounds[-1]:
                self.get_logger().warn(f"nozzle [nozzle_id={nozzle_configuration['id']}] will never be used: spray_height [{nozzle_configuration['spray_height']}] is outside any layer bound [min: {self.canopy_layer_bounds[0]}, max: {self.canopy_layer_bounds[-1]}] in file {nozzles_configuration_file_path}.")

            for z_1, z_2 in self.canopy_layer_bound_pairs:
                if z_1 <= nozzle_configuration['spray_height'] < z_2:
                    side = SprayingSide.from_str(nozzle_configuration['side'])
                    self.nozzles_by_side_layer[(side, z_1)].append(nozzle_configuration)

        self.get_logger().info(f"configured nozzles by side and layer:")
        for (side, z_1), nozzles in self.nozzles_by_side_layer.items():
            self.get_logger().info(f"side: {side.name.lower()}, layer: {z_1}, nozzles: {list(map(lambda n: n['id'], nozzles))}")

        # nozzle rate function variables
        with open(nozzle_rate_lookup_table_file_path, 'r') as f:
            nozzle_rate_lookup_table = yaml.safe_load(f)
        if not isinstance(nozzle_rate_lookup_table, dict):
            self.get_logger().fatal(f"nozzle_rate_lookup_table is not of type dict in file {nozzle_rate_lookup_table_file_path}")
            raise TypeError("one or more parameters have the wrong type")
        for k, v in nozzle_rate_lookup_table.items():
            if not isinstance(k, (float, int)) or not isinstance(v, (float, int)):
                self.get_logger().fatal(f"key or value in nozzle_rate_lookup_table is not of type float or int in file {nozzle_rate_lookup_table_file_path}")
                raise TypeError("one or more parameters have the wrong type")
        if len(nozzle_rate_lookup_table) < 2:
            self.get_logger().fatal(f"less than 2 key-value pairs specified in nozzle_rate_lookup_table in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        if not np.all(np.diff(np.array(list(nozzle_rate_lookup_table.keys()))) > 0):
            self.get_logger().fatal(f"keys in nozzle_rate_lookup_table are not monotonically increasing in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        if not np.all(np.diff(np.array(list(nozzle_rate_lookup_table.values()))) > 0):
            self.get_logger().fatal(f"values in nozzle_rate_lookup_table are not monotonically increasing in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        min_lut_key = np.min(list(nozzle_rate_lookup_table.keys()))
        if min_lut_key < 0.0:
            self.get_logger().fatal(f"smaller key of nozzle_rate_lookup_table [{min_lut_key}] is not greater or equal to 0 in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        min_lut_value = np.min(list(nozzle_rate_lookup_table.values()))
        if min_lut_value < 0.0:
            self.get_logger().fatal(f"minimum value of nozzle_rate_lookup_table [{min_lut_value}] is not greater or equal to 0 in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        max_lut_value = np.max(list(nozzle_rate_lookup_table.values()))
        if max_lut_value > 1.0:
            self.get_logger().fatal(f"maximum value of nozzle_rate_lookup_table [{max_lut_value}] is not less or equal to 1 in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")

        self.nozzle_rate_lookup_table_keys = list(nozzle_rate_lookup_table.keys())
        self.nozzle_rate_lookup_table_values = list(nozzle_rate_lookup_table.values())

        # spraying variables
        self.active_spraying_requests: dict[str, SprayingRequest] = dict()
        self.fan_rpm: int = 0
        self.last_velocity_time: Time | None = None
        self.current_velocity: float | None = None

        # publishers, subscribers and services
        qos_reliable_transient_local = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.platform_state_sub = self.create_subscription(PlatformState, '/asb_platform_controller/platform_state', self.platform_state_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)
        self.velocity_odom_sub = self.create_subscription(Odometry, 'velocity_odom', self.velocity_odom_callback, 1)
        self.canopy_data_sub = self.create_subscription(CanopyDataArray, 'canopy_data', self.canopy_data_callback, 10)
        self.init_canopy_region_client = self.create_client(InitializeCanopyRegion, 'initialize_canopy_region')
        self.suspend_canopy_region_client = self.create_client(SuspendCanopyRegion, 'suspend_canopy_region')
        self.canopy_region_of_interest_pub = self.create_publisher(CanopyRegionOfInterest, 'canopy_region_of_interest', qos_profile=qos_reliable_transient_local)
        self.spray_regulator_status_pub = self.create_publisher(SprayRegulatorStatus, 'spray_regulator_status', qos_profile=qos_reliable_transient_local)
        self.fan_command_pub = self.create_publisher(FanCmd, '/asb_platform_controller/fan_cmd', qos_profile=rclpy.qos.qos_profile_sensor_data)
        self.canopy_depth_pub = self.create_publisher(CanopyLayerDepthArray, '/canopy_layer_depth', qos_profile=rclpy.qos.qos_profile_sensor_data)
        self.nozzles_command_pub = self.create_publisher(NozzleCommandArray, '/nozzles_command', qos_profile=rclpy.qos.qos_profile_sensor_data)

        # services that depend on other services
        while not self.init_canopy_region_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('initialize_canopy_region service not available, waiting...', throttle_duration_sec=5.0)
        self.start_row_spraying_service = self.create_service(StartRowSpraying, 'start_row_spraying', self.start_row_spraying_callback)
        self.stop_row_spraying_service = self.create_service(StopRowSpraying, 'stop_row_spraying', self.stop_row_spraying_callback)

        self.create_timer(0.05, self.timer_callback)

    def timer_callback(self) -> None:
        if len(self.active_spraying_requests) > 0:
            self.fan_command_pub.publish(FanCmd(stamp=self.get_clock().now().to_msg(), velocity_rpm=self.fan_velocity_target_rpm))
            # TODO activate pump, if enabled by parameter
        else:
            self.fan_command_pub.publish(FanCmd(stamp=self.get_clock().now().to_msg(), velocity_rpm=0))
            # TODO disable pump

        if self.last_velocity_time is None:
            self.get_logger().info(f"waiting to receive velocity message", throttle_duration_sec=5.0)
            return

        velocity_age = self.get_clock().now() - self.last_velocity_time

        nozzle_command_msg = NozzleCommandArray(stamp=self.get_clock().now().to_msg())
        for row_id, spraying_request in self.active_spraying_requests.items():
            if spraying_request.last_canopy_data_msg is not None:
                canopy_data_age = self.get_clock().now() - Time.from_msg(spraying_request.last_canopy_data_msg.header.stamp)
                if canopy_data_age < self.canopy_data_timeout and velocity_age < self.velocity_timeout:
                    if self.fan_rpm >= self.fan_velocity_threshold_rpm and self.current_velocity >= self.velocity_threshold:

                        # compute nozzles rate
                        for z_1, _ in self.canopy_layer_bound_pairs:
                            mean_depth = spraying_request.mean_depth[z_1]
                            rate = self.current_velocity / self.normalizing_velocity * np.interp(mean_depth, self.nozzle_rate_lookup_table_keys, self.nozzle_rate_lookup_table_values, left=0.0)
                            nozzles = self.nozzles_by_side_layer[(spraying_request.side, z_1)]
                            for nozzle in nozzles:
                                nozzle_command_msg.nozzle_command_array.append(NozzleCommand(
                                    nozzle_id=nozzle['id'],
                                    rate=rate,
                                ))

                        self.spray_regulator_status_pub.publish(SprayRegulatorStatus(
                            header=spraying_request.last_canopy_data_msg.header,
                            row_id=spraying_request.last_canopy_data_msg.canopy_id,
                            status=SprayRegulatorStatus.STATUS_OK,
                        ))

                else:
                    if canopy_data_age > self.canopy_data_timeout:
                        self.get_logger().warn(f"canopy data age [{canopy_data_age}] older than timeout [{self.canopy_data_timeout}]. Can not compute spray regulation for row {row_id}.")
                    if velocity_age > self.velocity_timeout:
                        self.get_logger().warn(f"last velocity message age [{velocity_age}] older than timeout [{self.velocity_timeout}]. Can not compute spray regulation for row {row_id}.")

                    self.spray_regulator_status_pub.publish(SprayRegulatorStatus(
                        header=Header(
                            frame_id=spraying_request.last_canopy_data_msg.header.frame_id,
                            stamp=self.get_clock().now().to_msg(),
                        ),
                        row_id=spraying_request.last_canopy_data_msg.canopy_id,
                        status=SprayRegulatorStatus.STATUS_FAILED,
                    ))
            else:
                canopy_data_wait_time = self.get_clock().now() - spraying_request.init_time
                if canopy_data_wait_time > self.canopy_data_timeout:
                    self.get_logger().warn(f"waited for first canopy data [{canopy_data_wait_time}] longer than timeout [{self.canopy_data_timeout}] for row {row_id}")

        self.nozzles_command_pub.publish(nozzle_command_msg)

    def velocity_odom_callback(self, velocity_odom_msg: Odometry) -> None:
        self.last_velocity_time = Time.from_msg(velocity_odom_msg.header.stamp)
        self.current_velocity = velocity_odom_msg.twist.twist.linear.x

    def canopy_data_callback(self, canopy_data_array_msg: CanopyDataArray) -> None:
        canopy_layer_depth_array_msg = CanopyLayerDepthArray()
        canopy_data_msg: CanopyData
        for canopy_data_msg in canopy_data_array_msg.canopy_data_array:
            if canopy_data_msg.canopy_id in self.active_spraying_requests:
                self.active_spraying_requests[canopy_data_msg.canopy_id].last_canopy_data_msg = canopy_data_msg

                layer_depth_sum_count = defaultdict(lambda: [0.0, 0])
                for _, y_depth, z in zip(canopy_data_msg.depth_x_array, canopy_data_msg.depth_y_array, canopy_data_msg.depth_z_array):
                    for layer_z_1, layer_z_2 in self.canopy_layer_bound_pairs:
                        if layer_z_1 < z < layer_z_2:
                            layer_depth_sum_count[layer_z_1][0] += y_depth
                            layer_depth_sum_count[layer_z_1][1] += 1

                x = (canopy_data_msg.roi.x_1 + canopy_data_msg.roi.x_2) / 2
                x_round = np.round(x / canopy_data_msg.resolution) * canopy_data_msg.resolution
                for layer_z_1, _ in self.canopy_layer_bound_pairs:
                    depth_sum, count = layer_depth_sum_count[layer_z_1]
                    self.active_spraying_requests[canopy_data_msg.canopy_id].mean_depth[layer_z_1] = (depth_sum / count) if count > 0 else 0.0

                canopy_layer_depth = CanopyLayerDepth(
                    canopy_id=canopy_data_msg.canopy_id,
                    header=canopy_data_msg.header,
                    roi=canopy_data_msg.roi,
                    resolution=canopy_data_msg.resolution,
                    x=x_round,
                    mean_depth=self.active_spraying_requests[canopy_data_msg.canopy_id].mean_depth.values(),
                    canopy_layer_bounds=self.canopy_layer_bounds,
                )
                canopy_layer_depth_array_msg.canopy_layer_depth_array.append(canopy_layer_depth)

        self.canopy_depth_pub.publish(canopy_layer_depth_array_msg)

    def platform_state_callback(self, platform_state_msg: PlatformState) -> None:
        self.fan_rpm = platform_state_msg.fan_motor_velocity_rpm

    def start_row_spraying_callback(self, request: StartRowSpraying_Request, response: StartRowSpraying_Response) -> StartRowSpraying_Response:
        p_1: PointStamped = request.start
        p_2: PointStamped = request.end
        canopy_radius = self.max_canopy_width/2
        canopy_length = np.linalg.norm(np.array([p_2.point.x, p_2.point.y]) - np.array([p_1.point.x, p_1.point.y]))
        canopy_yaw = np.arctan2(p_2.point.y - p_1.point.y, p_2.point.x - p_1.point.x)
        canopy_q = quaternion_from_euler(0, 0, canopy_yaw)

        self.broadcast_static_transform(child_frame_id=request.row_id, p=p_1, q=canopy_q)

        init_canopy_region_response_future = self.init_canopy_region_client.call_async(InitializeCanopyRegion_Request(
            canopy_id=request.row_id,
            canopy_frame_id=request.row_id,
            min_x=-canopy_radius,
            max_x=canopy_length + canopy_radius,
            min_y=-canopy_radius,
            max_y=canopy_radius,
            min_z=self.canopy_layer_bounds[0],
            max_z=self.canopy_layer_bounds[-1],
            roi=CanopyRegionOfInterest(
                frame_id='base_link',
                x_1=-1.0,
                x_2=1.0,
            ),
        ))
        init_canopy_region_response_future.add_done_callback(lambda f: self.init_canopy_region_response_callback(f, request.row_id, SprayingSide.from_msg(request)))
        response.result = True
        return response

    def init_canopy_region_response_callback(self, future: Future, row_id: str, spraying_side: SprayingSide) -> None:
        result = future.result().result
        if not result:
            self.get_logger().error(f"start_row_spraying_response_callback: row_id: {row_id} result: {result}")
            self.spray_regulator_status_pub.publish(SprayRegulatorStatus(
                header=Header(stamp=self.get_clock().now().to_msg()),
                row_id=row_id,
                status=SprayRegulatorStatus.STATUS_FAILED,
            ))
        else:
            self.active_spraying_requests[row_id] = SprayingRequest(side=spraying_side, init_time=self.get_clock().now())

    def stop_row_spraying_callback(self, request: StopRowSpraying_Request, response: StopRowSpraying_Response) -> StopRowSpraying_Response:
        self.spray_regulator_status_pub.publish(SprayRegulatorStatus(
            header=Header(stamp=self.get_clock().now().to_msg()),
            row_id=request.row_id,
            status=SprayRegulatorStatus.STATUS_STOPPED,
        ))

        self.active_spraying_requests.pop(request.row_id)

        self.suspend_canopy_region_client.call_async(SuspendCanopyRegion_Request(
            canopy_id=request.row_id,
        ))
        response.result = True
        return response

    def broadcast_static_transform(self, child_frame_id: str, p: PointStamped, q: list[float]) -> None:
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
