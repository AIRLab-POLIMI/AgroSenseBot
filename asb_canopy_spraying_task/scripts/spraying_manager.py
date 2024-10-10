#!/usr/bin/python3

from __future__ import annotations

import os
from enum import Enum

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy import Future
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.timer import Timer

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, PointStamped
from asb_msgs.msg import (CanopyRegionOfInterest,
                          CanopyDataArray, CanopyData,
                          CanopyLayerDepthArray, CanopyLayerDepth,
                          PlatformState, FanCmd, NozzleCommandArray, NozzleCommand, PumpCmd)
from asb_msgs.srv import InitializeCanopyRegion, InitializeCanopyRegion_Request, SuspendCanopyRegion, SuspendCanopyRegion_Request

from collections import defaultdict
import numpy as np

from spraying_task_plan import TaskPlanItem

from typing_extensions import Self
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from spraying_task_sm import SprayingTaskPlanExecutor

np.set_printoptions(precision=2)


class SprayingStatus(Enum):
    NOT_SPRAYING = 0
    STARTING = 1
    STARTED = 2
    FAILURE = 3


class SprayingSide(Enum):
    LEFT = 0
    RIGHT = 1
    UNKNOWN = 2

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


class SprayingManager:

    def __init__(self, node: SprayingTaskPlanExecutor):
        self._node = node

        self._node.declare_parameter('canopy_data_timeout', rclpy.Parameter.Type.DOUBLE)
        self._canopy_data_timeout = Duration(seconds=self._node.get_parameter('canopy_data_timeout').get_parameter_value().double_value)

        self._node.declare_parameter('velocity_timeout', rclpy.Parameter.Type.DOUBLE)
        self._velocity_timeout = Duration(seconds=self._node.get_parameter('velocity_timeout').get_parameter_value().double_value)

        self._node.declare_parameter('fan_rpm_timeout', rclpy.Parameter.Type.DOUBLE)
        self._fan_rpm_timeout = Duration(seconds=self._node.get_parameter('fan_rpm_timeout').get_parameter_value().double_value)

        self._node.declare_parameter('canopy_roi_frame_id', rclpy.Parameter.Type.STRING)
        self._canopy_roi_frame_id = self._node.get_parameter('canopy_roi_frame_id').get_parameter_value().string_value

        self._node.declare_parameter('nozzles_configuration_file_path', rclpy.Parameter.Type.STRING)
        nozzles_configuration_file_path = os.path.expanduser(self._node.get_parameter('nozzles_configuration_file_path').get_parameter_value().string_value)

        self._node.declare_parameter('nozzle_rate_lookup_table_file_path', rclpy.Parameter.Type.STRING)
        nozzle_rate_lookup_table_file_path = os.path.expanduser(self._node.get_parameter('nozzle_rate_lookup_table_file_path').get_parameter_value().string_value)

        self._node.declare_parameter('enable_pump', rclpy.Parameter.Type.BOOL)
        self._enable_pump = self._node.get_parameter('enable_pump').get_parameter_value().bool_value

        # parameters from task plan configuration
        self._fan_velocity_threshold_rpm = self._node.task_plan.fan_velocity_threshold_rpm
        self._fan_velocity_target_rpm = self._node.task_plan.fan_velocity_target_rpm
        self._max_canopy_width = self._node.task_plan.max_canopy_width
        self._canopy_roi_x_1 = self._node.task_plan.canopy_roi_x_1
        self._canopy_roi_x_2 = self._node.task_plan.canopy_roi_x_2
        self._normalizing_velocity = self._node.task_plan.normalizing_velocity
        self._canopy_layer_bounds = self._node.task_plan.canopy_layer_bounds

        # canopy layer bound configuration variables
        if len(self._canopy_layer_bounds) < 2:
            self._node.get_logger().fatal(f"canopy_layer_bounds should have at least two values, but it has {len(self._canopy_layer_bounds)}")
            raise ValueError("one or more parameters are not correct")
        if not np.all(np.diff(np.array(self._canopy_layer_bounds)) > 0):
            self._node.get_logger().fatal(f"height values in canopy_layer_bounds are not monotonically increasing, but they should be")
            raise ValueError("one or more parameters are not correct")

        self._canopy_layer_bound_pairs = list(zip(self._canopy_layer_bounds[0:-1], self._canopy_layer_bounds[1:]))

        # nozzles configuration variables
        self._nozzles_by_side_layer: defaultdict[(SprayingSide, float), list] = defaultdict(list)
        with open(nozzles_configuration_file_path, 'r') as f:
            nozzles_configuration = yaml.safe_load(f)
        if not isinstance(nozzles_configuration, list):
            self._node.get_logger().fatal(f"nozzles_configuration is not of type list in file {nozzles_configuration_file_path}")
            raise TypeError("one or more parameters have the wrong type")
        for nozzle_configuration in nozzles_configuration:
            if 'id' not in nozzle_configuration or 'side' not in nozzle_configuration or 'spray_height' not in nozzle_configuration:
                self._node.get_logger().fatal(f"nozzle configuration does not have one or more of the required fields in file {nozzles_configuration_file_path}")
                raise ValueError("one or more parameters are not correct")
            if not isinstance(nozzle_configuration['id'], str):
                self._node.get_logger().fatal(f"nozzle id is not of type str [nozzle_id={nozzle_configuration['id']}] in file {nozzles_configuration_file_path}")
                raise TypeError("one or more parameters have the wrong type")
            if not nozzle_configuration['side'] in ['left', 'right']:
                self._node.get_logger().fatal(f"nozzle side is neither left nor right [nozzle_id={nozzle_configuration['id']}] in nozzles configuration file [{nozzles_configuration_file_path}]")
                raise ValueError("one or more parameters are not correct")
            if not isinstance(nozzle_configuration['spray_height'], (float, int)):
                self._node.get_logger().fatal(f"nozzle spray_height is not of type float or int [nozzle_id={nozzle_configuration['id']}] in file {nozzles_configuration_file_path}")
                raise TypeError("one or more parameters have the wrong type")

            if not self._canopy_layer_bounds[0] < nozzle_configuration['spray_height'] < self._canopy_layer_bounds[-1]:
                self._node.get_logger().warn(f"nozzle [nozzle_id={nozzle_configuration['id']}] will never be used: spray_height [{nozzle_configuration['spray_height']}] is outside any layer bound [min: {self._canopy_layer_bounds[0]}, max: {self._canopy_layer_bounds[-1]}] in file {nozzles_configuration_file_path}.")

            for z_1, z_2 in self._canopy_layer_bound_pairs:
                if z_1 <= nozzle_configuration['spray_height'] < z_2:
                    side = SprayingSide.from_str(nozzle_configuration['side'])
                    self._nozzles_by_side_layer[(side, z_1)].append(nozzle_configuration)

        self._node.get_logger().info(f"configured nozzles by side and layer:")
        for (side, z_1), nozzles in self._nozzles_by_side_layer.items():
            self._node.get_logger().info(f"side: {side.name.lower()}, layer: {z_1}, nozzles: {list(map(lambda n: n['id'], nozzles))}")

        # nozzle rate function variables
        with open(nozzle_rate_lookup_table_file_path, 'r') as f:
            nozzle_rate_lookup_table = yaml.safe_load(f)
        if not isinstance(nozzle_rate_lookup_table, dict):
            self._node.get_logger().fatal(f"nozzle_rate_lookup_table is not of type dict in file {nozzle_rate_lookup_table_file_path}")
            raise TypeError("one or more parameters have the wrong type")
        for k, v in nozzle_rate_lookup_table.items():
            if not isinstance(k, (float, int)) or not isinstance(v, (float, int)):
                self._node.get_logger().fatal(f"key or value in nozzle_rate_lookup_table is not of type float or int in file {nozzle_rate_lookup_table_file_path}")
                raise TypeError("one or more parameters have the wrong type")
        if len(nozzle_rate_lookup_table) < 2:
            self._node.get_logger().fatal(f"less than 2 key-value pairs specified in nozzle_rate_lookup_table in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        if not np.all(np.diff(np.array(list(nozzle_rate_lookup_table.keys()))) > 0):
            self._node.get_logger().fatal(f"keys in nozzle_rate_lookup_table are not monotonically increasing in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        if not np.all(np.diff(np.array(list(nozzle_rate_lookup_table.values()))) > 0):
            self._node.get_logger().fatal(f"values in nozzle_rate_lookup_table are not monotonically increasing in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        min_lut_key = np.min(list(nozzle_rate_lookup_table.keys()))
        if min_lut_key < 0.0:
            self._node.get_logger().fatal(f"smaller key of nozzle_rate_lookup_table [{min_lut_key}] is not greater or equal to 0 in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        min_lut_value = np.min(list(nozzle_rate_lookup_table.values()))
        if min_lut_value < 0.0:
            self._node.get_logger().fatal(f"minimum value of nozzle_rate_lookup_table [{min_lut_value}] is not greater or equal to 0 in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        max_lut_value = np.max(list(nozzle_rate_lookup_table.values()))
        if max_lut_value > 1.0:
            self._node.get_logger().fatal(f"maximum value of nozzle_rate_lookup_table [{max_lut_value}] is not less or equal to 1 in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")

        self._nozzle_rate_lookup_table_keys = list(nozzle_rate_lookup_table.keys())
        self._nozzle_rate_lookup_table_values = list(nozzle_rate_lookup_table.values())

        # spraying variables
        self.spraying_status: SprayingStatus = SprayingStatus.NOT_SPRAYING
        self._active_spraying_requests: dict[str, SprayingRequest] = dict()
        self._last_fan_rpm_time: Time | None = None
        self._fan_rpm: int = 0
        self._last_velocity_time: Time | None = None
        self._current_velocity: float | None = None
        self._service_call_max_attempts: int = 3

        # publishers, subscribers and services
        qos_reliable_transient_local = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._tf_static_broadcaster = StaticTransformBroadcaster(node)
        self._platform_state_sub = self._node.create_subscription(PlatformState, '/asb_platform_controller/platform_state', self._platform_state_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)
        self._velocity_odom_sub = self._node.create_subscription(Odometry, 'velocity_odom', self._velocity_odom_callback, 1)
        self._canopy_data_sub = self._node.create_subscription(CanopyDataArray, 'canopy_data', self._canopy_data_callback, 10)
        self._canopy_region_of_interest_pub = self._node.create_publisher(CanopyRegionOfInterest, 'canopy_region_of_interest', qos_profile=qos_reliable_transient_local)
        self._fan_command_pub = self._node.create_publisher(FanCmd, '/asb_platform_controller/fan_cmd', qos_profile=rclpy.qos.qos_profile_sensor_data)
        self._pump_command_pub = self._node.create_publisher(PumpCmd, '/asb_platform_controller/pump_cmd', qos_profile=rclpy.qos.qos_profile_sensor_data)
        self._canopy_depth_pub = self._node.create_publisher(CanopyLayerDepthArray, '/canopy_layer_depth', qos_profile=rclpy.qos.qos_profile_sensor_data)
        self._nozzles_command_pub = self._node.create_publisher(NozzleCommandArray, '/nozzles_command', qos_profile=rclpy.qos.qos_profile_sensor_data)

        self._init_canopy_region_client = self._node.create_client(InitializeCanopyRegion, 'initialize_canopy_region')
        self._suspend_canopy_region_client = self._node.create_client(SuspendCanopyRegion, 'suspend_canopy_region')
        self._init_canopy_region_timeout_timer: dict[str, Timer] = dict()
        self._init_canopy_region_timeout_counter: dict[str, int] = dict()
        self._suspend_canopy_region_timeout_timer: dict[str, Timer] = dict()
        self._suspend_canopy_region_timeout_counter: dict[str, int] = dict()

        self._node.create_timer(0.05, self._timer_callback)

    def setup(self):
        while not self._init_canopy_region_client.wait_for_service(timeout_sec=0.1):
            self._node.get_logger().info('initialize_canopy_region service not available, waiting...', throttle_duration_sec=5.0)

        while self._last_velocity_time is None:
            self._node.get_logger().info(f"waiting to receive velocity message...", throttle_duration_sec=5.0)

    def _timer_callback(self) -> None:

        def zero_all_cmds():
            self._fan_command_pub.publish(FanCmd(stamp=self._node.get_clock().now().to_msg(), velocity_rpm=0))
            self._pump_command_pub.publish(PumpCmd(stamp=self._node.get_clock().now().to_msg(), pump_cmd=False))
            self._nozzles_command_pub.publish(NozzleCommandArray(stamp=self._node.get_clock().now().to_msg()))

        if self._node.dry_run:
            if len(self._active_spraying_requests) == 0:
                self.spraying_status = SprayingStatus.NOT_SPRAYING
            else:
                self.spraying_status = SprayingStatus.STARTED
            zero_all_cmds()
            return

        # if there was a failure, zero all commands and wait for the spraying state to be cleared
        if self.spraying_status == SprayingStatus.FAILURE:
            zero_all_cmds()
            return

        # if there is nothing to do, zero all commands
        if len(self._active_spraying_requests) == 0:
            self.spraying_status = SprayingStatus.NOT_SPRAYING
            zero_all_cmds()
            return

        # if the velocity is not available, go to FAILED state
        velocity_age = self._node.get_clock().now() - self._last_velocity_time
        if velocity_age > self._velocity_timeout:
            self._node.get_logger().error(f"last velocity message age [{velocity_age}] older than timeout [{self._velocity_timeout}]. Can not compute spray regulation.")
            self.spraying_status = SprayingStatus.FAILURE
            zero_all_cmds()
            return

        # if the fan velocity is not available, go to FAILED state
        fan_rpm_age = self._node.get_clock().now() - self._last_fan_rpm_time
        if fan_rpm_age > self._fan_rpm_timeout:
            self._node.get_logger().error(f"last fan_rpm message age [{fan_rpm_age}] older than timeout [{self._fan_rpm_timeout}]. Can not operate sprayer.")
            self.spraying_status = SprayingStatus.FAILURE
            zero_all_cmds()
            return

        # wait until the fan has spun up
        self._fan_command_pub.publish(FanCmd(stamp=self._node.get_clock().now().to_msg(), velocity_rpm=self._fan_velocity_target_rpm))
        self._pump_command_pub.publish(PumpCmd(stamp=self._node.get_clock().now().to_msg(), pump_cmd=self._enable_pump))
        if self._fan_rpm < self._fan_velocity_threshold_rpm:
            self._nozzles_command_pub.publish(NozzleCommandArray(stamp=self._node.get_clock().now().to_msg()))
            self.spraying_status = SprayingStatus.STARTING
            return

        # if everything is ok, compute the nozzle command for the requested sides
        nozzle_command_msg = NozzleCommandArray(stamp=self._node.get_clock().now().to_msg())
        for row_id, spraying_request in self._active_spraying_requests.items():

            # if we didn't receive any canopy data since initialization, there is a problem
            if spraying_request.last_canopy_data_msg is None:
                canopy_data_init_wait_time = self._node.get_clock().now() - spraying_request.init_time
                if canopy_data_init_wait_time <= self._canopy_data_timeout:
                    self._node.get_logger().warn(f"still waiting for first canopy data [{canopy_data_init_wait_time.nanoseconds/1e9} s] for row {row_id}, can not compute spray regulation")
                    continue
                else:
                    # if we still didn't receive the canopy data after the timeout, go to failure state
                    self._node.get_logger().error(f"waited for first canopy data [{canopy_data_init_wait_time.nanoseconds/1e9} s] longer than timeout [{self._canopy_data_timeout.nanoseconds / 1e9} s] for row {row_id}")
                    self.spraying_status = SprayingStatus.FAILURE
                    zero_all_cmds()
                    return

            # if we didn't receive the canopy data for longer than timeout, there is a problem, go to failure state
            canopy_data_age = self._node.get_clock().now() - Time.from_msg(spraying_request.last_canopy_data_msg.header.stamp)
            if canopy_data_age > self._canopy_data_timeout:
                self._node.get_logger().error(f"canopy data age [{canopy_data_age.nanoseconds/1e9} s] older than timeout [{self._canopy_data_timeout.nanoseconds / 1e9} s]. Can not compute spray regulation for row {row_id}.")
                self.spraying_status = SprayingStatus.FAILURE
                zero_all_cmds()
                return

            # compute nozzle rates
            for z_1, _ in self._canopy_layer_bound_pairs:
                mean_depth = spraying_request.mean_depth[z_1]
                rate = self._current_velocity / self._normalizing_velocity * np.interp(mean_depth, self._nozzle_rate_lookup_table_keys, self._nozzle_rate_lookup_table_values, left=0.0)
                nozzles = self._nozzles_by_side_layer[(spraying_request.side, z_1)]
                for nozzle in nozzles:
                    nozzle_command_msg.nozzle_command_array.append(NozzleCommand(
                        nozzle_id=nozzle['id'],
                        rate=rate,
                    ))

        self.spraying_status = SprayingStatus.STARTED
        self._nozzles_command_pub.publish(nozzle_command_msg)

    def _velocity_odom_callback(self, velocity_odom_msg: Odometry) -> None:
        self._last_velocity_time = Time.from_msg(velocity_odom_msg.header.stamp)
        self._current_velocity = velocity_odom_msg.twist.twist.linear.x

    def _platform_state_callback(self, platform_state_msg: PlatformState) -> None:
        self._last_fan_rpm_time = Time.from_msg(platform_state_msg.stamp)
        self._fan_rpm = platform_state_msg.fan_motor_velocity_rpm

    def _canopy_data_callback(self, canopy_data_array_msg: CanopyDataArray) -> None:
        canopy_layer_depth_array_msg = CanopyLayerDepthArray()
        canopy_data_msg: CanopyData
        for canopy_data_msg in canopy_data_array_msg.canopy_data_array:
            if canopy_data_msg.canopy_id in self._active_spraying_requests:
                self._active_spraying_requests[canopy_data_msg.canopy_id].last_canopy_data_msg = canopy_data_msg

                layer_depth_sum_count = defaultdict(lambda: [0.0, 0])
                for _, y_depth, z in zip(canopy_data_msg.depth_x_array, canopy_data_msg.depth_y_array, canopy_data_msg.depth_z_array):
                    for layer_z_1, layer_z_2 in self._canopy_layer_bound_pairs:
                        if layer_z_1 < z < layer_z_2:
                            layer_depth_sum_count[layer_z_1][0] += y_depth
                            layer_depth_sum_count[layer_z_1][1] += 1

                x = (canopy_data_msg.roi.x_1 + canopy_data_msg.roi.x_2) / 2
                x_round = np.round(x / canopy_data_msg.resolution) * canopy_data_msg.resolution
                for layer_z_1, _ in self._canopy_layer_bound_pairs:
                    depth_sum, count = layer_depth_sum_count[layer_z_1]
                    self._active_spraying_requests[canopy_data_msg.canopy_id].mean_depth[layer_z_1] = (depth_sum / count) if count > 0 else 0.0

                canopy_layer_depth = CanopyLayerDepth(
                    canopy_id=canopy_data_msg.canopy_id,
                    header=canopy_data_msg.header,
                    roi=canopy_data_msg.roi,
                    resolution=canopy_data_msg.resolution,
                    x=x_round,
                    mean_depth=self._active_spraying_requests[canopy_data_msg.canopy_id].mean_depth.values(),
                    canopy_layer_bounds=self._canopy_layer_bounds,
                )
                canopy_layer_depth_array_msg.canopy_layer_depth_array.append(canopy_layer_depth)

        self._canopy_depth_pub.publish(canopy_layer_depth_array_msg)

    def start_spray_regulator(self, item: TaskPlanItem) -> None:
        if len(self._active_spraying_requests) > 0:
            self._node.get_logger().error(f"start row spraying while already spraying")

        if item.get_left_row_id() is None and item.get_right_row_id() is None:
            self._node.get_logger().error(f"neither left nor right row spraying in row item [{item.get_item_id()}]")

        if item.get_left_row_id() is not None:
            left_row = self._node.task_plan.get_row(item.get_left_row_id())
            self._node.get_logger().info(f"starting spray regulator for left row [{left_row.get_row_id()}]")

            self._init_canopy_volume_estimation(
                row_id=left_row.get_row_id(),
                side=SprayingSide.LEFT,
                p_1=left_row.get_start_point(),
                p_2=left_row.get_end_point()
            )

        if item.get_right_row_id() is not None:
            right_row = self._node.task_plan.get_row(item.get_right_row_id())
            self._node.get_logger().info(f"starting spray regulator for right row [{right_row.get_row_id()}]")

            self._init_canopy_volume_estimation(
                row_id=right_row.get_row_id(),
                side=SprayingSide.RIGHT,
                p_1=right_row.get_start_point(),
                p_2=right_row.get_end_point()
            )

    def _init_canopy_volume_estimation(self, row_id: str, side: SprayingSide, p_1: PointStamped, p_2: PointStamped):
        canopy_radius = self._max_canopy_width / 2
        canopy_length = np.linalg.norm(np.array([p_2.point.x, p_2.point.y]) - np.array([p_1.point.x, p_1.point.y]))
        canopy_yaw = np.arctan2(p_2.point.y - p_1.point.y, p_2.point.x - p_1.point.x)
        canopy_q = quaternion_from_euler(0, 0, canopy_yaw)

        self._broadcast_static_transform(child_frame_id=row_id, p=p_1, q=canopy_q)

        request = InitializeCanopyRegion_Request(
            canopy_id=row_id,
            canopy_frame_id=row_id,
            min_x=-canopy_radius,
            max_x=canopy_length + canopy_radius,
            min_y=-canopy_radius,
            max_y=canopy_radius,
            min_z=self._canopy_layer_bounds[0],
            max_z=self._canopy_layer_bounds[-1],
            roi=CanopyRegionOfInterest(
                frame_id=self._canopy_roi_frame_id,
                x_1=self._canopy_roi_x_1,
                x_2=self._canopy_roi_x_2,
            ),
        )

        self._init_canopy_region_timeout_counter[row_id] = 0
        if row_id in self._init_canopy_region_timeout_timer:
            self._init_canopy_region_timeout_timer[row_id].cancel()

        init_canopy_region_response_future = self._init_canopy_region_client.call_async(request)
        self._init_canopy_region_timeout_timer[row_id] = self._node.create_timer(1.0, lambda: self._init_canopy_region_timeout_callback(request, row_id, side))
        init_canopy_region_response_future.add_done_callback(lambda f: self._init_canopy_region_response_callback(f, row_id, side))

    def _init_canopy_region_timeout_callback(self, request: InitializeCanopyRegion_Request, row_id: str, side: SprayingSide):
        self._init_canopy_region_timeout_timer[row_id].cancel()
        self._init_canopy_region_timeout_counter[row_id] += 1

        if self._init_canopy_region_timeout_counter[row_id] < self._service_call_max_attempts:
            self._node.get_logger().warn(f"init_canopy_region service call timeout, retrying (retry attempt {self._init_canopy_region_timeout_counter[row_id]} of {self._service_call_max_attempts}). [row_id={row_id}]")
        else:
            self.spraying_status = SprayingStatus.FAILURE
            self._init_canopy_region_timeout_counter[row_id] = 0
            self._node.get_logger().error(f"init_canopy_region service call timeout, max attempts reached [{self._service_call_max_attempts}] [row_id={row_id}].")
            return

        # retry
        init_canopy_region_response_future = self._init_canopy_region_client.call_async(request)
        self._init_canopy_region_timeout_timer[row_id] = self._node.create_timer(1.0, lambda: self._init_canopy_region_timeout_callback(request, row_id, side))
        init_canopy_region_response_future.add_done_callback(lambda f: self._init_canopy_region_response_callback(f, row_id, side))

    def _init_canopy_region_response_callback(self, future: Future, row_id: str, spraying_side: SprayingSide) -> None:
        self._init_canopy_region_timeout_counter[row_id] = 0
        self._init_canopy_region_timeout_timer[row_id].cancel()

        result = future.result().result
        if result:
            self._active_spraying_requests[row_id] = SprayingRequest(side=spraying_side, init_time=self._node.get_clock().now())
        else:
            self._node.get_logger().error(f"init_canopy_region_response: row_id: {row_id} result: {result}")
            self.spraying_status = SprayingStatus.FAILURE

    def stop_spray_regulator(self) -> None:
        for row_id in self._init_canopy_region_timeout_timer.keys():
            self._init_canopy_region_timeout_counter[row_id] = 0
            self._init_canopy_region_timeout_timer[row_id].cancel()

        if len(self._active_spraying_requests) == 0:
            self._node.get_logger().warn(f"stop row spraying while already not spraying")
            return

        for row_id in list(self._active_spraying_requests.keys()):
            self._active_spraying_requests.pop(row_id)
            self._suspend_canopy_volume_estimation(row_id)

    def _suspend_canopy_volume_estimation(self, row_id):
        self._suspend_canopy_region_timeout_counter[row_id] = 0
        if row_id in self._suspend_canopy_region_timeout_timer:
            self._suspend_canopy_region_timeout_timer[row_id].cancel()

        suspend_canopy_region_response_future = self._suspend_canopy_region_client.call_async(SuspendCanopyRegion_Request(canopy_id=row_id))
        self._suspend_canopy_region_timeout_timer[row_id] = self._node.create_timer(1.0, lambda: self._suspend_canopy_region_timeout_callback(row_id))
        suspend_canopy_region_response_future.add_done_callback(lambda f: self._suspend_canopy_region_response_callback(f, row_id))

    def _suspend_canopy_region_timeout_callback(self, row_id: str):
        self._suspend_canopy_region_timeout_timer[row_id].cancel()
        self._suspend_canopy_region_timeout_counter[row_id] += 1
        if self._suspend_canopy_region_timeout_counter[row_id] < self._service_call_max_attempts:
            self._node.get_logger().warn(f"suspend_canopy_region service call timeout, retrying (retry attempt {self._suspend_canopy_region_timeout_counter[row_id]} of {self._service_call_max_attempts}). [row_id={row_id}]")
        else:
            self.spraying_status = SprayingStatus.FAILURE
            self._suspend_canopy_region_timeout_counter[row_id] = 0
            self._node.get_logger().error(f"suspend_canopy_region service call timeout, max attempts reached [{self._service_call_max_attempts}]. [row_id={row_id}]")
            return

        # retry
        suspend_canopy_region_response_future = self._suspend_canopy_region_client.call_async(SuspendCanopyRegion_Request(canopy_id=row_id))
        self._suspend_canopy_region_timeout_timer[row_id] = self._node.create_timer(1.0, lambda: self._suspend_canopy_region_timeout_callback(row_id))
        suspend_canopy_region_response_future.add_done_callback(lambda f: self._suspend_canopy_region_response_callback(f, row_id))

    def _suspend_canopy_region_response_callback(self, future: Future, row_id: str) -> None:
        self._suspend_canopy_region_timeout_counter[row_id] = 0
        self._suspend_canopy_region_timeout_timer[row_id].cancel()

        result = future.result().result
        if not result:
            self._node.get_logger().error(f"suspend_canopy_region_response_callback: row_id: {row_id} result: {result}")

    def _broadcast_static_transform(self, child_frame_id: str, p: PointStamped, q: list[float]) -> None:
        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = p.header.frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = p.point.x
        t.transform.translation.y = p.point.y
        t.transform.translation.z = p.point.z
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self._tf_static_broadcaster.sendTransform(t)
