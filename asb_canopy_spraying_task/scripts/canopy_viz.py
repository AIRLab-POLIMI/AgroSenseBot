#!/usr/bin/python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from asb_msgs.msg import CanopyDataArray, CanopyData

import yaml
import os
from datetime import datetime
from collections import defaultdict
import numpy as np
import cv2
import time

from spraying_task_plan import SprayingTaskPlan

np.set_printoptions(precision=2)


class CanopyDataViz(Node):

    def __init__(self):
        super().__init__('canopy_data_viz')

        self.bridge = CvBridge()

        # task plan variables
        self.declare_parameter('task_plan_file_path', rclpy.Parameter.Type.STRING)
        self.task_plan_file_path = os.path.expanduser(self.get_parameter('task_plan_file_path').get_parameter_value().string_value)

        self.task_plan: SprayingTaskPlan = SprayingTaskPlan.load(self.task_plan_file_path)

        # nozzle rate function variables
        self.declare_parameter('nozzle_rate_lookup_table_file_path', rclpy.Parameter.Type.STRING)
        nozzle_rate_lookup_table_file_path = os.path.expanduser(self.get_parameter('nozzle_rate_lookup_table_file_path').get_parameter_value().string_value)

        with open(nozzle_rate_lookup_table_file_path, 'r') as f:
            nozzle_rate_lookup_table = yaml.safe_load(f)
        if not isinstance(nozzle_rate_lookup_table, dict):
            self.get_logger().fatal(f"nozzle_rate_lookup_table is not of type dict in file {nozzle_rate_lookup_table_file_path}")
            raise TypeError("one or more parameters have the wrong type")
        for k, v in nozzle_rate_lookup_table.items():
            if not isinstance(k, (float, int)) or not isinstance(v, (float, int)):
                self.get_logger().fatal(f"desired nozzle flow rate (dict key) or nozzle valve rate command (dict value) in nozzle_rate_lookup_table is not of type float or int in file {nozzle_rate_lookup_table_file_path}")
                raise TypeError("one or more parameters have the wrong type")
        if len(nozzle_rate_lookup_table) < 2:
            self.get_logger().fatal(f"less than 2 key-value pairs specified in nozzle_rate_lookup_table in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        if not np.all(np.diff(np.array(list(nozzle_rate_lookup_table.keys()))) > 0):
            self.get_logger().fatal(f"desired nozzle flow rate values (dict keys) in nozzle_rate_lookup_table are not monotonically increasing in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        min_lut_key = np.min(list(nozzle_rate_lookup_table.keys()))
        if min_lut_key < 0.0:
            self.get_logger().fatal(f"smallest desired nozzle flow rate (dict key) of nozzle_rate_lookup_table [{min_lut_key}] is not greater or equal to 0 in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        min_lut_value = np.min(list(nozzle_rate_lookup_table.values()))
        if min_lut_value < 0.0:
            self.get_logger().fatal(f"minimum nozzle valve rate command (dict value) of nozzle_rate_lookup_table [{min_lut_value}] is not greater or equal to 0 in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")
        max_lut_value = np.max(list(nozzle_rate_lookup_table.values()))
        if max_lut_value > 1.0:
            self.get_logger().fatal(f"maximum nozzle valve rate command (dict value) of nozzle_rate_lookup_table [{max_lut_value}] is not less or equal to 1 in file {nozzle_rate_lookup_table_file_path}")
            raise ValueError("one or more parameters are not correct")

        nozzle_flow_rates = np.array(list(nozzle_rate_lookup_table.keys()))  # desired nozzle flow rate [L/s]
        f_min = min(nozzle_flow_rates)
        f_max = max(nozzle_flow_rates)

        self.get_logger().info(f"f_min = {f_min:.6f} L/s")
        self.get_logger().info(f"f_max = {f_max:.6f} L/s")

        # depth limits
        num_nozzles = len(self.task_plan.canopy_layer_bounds) - 1
        self.get_logger().info(f"num nozzles = {num_nozzles}")

        self._d_ref = self.task_plan.canopy_ref_depth
        self.get_logger().info(f"d_ref = {self._d_ref:.3f} m")

        self._d_min = self.task_plan.canopy_min_depth
        self.get_logger().info(f"d_min = {self._d_min:.3f} m")

        self._d_max = self.task_plan.max_canopy_width
        self.get_logger().info(f"d_max = {self._d_max:.3f} m")

        self._d_min_flow_rate = 2E4 * self._d_ref * num_nozzles * f_min / (self.task_plan.inter_row_target_velocity * self.task_plan.inter_row * self.task_plan.hectare_ref_volume)
        self.get_logger().info(f"d_min_flow_rate = {self._d_min_flow_rate:.3f} m")

        self._d_max_flow_rate = 2E4 * self._d_ref * num_nozzles * f_max / (self.task_plan.inter_row_target_velocity * self.task_plan.inter_row * self.task_plan.hectare_ref_volume)
        self.get_logger().info(f"d_max_flow_rate = {self._d_max_flow_rate:.3f} m")

        # run time variables
        self._canopy_data_sub = self.create_subscription(CanopyDataArray, 'canopy_data', self._canopy_data_callback, 10)
        self._canopy_data_viz_pubs = dict()
        self._canopy_data: defaultdict[str, dict[(int, int), float]] = defaultdict(dict)
        self._x_min: defaultdict[str, int] = defaultdict(int)
        self._x_max: defaultdict[str, int] = defaultdict(int)
        self._z_min: defaultdict[str, int] = defaultdict(int)
        self._z_max: defaultdict[str, int] = defaultdict(int)
        self.prev_canopy_data_array_msg: CanopyDataArray | None = None

        self._publish_period: float = 1.0
        self._last_publish_time: float = 0.0
        self._i: int = 0

        self._crop_size = 10.0  # [m]

        date_time_stamp = datetime.now().strftime("%Y-%m-%d__%H-%M-%S")
        self._out_dir = os.path.expanduser(f"~/tmp/canopy_viz/{date_time_stamp}/")
        os.makedirs(self._out_dir, exist_ok=True)

        # color maps
        cm_im_width = 1000
        cm_im_height = 100

        # make natural color map image
        self._background_color_natural = (0, 0, 0)  # (B, G, R)
        cmn_filename = os.path.join(self._out_dir, "natural_color_map.png")
        cmn_im = np.full(shape=(cm_im_height, cm_im_width, 3), fill_value=self._background_color_natural, dtype=np.uint8)
        for x in range(cm_im_height, cm_im_width):
            cmn_im[:, x] = self.color_map_natural((x - cm_im_height)/(cm_im_width - 2 * cm_im_height) * self.task_plan.max_canopy_width)
        cv2.imwrite(cmn_filename, cmn_im)

        # make command color map image
        self._background_color_command = (150, 150, 0)  # (B, G, R)
        cmc_filename = os.path.join(self._out_dir, "command_color_map.png")
        cmc_im = np.full(shape=(cm_im_height, cm_im_width, 3), fill_value=self._background_color_command, dtype=np.uint8)
        for x in range(cm_im_height, cm_im_width):
            cmc_im[:, x] = self.color_map_command((x - cm_im_height)/(cm_im_width - 2 * cm_im_height) * self.task_plan.max_canopy_width)
        cv2.imwrite(cmc_filename, cmc_im)

    def _canopy_data_callback(self, canopy_data_array_msg: CanopyDataArray) -> None:
        now = time.time()
        if now - self._last_publish_time < self._publish_period:
            do_publish: bool = False
        else:
            do_publish: bool = True
            self._last_publish_time = now

        canopy_data_msg: CanopyData
        for canopy_data_msg in canopy_data_array_msg.canopy_data_array:

            canopy_id = canopy_data_msg.canopy_id.replace('/', '__')
            res = canopy_data_msg.resolution

            if canopy_id not in self._canopy_data_viz_pubs:
                self._canopy_data_viz_pubs[canopy_id] = self.create_publisher(Image, f"canopy_data_viz/{canopy_id}", 10)

            for x, y_depth, z in zip(canopy_data_msg.depth_x_array, canopy_data_msg.depth_y_array, canopy_data_msg.depth_z_array):
                xi = int(x/res)
                zi = int(z/res)
                self._canopy_data[canopy_id][(xi, zi)] = y_depth
                self._x_min[canopy_id] = min(self._x_min[canopy_id], xi)
                self._x_max[canopy_id] = max(self._x_max[canopy_id], xi)
                self._z_min[canopy_id] = min(self._z_min[canopy_id], zi)
                self._z_max[canopy_id] = max(self._z_max[canopy_id], zi)

            if do_publish:

                im_height = self._z_max[canopy_id] - self._z_min[canopy_id] + 1
                im_width = self._x_max[canopy_id] - self._x_min[canopy_id] + 1
                if im_height <= 1 or im_width <= 1:
                    continue

                # command canopy image
                im_command = np.full(shape=(im_height, im_width, 3), fill_value=self._background_color_command, dtype=np.uint8)
                for xi, zi in self._canopy_data[canopy_id].keys():
                    x_im = xi - self._x_min[canopy_id]
                    z_im = self._z_max[canopy_id] - zi
                    d = self._canopy_data[canopy_id][(xi, zi)]
                    im_command[z_im, x_im] = self.color_map_command(d)

                im_command_filename = os.path.join(self._out_dir, f"command_{self._i:05d}_{canopy_id}.png")
                cv2.imwrite(im_command_filename, im_command)

                # natural canopy image
                im_natural = np.full(shape=(im_height, im_width, 3), fill_value=self._background_color_natural, dtype=np.uint8)
                for xi, zi in self._canopy_data[canopy_id].keys():
                    x_im = xi - self._x_min[canopy_id]
                    z_im = self._z_max[canopy_id] - zi
                    d = self._canopy_data[canopy_id][(xi, zi)]
                    im_natural[z_im, x_im] = self.color_map_natural(d)

                im_natural_filename = os.path.join(self._out_dir, f"natural_{self._i:05d}_{canopy_id}.png")
                cv2.imwrite(im_natural_filename, im_natural)

                if self.prev_canopy_data_array_msg is not None:

                    prev_canopy_data_msg: CanopyData | None = None
                    for m in self.prev_canopy_data_array_msg.canopy_data_array:
                        if canopy_data_msg.canopy_id == m.canopy_id:
                            prev_canopy_data_msg = m

                    if prev_canopy_data_msg is not None:

                        x_roi_1 = int(np.round(canopy_data_msg.roi.x_1 / res))
                        x_roi_2 = int(np.round(canopy_data_msg.roi.x_2 / res))
                        x_roi = int(x_roi_1 / 2 + x_roi_2 / 2)

                        prev_x_roi_1 = int(np.round(prev_canopy_data_msg.roi.x_1 / res))
                        prev_x_roi_2 = int(np.round(prev_canopy_data_msg.roi.x_2 / res))
                        prev_x_roi = int(prev_x_roi_1 / 2 + prev_x_roi_2 / 2)

                        x_offset = 100
                        new_width = im_width + 2 * x_offset
                        im_natural_enlarged = np.zeros((im_height, new_width, 3), dtype=im_natural.dtype)
                        im_natural_enlarged[:, x_offset:x_offset+im_width, :] = im_natural

                        if 0 < x_roi_1 + x_offset < new_width:
                            im_natural_enlarged[:, x_roi_1+x_offset] = (255, 255, 255)  # white
                        else:
                            self.get_logger().error(f"im_width: {im_width}  new_width: {new_width}    x_roi_1 + x_offset: {x_roi_1 + x_offset}")

                        if 0 < x_roi_2 + x_offset < new_width:
                            im_natural_enlarged[:, x_roi_2+x_offset] = (0, 255, 255)   # yellow
                        else:
                            self.get_logger().error(f"im_width: {im_width}  new_width: {new_width}    x_roi_2 + x_offset: {x_roi_2 + x_offset}")

                        if x_roi > prev_x_roi:
                            x_crop_min = min(x_roi_1, x_roi_2) - int(self._crop_size / res)
                            x_crop_max = max(x_roi_1, x_roi_2) + 5
                        else:
                            x_crop_min = min(x_roi_1, x_roi_2) - 5
                            x_crop_max = max(x_roi_1, x_roi_2) + int(self._crop_size / res)

                        im_natural_cropped = crop_with_padding_rgb(im_natural_enlarged, x_crop_min+x_offset, x_crop_max+x_offset)
                        self._canopy_data_viz_pubs[canopy_id].publish(self.bridge.cv2_to_imgmsg(im_natural_cropped, "bgr8"))

                self._i += 1

        if do_publish:
            self.prev_canopy_data_array_msg = canopy_data_array_msg

    def color_map_natural(self, d: float) -> tuple[int, int, int]:  # (B, G, R)

        if d <= self._d_min:
            return 0, 0, 0  # black

        elif d >= self._d_max:
            return 0, 0, 255  # red

        else:
            df = d / self._d_max
            di = int(df * 255)
            return 0, di, 0  # blue -> white

    def color_map_command(self, d: float) -> tuple[int, int, int]:  # (B, G, R)

        if d <= self._d_min:
            return 0, 0, 0  # black

        elif d <= self._d_min_flow_rate:
            return 255, 0, 255  # purple

        elif d >= min(self._d_max, self._d_max_flow_rate):
            return 0, 0, 255  # red

        else:
            if d < self._d_ref:
                return 0, 255, 0  # green
            else:
                return 255, 0, 0  # blue

def crop_with_padding_rgb(im, i_roi_min: int, i_roi_max: int):
    im_height, im_width, _ = im.shape
    crop_width = i_roi_max - i_roi_min

    src_start = max(i_roi_min, 0)
    src_end = min(i_roi_max, im_width)
    dst_start = max(-i_roi_min, 0)
    dst_end = dst_start + (src_end - src_start)

    result = np.zeros((im_height, crop_width, 3), dtype=im.dtype)
    result[:, dst_start:dst_end, :] = im[:, src_start:src_end, :]

    return result


def main(args=None):
    rclpy.init(args=args)
    node = CanopyDataViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
