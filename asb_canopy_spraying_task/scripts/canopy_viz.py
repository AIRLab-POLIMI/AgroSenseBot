#!/usr/bin/python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from asb_msgs.msg import CanopyDataArray, CanopyData

import os
from collections import defaultdict
import numpy as np
import cv2

np.set_printoptions(precision=2)


class CanopyDataViz(Node):

    def __init__(self):
        super().__init__('canopy_data_viz')

        self.bridge = CvBridge()

        self._canopy_data_sub = self.create_subscription(CanopyDataArray, 'canopy_data', self._canopy_data_callback, 10)
        self._canopy_data_viz_pubs = dict()
        self._canopy_data: defaultdict[str, dict[(int, int), float]] = defaultdict(dict)
        self._x_min: defaultdict[str, int] = defaultdict(int)
        self._x_max: defaultdict[str, int] = defaultdict(int)
        self._z_min: defaultdict[str, int] = defaultdict(int)
        self._z_max: defaultdict[str, int] = defaultdict(int)

        self.i: int = 0

        self.out_dir = os.path.expanduser("~/tmp/canopy_viz/")
        os.makedirs(self.out_dir, exist_ok=True)

    def _canopy_data_callback(self, canopy_data_array_msg: CanopyDataArray) -> None:
        canopy_data_msg: CanopyData
        for canopy_data_msg in canopy_data_array_msg.canopy_data_array:

            canopy_id = canopy_data_msg.canopy_id
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

            im_height = self._z_max[canopy_id] - self._z_min[canopy_id] + 1
            im_width = self._x_max[canopy_id] - self._x_min[canopy_id] + 1
            if im_height == 0 or im_width == 0:
                continue

            background: tuple[int, int, int] = (0, 0, 0)  # (B, G, R)
            im = np.full(shape=(im_height, im_width, 3), fill_value=background, dtype=np.uint8)
            for xi, zi in self._canopy_data[canopy_id].keys():
                x_im = xi - self._x_min[canopy_id]
                z_im = self._z_max[canopy_id] - zi
                di = int(self._canopy_data[canopy_id][(xi, zi)] / 1.2 * 255)
                if di == 0:
                    im[z_im, x_im] = background
                elif di == 255:
                    im[z_im, x_im] = (0, 0, 255)  # (B, G, R)
                else:
                    im[z_im, x_im] = (0, di, 0)  # (B, G, R)

            self._canopy_data_viz_pubs[canopy_id].publish(self.bridge.cv2_to_imgmsg(im, "bgr8"))

            filename = os.path.join(self.out_dir, f"{canopy_id}_{self.i:010d}.png")
            cv2.imwrite(filename, im)
            self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = CanopyDataViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
