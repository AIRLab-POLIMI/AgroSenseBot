#!/usr/bin/python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from asb_msgs.msg import CanopyDataArray, CanopyData

from collections import defaultdict
import numpy as np

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

    def _canopy_data_callback(self, canopy_data_array_msg: CanopyDataArray) -> None:
        canopy_data_msg: CanopyData
        for canopy_data_msg in canopy_data_array_msg.canopy_data_array:

            i = canopy_data_msg.canopy_id
            res = canopy_data_msg.resolution

            if i not in self._canopy_data_viz_pubs:
                self._canopy_data_viz_pubs[i] = self.create_publisher(Image, f"canopy_data_viz/{i}", 10)

            for x, y_depth, z in zip(canopy_data_msg.depth_x_array, canopy_data_msg.depth_y_array, canopy_data_msg.depth_z_array):
                xi = int(x/res)
                zi = int(z/res)
                self._canopy_data[i][(xi, zi)] = y_depth
                self._x_min[i] = min(self._x_min[i], xi)
                self._x_max[i] = max(self._x_max[i], xi)
                self._z_min[i] = min(self._z_min[i], zi)
                self._z_max[i] = max(self._z_max[i], zi)

            im_height = self._z_max[i] - self._z_min[i] + 1
            im_width = self._x_max[i] - self._x_min[i] + 1
            if im_height == 0 or im_width == 0:
                continue

            im = np.zeros((im_height, im_width, 3), np.uint8)
            for xi, zi in self._canopy_data[i].keys():
                x_im = xi - self._x_min[i]
                z_im = self._z_max[i] - zi
                di = int(self._canopy_data[i][(xi, zi)] / 0.4 * 255)
                im[z_im, x_im] = (di, 0, 0) if di <= 255 else (255, 255, 255)  # (B, G, R)

            self._canopy_data_viz_pubs[i].publish(self.bridge.cv2_to_imgmsg(im, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = CanopyDataViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
