#!/usr/bin/python3
from collections import defaultdict

import rclpy
from rclpy.node import Node
from asb_msgs.msg import CanopyData

import matplotlib.pyplot as plt
import numpy as np


class CanopyDensityPlotter(Node):

    def __init__(self):
        super().__init__('canopy_data_plotter')
        self.canopy_data_sub_ = self.create_subscription(CanopyData, '/canopy_data', self.canopy_data_callback, 10)

        plt.ion()  # turning interactive mode on

        # preparing the data
        self.x_ = [0]
        self.y_ = [0]
        plt.xlim(-1, 1)
        plt.ylim(0, 1)

        # plotting the first frame
        self.graph_ = plt.plot([1.0, 21.0], [4.0, 2.0])[0]
        plt.xlim(-2, 22)
        plt.ylim(0, 5)
        plt.pause(0.01)

    def canopy_data_callback(self, canopy_data_msg: CanopyData):
        self.graph_.remove()
        self.graph_ = plt.plot(canopy_data_msg.volume_x_array, np.array(canopy_data_msg.volume_y_array) / canopy_data_msg.resolution, color='g', label='normalized volume')[0]
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = CanopyDensityPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
