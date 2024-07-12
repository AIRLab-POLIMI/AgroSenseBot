#!/usr/bin/python3


import rclpy
from rclpy.node import Node
from asb_msgs.msg import CanopyDensity

import matplotlib.pyplot as plt
import numpy as np


class CanopyDensityPlotter(Node):

    def __init__(self):
        super().__init__('canopy_density_plotter')
        self.canopy_density_sub_ = self.create_subscription(CanopyDensity, '/canopy_density', self.canopy_density_callback, 10)

        plt.ion()  # turning interactive mode on

        # preparing the data
        self.x_ = [0]
        self.y_ = [0]
        plt.xlim(-1, 1)
        plt.ylim(0, 1)

        # plotting the first frame
        self.graph_ = plt.plot(self.x_, self.y_)[0]
        plt.xlim(-2, 22)
        plt.ylim(0, 0.1)
        plt.pause(0.01)

    def canopy_density_callback(self, canopy_density_msg: CanopyDensity):
        self.get_logger().info("canopy_density_callback")
        self.graph_.remove()

        # print(list(canopy_density_msg.x_array), list(canopy_density_msg.y_array))
        print(np.max(canopy_density_msg.y_array))

        # plotting newer graph
        self.graph_ = plt.plot(canopy_density_msg.x_array, canopy_density_msg.y_array, color='g')[0]
        # plt.xlim(canopy_density_msg.x_array[0], canopy_density_msg.x_array[-1])
        # plt.ylim(0, np.max(canopy_density_msg.y_array))

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

# plt.ion()  # turning interactive mode on
#
# # preparing the data
# y = [random.randint(1, 10) for i in range(20)]
# x = [*range(1, 21)]
#
# # plotting the first frame
# graph = plt.plot(x, y)[0]
# plt.ylim(0, 10)
# plt.pause(1)
#
# # the update loop
# for i in range(1000):
#     # updating the data
#     y.append(random.randint(1, 10))
#     x.append(x[-1]+1)
#
#     # removing the older graph
#     graph.remove()
#
#     # plotting newer graph
#     graph = plt.plot(x, y, color='g')[0]
#     plt.xlim(x[0], x[-1])
#
#     # calling pause function for 0.25 seconds
#     # plt.pause(0.01)
