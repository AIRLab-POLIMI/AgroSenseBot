#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from microstrain_inertial_msgs.msg import MipHeader, MipGnssFixInfo, MipFilterGnssDualAntennaStatus, HumanReadableStatus


class GnssStatusPublisher(Node):

    def __init__(self):
        super().__init__('gnss_status_publisher')

        qos_reliable_volatile_depth_1 = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.gnss_1_fix_status_pub = self.create_publisher(
            MipGnssFixInfo,
            '/mip/gnss_1/fix_info',
            qos_profile=qos_reliable_volatile_depth_1
        )

        self.gnss_2_fix_status_pub = self.create_publisher(
            MipGnssFixInfo,
            '/mip/gnss_2/fix_info',
            qos_profile=qos_reliable_volatile_depth_1
        )

        self.gnss_dual_antenna_fix_status_pub = self.create_publisher(
            MipFilterGnssDualAntennaStatus,
            '/mip/filter/gnss_dual_antenna_status',
            qos_profile=qos_reliable_volatile_depth_1
        )

        self.ekf_status_pub = self.create_publisher(
            HumanReadableStatus,
            '/ekf/status',
            qos_profile=qos_reliable_volatile_depth_1
        )

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.gnss_1_fix_status_pub.publish(MipGnssFixInfo(
            header=MipHeader(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                ),
            ),
            fix_type=MipGnssFixInfo.FIX_TYPE_FIX_RTK_FIXED,
        ))

        self.gnss_2_fix_status_pub.publish(MipGnssFixInfo(
            header=MipHeader(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                ),
            ),
            fix_type=MipGnssFixInfo.FIX_TYPE_FIX_RTK_FIXED,
        ))

        self.gnss_dual_antenna_fix_status_pub.publish(MipFilterGnssDualAntennaStatus(
            header=MipHeader(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                ),
            ),
            fix_type=MipFilterGnssDualAntennaStatus.FIX_TYPE_FIX_DA_FIXED,
        ))

        self.ekf_status_pub.publish(HumanReadableStatus(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
            ),
            gnss_state=HumanReadableStatus.GNSS_STATE_RTK_FIXED,
            dual_antenna_fix_type=HumanReadableStatus.DUAL_ANTENNA_FIX_TYPE_FIXED,
            filter_state=HumanReadableStatus.FILTER_STATE_GQ7_FULL_NAV,
            status_flags=[HumanReadableStatus.STATUS_FLAGS_GQ7_FILTER_CONDITION_STABLE],
        ))


def main(args=None):
    rclpy.init(args=args)
    node = GnssStatusPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
