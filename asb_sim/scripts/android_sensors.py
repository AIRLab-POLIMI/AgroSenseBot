#! /usr/bin/python3

import threading


import websocket
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

import numpy as np
import scipy
import scipy.signal
from collections import deque


class LiveFilter:
    """Base class for live filters.
    """
    def process(self, x):
        # do not process NaNs
        if np.isnan(x):
            return x

        return self._process(x)

    def __call__(self, x):
        return self.process(x)

    def _process(self, x):
        raise NotImplementedError("Derived class must implement _process")


class LiveLFilter(LiveFilter):
    def __init__(self, b, a):
        """Initialize live filter based on difference equation.

        Args:
            b (array-like): numerator coefficients obtained from scipy.
            a (array-like): denominator coefficients obtained from scipy.
        """
        self.b = b
        self.a = a
        self._xs = deque([0] * len(b), maxlen=len(b))
        self._ys = deque([0] * (len(a) - 1), maxlen=len(a)-1)

    def _process(self, x):
        """Filter incoming data with standard difference equations.
        """
        self._xs.appendleft(x)
        y = np.dot(self.b, self._xs) - np.dot(self.a[1:], self._ys)
        y = y / self.a[0]
        self._ys.appendleft(y)

        return y


class AndroidSensorsPublisher(Node):
    """
    Publish to ROS2 the sensors data from an Android phone running the Sensor Server app (https://github.com/umer0586/SensorServer)
    """

    def __init__(self):
        super().__init__('scan_test_publisher')
        self.print_debug_ = False
        self.covariance_imu_ = 0.01
        self.covariance_gnss_ = 0.1

        self.imu_gyro_unfiltered_pub_ = self.create_publisher(Imu, "/imu/unfiltered", 10)
        self.imu_gyro_pub_ = self.create_publisher(Imu, "/imu", 10)
        self.imu_accelerometer_pub_ = self.create_publisher(Imu, "/imu/accelerometer", 10)
        self.gnss_pub_ = self.create_publisher(NavSatFix, "/gnss/fix", 10)

        b, a = scipy.signal.iirfilter(4, Wn=5.0, fs=200, btype="low", ftype="butter")
        print("b: ", b)
        print("a: ", a)
        self.filter_ = LiveLFilter(b, a)

        self.shutting_down_ = False

        ws_ip = "10.42.0.207"
        # ws_ip = "192.168.1.54"
        ws_port = 8080

        url_gyro = f"ws://{ws_ip}:{ws_port}/sensor/connect?type=android.sensor.gyroscope"
        self.ws_gyro_ = websocket.WebSocketApp(
            url_gyro,
            on_open=lambda ws: self.get_logger().info("Websocket gyro connected"),
            on_message=lambda ws, msg: self.on_ws_gyro_message(ws, msg),
            on_error=lambda ws, error: self.get_logger().error(f"Websocket gyro error occurred: {error}"),
            on_close=lambda ws, code, reason: self.get_logger().info(f"Websocket gyro connection closed. Code: {code}. Reason: {reason}"),
        )

        self.ws_gyro_thread_ = threading.Thread(target=self.ws_gyro_.run_forever)
        self.ws_gyro_thread_.start()

        url_accelerometer = f"ws://{ws_ip}:{ws_port}/sensor/connect?type=android.sensor.accelerometer"
        self.ws_accelerometer_ = websocket.WebSocketApp(
            url_accelerometer,
            on_open=lambda ws: self.get_logger().info("Websocket accelerometer connected"),
            on_message=lambda ws, msg: self.on_ws_accelerometer_message(ws, msg),
            on_error=lambda ws, error: self.get_logger().error(f"Websocket accelerometer error occurred: {error}"),
            on_close=lambda ws, code, reason: self.get_logger().info(f"Websocket accelerometer connection closed. Code: {code}. Reason: {reason}"),
        )

        self.ws_accelerometer_thread_ = threading.Thread(target=self.ws_accelerometer_.run_forever)
        self.ws_accelerometer_thread_.start()

        url_gnss = f"ws://{ws_ip}:{ws_port}/gps"
        self.ws_gnss_ = websocket.WebSocketApp(
            url_gnss,
            on_open=lambda ws: self.get_logger().info("Websocket GNSS connected"),
            on_message=lambda ws, msg: self.on_ws_gnss_message(ws, msg),
            on_error=lambda ws, error: self.get_logger().error(f"Websocket GNSS error occurred: {error}"),
            on_close=lambda ws, code, reason: self.get_logger().info(f"Websocket GNSS connection closed. Code: {code}. Reason: {reason}"),
        )

        self.ws_gnss_thread_ = threading.Thread(target=self.ws_gnss_.run_forever)
        self.ws_gnss_thread_.start()

    def stop(self):
        self.get_logger().info("Closing websockets")
        self.shutting_down_ = True
        self.ws_gnss_.close()
        self.ws_gyro_.close()
        self.ws_accelerometer_.close()

        self.ws_gnss_thread_.join()
        self.ws_gyro_thread_.join()
        self.ws_accelerometer_thread_.join()

    def on_ws_gyro_message(self, _, message):
        if self.shutting_down_:
            return

        now = self.get_clock().now().to_msg()

        x, y, z = json.loads(message)['values']
        if self.print_debug_:
            self.get_logger().info(f"Websocket gyro received: x = {x:+.3f} y={y:+.3f}, z={z:+.3f}")

        cov = self.covariance_imu_
        msg_unfiltered = Imu()
        msg_unfiltered.header.stamp = now
        msg_unfiltered.header.frame_id = "imu_link"
        msg_unfiltered.angular_velocity.x = x
        msg_unfiltered.angular_velocity.y = y
        msg_unfiltered.angular_velocity.z = z
        msg_unfiltered.angular_velocity_covariance = [
            cov, 0.0, 0.0,
            0.0, cov, 0.0,
            0.0, 0.0, cov,
        ]
        self.imu_gyro_unfiltered_pub_.publish(msg_unfiltered)

        z_filtered = self.filter_(z)

        msg_filtered = Imu()
        msg_filtered.header.stamp = now
        msg_filtered.header.frame_id = "imu_link"
        msg_filtered.angular_velocity.x = x
        msg_filtered.angular_velocity.y = y
        msg_filtered.angular_velocity.z = z_filtered
        msg_filtered.angular_velocity_covariance = [
            cov, 0.0, 0.0,
            0.0, cov, 0.0,
            0.0, 0.0, cov,
        ]
        self.imu_gyro_pub_.publish(msg_filtered)

    def on_ws_accelerometer_message(self, _, message):
        if self.shutting_down_:
            return

        x, y, z = json.loads(message)['values']
        if self.print_debug_:
            self.get_logger().info(f"Websocket accelerometer received: x = {x:+.3f} y={y:+.3f}, z={z:+.3f}")

        cov = self.covariance_imu_
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        msg.linear_acceleration.x = x
        msg.linear_acceleration.y = y
        msg.linear_acceleration.z = z
        msg.linear_acceleration_covariance = [
            cov, 0.0, 0.0,
            0.0, cov, 0.0,
            0.0, 0.0, cov,
        ]
        self.imu_accelerometer_pub_.publish(msg)

    def on_ws_gnss_message(self, _, message):
        if self.shutting_down_:
            return

        longitude = json.loads(message)['longitude']
        latitude = json.loads(message)['latitude']
        altitude = json.loads(message)['altitude']
        if self.print_debug_:
            self.get_logger().info(f"Websocket GNSS received: longitude = {longitude:+.6f} latitude={latitude:+.6f}, altitude={altitude:+.6f}")
        self.get_logger().info(f"Websocket GNSS received: longitude = {longitude:+.6f} latitude={latitude:+.6f}, altitude={altitude:+.6f}")

        cov = self.covariance_gnss_
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gnss_link"
        msg.longitude = longitude
        msg.latitude = latitude
        msg.altitude = altitude
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance = [
            cov, 0.0, 0.0,
            0.0, cov, 0.0,
            0.0, 0.0, cov,
        ]
        self.gnss_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    android_sensors_publisher = AndroidSensorsPublisher()
    try:
        rclpy.spin(android_sensors_publisher)
    except KeyboardInterrupt:
        android_sensors_publisher.stop()


if __name__ == '__main__':
    main()
