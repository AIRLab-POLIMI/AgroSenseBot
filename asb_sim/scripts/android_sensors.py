#! /usr/bin/python3

import threading
import websocket
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix


class AndroidSensorsPublisher(Node):
    """
    Publish to ROS2 the sensors data from an Android phone running the Sensor Server app (https://github.com/umer0586/SensorServer)
    """

    def __init__(self):
        super().__init__('scan_test_publisher')
        self.print_debug_ = False
        self.covariance_imu_ = 0.01
        self.covariance_gnss_ = 0.1

        self.imu_gyro_pub_ = self.create_publisher(Imu, "/imu", 10)
        self.imu_accelerometer_pub_ = self.create_publisher(Imu, "/imu/accelerometer", 10)
        self.gnss_pub_ = self.create_publisher(NavSatFix, "/gnss/fix", 10)

        self.shutting_down_ = False

        ws_ip = "192.168.1.54"
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

        x, y, z = json.loads(message)['values']
        if self.print_debug_:
            self.get_logger().info(f"Websocket gyro received: x = {x:+.3f} y={y:+.3f}, z={z:+.3f}")

        cov = self.covariance_imu_
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        msg.angular_velocity.x = x
        msg.angular_velocity.y = y
        msg.angular_velocity.z = z
        msg.angular_velocity_covariance = [
            cov, 0.0, 0.0,
            0.0, cov, 0.0,
            0.0, 0.0, cov,
        ]
        self.imu_gyro_pub_.publish(msg)

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
