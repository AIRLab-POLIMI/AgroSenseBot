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
        self.shutting_down_ = False
        self.covariance_imu_ = 0.01
        self.covariance_gnss_ = 0.0001

        self.imu_pub_ = self.create_publisher(Imu, "/imu", 10)
        self.gnss_pub_ = self.create_publisher(NavSatFix, "/gnss", 10)

        url_imu = "ws://192.168.1.54:8080/sensor/connect?type=android.sensor.gyroscope"
        self.ws_imu_ = websocket.WebSocketApp(
            url_imu,
            on_open=lambda ws: print("Websocket IMU connected"),
            on_message=lambda ws, msg: self.on_ws_imu_message(ws, msg),
            on_error=lambda ws, error: print(f"Websocket IMU error occurred: {error}"),
            on_close=lambda ws, code, reason: print(f"Websocket IMU connection closed. Code: {code}. Reason: {reason}"),
        )

        self.ws_imu_thread_ = threading.Thread(target=self.ws_imu_.run_forever)
        self.ws_imu_thread_.start()

        url_gnss = "ws://192.168.1.54:8080/gps"
        self.ws_gnss_ = websocket.WebSocketApp(
            url_gnss,
            on_open=lambda ws: print("Websocket GNSS connected"),
            on_message=lambda ws, msg: self.on_ws_gnss_message(ws, msg),
            on_error=lambda ws, error: print(f"Websocket GNSS error occurred: {error}"),
            on_close=lambda ws, code, reason: print(f"Websocket GNSS connection closed. Code: {code}. Reason: {reason}"),
        )

        self.ws_gnss_thread_ = threading.Thread(target=self.ws_gnss_.run_forever)
        self.ws_gnss_thread_.start()

    def stop(self):
        print("Closing websockets")
        self.shutting_down_ = True
        self.ws_gnss_.close()
        self.ws_imu_.close()

        self.ws_gnss_thread_.join()
        self.ws_imu_thread_.join()

    def on_ws_imu_message(self, _, message):
        if self.shutting_down_:
            return

        x, y, z = json.loads(message)['values']
        if self.print_debug_:
            print(f"Websocket IMU received: x = {x:+.3f} y={y:+.3f}, z={z:+.3f}")

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
        self.imu_pub_.publish(msg)

    def on_ws_gnss_message(self, _, message):
        if self.shutting_down_:
            return

        longitude = json.loads(message)['longitude']
        latitude = json.loads(message)['latitude']
        altitude = json.loads(message)['altitude']
        if self.print_debug_:
            print(f"Websocket GNSS received: longitude = {longitude:+.6f} latitude={latitude:+.6f}, altitude={altitude:+.6f}")
        print(f"Websocket GNSS received: longitude = {longitude:+.6f} latitude={latitude:+.6f}, altitude={altitude:+.6f}")

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
