#!/usr/bin/python3

import rclpy
import std_msgs.msg
from rclpy.node import Node
import sys
import serial
import struct
import time
from cobs import cobs
from std_msgs.msg import Float32


class ArduinoSensorsDriver(Node):

    def __init__(self):
        super().__init__('arduino_sensors_driver')

        self.serial_interface = serial.Serial('/dev/arduino_onboard', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=3)

        self.cobs_delimiter = b'\x00'
        self.payload_size_uint32 = 3
        self.payload_size_bytes = 4 * self.payload_size_uint32

        self.t_prev = time.perf_counter()
        self.error_count = 0

        self._sprayer_pressure_transmitter_pressure_pub = self.create_publisher(Float32, f"/onboard_sensors/sprayer_pressure/pressure", 10)
        self._sprayer_pressure_transmitter_voltage_pub = self.create_publisher(Float32, f"/onboard_sensors/sprayer_pressure/voltage", 10)
        self._timer = self.create_timer(0.0001, self._timer_callback)

    def _timer_callback(self):
        try:
            encoded_data = self.serial_interface.read_until(self.cobs_delimiter)  # read until the COBS packet ending delimiter is found
            n = len(encoded_data)

            if n > 0:
                decoded_data = cobs.decode(encoded_data[0:-1])  # discard the delimiter byte and decode the data
                n_decoded = len(decoded_data)

                if n_decoded == self.payload_size_bytes:

                    t, adc_value, delta_analog_read = struct.unpack('I' * self.payload_size_uint32, decoded_data)  # python data types: https://docs.python.org/3/library/struct.html#format-characters
                    sensor_value_voltage = adc_value / (2**12 - 1) * 5  # [V]
                    sensor_value_pressure = (sensor_value_voltage - 0.5) / 4 * 1.2  # [MPa]

                    self._sprayer_pressure_transmitter_pressure_pub.publish(Float32(data=sensor_value_pressure))
                    self._sprayer_pressure_transmitter_voltage_pub.publish(Float32(data=sensor_value_voltage))

                else:
                    self.get_logger().error(f"Decode error ({self.error_count})")
                    self.error_count += 1

            else:
                self.get_logger().error(f"Decode error ({self.error_count})")
                self.error_count += 1

        except (serial.serialutil.SerialException, cobs.DecodeError):
            self.get_logger().error("Serial or COBS decode error:", sys.exc_info()[0])


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSensorsDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
