#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import sys
import serial
import struct
from cobs import cobs
from std_msgs.msg import Float32


class ArduinoSensorsDriver(Node):

    def __init__(self):
        super().__init__('arduino_sensors_driver')

        self.serial_interface = serial.Serial('/dev/arduino_onboard', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=3)

        self.cobs_delimiter = b'\x00'
        self.payload_struct_format = 'Ifff'

        self.t_prev = 0
        self.error_count = 0
        self.first_message = True

        self._sprayer_pressure_transmitter_pressure_pub = self.create_publisher(Float32, f"/onboard_sensors/sprayer_pressure/pressure", 10)
        self._sprayer_pressure_transmitter_voltage_pub = self.create_publisher(Float32, f"/onboard_sensors/sprayer_pressure/voltage", 10)
        self._ambient_pressure_pub = self.create_publisher(Float32, f"/onboard_sensors/ambient_pressure", 10)
        self._ambient_temperature_pub = self.create_publisher(Float32, f"/onboard_sensors/ambient_temperature", 10)
        self._ambient_relative_humidity_pub = self.create_publisher(Float32, f"/onboard_sensors/ambient_relative_humidity", 10)
        self._timer = self.create_timer(0.0001, self._timer_callback)

    def _timer_callback(self):

        try:
            if self.first_message:
                self.serial_interface.read_until(self.cobs_delimiter)  # discard the first message (it probably contains a partial payload)
                self.first_message = False
                return

            encoded_data = self.serial_interface.read_until(self.cobs_delimiter)  # read until the COBS packet ending delimiter is found
            n = len(encoded_data)

            if n > 0:
                decoded_data = cobs.decode(encoded_data[0:-1])  # discard the delimiter byte and decode the data
                n_decoded = len(decoded_data)

                if n_decoded == struct.calcsize(self.payload_struct_format):

                    payload = struct.unpack(self.payload_struct_format, decoded_data)  # python data types: https://docs.python.org/3/library/struct.html#format-characters
                    adc_value, ambient_pressure, ambient_temperature, ambient_relative_humidity = payload

                    sensor_value_voltage = adc_value / (2**12 - 1) * 5  # [V]
                    sensor_value_pressure = (sensor_value_voltage - 0.5) / 4 * 1.2  # [MPa]

                    self._sprayer_pressure_transmitter_pressure_pub.publish(Float32(data=sensor_value_pressure))
                    self._sprayer_pressure_transmitter_voltage_pub.publish(Float32(data=sensor_value_voltage))
                    self._ambient_pressure_pub.publish(Float32(data=ambient_pressure))
                    self._ambient_temperature_pub.publish(Float32(data=ambient_temperature))
                    self._ambient_relative_humidity_pub.publish(Float32(data=ambient_relative_humidity))

                else:
                    self.error_count += 1
                    self.get_logger().error(f"Decode error, payload length = {n_decoded}, it should be {struct.calcsize(self.payload_struct_format)} (accumulated errors: {self.error_count})")

            else:
                self.error_count += 1
                self.get_logger().error(f"Decode error, data length = {n} (accumulated errors: {self.error_count})")

        except (serial.serialutil.SerialException, cobs.DecodeError):
            self.error_count += 1
            self.get_logger().error(f"Serial or COBS decode error: {sys.exc_info()[0]} (accumulated errors: {self.error_count})")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSensorsDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
