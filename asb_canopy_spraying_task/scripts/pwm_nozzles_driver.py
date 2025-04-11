#!/usr/bin/python3
import os

import rclpy
import yaml
from asb_msgs.msg import NozzleCommandArray, NozzleCommand
from rclpy import qos
from rclpy.node import Node

import time
import datetime
import can
from rclpy.time import Time, Duration


def compute_tick_milliseconds() -> int:
    now: datetime.datetime = datetime.datetime.now()
    midnight: datetime.datetime = now.replace(hour=0, minute=0, second=0, microsecond=0)
    milliseconds_since_midnight = int((now - midnight).seconds*1E3 + (now - midnight).microseconds/1E3)
    return milliseconds_since_midnight


class PwmNozzlesDriver(Node):

    def __init__(self):
        super().__init__('pwm_nozzles_driver')

        can_channel_name: str = "can3"
        send_test_messages: bool = False

        self.nozzles_command_timeout: float = 1.0  # s
        read_valve_state_rate = 1.0  # Hz
        self.valve_command_rate = 20.0  # Hz
        set_valve_address_response_timeout = 1.0  # s
        self.valve_state_response_timeout = Duration(seconds=7.0)

        self.declare_parameter('nozzles_configuration_file_path', rclpy.Parameter.Type.STRING)
        nozzles_configuration_file_path = os.path.expanduser(self.get_parameter('nozzles_configuration_file_path').get_parameter_value().string_value)

        self.id_to_valve_address: dict[str, int] = dict()
        self.valve_address_to_nozzle_id: dict[int, str] = dict()
        self.valve_addresses: list[int] = list()
        with open(nozzles_configuration_file_path, 'r') as f:
            nozzles_configuration = yaml.safe_load(f)
        if not isinstance(nozzles_configuration, list):
            self.get_logger().fatal(f"nozzles_configuration is not of type list in file {nozzles_configuration_file_path}")
            raise TypeError("one or more parameters have the wrong type")
        for nozzle_configuration in nozzles_configuration:
            if 'id' not in nozzle_configuration or 'hardware_valve_address' not in nozzle_configuration:
                self.get_logger().fatal(f"nozzle configuration does not have one or more of the required fields in file {nozzles_configuration_file_path}")
                raise ValueError("one or more parameters are not correct")

            if not isinstance(nozzle_configuration['id'], str):
                self.get_logger().fatal(f"nozzle id is not of type str [nozzle_id={nozzle_configuration['id']}] in file {nozzles_configuration_file_path}")
                raise TypeError("one or more parameters have the wrong type")
            if not isinstance(nozzle_configuration['hardware_valve_address'], int):
                self.get_logger().fatal(f"hardware_valve_address is not of type int [hardware_valve_address={nozzle_configuration['hardware_valve_address']}] in file {nozzles_configuration_file_path}")
                raise TypeError("one or more parameters have the wrong type")
            if not (1 <= nozzle_configuration['hardware_valve_address'] <= 127):
                self.get_logger().fatal(f"hardware_valve_address is not in the range 1...127 [hardware_valve_address={nozzle_configuration['hardware_valve_address']}] in file {nozzles_configuration_file_path}")
                raise ValueError("one or more parameters have incorrect value")

            self.valve_addresses.append(nozzle_configuration['hardware_valve_address'])
            self.id_to_valve_address[nozzle_configuration['id']] = nozzle_configuration['hardware_valve_address']
            self.valve_address_to_nozzle_id[nozzle_configuration['hardware_valve_address']] = nozzle_configuration['id']

        self.valve_addresses = sorted(self.valve_addresses)

        if len(self.valve_addresses) != len(set(self.valve_addresses)):
            self.get_logger().fatal(f"some hardware_valve_address value is not unique in file {nozzles_configuration_file_path}")
            raise ValueError("one or more parameters have incorrect value")

        self.get_logger().info(f"valve addresses: {self.valve_addresses}")
        self.get_logger().info(f"nozzle ids to valve addresses: {self.id_to_valve_address}")

        self.send_valve_read_command: bool = False
        self.last_nozzles_command: NozzleCommandArray | None = None
        self.nozzles_command_sub = self.create_subscription(NozzleCommandArray, '/nozzles_command', callback=self.nozzles_command_callback, qos_profile=qos.qos_profile_sensor_data)

        self.last_valve_state_stamp: dict[int, Time] = dict()
        for valve_address in self.valve_addresses:
            self.last_valve_state_stamp[valve_address] = self.get_clock().now()

        # configure valve addresses
        try:
            self.can_bus = can.Bus(interface='socketcan', channel=can_channel_name, bitrate=250000, receive_own_messages=send_test_messages)
            self.can_listener = can.BufferedReader()
            self.notifier = can.Notifier(self.can_bus, [self.can_listener])

            already_configured = False
            for a in self.valve_addresses:
                if already_configured:
                    break

                time.sleep(0.01)
                self.broadcast_sync(can_bus=self.can_bus, groups_number=1)

                time.sleep(0.01)
                self.set_valve_address_command(can_bus=self.can_bus, valve_address=a)

                if send_test_messages:
                    self.test_set_valve_address_command_response(can_bus=self.can_bus, valve_address=a, fill_data_array=True)

                start_time = time.time()
                while True:
                    m = self.can_listener.get_message(timeout=0.001)
                    if time.time() - start_time > set_valve_address_response_timeout:
                        self.get_logger().info(f"set_valve_address_command timeout, address: {a}, timeout: {set_valve_address_response_timeout}. Assuming valves are already configured.")
                        already_configured = True
                        break
                    if m is not None and m.arbitration_id == 0x100 and m.data[0] == a and (len(m.data) == 1 or m.data[1] == 0):
                        self.get_logger().info(f"valve address {a} set for nozzle {self.valve_address_to_nozzle_id[a]}. It took {(time.time() - start_time)*1000:.1f} ms.")
                        break

        except OSError as e:
            self.get_logger().fatal(f"Could not open CAN socket: {e}")
            raise KeyboardInterrupt

        # Create timers for sending the commands, sending the state requests, and reading the state responses
        self.create_timer(1.0 / read_valve_state_rate, self.read_valve_state_timer_callback)
        self.create_timer(1.0 / self.valve_command_rate, self.valve_command_timer_callback)
        self.create_timer(1.0 / read_valve_state_rate, self.valve_state_response_timer_callback)

    def nozzles_command_callback(self, msg: NozzleCommandArray) -> None:
        self.last_nozzles_command = msg

    def read_valve_state_timer_callback(self) -> None:
        self.send_valve_read_command = True

    def valve_command_timer_callback(self) -> None:
        if self.last_nozzles_command is None:
            nozzles_command = NozzleCommandArray()
        elif (self.get_clock().now() - Time.from_msg(self.last_nozzles_command.stamp)).nanoseconds / 1E9 > self.nozzles_command_timeout:
            self.get_logger().warn(f"nozzle command age too old")
            self.last_nozzles_command = None
            nozzles_command = NozzleCommandArray()
        else:
            nozzles_command = self.last_nozzles_command

        self.send_valve_commands(nozzles_command)

    def valve_state_response_timer_callback(self):
        now: Time = self.get_clock().now()
        start_time = time.time()
        while True:
            m = self.can_listener.get_message(timeout=0.001)
            if m is None:
                break
            if time.time() - start_time > 1.0 / 10 / self.valve_command_rate:  # prevent this timer execution from being longer then the command timer
                return
            if 0x481 <= m.arbitration_id <= 0x4FF:
                valve_address = m.arbitration_id - 0x480
                self.last_valve_state_stamp[valve_address] = now

        for valve_address in self.valve_addresses:
            age = now - self.last_valve_state_stamp[valve_address]
            self.get_logger().debug(f"valve_state_response_timer_callback: valve_address: {valve_address}  state age: {age.nanoseconds/1E9:0.3f} s")
            if age > self.valve_state_response_timeout:
                self.get_logger().warn(f"valve_state_response_timer_callback: valve_address: {valve_address}  state age: {age.nanoseconds/1E9:0.3f} s")

    def send_valve_commands(self, nozzles_command: NozzleCommandArray, shutting_down=False):

        valve_rates: dict[int, float] = dict()
        for valve_address in self.valve_addresses:
            valve_rates[valve_address] = 0.0

        nozzle_command: NozzleCommand
        for nozzle_command in nozzles_command.nozzle_command_array:
            if nozzle_command.nozzle_id in self.id_to_valve_address:
                valve_rates[self.id_to_valve_address[nozzle_command.nozzle_id]] = min(1.0, max(0.0, nozzle_command.rate))
            else:
                self.get_logger().warn(f"trying to set rate of unknown nozzle [nozzle_id: {nozzle_command.nozzle_id}]")

        for valve_address, valve_rate in valve_rates.items():
            self.control_valve_state_command(can_bus=self.can_bus, valve_address=valve_address, rate=valve_rate)

        if self.send_valve_read_command and not shutting_down:
            self.send_valve_read_command = False
            # self.broadcast_sync(can_bus=self.can_bus, groups_number=1)
            # self.broadcast_read_valve_state_command(can_bus=self.can_bus)

    def shutdown(self) -> None:

        nozzles_command = NozzleCommandArray()
        self.send_valve_commands(nozzles_command, shutting_down=True)

        self.get_logger().info("closing CAN socket")
        self.notifier.stop()
        self.can_bus.shutdown()

    def control_valve_state_command(self, can_bus: can.Bus, valve_address: int, rate: float):
        if not isinstance(valve_address, int):
            raise TypeError("not isinstance(valve_address, int)")
        if not (0x01 <= valve_address <= 0x7F):
            raise ValueError("not (0x01 <= valve_address <= 0x7F)")

        if not isinstance(rate, float):
            raise TypeError("not isinstance(rate, float)")
        if not (0.0 <= rate <= 1.0):
            raise ValueError("not (0.0 <= rate <= 1.0)")

        valve_open_state: int = 100  # 0, 100 [%], byte 1
        main_frequency: int = 10  # 10, 20 [Hz], byte 2
        main_active_perc: int = int(100 * rate)  # 0...100 [%], byte 3
        hold_phase_frequency: int = 5  # 0...20 [kHz], byte 4
        hold_phase_active_perc: int = 30  # 0...100 [%], byte 5
        pick_phase_duration: int = 130  # 0...200 [ms/10], byte 6

        msg = can.Message(
            arbitration_id=valve_address,
            data=[
                0,
                valve_open_state,
                main_frequency,
                main_active_perc,
                hold_phase_frequency,
                hold_phase_active_perc,
                pick_phase_duration,
                0,
            ],
            is_extended_id=False
        )

        try:
            can_bus.send(msg)
            self.get_logger().debug(f"control_valve_state_command: message sent on {can_bus.channel_info}, "
                                    f"valve_address: {valve_address}, rate: {rate:0.3f}, main_active_perc: {main_active_perc}")
        except can.CanError:
            self.get_logger().fatal("CanError, control_valve_state_command: message could not be sent")

    def broadcast_sync(self, can_bus: can.Bus, groups_number: int):
        if not isinstance(groups_number, int):
            raise TypeError("not isinstance(groups_number, int)")
        if not (1 <= groups_number <= 4):
            raise ValueError("not (1 <= groups_number <= 4)")

        tick: int = compute_tick_milliseconds()

        if not isinstance(tick, int):
            raise TypeError("not isinstance(tick, int)")
        if not (0 <= tick <= 2**32 - 1):
            raise ValueError("not (0 <= tick <= 2**32 - 1)")

        tick_0, tick_1, tick_2, tick_3 = tick.to_bytes(length=4, byteorder='little', signed=False)

        msg = can.Message(
            arbitration_id=0x80,
            data=[
                groups_number,
                tick_0,
                tick_1,
                tick_2,
                tick_3,
                0,
                0,
                0,
            ],
            is_extended_id=False
        )

        try:
            can_bus.send(msg)
            self.get_logger().debug(f"broadcast_sync: message sent on {can_bus.channel_info}, groups_number: {groups_number}, tick: {tick}")
        except can.CanError:
            self.get_logger().fatal("CanError, broadcast_sync: message could not be sent")

    def set_valve_address_command(self, can_bus: can.Bus, valve_address: int):
        if not isinstance(valve_address, int):
            raise TypeError("not isinstance(valve_address, int)")
        if not (1 <= valve_address <= 127):
            raise ValueError("not (1 <= valve_address <= 127)")

        msg = can.Message(
            arbitration_id=0x100,
            data=[
                valve_address,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            ],
            is_extended_id=False
        )

        try:
            can_bus.send(msg)
            self.get_logger().debug(f"set_valve_address_command: message sent on {can_bus.channel_info}, valve_address: {valve_address}")
        except can.CanError:
            self.get_logger().fatal("CanError, set_valve_address_command: message could not be sent")

    def ___reset_valve_address_command(self, can_bus: can.Bus):

        msg = can.Message(
            arbitration_id=0x500,
            data=[
                1,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            ],
            is_extended_id=False
        )

        try:
            can_bus.send(msg)
            self.get_logger().debug(f"reset_valve_address_command: message sent on {can_bus.channel_info}")
        except can.CanError:
            self.get_logger().fatal("CanError, reset_valve_address_command: message could not be sent")

    def test_set_valve_address_command_response(self, can_bus: can.Bus, valve_address: int, fill_data_array: bool):
        if not isinstance(valve_address, int):
            raise TypeError("not isinstance(valve_address, int)")
        if not (1 <= valve_address <= 127):
            raise ValueError("not (1 <= valve_address <= 127)")

        msg = can.Message(
            arbitration_id=0x100,
            data=[valve_address, 0, 0, 0, 0, 0, 0, 0, ] if fill_data_array else [valve_address],
            is_extended_id=False
        )

        try:
            can_bus.send(msg)
            self.get_logger().debug(f"test_set_valve_address_command_response: message sent on {can_bus.channel_info}, valve_address: {valve_address}")
        except can.CanError:
            self.get_logger().fatal("CanError, test_set_valve_address_command_response: message could not be sent")

    def broadcast_read_valve_state_command(self, can_bus: can.Bus):
        msg = can.Message(
            arbitration_id=0x480,
            data=[0, 0, 0, 0, 0, 0, 0, 0],
            is_extended_id=False
        )

        try:
            can_bus.send(msg)
            self.get_logger().debug(f"broadcast_read_valve_state_command: message sent on {can_bus.channel_info}")
        except can.CanError:
            self.get_logger().fatal("CanError, broadcast_read_valve_state_command: message could not be sent")


def main(args=None):
    rclpy.init(args=args)
    node = PwmNozzlesDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.shutdown()


if __name__ == '__main__':
    main()
