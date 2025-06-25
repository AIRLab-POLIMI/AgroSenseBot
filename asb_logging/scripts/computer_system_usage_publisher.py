#! /usr/bin/python3

import psutil

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from asb_msgs.msg import SystemUsage, ProcessCpuUsage


class ComputerSystemUsagePublisher(Node):
    def __init__(self):
        super().__init__('computer_system_usage_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.publisher_ = self.create_publisher(SystemUsage, '/asb_computer_system_usage', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Prime cpu_percent() for accuracy
        for proc in psutil.process_iter():
            try:
                proc.cpu_percent(interval=None)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

    def timer_callback(self):
        cpu_percents = psutil.cpu_percent(percpu=True)
        mem = psutil.virtual_memory()

        msg = SystemUsage()
        msg.cpu_core_usage_percent = cpu_percents
        msg.mem_total = float(mem.total) / 2**20
        msg.mem_available = float(mem.available) / 2**20
        msg.mem_used = float(mem.used) / 2**20
        msg.mem_used_percent = float(mem.percent)

        proc: psutil.Process
        for proc in psutil.process_iter(attrs=['pid', 'name', 'exe', 'cmdline', 'cpu_percent', 'cpu_times', 'memory_info']):
            try:
                cpu_percent = proc.cpu_percent(interval=None)
                cpu_times = proc.cpu_times()
                total_time = cpu_times.user + cpu_times.system

                proc_msg = ProcessCpuUsage()
                proc_msg.pid = proc.info['pid']
                proc_msg.name = proc.info['name']
                proc_msg.exe = str(proc.info['exe'])
                proc_msg.cmdline = str(proc.info['cmdline'])
                proc_msg.cpu_percent = float(cpu_percent)
                proc_msg.cpu_time_total = float(total_time)
                proc_msg.mem_rss = float(proc.info['memory_info'].rss) / 2**20

                msg.processes.append(proc_msg)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ComputerSystemUsagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
