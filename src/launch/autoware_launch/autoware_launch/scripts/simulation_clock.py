#!/usr/bin/env python3

import time

import rclpy
from rclpy.clock import Clock as RclpyClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class SimulationClock(Node):
    def __init__(self):
        super().__init__("simulation_clock")
        rate_hz = self.declare_parameter("publish_rate_hz", 100.0).value
        if rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        self._publisher = self.create_publisher(Clock, "/clock", 1)
        self._start_ns = time.monotonic_ns()
        steady_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish, clock=steady_clock)

    def _publish(self):
        elapsed_ns = time.monotonic_ns() - self._start_ns
        msg = Clock()
        msg.clock.sec = elapsed_ns // 1_000_000_000
        msg.clock.nanosec = elapsed_ns % 1_000_000_000
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationClock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
