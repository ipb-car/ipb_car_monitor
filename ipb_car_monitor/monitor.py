#!/usr/bin/env python3

import sys

import rclpy
from ipb_car_monitor.view import View


class Monitor:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("monitor")
        self.rate = self.node.create_rate(20)
        self.view = View(self.node)

    def run(self):
        self.view.spin()
        self.rate.sleep()


def main():
    watch = Monitor()
    watch.run()


if __name__ == "__main__":
    main()
