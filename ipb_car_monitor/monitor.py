#!/usr/bin/env python3

import sys
import rospy

from controller.controller import Controller
from model.model import Model
from view.view import View


class Monitor:
    cameras_status = None
    sensors_status = None

    def __init__(self):
        # Init ROS node
        rospy.init_node("monitor", anonymous=True, disable_signals=False)

        # Init classes
        self.controller = Controller()
        self.model = Model()
        self.view = View(self.controller, self.model)

    def run(self):
        rate = rospy.Rate(20)  # 20hz

        while self.controller.running_status():
            self.model.spin()
            self.view.spin()
            self.controller.spin()

            rate.sleep()

        self.quit()

    def quit(self):
        rospy.logerr("quitting")
        self.view.quit()
        self.model.quit()
        sys.exit(0)


if __name__ == "__main__":
    watch = Monitor()
    watch.run()
