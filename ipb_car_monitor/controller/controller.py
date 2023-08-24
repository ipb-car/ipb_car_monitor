#!/usr/bin/env python3
# pylint: disable=import-error

"""
TODO: (pylint) add module description.
"""

import rospy


class Controller:
    """
    TODO: (pylint) add class description.
    """

    def __init__(self):
        self.is_running = True

    def action_shutdown(self):
        """
        Command a shutting down action.
        """
        self.is_running = False

    def running_status(self):
        """
        Retrieve the running status of the monitor.
        """

        if rospy.is_shutdown() or not self.is_running:
            return False

        return True

    def spin(self):
        pass

    def quit(self):
        pass
