#!/usr/bin/env python3
# pylint: disable=import-error

"""
TODO: Add module description.
"""

from model.sensor_status import SensorStatus
from model.cameras_consistency import CamerasConsistency


class Model:
    """
    TODO: Add module description.
    """

    def __init__(self):
        # Init class
        self.cameras_status = CamerasConsistency()
        self.sensors_status = SensorStatus()

    def get_cameras(self):
        """
        Get camera object containing the current status.
        """
        return self.cameras_status.get_cameras()

    def get_ptp_sensors(self):
        """
        Get sensor object containing the current status.
        """
        return self.sensors_status.get_ptp_sensors()

    def get_cameras_logs(self):
        """
        Get all the saved logs of the cameras.
        """
        return self.cameras_status.get_logs()

    def get_ptp_logs(self):
        """
        Get all the saved logs of the sensors.
        """
        return self.sensors_status.get_logs()

    def spin(self):
        """
        General spin of the model module.
        """
        self.cameras_status.spin()
        self.sensors_status.spin()

    def quit(self):
        pass
