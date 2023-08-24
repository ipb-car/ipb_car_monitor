#!/usr/bin/env python3
# pylint: disable=import-error

"""
TODO: (pylint) add module description.
"""

import subprocess
from importlib import import_module
import time
import sys
from typing import Optional
from collections import deque

import numpy as np
import rospy
from std_msgs.msg import Bool


def is_rpi_synchronized():
    """ssh into pi and checks if it is synched."""
    process = subprocess.Popen('ssh pi "timedatectl"'.split(), stdout=subprocess.PIPE)
    try:
        stdout, _ = process.communicate(timeout=5)
        stdout = stdout.decode("utf-8")
        return "System clock synchronized: yes" in stdout
    except subprocess.TimeoutExpired:
        print("[WARNING] No connection to rpi!")
        return False


def trim_ellipsis(string: str, max_length: int):
    """Trimming strings longer than the max length for better display"""
    return string if len(string) <= max_length else string[: max_length - 3] + "..."


class Sensor:
    """This class holds the sensor data that should be displayed on the monitor.
    The information includes the sensor ID, its current status, and the current FPS.
    """

    topic_name = None
    alive = False
    sync = False
    fps = 0
    offset = 0
    count_errors = 0
    count_success = 0

    def __init__(self, topic_name: str):
        """Initialize sensor with an topic_name."""
        self.topic_name = topic_name

    def __str__(self):
        if not self.alive:
            status = "dead"
        elif not self.sync:
            status = "not sync"
        else:
            status = "sync"

        id_column = f"Id: {trim_ellipsis(self.topic_name, 18)}"
        status_column = f"status: {status}"
        fps_column = f"fps: {self.fps:5.1f}"
        string = f"{id_column:<28}{status_column:<19}{fps_column:<33}"

        return string


class SensorStatus:
    """Node for capturing sensor information to provide an overview of the current sensor status.
    TODO: merge with cameras_consistency.py
    """

    # Check time interval
    TIME_INTERVAL = 0.5  # in sec
    # Window size for fps counting
    FPS_WINDOW_SIZE = 25
    # Range of neighbor checking
    RANGE = 2
    # Same value as in include/parameters.h
    PTP_OFFSET_LIMIT = 700000000  # 700ms (in ns)
    PTP_OFFSET_LIMIT_DU = rospy.Duration(secs=0, nsecs=PTP_OFFSET_LIMIT)

    # Status
    STATUS_ALL_SYNC = 0
    STATUS_NOT_SYNC = 1

    def __init__(self):
        # Init param
        self.past_timer = time.time()

        self.id_reference_time = ""

        self.sensors = {}

        # Dict containing all prev timestamps in this time interval
        self.times = {}
        # Dict containing sliding windows of prev system times for FPS computing.
        self.sys_times_window = {}
        # List contaning all the prev status
        self.status = []
        # Past timer variable
        self.past_timer = 0

        # Numpy array containing the diff. of each
        # sens. in relation to other sens.
        self.np_sens_diff = None
        # Numpy array containing the index of the
        # most reliable sens.
        self.idx_reference_time = None
        # Dict of sensors with errors {id: idx, ...}
        self.sensors_error = {}
        # List of dead sensors
        self.sensors_dead = {}

        # logs logs
        self.logs = ""

        # topic list
        self.topic_list = []

        self.monitored_topics = str(rospy.get_param("/topics_to_monitor")).split(" ")
        self.ready_to_record_pub = rospy.Publisher(
            "ready_to_record", Bool, queue_size=1, latch=True
        )
        self._binary_subs = {}
        self.published_ready_to_record = False

        # ==================
        self.listen_topics()

    def listen_topics(self):
        """Initialize sensor subscribers."""

        topic_list = rospy.get_published_topics()
        new_topics = [
            (topic_name, topic_type)
            for topic_name, topic_type in topic_list
            if (
                topic_name not in self.topic_list
                and topic_name in self.monitored_topics
            )
        ]

        for topic_name, _ in new_topics:
            self.topic_list.append(topic_name)
            self._binary_subs[topic_name] = rospy.Subscriber(
                topic_name,
                rospy.AnyMsg,
                self.binary_callback,
                topic_name,
            )

        if not self.published_ready_to_record:
            self.check_ready_to_record()

    def check_ready_to_record(self):
        for topic_name in self.monitored_topics:
            if topic_name not in self.topic_list:
                return
        if rospy.is_shutdown():
            return
        if is_rpi_synchronized():
            self.ready_to_record_pub.publish((Bool(True)))

    def binary_callback(self, data, topic_name):
        connection_header = data._connection_header["type"].split("/")
        ros_pkg = connection_header[0] + ".msg"
        msg_type = connection_header[1]
        msg_class = getattr(import_module(ros_pkg), msg_type)
        self._binary_subs[topic_name].unregister()
        rospy.Subscriber(topic_name, msg_class, self.topic_callback, topic_name)

    def topic_callback(self, msg, topic_name):
        """Topic sensor frame header callback."""
        if topic_name not in self.times.keys():
            self.logs += "{} - sensor connected \n".format(topic_name)
            self.sensors[topic_name] = Sensor(topic_name)
            self.times[topic_name] = []
            self.sys_times_window[topic_name] = deque(maxlen=self.FPS_WINDOW_SIZE)
            if msg.header.frame_id == "clock":
                self.id_reference_time = topic_name
        self.times[topic_name].append(msg.header.stamp)
        self.sys_times_window[topic_name].append(time.time())

    def time_to_check(self):
        return (
            self.times.keys() and (time.time() - self.past_timer) > self.TIME_INTERVAL
        )

    def check_life(self):
        sensors_ts = list(self.times.items()).copy()
        for id_sens, timestamps in sensors_ts:
            if len(timestamps) == 0:
                self.sensors_dead[id_sens] = 0
                del self.times[id_sens]
            elif id_sens in self.sensors_dead.keys():
                del self.sensors_dead[id_sens]

    def check_consistency(self):
        """Compute the timestamp abs difference between each sensor.
        ex: [sens1, sens2, sens3]
        -> [[sens1 - sens1, sens1 - sens2, sens1 - sens3], [sens2 - sens1, ...]
        """
        sens_diff = []
        for idx, id_sens_i in enumerate(list(self.times.keys())):
            # Get the index of the reference time
            if id_sens_i == self.id_reference_time:
                self.idx_reference_time = idx

            # Compare the timestamp differences between each sensors
            diff = []
            for id_sens_j in list(self.times.keys()):
                diff.append(abs(self.times[id_sens_j][0] - self.times[id_sens_i][0]))
            sens_diff.append(diff)

        if len(sens_diff) == 0:
            return

        # To numpy array
        self.np_sens_diff = np.array(sens_diff)

        # Matrix containing per each position a boolean value referign
        # whether the sensor respected the right offset interval.
        np_diff_bool = self.np_sens_diff < self.PTP_OFFSET_LIMIT_DU

        # Get the indices of the non reliable sensors in respect to the
        # most reliable.
        idxs_wrong_ptp = np.where(np_diff_bool[self.idx_reference_time] == False)

        # Save sensors with error
        for idx_sens, id_sens in enumerate(list(self.times.keys())):
            # if rpi: check if it is synched with gps
            if self.id_reference_time == id_sens:
                if not is_rpi_synchronized():
                    self.sensors_error[id_sens] = idx_sens
            # Check if time difference to big
            offset = self.np_sens_diff[self.idx_reference_time, idx_sens]
            self.sensors[id_sens].offset = offset
            if idx_sens in idxs_wrong_ptp[0]:
                self.sensors_error[id_sens] = idx_sens

    def compute_fps(self):
        """Compute the fps by counting the number of frames in the windows."""
        for id_sens in self.sensors.keys():
            rate = 0.0
            if self.sensors[id_sens].alive:
                n = len(self.sys_times_window[id_sens])
                if n < 2:
                    continue
                rate = (n - 1) / (
                    self.sys_times_window[id_sens][n - 1]
                    - self.sys_times_window[id_sens][0]
                )
            self.sensors[id_sens].fps = rate

    def update_status(self):
        for id_sens in self.sensors.keys():
            # Sensor dead
            if id_sens in self.sensors_dead.keys():
                if self.sensors[id_sens].alive:
                    self.logs += "[{}] Sensor dead \n".format(id_sens)
                self.sensors[id_sens].alive = False
                self.sensors[id_sens].sync = False
                self.sensors[id_sens].count_errors += 1

            # Sensor error
            elif id_sens in self.sensors_error.keys():
                self.logs += "[{}] Sensor error (offset: {}) \n".format(
                    id_sens, self.sensors[id_sens].offset
                )
                self.sensors[id_sens].alive = True
                self.sensors[id_sens].sync = False
                self.sensors[id_sens].count_errors += 1

            # Sensor success
            else:
                self.sensors[id_sens].alive = True
                self.sensors[id_sens].sync = True
                self.sensors[id_sens].count_success += 1

    def refresh(self):
        # Clean lists
        for sens in list(self.times.keys()):
            self.times[sens] = []

        self.sensors_error = {}

        # Save last check time
        self.past_timer = time.time()

    def get_ptp_sensors(self):
        return self.sensors.values()

    def get_logs(self):
        return self.logs

    def spin(self):
        if self.time_to_check():
            self.check_life()
            self.check_consistency()
            self.update_status()
            self.refresh()
            self.compute_fps()
        self.listen_topics()
