#!/usr/bin/env python3
# pylint: disable=import-error

"""
TODO: (pylint) add module description.
"""

import time
import rclpy
import numpy as np

from sensor_msgs.msg import Image


class Camera:
    """
    TODO: (pylint) add class description.
    """

    identifier = None
    alive = False
    sync = False
    mismatches = 0
    count_errors = 0
    count_success = 0

    def __init__(self, identifier):
        self.identifier = identifier

    def __str__(self):
        if not self.alive:
            status = "dead"
        elif not self.sync:
            status = "not consistent"
        else:
            status = "consistent"

        string = ""
        string += f"Id: {self.identifier}\t"
        string += f"status: {status}"
        return string


class CamerasConsistency:
    """
    The Camera Consistency checking is able to determine which
    sensor connected to the PTP protocol, is experiencing delays.

    TODO: merge with ptp_status.py
    """

    # Check timestamps interval
    INTERVAL = 20
    # Check time interval
    TIME_INTERVAL = 3  # in sec
    # Range of neighbor checking
    RANGE = 2
    # Same value as in include/parameters.h
    PTP_OFFSET_LIMIT = 1000000  # 1ms (in ns)
    # Max number of allowed mismatches
    MAX_MISMATCHES = INTERVAL * 0.18  # 18%

    # Status
    STATUS_ALL_SYNC = 0
    STATUS_NOT_SYNC = 1

    # Camera obj
    cameras = {}

    # Dict containing all prev timestamps
    times = {}
    # List contaning all the prev status
    status = []
    # Past timer variable
    past_timer = 0
    # Previous frame rate
    frame_rate = 0

    # Check map of each camera timestamps
    # within a certain INTERVAL
    check_map = 0

    # Dict of cameras with errors {id: idx, ...}
    cameras_error = {}
    # List of dead cameras
    cameras_dead = {}

    # logs logs
    logs = ""

    # Topic list
    topic_list = []

    def __init__(self, node):
        # Init param
        self.node = node
        self.past_timer = time.time()
        self.listen_topics()

    def listen_topics(self):
        # Initialize camera image subscribers
        topic_list = self.node.get_topic_names_and_types()
        new_topics = [topic for topic in topic_list if topic not in self.topic_list]

        for topic in new_topics:
            self.topic_list.append(topic)
            if topic[0].split("/")[-1] == "image_raw":
                self.node.create_subscription(Image, topic[0], self.camera_callback)

    def camera_callback(self, msg):
        """Topic camera image header callback."""
        identifier = msg.header.frame_id
        if identifier not in self.times.keys():
            self.logs += "[{}] Camera connected \n".format(identifier)
            self.cameras[identifier] = Camera(msg.header.frame_id)
            self.times[identifier] = []

        self.times[identifier].append(msg.header.stamp)

    def time_to_check(self):
        if len(self.times.keys()) == 0:
            return False

        master_cam = self.get_master_cam()
        right_time = time.time() - self.past_timer > self.TIME_INTERVAL

        return master_cam != None or right_time

    def check_life(self):
        sensors_ts = list(self.times.items()).copy()
        for id_sens, timestamps in sensors_ts:
            if len(timestamps) == 0:
                self.cameras_dead[id_sens] = 0
                del self.times[id_sens]
            elif id_sens in self.cameras_dead.keys():
                del self.cameras_dead[id_sens]

    def right_amount(self, cam):
        """Check if the number of timestamps registered from the
        topic callback (handler) is equal to the number of self.INTERVAL.

        The comparison is not made directly but by checking if the
        rest of the division is equal to 0. This in order to handle
        situations in which we skipped the checking synchronization
        at the right interval, but still be possible for the next one.
        """
        return (len(self.times[cam]) != 0) and (
            (len(self.times[cam]) / self.INTERVAL) % 1
        ) == 0

    def get_master_cam(self):
        # Get cam with right amount of data i.e with
        # the correct number of timestamps == self.INTERVAL.
        # TODO: insert into is_right_amount in order to remove delay
        for cam in enumerate(self.times.keys()):
            if self.right_amount(cam[1]):
                return cam

        return None

    def check_consistency(self):
        """
        This function allow to spot the t ... (deleted for mistake)

        (the incremental numbers are called steps)
        """
        check_map = np.zeros((len(self.times.keys()), self.INTERVAL))
        check_map[:, :] = -1
        master_cam = self.get_master_cam()

        # If no one has the right amount of data
        if master_cam is None:
            return

        # ==================================================
        # Check the consistency of the data by computing the
        # correspondent timestamps for every different camera.
        for step in range(self.INTERVAL):
            for idx_cam, id_cam in enumerate(self.times.keys()):
                corr_step = -1

                # Range loop which checks the timestamp offset between the
                # main camera and the neighbors of the current step of the
                # other cameras.
                # ex: step = 8, RANGE = 2
                # -> (check between the neighbors):
                # -> [6,7,8,9,10]
                for i in range(-self.RANGE, self.RANGE + 1, 1):
                    # Check if out of index, i.e. between the INTERVAL
                    if not (0 <= step + i <= len(self.times[id_cam]) - 1):
                        continue

                    # Get offset of neighbor timestamps
                    offset = abs(
                        self.times[id_cam][step + i] - self.times[master_cam[1]][step]
                    )

                    # Check if the offest of the two timestamps is smaller than a
                    # certain threshold: this means that the ts are corresponding.
                    corr_step = (
                        step + i if offset < self.PTP_OFFSET_LIMIT else corr_step
                    )
                # Assign the corresponding step id
                check_map[idx_cam][step] = corr_step

        # Save cameras with error
        for idx_cam, id_cam in enumerate(self.times.keys()):
            # Check for the number of mismatches (-1)
            mismatches = np.where(check_map[idx_cam] == -1)[0].size
            perc_diff = mismatches / self.INTERVAL * 100
            self.cameras[id_cam].mismatches = round(perc_diff)
            if mismatches > self.MAX_MISMATCHES:
                self.cameras_error[id_cam] = idx_cam

    def save_results(self):
        """Check the consistency of the timestamp map extracted
        from the function this.check_consistency.
        """
        for id_cam in self.cameras.keys():
            # Sensor dead
            if id_cam in self.cameras_dead:
                if self.cameras[id_cam].alive == True:
                    self.logs += "[{}] Camera dead \n".format(id_cam)
                self.cameras[id_cam].alive = False
                self.cameras[id_cam].sync = False
                self.cameras[id_cam].count_errors += 1
            # Camera error
            elif id_cam in self.cameras_error.keys():
                self.logs += "[{}] Camera mismatched of {}%\n".format(
                    id_cam, self.cameras[id_cam].mismatches
                )
                self.cameras[id_cam].alive = True
                self.cameras[id_cam].sync = False
                self.cameras[id_cam].count_errors += 1
            # Camera success
            else:
                self.cameras[id_cam].alive = True
                self.cameras[id_cam].sync = True
                self.cameras[id_cam].count_success += 1

    def refresh(self):
        # Clean lists
        for cam in self.times.keys():
            self.times[cam] = []

        self.cameras_error = {}
        # self.cameras_dead = []

        # Save last check time
        self.past_timer = time.time()

    def get_cameras(self):
        return self.cameras.values()

    def get_logs(self):
        return self.logs

    def spin(self):
        if self.time_to_check():
            self.check_life()
            self.check_consistency()
            self.save_results()
            self.refresh()
        self.listen_topics()
