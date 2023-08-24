#!/usr/bin/env python3
# pylint: disable=import-error
# pylint: disable=too-many-instance-attributes

"""
TODO: (pylint) add module description.
"""

import os
import curses
import threading

from ipb_car_monitor.cameras_consistency import CamerasConsistency
from ipb_car_monitor.sensor_status import SensorStatus


class View:
    """View class of the MVC architecture.
    Display the information about the sensors.
    """

    def __init__(self, node):
        self.node = node
        self.cameras_status = CamerasConsistency(self.node)
        self.sensors_status = SensorStatus(self.node)

        self.is_running = True

        # Screen size
        self.screen_height = 0
        self.screen_width = 0

        # Windows
        self.win_sensors = None
        self.win_cameras = None
        self.win_status_sensors = None
        self.win_status_cameras = None
        self.win_status_bar = None
        self.pad = None

        # Windows size
        self.dim_win_ptp = []
        self.dim_win_cameras = []
        self.dim_win_ptp_logs = []
        self.dim_win_cameras_logs = []
        self.dim_win_status_bar = []

        # Curses initialization
        self.stdscr = curses.initscr()
        self.stdscr.keypad(True)
        curses.noecho()
        curses.cbreak()

        # Define colors
        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(4, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(5, curses.COLOR_RED, curses.COLOR_BLACK)

        # Assign background color
        self.stdscr.bkgd(curses.color_pair(1))
        self.stdscr.clear()
        self.stdscr.refresh()

    def compute_win_sizes(self):
        """Compute the window size.
        Return true if the size changed, false otherwise.
        """

        screen_width, screen_height = os.get_terminal_size()
        screen_height = int(screen_height)
        screen_width = int(screen_width)

        # Size not changed
        if screen_height == self.screen_height and screen_width == self.screen_width:
            return False

        curses.resize_term(screen_height, screen_width)
        self.screen_height = screen_height
        self.screen_width = screen_width
        half_height = screen_height // 2
        half_width = screen_width // 2

        # [height, width, begin_y, begin_x]
        self.dim_win_ptp = [half_height, half_width, 0, 0]
        self.dim_win_cameras = [half_height, half_width, 0, half_width]
        self.dim_win_ptp_logs = [half_height - 3, half_width, half_height, 0]
        self.dim_win_cameras_logs = [
            half_height - 3,
            half_width,
            half_height,
            half_width,
        ]
        self.dim_win_status_bar = [3, screen_width, screen_height - 3, 0]

        return True

    def update_windows(self):
        """Update windows size and content."""

        if self.compute_win_sizes():
            self.win_sensors = curses.newwin(*self.dim_win_ptp)
            self.win_cameras = curses.newwin(*self.dim_win_cameras)
            self.win_status_sensors = curses.newwin(*self.dim_win_ptp_logs)
            self.win_status_cameras = curses.newwin(*self.dim_win_cameras_logs)
            self.win_status_bar = curses.newwin(*self.dim_win_status_bar)
            self.pad = curses.newpad(200, 200)

        # Show the menu
        self.display_status(
            self.sensors_status.get_ptp_sensors(),
            self.win_sensors,
            self.dim_win_ptp,
            "Sensor status",
        )
        self.display_status(
            self.cameras_status.get_cameras(),
            self.win_cameras,
            self.dim_win_cameras,
            "Camera consistency",
        )
        self.display_logs(
            self.sensors_status.get_logs(),
            self.win_status_sensors,
            self.dim_win_ptp_logs,
            "PTP logs",
        )
        self.display_logs(
            self.cameras_status.get_logs(),
            self.win_status_cameras,
            self.dim_win_cameras_logs,
            "Camera logs",
        )
        self.display_status_bar(self.win_status_bar)

    def display_status(self, sensors, win, dim, title):
        """Display the sensors status."""

        # Assign color and box shape
        win.bkgd(curses.color_pair(2))
        win.box()
        # Display title
        row = 1
        win.addstr(row, 2, title, curses.A_BOLD)
        row += 2

        # Display sensors info
        for sensor in sensors:
            space = 2

            # Dislay sensor status
            color_type = 3 if sensor.sync else 4
            color_type = color_type if sensor.alive else 5
            win.addstr(row, space, "\u25CF  ", curses.color_pair(color_type))
            space += 3

            # Check and cut string length if the
            # window size is smaller.
            device_info = str(sensor).expandtabs()
            space_left = (dim[1] - 2) - (space + len(device_info))
            if space_left < 0:
                sub_space = space_left - 3
                device_info = device_info[:sub_space] + "..."

            # Print device info
            win.addstr(row, space, device_info, curses.color_pair(2))
            space += len(device_info)

            # Clear everything after the end of the string
            space_left = (dim[1] - space) - 1
            win.addstr(row, space, " " * space_left, curses.color_pair(2))

            row += 1

        win.refresh()

    def display_logs(self, logs, win, dim, title):
        """Display the sensors logs."""

        # win.clear()
        win.bkgd(curses.color_pair(2))
        win.box()

        row = 1
        win.addstr(row, 2, title, curses.A_BOLD)
        row += 2

        b_height = len(logs.split("\n"))
        line = b_height - (dim[0] - row - 1)

        line = line if line > 0 else 0

        for msg in logs.split("\n")[line:]:
            # Check and cut string length if the
            # window size is smaller.
            space = 2
            space_left = (dim[1] - 2) - (space + len(msg))
            if space_left < 0:
                sub_space = space_left - 3
                msg = msg[:sub_space] + "..."

            win.addstr(row, space, msg)
            space += len(msg)

            # Clear everything after the end of the string
            space_left = (dim[1] - space) - 1
            win.addstr(row, space, " " * space_left, curses.color_pair(2))

            row += 1

        win.refresh()

    def display_status_bar(self, win):
        """Display the status bar.
        Display key commans, windows size.
        """
        # win.clear()
        win.bkgd(curses.color_pair(2))
        win.box()
        win.addstr(1, 2, "q: Exit")
        win.addstr(1, 10, "||")
        win.addstr(1, 14, "Size: {}x{}".format(self.screen_height, self.screen_width))
        win.refresh()

    def quit(self):
        """Quit method invoked when program as to be shutted down."""

        # Close GUI
        curses.nocbreak()
        self.stdscr.keypad(0)
        curses.echo()
        curses.endwin()

    def spin(self):
        self.sensors_status.spin()
        self.cameras_status.spin()
        self.update_windows()
        self.stdscr.refresh()
