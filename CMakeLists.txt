cmake_minimum_required(VERSION 3.16.3)
project(ipb_car_monitor)
find_package(catkin REQUIRED)
catkin_package()
install(PROGRAMS ipb_car_monitor/monitor.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts)
