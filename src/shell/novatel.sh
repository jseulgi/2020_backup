#!/bin/bash

roslaunch novatel_gps_driver tester_for_usb.launch
gnome-terminal -e command roslaunch ouster_ros os1.launch;
gnome-terminal -e command rosrun pcan pcan_pub_write;
gnome-terminal -e command rosrun autonomous_vehicle CanNodegen;

