#!/bin/bash

roslaunch /home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_dependence.launch &
sleep 10
roslaunch /home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_camera.launch &
sleep 5
roslaunch /home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_sensor.launch &
sleep 5
roslaunch /home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_functions.launch
