#!/bin/bash

# Configuring AutoLoader Cameras
$(/home/labuser/10k_ws/src/AutoLoader/openpnp-capture/build/linux/tests/openpnp-capture-autoloader)

autoloader_running=$(/home/labuser/10k_ws/src/AutoLoader/autoloader_check.bash)
if [ "$autoloader_running" = "false" ]; then
    cd ~/10k_ws
    source /opt/ros/humble/setup.bash
    source ~/10k_ws/install/setup.bash
    ros2 launch autoloader_runner mvp.launch.py
fi