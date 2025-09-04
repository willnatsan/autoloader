#!/bin/bash

source /opt/ros/humble/setup.bash

# Check if any running node contains "autoloader"
if ros2 node list | grep -q "autoloader"; then
    echo "true"
else
    echo "false"
fi
