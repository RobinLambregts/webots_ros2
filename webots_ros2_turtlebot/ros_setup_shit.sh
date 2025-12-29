#!/usr/bin/env bash

echo "oude build shit verwijderen..."
rm -rf build/ install/ log/
echo "global ros environment laden..."
source /opt/ros/jazzy/setup.bash
echo "(re)builden..."
colcon build
echo "local environment laden..."
source install/setup.bash

echo "okay, ros2+webots zijn ready for use (normaal gezien)"