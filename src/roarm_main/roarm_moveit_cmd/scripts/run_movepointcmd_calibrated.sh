#!/bin/bash
# Script to run movepointcmd with calibration parameters loaded
# Usage: ./run_movepointcmd_calibrated.sh

# Get the package share directory
PACKAGE_SHARE=$(ros2 pkg prefix roarm_moveit_cmd)/share/roarm_moveit_cmd

# Path to calibration config
CALIB_CONFIG="${PACKAGE_SHARE}/config/calibration_offsets.yaml"

echo "Loading calibration parameters from: ${CALIB_CONFIG}"
echo "Starting movepointcmd node with calibration offsets..."

# Run movepointcmd with parameters from calibration config
ros2 run roarm_moveit_cmd movepointcmd --ros-args --params-file "${CALIB_CONFIG}"
