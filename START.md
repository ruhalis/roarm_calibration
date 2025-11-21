sudo chmod 666 /dev/ttyUSB0

ros2 run roarm_driver roarm_driver

ros2 run roarm_moveit_cmd movepointcmd --ros-args --params-file install/roarm_moveit_cmd/share/roarm_moveit_cmd/config/calibration_offsets.yaml 

ros2 launch roarm_moveit_cmd command_control.launch.py

ros2 run roarm_moveit_cmd getposecmd


  ros2 run roarm_moveit_cmd calibrate_roarm.py \
    --targets-config
  src/roarm_main/roarm_moveit_cmd/config/calibration_targets.yaml \
    --loops 4 \
    --output-dir .