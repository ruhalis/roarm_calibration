ruhalis@ruhalis:~/roarm_calibration$ sudo chmod 666 /dev/ttyUSB0
ruhalis@ruhalis:~/roarm_calibration$ ros2 run roarm_driver roarm_driver
ros2 run roarm_moveit_cmd movepointcmd --ros-args --params-file install/roarm_moveit_cmd/share/roarm_moveit_cmd/config/calibration_offsets.yaml 

ros2 launch roarm_moveit_cmd command_control.launch.py

ros2 run roarm_moveit_cmd getposecmd