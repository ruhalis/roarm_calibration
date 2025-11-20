# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 Humble workspace for the RoArm-M2-S robotic arm, integrating MoveIt2 for motion planning and calibration. The project is designed for Ubuntu 22.04 with ROS2 Humble.

## Build and Compilation

### Initial Setup (First Time Only)
```bash
./build_first.sh
```
This runs:
- `colcon build` (full workspace build)
- Adds workspace to `~/.bashrc`

### Regular Development Build
```bash
./build_common.sh
```
This runs:
- `colcon build` (full workspace)
- Sources the workspace: `source install/setup.bash`

### Build Single Package
```bash
colcon build --packages-select <package_name>
source install/setup.bash
```

## Hardware Connection

The RoArm-M2-S connects via USB serial (typically `/dev/ttyUSB0`). Before running the driver:
```bash
sudo chmod 666 /dev/ttyUSB0
```

If using a different serial port, modify line 15 in `src/roarm_main/roarm_driver/roarm_driver/roarm_driver.py`:
```python
serial_port = "/dev/ttyUSB0"  # Change as needed
```

## Common Launch Commands

### Core Control

**Driver Node (Required for Physical Robot):**
```bash
ros2 run roarm_driver roarm_driver
```

**Basic Joint Control with Rviz2:**
```bash
ros2 launch roarm_description display.launch.py
```

**Interactive MoveIt2 Control:**
```bash
ros2 launch roarm_moveit interact.launch.py
```

**Command-Based Control (Services/Actions):**
```bash
ros2 launch roarm_moveit_cmd command_control.launch.py
```

**Calibrated Command Control:**
```bash
ros2 launch roarm_moveit_cmd command_control_calibrated.launch.py
```

### Servo/Keyboard/Gamepad Control

Start servo control demo:
```bash
ros2 launch moveit_servo demo.launch.py
```

In separate terminals:
```bash
ros2 run roarm_moveit_cmd setgrippercmd
ros2 run roarm_moveit_cmd keyboardcontrol
```

## Calibration

### Running Calibration Script
```bash
ros2 run roarm_moveit_cmd calibrate_roarm.py \
  --target 0.2 -0.1 -0.1 \
  --iterations 20 \
  --home 0.2 0.0 0.1 \
  --settling-time 1.0 \
  --output-dir .
```

This generates:
- `calibration_YYYYMMDD_HHMMSS.json` - Full results
- `calibration_YYYYMMDD_HHMMSS.csv` - Tabular data

### Using Calibrated Control

Calibration offsets are stored in:
```
src/roarm_main/roarm_moveit_cmd/config/calibration_offsets.yaml
```

To use calibrated control, launch with:
```bash
ros2 launch roarm_moveit_cmd command_control_calibrated.launch.py
```

Or when running nodes directly with calibration:
```bash
ros2 run roarm_moveit_cmd movepointcmd \
  --ros-args --params-file install/roarm_moveit_cmd/share/roarm_moveit_cmd/config/calibration_offsets.yaml
```

## Service Commands

These require `command_control.launch.py` or `command_control_calibrated.launch.py` to be running.

**Get Current Pose:**
```bash
# Terminal 1
ros2 run roarm_moveit_cmd getposecmd

# Terminal 2
ros2 service call /get_pose_cmd roarm_moveit/srv/GetPoseCmd
```

**Move to Position:**
```bash
# Terminal 1
ros2 run roarm_moveit_cmd movepointcmd

# Terminal 2
ros2 service call /move_point_cmd roarm_moveit/srv/MovePointCmd "{x: 0.2, y: 0, z: 0}"
```

**Control Gripper:**
```bash
# Terminal 1
ros2 run roarm_moveit_cmd setgrippercmd

# Terminal 2
ros2 topic pub /gripper_cmd std_msgs/msg/Float32 "{data: 0.0}" -1
```

**Draw Circle:**
```bash
# Terminal 1
ros2 run roarm_moveit_cmd movecirclecmd

# Terminal 2
ros2 service call /move_circle_cmd roarm_moveit/srv/MoveCircleCmd "{x: 0.2, y: 0, z: 0, radius: 0.1}"
```

## Architecture

### Package Structure

**roarm_main/** - Core robotic arm functionality
- `roarm_driver/` - Hardware driver (serial communication with ESP32)
- `roarm_description/` - URDF robot models
- `roarm_moveit/` - MoveIt2 configuration (kinematics, planning, controllers)
- `roarm_moveit_ikfast_plugins/` - Fast IK solver plugins
- `roarm_moveit_cmd/` - Control command nodes (C++ service implementations)
  - `scripts/calibrate_roarm.py` - Calibration automation script
  - `config/calibration_offsets.yaml` - Calibration compensation parameters
  - `src/movepointcmd.cpp` - Move to XYZ position service
  - `src/getposecmd_moveit2.cpp` - Get current pose service
  - `src/setgrippercmd.cpp` - Gripper control action
  - `src/movecirclecmd.cpp` - Circular motion planner
  - `src/keyboardcontrol.cpp` - Keyboard input handler
- `moveit_servo/` - Real-time servo control for keyboard

### Calibration System

The calibration system measures positioning errors by:
1. Moving to target positions repeatedly
2. Reading actual positions via forward kinematics
3. Calculating systematic errors (X, Y, Z offsets)
4. Generating compensation parameters in `calibration_offsets.yaml`

The calibrated launch file (`command_control_calibrated.launch.py`) loads these offsets and applies them before inverse kinematics solving, improving accuracy from ~14mm to ~5mm error.

### Control Flow

1. **Hardware Layer**: `roarm_driver` communicates via serial with ESP32 controller
2. **ROS2 Control**: `ros2_control_node` manages joint controllers
3. **MoveIt2 Layer**: Motion planning, kinematics solving, collision checking
4. **Command Layer**: `roarm_moveit_cmd` services expose high-level commands
5. **UI Layer**: Rviz2, keyboard control publish to ROS2 topics/services

### Key Topics and Services

- `/robot_description` - URDF model
- `/joint_states` - Current joint positions
- `/move_point_cmd` - Service to move end-effector to XYZ
- `/get_pose_cmd` - Service to get current end-effector pose
- `/gripper_cmd` - Topic for gripper control
- `/led_ctrl` - Topic for gripper LED control (0-255)

## Python Dependencies

Install via:
```bash
python3 -m pip install -r requirements.txt
```

## Notes

- Always source the workspace after building: `source install/setup.bash`
- Serial port permissions must be set each time the robot is connected
- Rviz2 view controls: Left-drag (pan), Right-drag (rotate), Scroll (zoom), Middle-drag (vertical)
- When using MoveIt2, clicking "Plan & Execute" is required to move the physical robot
