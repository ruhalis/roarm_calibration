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

The calibration system supports two modes: **single-target** (backward compatible) and **multi-target** (comprehensive workspace mapping).

### Single-Target Calibration (Quick Test)

Test a single position repeatedly to measure accuracy and repeatability:

```bash
ros2 run roarm_moveit_cmd calibrate_roarm.py \
  --target 0.2 -0.1 -0.1 \
  --iterations 20 \
  --home 0.2 0.0 0.1 \
  --settling-time 1.0 \
  --output-dir .
```

**Parameters:**
- `--target X Y Z` - Target position to test
- `--iterations N` - Number of repetitions (default: 3)
- `--home X Y Z` - Home position to return to (default: 0.2 0.0 0.1)
- `--settling-time S` - Wait time after movement (default: 1.0s)
- `--output-dir PATH` - Output directory (default: current directory)

**Output files:**
- `calibration_YYYYMMDD_HHMMSS.json` - Full results with statistics
- `calibration_YYYYMMDD_HHMMSS.csv` - Tabular data

### Multi-Target Calibration (Comprehensive)

Test multiple positions across the workspace to map positioning errors:

```bash
ros2 run roarm_moveit_cmd calibrate_roarm.py \
  --targets-config src/roarm_main/roarm_moveit_cmd/config/calibration_targets.yaml \
  --loops 2 \
  --output-dir .
```

**Parameters:**
- `--targets-config PATH` - YAML file with target positions
- `--loops N` - Number of complete cycles through all targets (default: 1)
- `--settling-time S` - Override config settling time (optional)
- `--home X Y Z` - Override config home position (optional)
- `--output-dir PATH` - Output directory (default: current directory)

**Movement Pattern (per loop):**
```
home → target1 → home → target2 → home → ... → target9 → home
```

**Default Config:** `config/calibration_targets.yaml` includes 9 targets in a 3×3 grid:
- 3 height levels (low/mid/high: -0.10m, 0.0m, +0.10m)
- 3 radial distances (front/mid/back: 0.15m, 0.20m, 0.25m)
- 3 lateral positions (left/center/right: -0.10m, 0.0m, +0.10m)

**Output files:**
- `calibration_multi_YYYYMMDD_HHMMSS.json` - Full results with per-target and global statistics
- `calibration_multi_YYYYMMDD_HHMMSS.csv` - Comprehensive tabular data

**Example output:** With 2 loops and 9 targets:
- Total movements: 36 (2 loops × 9 targets × 2 moves)
- Estimated time: ~30-40 minutes
- Statistics: Per-target errors + global workspace accuracy
- Comparison: Best/worst performing regions

### Customizing Target Positions

Edit `config/calibration_targets.yaml` to customize target positions:

```yaml
targets:
  - name: "custom_target_1"
    x: 0.18
    y: -0.05
    z: 0.05
    description: "Custom test position"

  - name: "custom_target_2"
    x: 0.22
    y: 0.05
    z: -0.05
    description: "Another test position"
```

**Workspace limits** (safety validation):
```yaml
workspace_limits:
  x_min: 0.10
  x_max: 0.30
  y_min: -0.15
  y_max: 0.15
  z_min: -0.15
  z_max: 0.15
```

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

### Analyzing Calibration Results

**Single-target mode:** Provides statistics for one position (mean error, std dev, repeatability)

**Multi-target mode:** Provides:
1. **Per-target statistics:** Individual accuracy metrics for each position
2. **Global statistics:** Overall workspace accuracy (mean, std dev, RMS)
3. **Target comparison:** Ranking of best/worst performing regions
4. **Position-dependent errors:** Data to identify if errors vary across workspace

Use multi-target results to:
- Identify workspace regions with higher/lower accuracy
- Determine if constant offsets are sufficient or if position-dependent compensation is needed
- Plan future polynomial compensation implementation

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
1. Moving to target positions repeatedly (single or multiple targets)
2. Reading actual positions via forward kinematics
3. Calculating systematic errors (X, Y, Z offsets)
4. Generating compensation parameters in `calibration_offsets.yaml`

**Modes:**
- **Single-target:** Tests one position repeatedly for quick accuracy assessment
- **Multi-target:** Tests 9 positions across workspace for comprehensive error mapping

**Movement pattern (multi-target):** Home → Target1 → Home → Target2 → Home ... (configurable loops)

The calibrated launch file (`command_control_calibrated.launch.py`) loads these offsets and applies them before inverse kinematics solving, improving accuracy from ~14mm to ~5mm error.

**Files:**
- `scripts/calibrate_roarm.py` - Automated calibration script (single/multi-target)
- `config/calibration_targets.yaml` - Multi-target position definitions (9 targets in 3×3 grid)
- `config/calibration_offsets.yaml` - Compensation parameters applied during control

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
