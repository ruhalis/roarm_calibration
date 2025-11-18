# RoArm-M2-S Calibration Enhancement Plan

## Executive Summary

This document outlines a multi-phase approach to improve the positioning accuracy of the RoArm-M2-S robotic arm from the current ~14.6mm euclidean error to an expected ~2-3mm error through systematic calibration.

**Current Status:**
- Systematic positioning errors: ~10mm in X and Z axes
- Excellent repeatability: <0.35mm standard deviation
- No existing calibration compensation mechanisms

**Expected Improvements:**
- Phase 1 (Quick Fix): ~60-70% error reduction immediately
- Phase 3 (Geometric Calibration): ~80-90% total error reduction
- Final Expected Accuracy: ~2-3mm euclidean error

---

## Phase 1: Quick Offset Compensation ✅ COMPLETED

### Objective
Reduce positioning errors by 60-70% using existing calibration data with constant offset compensation.

### Implementation (Completed)

#### 1. Created Calibration Configuration File
**File:** `src/roarm_main/roarm_moveit_cmd/config/calibration_offsets.yaml`

```yaml
calibration:
  enabled: true
  method: "constant_offset"

  offsets:
    x: -0.00997215  # meters (-9.97mm mean error)
    y: -0.00005325  # meters (-0.05mm mean error)
    z:  0.01067700  # meters (+10.68mm mean error)
```

**Source:** Calibration data from `calibration_20251118_105156.json`
- Target position mean errors: X=-9.97mm, Y=-0.05mm, Z=+10.68mm
- Home position mean errors: X=-9.96mm, Y=+0.17mm, Z=+11.20mm

#### 2. Modified Motion Command Handler
**File:** `src/roarm_main/roarm_moveit_cmd/src/movepointcmd.cpp`

**Changes:**
- Added `CalibrationOffsets` struct to hold parameters
- Load calibration parameters from ROS2 parameter server
- Apply offsets to commanded positions before inverse kinematics
- Added logging to show original vs. corrected commands

**Key Code Section (lines 24-59):**
```cpp
// Load calibration offset parameters from parameter server
CalibrationOffsets calib;
node->declare_parameter("calibration.enabled", false);
node->declare_parameter("calibration.offsets.x", 0.0);
node->declare_parameter("calibration.offsets.y", 0.0);
node->declare_parameter("calibration.offsets.z", 0.0);

calib.enabled = node->get_parameter("calibration.enabled").as_bool();
calib.x = node->get_parameter("calibration.offsets.x").as_double();
calib.y = node->get_parameter("calibration.offsets.y").as_double();
calib.z = node->get_parameter("calibration.offsets.z").as_double();

// Apply calibration offsets if enabled
if (calib.enabled) {
    corrected_x = request->x - calib.x;
    corrected_y = request->y - calib.y;
    corrected_z = request->z - calib.z;
    // ... logging ...
}
```

#### 3. Updated Build System
**File:** `src/roarm_main/roarm_moveit_cmd/CMakeLists.txt`

- Config directory already installed (line 131)
- Added shell script installation (lines 139-141)

#### 4. Created Launch Helper Script
**File:** `src/roarm_main/roarm_moveit_cmd/scripts/run_movepointcmd_calibrated.sh`

Shell script to easily run movepointcmd with calibration parameters loaded.

### Usage Instructions

#### Option 1: Using the Helper Script (Recommended)
```bash
# Terminal 1: Start roarm_driver
sudo chmod 666 /dev/ttyUSB0
ros2 run roarm_driver roarm_driver

# Terminal 2: Launch command control
ros2 launch roarm_moveit_cmd command_control.launch.py

# Terminal 3: Start calibrated movepointcmd
ros2 run roarm_moveit_cmd run_movepointcmd_calibrated.sh
```

#### Option 2: Manual Parameter Loading
```bash
# Start movepointcmd with calibration config
ros2 run roarm_moveit_cmd movepointcmd --ros-args \
  --params-file $(ros2 pkg prefix roarm_moveit_cmd)/share/roarm_moveit_cmd/config/calibration_offsets.yaml
```

#### Option 3: Service Call with Running Node
```bash
# After starting movepointcmd normally, call the service
ros2 service call /move_point_cmd roarm_moveit/srv/MovePointCmd "{x: 0.2, y: -0.1, z: -0.1}"
```

### Testing Phase 1

To validate the offset compensation:

```bash
# 1. Run calibration at original test positions
ros2 run roarm_moveit_cmd calibrate_roarm \
  --target 0.2 -0.1 -0.1 \
  --home 0.2 0.0 0.1 \
  --iterations 10

# 2. Compare results to baseline (calibration_20251118_105156.json)
# Expected: ~60-70% reduction in euclidean error
# Before: 14.6mm average → After: ~5-7mm average
```

### Disable Calibration
To revert to uncalibrated behavior:

Edit `config/calibration_offsets.yaml`:
```yaml
calibration:
  enabled: false  # Change to false
```

Then rebuild: `. build_common.sh`

---

## Phase 2: Enhanced Data Collection

### Objective
Collect 3x3x3 grid calibration data across the workspace to enable geometric optimization.

### Tasks

#### 1. Enhance Calibration Script
**File:** `src/roarm_main/roarm_moveit_cmd/scripts/calibrate_roarm.py`

**Enhancements Needed:**
- Add `--grid` mode for automated grid testing
- Implement workspace bounds checking
- Add progress tracking with ETA
- Auto-save incremental results (in case of interruption)
- Generate error heatmaps and visualizations

**New Command-Line Arguments:**
```python
parser.add_argument('--grid', action='store_true',
                   help='Run grid-based calibration instead of single point')
parser.add_argument('--x-range', nargs=3, type=float,
                   default=[0.15, 0.20, 0.25],
                   help='X grid points (min, mid, max)')
parser.add_argument('--y-range', nargs=3, type=float,
                   default=[-0.10, 0.0, 0.10],
                   help='Y grid points (min, mid, max)')
parser.add_argument('--z-range', nargs=3, type=float,
                   default=[-0.10, 0.0, 0.10],
                   help='Z grid points (min, mid, max)')
parser.add_argument('--grid-iterations', type=int, default=5,
                   help='Iterations per grid point')
```

**Grid Pattern:** 3×3×3 = 27 test positions
- X: [0.15, 0.20, 0.25] meters
- Y: [-0.10, 0.0, +0.10] meters
- Z: [-0.10, 0.0, +0.10] meters

**Total Measurements:** 27 positions × 5 iterations = 135 measurements

#### 2. Run Grid Calibration
```bash
# Ensure system is ready
ros2 launch roarm_moveit_cmd command_control.launch.py

# Run grid calibration (estimated time: 1.5-2 hours)
ros2 run roarm_moveit_cmd calibrate_roarm --grid \
  --x-range 0.15 0.20 0.25 \
  --y-range -0.10 0.0 0.10 \
  --z-range -0.10 0.0 0.10 \
  --grid-iterations 5 \
  --output calibration_grid_$(date +%Y%m%d_%H%M%S)
```

#### 3. Data Analysis

**Create Analysis Script:** `scripts/analyze_calibration_grid.py`

**Analyses to perform:**
- Error magnitude vs. position (identify workspace regions with highest errors)
- Error direction patterns (systematic bias detection)
- Repeatability analysis per region
- Check for non-linear error patterns
- Generate 3D error heatmaps

**Outputs:**
- Error statistics per axis and position
- Visualization plots (matplotlib)
- Recommendations for optimization approach

---

## Phase 3: Geometric Calibration

### Objective
Update URDF link lengths and kinematic parameters based on optimization to achieve 80-90% error reduction.

### Background

Current kinematic parameters are defined in multiple locations:

1. **URDF Model:** `/src/roarm_main/roarm_description/urdf/roarm_description.urdf`
   - Joint origins (lines 88, 204, 281)
   - Link geometry transformations

2. **Custom IK Header:** `/src/roarm_main/roarm_moveit_cmd/include/roarm_moveit_cmd/ik.h`
   - Hardcoded link lengths (lines 4-18)
   - ARM_L1_LENGTH_MM = 126.06
   - ARM_L2_LENGTH_MM_A = 236.82
   - ARM_L3_LENGTH_MM_A_0 = 280.15

3. **IKFast Plugin:** `/src/roarm_main/roarm_moveit_ikfast_plugins/src/roarm_description_hand_ikfast_solver.cpp`
   - Auto-generated from URDF
   - Contains hardcoded transformations

**Identified Discrepancy:** Custom IK header values don't perfectly match URDF calculations.

### Tasks

#### 1. Create Link Length Optimization Script
**File:** `src/roarm_main/roarm_moveit_cmd/scripts/optimize_link_lengths.py`

**Optimization Algorithm:**
```python
import numpy as np
from scipy.optimize import minimize
import json

def forward_kinematics(joint_angles, L1, L2, L3, TCP_offset):
    """
    Compute end-effector position given joint angles and link lengths
    Returns: [x, y, z] position
    """
    # Implement FK using DH parameters
    # ...
    return position

def objective_function(link_params, calibration_data):
    """
    Minimize sum of squared errors between measured and FK positions

    link_params = [L1, L2, L3, TCP_x, TCP_y, TCP_z]
    """
    L1, L2, L3, tcp_x, tcp_y, tcp_z = link_params

    total_error = 0
    for measurement in calibration_data:
        commanded_pos = measurement['commanded']
        actual_pos = measurement['actual']
        joint_angles = measurement['joint_angles']  # Need to log these

        # Compute FK with current link estimates
        fk_pos = forward_kinematics(joint_angles, L1, L2, L3, [tcp_x, tcp_y, tcp_z])

        # Error between FK and actual measurement
        error = np.linalg.norm(np.array(fk_pos) - np.array(actual_pos))
        total_error += error**2

    return total_error

# Initial guess (current URDF values)
initial_params = [0.12606, 0.23682, 0.28015, 0.002, -0.2802, 0.0]

# Bounds (±10% of initial values)
bounds = [
    (0.120, 0.132),  # L1
    (0.225, 0.248),  # L2
    (0.266, 0.294),  # L3
    (-0.010, 0.010), # TCP_x
    (-0.290, -0.270),# TCP_y
    (-0.010, 0.010)  # TCP_z
]

# Run optimization
result = minimize(objective_function, initial_params,
                 args=(calibration_data,),
                 method='L-BFGS-B', bounds=bounds)

optimized_params = result.x
print(f"Optimized link lengths: L1={optimized_params[0]:.6f}, "
      f"L2={optimized_params[1]:.6f}, L3={optimized_params[2]:.6f}")
print(f"Optimized TCP offset: [{optimized_params[3]:.6f}, "
      f"{optimized_params[4]:.6f}, {optimized_params[5]:.6f}]")
```

**Requirements:**
- Calibration data must include joint angles (requires enhancement to calibrate_roarm.py)
- Need to subscribe to `/joint_states` topic during calibration

#### 2. Update URDF Files

**Files to Modify:**
- `/src/roarm_main/roarm_description/urdf/roarm_description.urdf`
- `/src/roarm_main/roarm_description/urdf/roarm_description_web_app.urdf`

**Changes Required:**

**Base Joint (Line ~88):**
```xml
<!-- Current -->
<origin xyz="0.0100000008759151 0 0.123059270461044" rpy="0 0 0"/>

<!-- Update with optimized L1 -->
<origin xyz="0.01 0 [OPTIMIZED_L1]" rpy="0 0 0"/>
```

**Link2 Joint (Line ~204):**
```xml
<!-- Current -->
<origin xyz="0.236815132922094 0.0300023995170449 0" rpy="0 0 0"/>

<!-- Update with optimized L2_A and L2_B -->
<origin xyz="[OPTIMIZED_L2_A] [OPTIMIZED_L2_B] 0" rpy="0 0 0"/>
```

**TCP Joint (Line ~281):**
```xml
<!-- Current -->
<origin xyz="0.002 -0.2802 0" rpy="-1.5708 0 -1.5708"/>

<!-- Update with optimized L3 and TCP offset -->
<origin xyz="[OPTIMIZED_TCP_X] [OPTIMIZED_TCP_Y] [OPTIMIZED_TCP_Z]" rpy="-1.5708 0 -1.5708"/>
```

#### 3. Update Custom IK Header

**File:** `/src/roarm_main/roarm_moveit_cmd/include/roarm_moveit_cmd/ik.h`

**Lines 4-18 Update:**
```cpp
// Update these constants with optimized values
#define ARM_L1_LENGTH_MM    [OPTIMIZED_L1 * 1000]
#define ARM_L2_LENGTH_MM_A  [OPTIMIZED_L2_A * 1000]
#define ARM_L2_LENGTH_MM_B  [OPTIMIZED_L2_B * 1000]
#define ARM_L3_LENGTH_MM_A_0  [OPTIMIZED_L3_A * 1000]
#define ARM_L3_LENGTH_MM_B_0  [OPTIMIZED_L3_B * 1000]
```

**Important:** After updating, disable Phase 1 constant offsets:
```yaml
# config/calibration_offsets.yaml
calibration:
  enabled: false  # Geometric calibration replaces constant offsets
```

#### 4. Regenerate IKFast Plugin

**Background:** The IKFast solver is auto-generated from the URDF using MoveIt's IKFast generator tool.

**Requirements:**
- MoveIt IKFast generator package
- OpenRAVE (IKFast backend)
- URDF must be updated first

**Process:**
```bash
# Install IKFast generator (if not already installed)
sudo apt install ros-humble-moveit-kinematics

# Generate new IKFast solver from updated URDF
# Note: This is a complex process - see MoveIt IKFast tutorial
# https://moveit.picknik.ai/humble/doc/examples/ikfast/ikfast_tutorial.html

rosrun moveit_kinematics create_ikfast_moveit_plugin.py \
  --robot roarm_description \
  --planning_group hand \
  --ikfast_output_path /tmp/ikfast_roarm.cpp
```

**Alternative (Simpler):**
If IKFast regeneration proves difficult, the custom IK solver in `ik.h` can be used exclusively. The existing system already uses both solvers, so updating the custom IK may be sufficient.

#### 5. Full Workspace Rebuild

```bash
cd ~/roarm_calibration

# Clean build (recommended after URDF changes)
rm -rf build/ install/ log/

# Rebuild everything
. build_common.sh

# Verify no build errors
echo $?  # Should output 0
```

#### 6. Validation Testing

```bash
# Test at original calibration points
ros2 run roarm_moveit_cmd calibrate_roarm \
  --target 0.2 -0.1 -0.1 \
  --home 0.2 0.0 0.1 \
  --iterations 20 \
  --output calibration_optimized_$(date +%Y%m%d_%H%M%S)

# Test across grid
ros2 run roarm_moveit_cmd calibrate_roarm --grid \
  --grid-iterations 3 \
  --output calibration_grid_optimized_$(date +%Y%m%d_%H%M%S)
```

**Expected Results:**
- Euclidean error: ~2-3mm (down from 14.6mm)
- 80-90% error reduction
- Errors should be evenly distributed (no systematic bias)

---

## Phase 4: Validation & Documentation

### Objective
Verify improvements, document the calibration procedure, and create maintenance guidelines.

### Tasks

#### 1. Comprehensive Validation Testing

**Test Suite:**

```bash
# 1. Original test positions (20 iterations)
ros2 run roarm_moveit_cmd calibrate_roarm \
  --target 0.2 -0.1 -0.1 \
  --home 0.2 0.0 0.1 \
  --iterations 20 \
  --output final_validation_original

# 2. Grid validation (3×3×3, 3 iterations each)
ros2 run roarm_moveit_cmd calibrate_roarm --grid \
  --grid-iterations 3 \
  --output final_validation_grid

# 3. Extreme workspace corners
ros2 run roarm_moveit_cmd calibrate_roarm \
  --target 0.25 -0.12 -0.12 \
  --iterations 10 \
  --output final_validation_corner1

ros2 run roarm_moveit_cmd calibrate_roarm \
  --target 0.15 0.12 0.08 \
  --iterations 10 \
  --output final_validation_corner2
```

#### 2. Generate Improvement Report

**Create Script:** `scripts/generate_improvement_report.py`

**Report Contents:**
- Before/after error statistics comparison
- Percentage improvement per axis
- Error distribution histograms
- Workspace error heatmap (3D visualization)
- Repeatability analysis
- Success rate statistics

**Example Output:**
```
=== RoArm-M2-S Calibration Improvement Report ===

Baseline (2025-11-18):
  Mean Euclidean Error: 14.62 mm
  X Error: -9.97 ± 0.32 mm
  Y Error: -0.05 ± 0.32 mm
  Z Error: +10.68 ± 0.23 mm

Phase 1 (Constant Offsets):
  Mean Euclidean Error: 5.23 mm
  Improvement: 64.2%

Phase 3 (Geometric Calibration):
  Mean Euclidean Error: 2.41 mm
  Improvement: 83.5%

  X Error: -0.12 ± 0.28 mm
  Y Error: +0.03 ± 0.31 mm
  Z Error: +0.08 ± 0.25 mm

Repeatability: EXCELLENT (<0.35mm σ maintained)
Success Rate: 100% (all moves completed)
```

#### 3. Update Documentation

**File:** `/home/ruhalis/roarm_calibration/CLAUDE.md`

Add new section:

```markdown
### Calibration

The RoArm-M2-S has been calibrated to improve positioning accuracy from ~15mm to ~2-3mm euclidean error.

**Calibration Status:**
- Baseline error: 14.6mm (2025-11-18)
- Current error: ~2.4mm (geometric calibration applied)
- Improvement: 83.5%

**Calibration Files:**
- Link parameters: `roarm_description/urdf/roarm_description.urdf`
- Optimized lengths: See git commit [COMMIT_HASH]
- Calibration data: `calibration_*.json` files in workspace root

**Re-calibration Procedure:**
If the robot is physically modified or accuracy degrades:

1. Collect baseline data:
   ```bash
   ros2 run roarm_moveit_cmd calibrate_roarm --grid --output recalibration_baseline
   ```

2. Run optimization:
   ```bash
   python3 scripts/optimize_link_lengths.py recalibration_baseline_grid.json
   ```

3. Update URDF and rebuild (see CALIBRATION_PLAN.md Phase 3)

**Maintenance:**
- Re-run single-point calibration monthly to monitor drift
- Full re-calibration recommended after any mechanical adjustments
- Keep calibration data files for historical tracking
```

#### 4. Create Maintenance Checklist

**File:** `CALIBRATION_MAINTENANCE.md`

```markdown
# Calibration Maintenance Checklist

## Monthly Accuracy Check
- [ ] Run baseline calibration test
- [ ] Compare to historical data
- [ ] Document any drift > 1mm
- [ ] Check mechanical connections for looseness

## After Mechanical Changes
- [ ] Re-tighten all joint screws
- [ ] Run full grid calibration
- [ ] Re-optimize link parameters if error > 5mm
- [ ] Update URDF and rebuild

## Calibration Data Archive
Keep all calibration files with naming convention:
`calibration_[type]_YYYYMMDD_HHMMSS.[json|csv]`

Types: baseline, grid, optimized, validation
```

#### 5. Commit Calibration Configuration

```bash
cd ~/roarm_calibration

# Add calibration files
git add src/roarm_main/roarm_moveit_cmd/config/calibration_offsets.yaml
git add src/roarm_main/roarm_moveit_cmd/scripts/run_movepointcmd_calibrated.sh
git add src/roarm_main/roarm_moveit_cmd/scripts/optimize_link_lengths.py
git add src/roarm_main/roarm_moveit_cmd/scripts/analyze_calibration_grid.py

# Commit Phase 1 changes
git commit -m "Add Phase 1 calibration: constant offset compensation

- Add calibration_offsets.yaml with measured offsets from 2025-11-18
- Modify movepointcmd.cpp to load and apply calibration parameters
- Add helper script to run with calibration enabled
- Expected improvement: 60-70% error reduction

Baseline: 14.6mm euclidean error
Target: ~5-7mm with constant offsets"

# Later: Commit Phase 3 changes
git add src/roarm_main/roarm_description/urdf/roarm_description.urdf
git add src/roarm_main/roarm_moveit_cmd/include/roarm_moveit_cmd/ik.h
git commit -m "Phase 3 calibration: optimized geometric parameters

- Update URDF link lengths based on grid calibration optimization
- Update custom IK solver constants
- Final accuracy: ~2.4mm euclidean error (83.5% improvement)

Optimized parameters:
  L1: [value] mm
  L2: [value] mm
  L3: [value] mm
  TCP offset: [x, y, z]"
```

#### 6. Backup Configuration

```bash
# Create calibration archive
cd ~/roarm_calibration
tar -czf calibration_archive_$(date +%Y%m%d).tar.gz \
  calibration_*.json \
  calibration_*.csv \
  src/roarm_main/roarm_description/urdf/ \
  src/roarm_main/roarm_moveit_cmd/config/calibration_offsets.yaml \
  CALIBRATION_PLAN.md \
  CALIBRATION_MAINTENANCE.md

# Move to safe location
mv calibration_archive_*.tar.gz ~/backups/
```

---

## Implementation Timeline

### Completed
- ✅ **Phase 1 Setup** - Calibration infrastructure created and built
  - Calibration config file
  - Modified movepointcmd.cpp
  - Build system updated
  - Helper scripts created

### Pending

**Next Session (1-2 hours):**
- Test Phase 1 offset compensation
- Validate 60-70% error reduction
- Begin Phase 2: enhance calibrate_roarm.py for grid mode

**Following Session (2-3 hours):**
- Run 3×3×3 grid calibration
- Analyze data patterns
- Implement optimization script

**Final Session (3-4 hours):**
- Run link length optimization
- Update URDF and IK parameters
- Rebuild workspace
- Run validation tests
- Generate improvement report
- Update documentation

**Total Estimated Time:** 6-10 hours across multiple sessions

---

## Technical Reference

### Key Files Modified

**Phase 1:**
1. `src/roarm_main/roarm_moveit_cmd/config/calibration_offsets.yaml` (NEW)
2. `src/roarm_main/roarm_moveit_cmd/src/movepointcmd.cpp` (MODIFIED)
3. `src/roarm_main/roarm_moveit_cmd/CMakeLists.txt` (MODIFIED)
4. `src/roarm_main/roarm_moveit_cmd/scripts/run_movepointcmd_calibrated.sh` (NEW)
5. `src/roarm_main/roarm_moveit_cmd/launch/command_control_calibrated.launch.py` (NEW)

**Phase 2 (Pending):**
6. `src/roarm_main/roarm_moveit_cmd/scripts/calibrate_roarm.py` (TO MODIFY)
7. `src/roarm_main/roarm_moveit_cmd/scripts/analyze_calibration_grid.py` (TO CREATE)

**Phase 3 (Pending):**
8. `src/roarm_main/roarm_description/urdf/roarm_description.urdf` (TO MODIFY)
9. `src/roarm_main/roarm_description/urdf/roarm_description_web_app.urdf` (TO MODIFY)
10. `src/roarm_main/roarm_moveit_cmd/include/roarm_moveit_cmd/ik.h` (TO MODIFY)
11. `src/roarm_main/roarm_moveit_cmd/scripts/optimize_link_lengths.py` (TO CREATE)

### Calibration Data Format

**JSON Structure:**
```json
{
  "timestamp": "2025-11-18T10:49:51.445747",
  "target": {"x": 0.2, "y": -0.1, "z": -0.1},
  "home": {"x": 0.2, "y": 0.0, "z": 0.1},
  "iterations": 20,
  "measurements": [
    {
      "iteration": 1,
      "joint_angles": [0.0, -1.57, 0.785],  // To be added in Phase 2
      "target_measurement": {
        "actual": {"x": 0.209, "y": -0.099, "z": -0.111},
        "errors": {"error_x": -0.009, "error_y": -0.001, "error_z": 0.011}
      }
    }
  ],
  "statistics": { ... }
}
```

### Parameter Hierarchy

**Calibration Parameter Priority:**
1. **Geometric Parameters (Phase 3)** - URDF link lengths (most fundamental)
2. **IK Solver Constants (Phase 3)** - ik.h definitions (must match URDF)
3. **Constant Offsets (Phase 1)** - calibration_offsets.yaml (temporary fix)

**Note:** Once Phase 3 is complete, Phase 1 constant offsets should be disabled as they become redundant with correct geometric parameters.

### Troubleshooting

**If accuracy doesn't improve after Phase 1:**
- Verify calibration parameters are being loaded (check logs)
- Ensure `calibration.enabled: true` in config
- Check parameter names match exactly
- Verify workspace was rebuilt after code changes

**If optimization fails in Phase 3:**
- Check calibration data includes joint angles
- Verify grid data covers sufficient workspace
- Try different optimization algorithms (Nelder-Mead, Powell)
- Check bounds are reasonable (±10% of initial values)

**If build fails:**
- Check C++ syntax in movepointcmd.cpp
- Verify all includes are present
- Clean build: `rm -rf build/ install/ log/`
- Check for missing dependencies

---

## References

**Calibration Data:**
- Baseline: `calibration_20251118_105156.json` (14.6mm error)
- Source measurements: 20 iterations, 2 positions

**Related Documentation:**
- Main documentation: `/home/ruhalis/roarm_calibration/CLAUDE.md`
- Startup instructions: `/home/ruhalis/roarm_calibration/START.md`

**ROS2 Packages:**
- roarm_moveit_cmd: Motion control commands
- roarm_description: Robot URDF models
- roarm_moveit: MoveIt2 configuration

**External Resources:**
- MoveIt2 IKFast Tutorial: https://moveit.picknik.ai/humble/doc/examples/ikfast/ikfast_tutorial.html
- ROS2 Parameters Guide: https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html

---

## Appendix A: Calibration Theory

### Why Systematic Errors Occur

1. **Manufacturing Tolerances:** Link lengths may differ from design by ±1-2mm
2. **Assembly Errors:** Joint zero positions may be offset during assembly
3. **Model Simplification:** URDF assumes rigid links, ignores flex/backlash
4. **Sensor Offsets:** End-effector position sensor may have mounting offset

### Why Constant Offsets Work (Phase 1)

With excellent repeatability (<0.35mm σ), errors are primarily **systematic** (constant bias) rather than random. A simple translation correction addresses the bulk of the error.

### Why Geometric Calibration Is Better (Phase 3)

Geometric calibration corrects the **root cause** (incorrect link parameters) rather than compensating for symptoms. This provides:
- Position-independent accuracy
- Correct behavior across entire workspace
- Valid forward/inverse kinematics
- Proper collision checking and planning

### Optimization Mathematics

**Objective:** Minimize positioning error across all calibration points

**Cost Function:**
```
J(L) = Σᵢ ||FK(qᵢ, L) - pᵢ||²

Where:
  L = [L1, L2, L3, TCP_x, TCP_y, TCP_z]  // Link parameters to optimize
  qᵢ = joint angles for measurement i
  pᵢ = actual measured position i
  FK = forward kinematics function
```

**Constraints:**
- Link lengths must be physically reasonable (±10% of design)
- TCP offset bounded to prevent unrealistic solutions
- Joint angle limits respected

---

## Appendix B: Quick Reference Commands

### Build & Source
```bash
cd ~/roarm_calibration
. build_common.sh
```

### Start System (Uncalibrated)
```bash
# Terminal 1: Driver
sudo chmod 666 /dev/ttyUSB0
ros2 run roarm_driver roarm_driver

# Terminal 2: Control system
ros2 launch roarm_moveit_cmd command_control.launch.py

# Terminal 3: Move command service
ros2 run roarm_moveit_cmd movepointcmd
```

### Start System (Phase 1 Calibrated)
```bash
# Terminals 1-2: Same as above

# Terminal 3: Calibrated move command
ros2 run roarm_moveit_cmd run_movepointcmd_calibrated.sh
```

### Run Calibration Tests
```bash
# Single point (20 iterations)
ros2 run roarm_moveit_cmd calibrate_roarm \
  --target 0.2 -0.1 -0.1 \
  --home 0.2 0.0 0.1 \
  --iterations 20

# Grid test (Phase 2+)
ros2 run roarm_moveit_cmd calibrate_roarm --grid \
  --grid-iterations 5
```

### Check Calibration Status
```bash
# View current offsets
cat $(ros2 pkg prefix roarm_moveit_cmd)/share/roarm_moveit_cmd/config/calibration_offsets.yaml

# View URDF link lengths
grep -A2 "joint name=\"link._to_link.\"" \
  $(ros2 pkg prefix roarm_description)/share/roarm_description/urdf/roarm_description.urdf
```

---

**Document Version:** 1.0
**Created:** 2025-11-18
**Status:** Phase 1 Complete, Phases 2-4 Pending
**Expected Completion:** Phase 4 validation and documentation
