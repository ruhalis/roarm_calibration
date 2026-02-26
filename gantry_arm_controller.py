#!/usr/bin/env python3
"""
Gantry + RoArm Coordinated Controller

Controls a 2-axis XY gantry (Arduino stepper motors) and a RoArm manipulator
to reach any target position in the combined workspace.

Strategy:
  1. Given target (x, y), compute how much the gantry must move so the arm's
     base is within ARM_REACH_RADIUS of the target.
  2. Move gantry to that position (relative mm commands over serial).
  3. Compute the local offset from the arm's base to the target.
  4. Command the RoArm to move its end-effector to that local offset.

The gantry Arduino expects: "dx_mm, dy_mm\n" (relative moves).
The RoArm is controlled via ROS2 MovePointCmd service (x, y, z in meters).
"""

import math
import time
import json
import serial
import argparse
import subprocess
import sys

# ============================================================
#  Arm geometry (from ik.h)
# ============================================================
L2A = 236.82
L2B = 30.0
L3A = 280.15
L3B = 1.73
L2 = math.sqrt(L2A**2 + L2B**2)
L3 = math.sqrt(L3A**2 + L3B**2)
ARM_MAX_REACH_MM = L2 + L3  # ~519mm theoretical max

ARM_REACH_RADIUS_MM = 200.0  # default comfortable reach radius in mm
ARM_DEFAULT_Z_M = 0.05       # default end-effector height in meters


class GantryController:
    """
    Talks to the Arduino running motor_control.c++ over serial.
    Tracks absolute position in software (starts at 0,0 after homing).
    Sends relative move commands in mm.
    """

    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # wait for Arduino reset
        self._drain_startup()
        self.pos_x = 0.0  # mm, current tracked position
        self.pos_y = 0.0

    def _drain_startup(self):
        """Read and discard the Arduino startup banner."""
        while self.ser.in_waiting:
            self.ser.readline()

    def move_relative(self, dx_mm: float, dy_mm: float, wait: bool = True):
        """
        Send a relative move command to the Arduino.
        dx_mm: motor 1 distance in mm (X axis)
        dy_mm: motor 2 distance in mm (Y axis)
        """
        cmd = f"{dx_mm:.2f}, {dy_mm:.2f}\n"
        self.ser.write(cmd.encode('utf-8'))
        self.pos_x += dx_mm
        self.pos_y += dy_mm

        if wait:
            self._wait_for_move(dx_mm, dy_mm)

        return self.pos_x, self.pos_y

    def move_absolute(self, target_x_mm: float, target_y_mm: float, wait: bool = True):
        """Move to an absolute position by computing the relative delta."""
        dx = target_x_mm - self.pos_x
        dy = target_y_mm - self.pos_y
        return self.move_relative(dx, dy, wait=wait)

    def _wait_for_move(self, dx_mm: float, dy_mm: float):
        """
        Estimate move time from distance and stepper parameters.
        motor_control.c++ uses maxSpeed=4000 steps/s, accel=2000 steps/s^2,
        3200 steps/rev, 125 mm/rev => 25.6 steps/mm
        Max speed: 4000/25.6 = 156.25 mm/s
        """
        steps_per_mm = 3200.0 / 125.0
        max_speed_mm_s = 4000.0 / steps_per_mm
        accel_mm_s2 = 2000.0 / steps_per_mm

        dist = max(abs(dx_mm), abs(dy_mm))
        if dist < 0.01:
            return

        # trapezoidal profile estimate
        t_accel = max_speed_mm_s / accel_mm_s2
        d_accel = 0.5 * accel_mm_s2 * t_accel**2

        if dist < 2 * d_accel:
            # triangular profile
            t_total = 2.0 * math.sqrt(dist / accel_mm_s2)
        else:
            d_cruise = dist - 2 * d_accel
            t_cruise = d_cruise / max_speed_mm_s
            t_total = 2 * t_accel + t_cruise

        time.sleep(t_total + 0.3)  # small buffer

    def home(self):
        """Reset tracked position to 0,0 (assumes physical homing was done)."""
        self.pos_x = 0.0
        self.pos_y = 0.0

    def get_position(self):
        return self.pos_x, self.pos_y

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


class RoArmController:
    """
    Controls the RoArm via ROS2 CLI.
    Sends MovePointCmd service calls for end-effector positioning.
    Sends JSON commands directly over serial for lower-level control.
    """

    def __init__(self, roarm_port: str = None, baud: int = 115200):
        self.roarm_port = roarm_port
        self.roarm_serial = None
        if roarm_port:
            self.roarm_serial = serial.Serial(roarm_port, baud, timeout=1)
            time.sleep(1)

    def move_to_xyz(self, x_m: float, y_m: float, z_m: float) -> bool:
        """
        Move end-effector to local coordinates (meters) using direct JSON serial.
        Uses the arm's native T:104 command for Cartesian positioning.
        x, y, z are in meters; the arm protocol uses mm internally.
        """
        if self.roarm_serial:
            return self._move_serial(x_m, y_m, z_m)
        else:
            return self._move_ros2(x_m, y_m, z_m)

    def _move_serial(self, x_m: float, y_m: float, z_m: float) -> bool:
        """Send Cartesian move via direct serial JSON command."""
        x_mm = x_m * 1000.0
        y_mm = y_m * 1000.0
        z_mm = z_m * 1000.0

        # T:104 is the Cartesian coordinate move command
        cmd = {"T": 104, "x": x_mm, "y": y_mm, "z": z_mm, "spd": 0, "acc": 10}
        cmd_str = json.dumps(cmd) + "\n"
        self.roarm_serial.write(cmd_str.encode('utf-8'))
        time.sleep(0.5)

        # read back response
        while self.roarm_serial.in_waiting:
            line = self.roarm_serial.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"  [RoArm] {line}")
        return True

    def _move_ros2(self, x_m: float, y_m: float, z_m: float) -> bool:
        """Move via ROS2 service call (requires ROS2 stack running)."""
        try:
            cmd = [
                "ros2", "service", "call",
                "/move_point_cmd",
                "roarm_moveit/srv/MovePointCmd",
                f"{{x: {x_m}, y: {y_m}, z: {z_m}}}"
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            print(f"  [ROS2] {result.stdout.strip()}")
            return "success=True" in result.stdout or "success: true" in result.stdout.lower()
        except Exception as e:
            print(f"  [ROS2] Error: {e}")
            return False

    def close(self):
        if self.roarm_serial and self.roarm_serial.is_open:
            self.roarm_serial.close()


def compute_gantry_target(target_x_mm: float, target_y_mm: float,
                          reach_radius_mm: float) -> tuple:
    """
    Given a target point and the arm's reach radius, compute where the
    gantry should position the arm's base.

    Strategy: move the gantry so the arm base is exactly (target - offset)
    where offset is along the direction from origin to target, with magnitude
    such that the remaining distance equals the reach radius.

    If the target is already within reach_radius from the current gantry
    position concept (i.e., the total distance is <= reach_radius), the
    gantry stays at 0,0 and the arm does all the work.
    """
    dist = math.sqrt(target_x_mm**2 + target_y_mm**2)

    if dist <= reach_radius_mm:
        # target is within arm reach from origin, no gantry move needed
        gantry_x = 0.0
        gantry_y = 0.0
    else:
        # move gantry so the arm only needs to reach 'reach_radius_mm'
        # gantry moves along the line from origin to target
        overshoot = dist - reach_radius_mm
        angle = math.atan2(target_y_mm, target_x_mm)
        gantry_x = overshoot * math.cos(angle)
        gantry_y = overshoot * math.sin(angle)

    arm_local_x = target_x_mm - gantry_x
    arm_local_y = target_y_mm - gantry_y

    return gantry_x, gantry_y, arm_local_x, arm_local_y


def goto_position(gantry: GantryController, arm: RoArmController,
                  target_x_mm: float, target_y_mm: float,
                  arm_z_m: float = ARM_DEFAULT_Z_M,
                  reach_radius_mm: float = ARM_REACH_RADIUS_MM):
    """
    Coordinated move: gantry positions the arm base, then arm reaches the target.
    """
    print(f"\n{'='*60}")
    print(f"TARGET: ({target_x_mm:.1f}, {target_y_mm:.1f}) mm")
    print(f"{'='*60}")

    gantry_x, gantry_y, arm_local_x, arm_local_y = compute_gantry_target(
        target_x_mm, target_y_mm, reach_radius_mm
    )

    print(f"  Gantry move to: ({gantry_x:.1f}, {gantry_y:.1f}) mm")
    print(f"  Arm local target: ({arm_local_x:.1f}, {arm_local_y:.1f}) mm")

    # Step 1: move gantry
    cur_gx, cur_gy = gantry.get_position()
    dx = gantry_x - cur_gx
    dy = gantry_y - cur_gy

    if abs(dx) > 0.1 or abs(dy) > 0.1:
        print(f"  Moving gantry by ({dx:.1f}, {dy:.1f}) mm ...")
        gantry.move_relative(dx, dy, wait=True)
        print(f"  Gantry at: ({gantry.pos_x:.1f}, {gantry.pos_y:.1f}) mm")
    else:
        print(f"  Gantry already in position")

    # Step 2: move arm end-effector to the local offset
    arm_x_m = arm_local_x / 1000.0
    arm_y_m = arm_local_y / 1000.0

    print(f"  Moving arm to local ({arm_x_m:.4f}, {arm_y_m:.4f}, {arm_z_m:.4f}) m ...")
    success = arm.move_to_xyz(arm_x_m, arm_y_m, arm_z_m)

    if success:
        print(f"  DONE - end-effector at global ({target_x_mm:.1f}, {target_y_mm:.1f}) mm")
    else:
        print(f"  WARNING - arm move may have failed")

    return success


def go_home(gantry: GantryController, arm: RoArmController,
            arm_z_m: float = ARM_DEFAULT_Z_M):
    """Return both gantry and arm to home (0, 0)."""
    print(f"\n{'='*60}")
    print("HOMING: returning to (0, 0)")
    print(f"{'='*60}")

    # First retract arm to safe position above base
    print("  Retracting arm to home...")
    arm.move_to_xyz(0.15, 0.0, arm_z_m)
    time.sleep(1)

    # Then move gantry to 0,0
    gx, gy = gantry.get_position()
    if abs(gx) > 0.1 or abs(gy) > 0.1:
        print(f"  Moving gantry from ({gx:.1f}, {gy:.1f}) to (0, 0) ...")
        gantry.move_absolute(0.0, 0.0, wait=True)
        print(f"  Gantry at home")
    else:
        print(f"  Gantry already at home")


def interactive_mode(gantry: GantryController, arm: RoArmController,
                     reach_radius_mm: float, arm_z_m: float):
    """Interactive command loop."""
    print(f"\n{'='*60}")
    print("INTERACTIVE GANTRY + ARM CONTROLLER")
    print(f"{'='*60}")
    print(f"  Arm reach radius: {reach_radius_mm:.0f} mm")
    print(f"  Arm Z height:     {arm_z_m:.3f} m")
    print(f"  Gantry position:  (0, 0) mm  [homed]")
    print()
    print("Commands:")
    print("  x, y          - go to position (mm), e.g.: 200, 200")
    print("  gantry x, y   - move gantry only (mm)")
    print("  arm x, y      - move arm only (mm, local coords)")
    print("  home           - return to (0, 0)")
    print("  pos            - show current positions")
    print("  radius N       - set arm reach radius (mm)")
    print("  z N            - set arm Z height (m)")
    print("  quit / exit    - exit")
    print()

    while True:
        try:
            raw = input(">> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not raw:
            continue

        parts = raw.lower().split()

        if parts[0] in ("quit", "exit", "q"):
            break

        elif parts[0] == "home":
            go_home(gantry, arm, arm_z_m)

        elif parts[0] == "pos":
            gx, gy = gantry.get_position()
            print(f"  Gantry: ({gx:.1f}, {gy:.1f}) mm")
            print(f"  Reach radius: {reach_radius_mm:.0f} mm")
            print(f"  Arm Z: {arm_z_m:.3f} m")

        elif parts[0] == "radius" and len(parts) >= 2:
            try:
                reach_radius_mm = float(parts[1])
                print(f"  Reach radius set to {reach_radius_mm:.0f} mm")
            except ValueError:
                print("  Invalid number")

        elif parts[0] == "z" and len(parts) >= 2:
            try:
                arm_z_m = float(parts[1])
                print(f"  Arm Z set to {arm_z_m:.3f} m")
            except ValueError:
                print("  Invalid number")

        elif parts[0] == "gantry":
            try:
                coords = raw[len("gantry"):].replace(",", " ").split()
                gx_mm = float(coords[0])
                gy_mm = float(coords[1])
                print(f"  Moving gantry to ({gx_mm:.1f}, {gy_mm:.1f}) mm ...")
                gantry.move_absolute(gx_mm, gy_mm, wait=True)
                print(f"  Gantry at: ({gantry.pos_x:.1f}, {gantry.pos_y:.1f}) mm")
            except (ValueError, IndexError):
                print("  Usage: gantry x, y")

        elif parts[0] == "arm":
            try:
                coords = raw[len("arm"):].replace(",", " ").split()
                ax_mm = float(coords[0])
                ay_mm = float(coords[1])
                ax_m = ax_mm / 1000.0
                ay_m = ay_mm / 1000.0
                print(f"  Moving arm to local ({ax_m:.4f}, {ay_m:.4f}, {arm_z_m:.4f}) m ...")
                arm.move_to_xyz(ax_m, ay_m, arm_z_m)
            except (ValueError, IndexError):
                print("  Usage: arm x, y  (mm, local to arm base)")

        else:
            # Default: parse as "x, y" target coordinates
            try:
                coords = raw.replace(",", " ").split()
                tx_mm = float(coords[0])
                ty_mm = float(coords[1])
                goto_position(gantry, arm, tx_mm, ty_mm, arm_z_m, reach_radius_mm)
            except (ValueError, IndexError):
                print("  Unknown command. Enter 'x, y' coordinates or type 'help'.")


def main():
    parser = argparse.ArgumentParser(
        description="Coordinated gantry + RoArm controller"
    )
    parser.add_argument(
        "--gantry-port", type=str, required=True,
        help="Serial port for the Arduino gantry (e.g., /dev/ttyUSB1 or COM3)"
    )
    parser.add_argument(
        "--roarm-port", type=str, default=None,
        help="Serial port for RoArm direct control (e.g., /dev/ttyUSB0). "
             "If omitted, uses ROS2 service calls instead."
    )
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Baud rate for serial connections (default: 115200)"
    )
    parser.add_argument(
        "--reach-radius", type=float, default=ARM_REACH_RADIUS_MM,
        help=f"Arm reach radius in mm (default: {ARM_REACH_RADIUS_MM})"
    )
    parser.add_argument(
        "--arm-z", type=float, default=ARM_DEFAULT_Z_M,
        help=f"Default arm Z height in meters (default: {ARM_DEFAULT_Z_M})"
    )
    parser.add_argument(
        "--goto", nargs=2, type=float, metavar=("X", "Y"),
        help="Go to position (mm) then exit. E.g.: --goto 200 200"
    )

    args = parser.parse_args()

    print("Connecting to gantry Arduino...")
    gantry = GantryController(args.gantry_port, args.baud)
    print(f"  Connected on {args.gantry_port}")

    arm = RoArmController(roarm_port=args.roarm_port, baud=args.baud)
    if args.roarm_port:
        print(f"  RoArm direct serial on {args.roarm_port}")
    else:
        print(f"  RoArm via ROS2 service calls")

    try:
        if args.goto:
            goto_position(
                gantry, arm,
                args.goto[0], args.goto[1],
                args.arm_z, args.reach_radius
            )
        else:
            interactive_mode(gantry, arm, args.reach_radius, args.arm_z)
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        print("Closing connections...")
        gantry.close()
        arm.close()
        print("Done.")


if __name__ == "__main__":
    main()
