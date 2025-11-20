"""
Calibration Measurement Module

Handles measurement logic for robot calibration including error calculation
and position validation.
"""

import time
import math
from typing import Dict, Tuple, Optional
from rclpy.node import Node
from .robot_interface import RobotInterface


class CalibrationMeasurement:
    """
    Handles calibration measurements and error calculations.
    """

    def __init__(self, robot: RobotInterface, logger):
        """
        Initialize calibration measurement.

        Args:
            robot: RobotInterface instance for robot communication
            logger: Logger instance for output
        """
        self.robot = robot
        self.logger = logger

    def validate_position(self, x: float, y: float, z: float,
                         limits: Optional[Dict] = None) -> Tuple[bool, str]:
        """
        Validate that a position is within workspace limits.

        Args:
            x, y, z: Position to validate
            limits: Optional workspace limits dictionary with keys:
                   x_min, x_max, y_min, y_max, z_min, z_max

        Returns:
            Tuple of (is_valid, error_message)
        """
        if limits is None:
            # Default safety limits
            limits = {
                'x_min': 0.10, 'x_max': 0.30,
                'y_min': -0.15, 'y_max': 0.15,
                'z_min': -0.15, 'z_max': 0.15
            }

        if not (limits['x_min'] <= x <= limits['x_max']):
            return False, f"X={x:.4f} outside range [{limits['x_min']}, {limits['x_max']}]"
        if not (limits['y_min'] <= y <= limits['y_max']):
            return False, f"Y={y:.4f} outside range [{limits['y_min']}, {limits['y_max']}]"
        if not (limits['z_min'] <= z <= limits['z_max']):
            return False, f"Z={z:.4f} outside range [{limits['z_min']}, {limits['z_max']}]"

        return True, "Position valid"

    def calculate_error(self, target: Tuple[float, float, float],
                       actual: Tuple[float, float, float]) -> Dict[str, float]:
        """
        Calculate positioning errors.

        Args:
            target: Target (x, y, z) position
            actual: Actual (x, y, z) position

        Returns:
            Dictionary with error metrics:
                - error_x: X-axis error
                - error_y: Y-axis error
                - error_z: Z-axis error
                - euclidean_error: 3D Euclidean distance error
        """
        error_x = target[0] - actual[0]
        error_y = target[1] - actual[1]
        error_z = target[2] - actual[2]

        euclidean_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)

        return {
            'error_x': error_x,
            'error_y': error_y,
            'error_z': error_z,
            'euclidean_error': euclidean_error
        }

    def measure_position(self, target_x: float, target_y: float, target_z: float,
                        settling_time: float, timeout: float = 10.0) -> Dict:
        """
        Move to a position and measure the actual position achieved.

        Args:
            target_x, target_y, target_z: Target position
            settling_time: Time to wait after movement before measuring
            timeout: Movement timeout

        Returns:
            Dictionary with measurement results:
                - move_success: bool
                - pose_success: bool
                - actual: {x, y, z} if successful
                - errors: {error_x, error_y, error_z, euclidean_error} if successful
                - error: error message if failed
        """
        measurement = {}

        # Move to target
        move_success, move_msg = self.robot.move_to_position(
            target_x, target_y, target_z, timeout
        )

        if not move_success:
            self.logger.error(f'Movement failed: {move_msg}')
            measurement['move_success'] = False
            measurement['error'] = move_msg
            return measurement

        measurement['move_success'] = True

        # Wait for settling
        time.sleep(settling_time)

        # Measure actual position
        pose_success, actual_x, actual_y, actual_z = self.robot.get_current_pose()

        if not pose_success:
            self.logger.error('Failed to get current pose')
            measurement['pose_success'] = False
            return measurement

        measurement['pose_success'] = True
        measurement['actual'] = {'x': actual_x, 'y': actual_y, 'z': actual_z}

        # Calculate errors
        errors = self.calculate_error(
            (target_x, target_y, target_z),
            (actual_x, actual_y, actual_z)
        )
        measurement['errors'] = errors

        return measurement

    def run_measurement_cycle(self, target: Dict, home: Dict,
                             settling_time: float, timeout: float = 10.0) -> Dict:
        """
        Run a complete measurement cycle: move to target, measure, return to home, measure.

        Args:
            target: Target position dict with 'x', 'y', 'z' keys
            home: Home position dict with 'x', 'y', 'z' keys
            settling_time: Settling time in seconds
            timeout: Movement timeout

        Returns:
            Dictionary with target_measurement and home_measurement results
        """
        result = {}

        # Measure at target
        self.logger.info(f'>> Moving to TARGET: {target.get("name", "unnamed")}')
        self.logger.info(f'   Position: ({target["x"]:.4f}, {target["y"]:.4f}, {target["z"]:.4f})')

        target_measurement = self.measure_position(
            target['x'], target['y'], target['z'],
            settling_time, timeout
        )

        if target_measurement.get('pose_success', False):
            actual = target_measurement['actual']
            errors = target_measurement['errors']
            self.logger.info(f'   Actual: ({actual["x"]:.4f}, {actual["y"]:.4f}, {actual["z"]:.4f})')
            self.logger.info(f'   Errors: X={errors["error_x"]:+.6f}, '
                           f'Y={errors["error_y"]:+.6f}, '
                           f'Z={errors["error_z"]:+.6f}, '
                           f'Euclidean={errors["euclidean_error"]:.6f}')

        result['target_measurement'] = target_measurement

        # Return to home
        self.logger.info('\n>> Returning to HOME')
        home_measurement = self.measure_position(
            home['x'], home['y'], home['z'],
            settling_time, timeout
        )

        if home_measurement.get('pose_success', False):
            actual = home_measurement['actual']
            errors = home_measurement['errors']
            self.logger.info(f'   Home actual: ({actual["x"]:.4f}, {actual["y"]:.4f}, {actual["z"]:.4f})')
            self.logger.info(f'   Home errors: X={errors["error_x"]:+.6f}, '
                           f'Y={errors["error_y"]:+.6f}, '
                           f'Z={errors["error_z"]:+.6f}')

        result['home_measurement'] = home_measurement

        return result
