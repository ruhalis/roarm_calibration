#!/usr/bin/env python3

"""
RoArm Calibration Script
Moves the robot arm to a target position, measures actual position,
calculates errors, and tests repeatability.
"""

import rclpy
from rclpy.node import Node
from roarm_moveit.srv import MovePointCmd, GetPoseCmd
import argparse
import time
import json
import csv
import math
from datetime import datetime
from typing import List, Dict, Tuple
import sys


class RoArmCalibrator(Node):
    """Node for calibrating RoArm positioning accuracy and repeatability."""

    def __init__(self):
        super().__init__('roarm_calibrator')

        # Create service clients
        self.move_client = self.create_client(MovePointCmd, '/move_point_cmd')
        self.get_pose_client = self.create_client(GetPoseCmd, '/get_pose_cmd')

        # Wait for services to be available
        self.get_logger().info('Waiting for /move_point_cmd service...')
        if not self.move_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service /move_point_cmd not available!')
            sys.exit(1)

        self.get_logger().info('Waiting for /get_pose_cmd service...')
        if not self.get_pose_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service /get_pose_cmd not available!')
            sys.exit(1)

        self.get_logger().info('All services available. Ready to calibrate.')

    def move_to_position(self, x: float, y: float, z: float) -> Tuple[bool, str]:
        """
        Move robot to specified position.

        Args:
            x, y, z: Target coordinates

        Returns:
            Tuple of (success, message)
        """
        request = MovePointCmd.Request()
        request.x = x
        request.y = y
        request.z = z

        self.get_logger().info(f'Moving to position: x={x:.4f}, y={y:.4f}, z={z:.4f}')

        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            return response.success, response.message
        else:
            return False, 'Service call failed'

    def get_current_pose(self) -> Tuple[bool, float, float, float]:
        """
        Get current robot pose.

        Returns:
            Tuple of (success, x, y, z)
        """
        request = GetPoseCmd.Request()

        future = self.get_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            return True, response.x, response.y, response.z
        else:
            return False, 0.0, 0.0, 0.0

    def calculate_error(self, target: Tuple[float, float, float],
                       actual: Tuple[float, float, float]) -> Dict[str, float]:
        """
        Calculate positioning errors.

        Args:
            target: Target (x, y, z) position
            actual: Actual (x, y, z) position

        Returns:
            Dictionary with error metrics
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

    def run_calibration(self, target_x: float, target_y: float, target_z: float,
                       iterations: int, settling_time: float,
                       home_x: float, home_y: float, home_z: float) -> Dict:
        """
        Run complete calibration routine.

        Args:
            target_x, target_y, target_z: Target position
            iterations: Number of times to repeat the measurement
            settling_time: Time to wait after movement before measuring (seconds)
            home_x, home_y, home_z: Home/base position to return to

        Returns:
            Dictionary with all calibration results
        """
        results = {
            'target': {'x': target_x, 'y': target_y, 'z': target_z},
            'home': {'x': home_x, 'y': home_y, 'z': home_z},
            'iterations': iterations,
            'settling_time': settling_time,
            'timestamp': datetime.now().isoformat(),
            'measurements': []
        }

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Starting calibration routine')
        self.get_logger().info(f'Target position: ({target_x:.4f}, {target_y:.4f}, {target_z:.4f})')
        self.get_logger().info(f'Iterations: {iterations}')
        self.get_logger().info(f'Settling time: {settling_time}s')
        self.get_logger().info('=' * 60)

        # Record initial position
        success, init_x, init_y, init_z = self.get_current_pose()
        if success:
            self.get_logger().info(f'Initial position: ({init_x:.4f}, {init_y:.4f}, {init_z:.4f})')
            results['initial_pose'] = {'x': init_x, 'y': init_y, 'z': init_z}

        # Run iterations
        for i in range(iterations):
            self.get_logger().info(f'\n--- Iteration {i+1}/{iterations} ---')

            # Move to target
            move_success, move_msg = self.move_to_position(target_x, target_y, target_z)
            if not move_success:
                self.get_logger().error(f'Movement failed: {move_msg}')
                results['measurements'].append({
                    'iteration': i + 1,
                    'move_success': False,
                    'error': move_msg
                })
                continue

            self.get_logger().info(f'Movement result: {move_msg}')

            # Wait for settling
            self.get_logger().info(f'Waiting {settling_time}s for settling...')
            time.sleep(settling_time)

            # Measure actual position
            pose_success, actual_x, actual_y, actual_z = self.get_current_pose()
            if not pose_success:
                self.get_logger().error('Failed to get current pose')
                results['measurements'].append({
                    'iteration': i + 1,
                    'move_success': True,
                    'pose_success': False
                })
                continue

            self.get_logger().info(f'Actual position: ({actual_x:.4f}, {actual_y:.4f}, {actual_z:.4f})')

            # Calculate errors
            errors = self.calculate_error(
                (target_x, target_y, target_z),
                (actual_x, actual_y, actual_z)
            )

            self.get_logger().info(f'Errors - X: {errors["error_x"]:.6f}, '
                                 f'Y: {errors["error_y"]:.6f}, '
                                 f'Z: {errors["error_z"]:.6f}')
            self.get_logger().info(f'Euclidean error: {errors["euclidean_error"]:.6f}')

            # Store measurement
            results['measurements'].append({
                'iteration': i + 1,
                'move_success': True,
                'pose_success': True,
                'actual': {'x': actual_x, 'y': actual_y, 'z': actual_z},
                'errors': errors
            })

        # Calculate statistics
        if results['measurements']:
            self._calculate_statistics(results)

        # Return to home position
        self.get_logger().info(f'\n--- Returning to home position ---')
        self.get_logger().info(f'Home position: ({home_x:.4f}, {home_y:.4f}, {home_z:.4f})')

        home_success, home_msg = self.move_to_position(home_x, home_y, home_z)
        results['return_home_success'] = home_success
        results['return_home_message'] = home_msg

        if home_success:
            time.sleep(settling_time)
            pose_success, final_x, final_y, final_z = self.get_current_pose()
            if pose_success:
                self.get_logger().info(f'Final position: ({final_x:.4f}, {final_y:.4f}, {final_z:.4f})')
                results['final_pose'] = {'x': final_x, 'y': final_y, 'z': final_z}

        return results

    def _calculate_statistics(self, results: Dict):
        """Calculate statistical metrics from measurements."""
        measurements = results['measurements']
        successful_measurements = [m for m in measurements if m.get('pose_success', False)]

        if not successful_measurements:
            self.get_logger().warning('No successful measurements for statistics')
            return

        # Extract error arrays
        errors_x = [m['errors']['error_x'] for m in successful_measurements]
        errors_y = [m['errors']['error_y'] for m in successful_measurements]
        errors_z = [m['errors']['error_z'] for m in successful_measurements]
        euclidean_errors = [m['errors']['euclidean_error'] for m in successful_measurements]

        # Calculate statistics
        stats = {
            'count': len(successful_measurements),
            'error_x': self._compute_stats(errors_x),
            'error_y': self._compute_stats(errors_y),
            'error_z': self._compute_stats(errors_z),
            'euclidean_error': self._compute_stats(euclidean_errors)
        }

        results['statistics'] = stats

        # Print statistics
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('CALIBRATION STATISTICS')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Successful measurements: {stats["count"]}')

        for axis in ['error_x', 'error_y', 'error_z', 'euclidean_error']:
            s = stats[axis]
            self.get_logger().info(f'\n{axis.upper()}:')
            self.get_logger().info(f'  Mean: {s["mean"]:.6f}')
            self.get_logger().info(f'  Std Dev: {s["std"]:.6f}')
            self.get_logger().info(f'  Min: {s["min"]:.6f}')
            self.get_logger().info(f'  Max: {s["max"]:.6f}')
            self.get_logger().info(f'  RMS: {s["rms"]:.6f}')

    def _compute_stats(self, values: List[float]) -> Dict[str, float]:
        """Compute statistical metrics for a list of values."""
        n = len(values)
        if n == 0:
            return {'mean': 0.0, 'std': 0.0, 'min': 0.0, 'max': 0.0, 'rms': 0.0}

        mean = sum(values) / n
        variance = sum((x - mean) ** 2 for x in values) / n
        std = math.sqrt(variance)
        rms = math.sqrt(sum(x ** 2 for x in values) / n)

        return {
            'mean': mean,
            'std': std,
            'min': min(values),
            'max': max(values),
            'rms': rms
        }


def save_results_json(results: Dict, filename: str):
    """Save calibration results to JSON file."""
    with open(filename, 'w') as f:
        json.dump(results, f, indent=2)
    print(f'Results saved to JSON: {filename}')


def save_results_csv(results: Dict, filename: str):
    """Save calibration results to CSV file."""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)

        # Write header
        writer.writerow(['Calibration Results'])
        writer.writerow(['Timestamp', results['timestamp']])
        writer.writerow(['Target X', results['target']['x']])
        writer.writerow(['Target Y', results['target']['y']])
        writer.writerow(['Target Z', results['target']['z']])
        writer.writerow(['Iterations', results['iterations']])
        writer.writerow([])

        # Write measurements
        writer.writerow(['Iteration', 'Move Success', 'Pose Success',
                        'Actual X', 'Actual Y', 'Actual Z',
                        'Error X', 'Error Y', 'Error Z', 'Euclidean Error'])

        for m in results['measurements']:
            if m.get('pose_success', False):
                writer.writerow([
                    m['iteration'],
                    m.get('move_success', False),
                    m.get('pose_success', False),
                    m['actual']['x'],
                    m['actual']['y'],
                    m['actual']['z'],
                    m['errors']['error_x'],
                    m['errors']['error_y'],
                    m['errors']['error_z'],
                    m['errors']['euclidean_error']
                ])
            else:
                writer.writerow([
                    m['iteration'],
                    m.get('move_success', False),
                    m.get('pose_success', False),
                    '', '', '', '', '', '', ''
                ])

        # Write statistics
        if 'statistics' in results:
            writer.writerow([])
            writer.writerow(['Statistics'])
            writer.writerow(['Metric', 'Mean', 'Std Dev', 'Min', 'Max', 'RMS'])

            stats = results['statistics']
            for metric in ['error_x', 'error_y', 'error_z', 'euclidean_error']:
                s = stats[metric]
                writer.writerow([
                    metric.upper(),
                    s['mean'],
                    s['std'],
                    s['min'],
                    s['max'],
                    s['rms']
                ])

    print(f'Results saved to CSV: {filename}')


def main():
    parser = argparse.ArgumentParser(
        description='Calibrate RoArm positioning accuracy and repeatability'
    )
    parser.add_argument('--target', nargs=3, type=float, required=True,
                       metavar=('X', 'Y', 'Z'),
                       help='Target position (x y z)')
    parser.add_argument('--iterations', type=int, default=3,
                       help='Number of iterations (default: 3)')
    parser.add_argument('--home', nargs=3, type=float, default=[0.2, 0.0, 0.0],
                       metavar=('X', 'Y', 'Z'),
                       help='Home/base position to return to (default: 0.2 0.0 0.0)')
    parser.add_argument('--settling-time', type=float, default=1.0,
                       help='Time to wait after movement before measuring in seconds (default: 1.0)')
    parser.add_argument('--output-dir', type=str, default='.',
                       help='Output directory for results files (default: current directory)')

    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()

    try:
        # Create calibrator node
        calibrator = RoArmCalibrator()

        # Run calibration
        results = calibrator.run_calibration(
            target_x=args.target[0],
            target_y=args.target[1],
            target_z=args.target[2],
            iterations=args.iterations,
            settling_time=args.settling_time,
            home_x=args.home[0],
            home_y=args.home[1],
            home_z=args.home[2]
        )

        # Generate filenames with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        json_filename = f'{args.output_dir}/calibration_{timestamp}.json'
        csv_filename = f'{args.output_dir}/calibration_{timestamp}.csv'

        # Save results
        save_results_json(results, json_filename)
        save_results_csv(results, csv_filename)

        print('\n' + '=' * 60)
        print('CALIBRATION COMPLETE')
        print('=' * 60)

    except KeyboardInterrupt:
        print('\nCalibration interrupted by user')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
