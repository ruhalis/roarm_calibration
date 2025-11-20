#!/usr/bin/env python3

"""
RoArm Calibration Script (Multi-Target Enhanced)
Moves the robot arm to multiple target positions, measures actual positions,
calculates errors, and tests repeatability across the workspace.
"""

import rclpy
from rclpy.node import Node
from roarm_moveit.srv import MovePointCmd, GetPoseCmd
import argparse
import time
import json
import csv
import math
import yaml
import os
from datetime import datetime
from typing import List, Dict, Tuple, Optional
from pathlib import Path
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

    def move_to_position(self, x: float, y: float, z: float, timeout: float = 10.0) -> Tuple[bool, str]:
        """
        Move robot to specified position.

        Args:
            x, y, z: Target coordinates
            timeout: Movement timeout in seconds

        Returns:
            Tuple of (success, message)
        """
        request = MovePointCmd.Request()
        request.x = x
        request.y = y
        request.z = z

        self.get_logger().info(f'Moving to position: x={x:.4f}, y={y:.4f}, z={z:.4f}')

        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if future.result() is not None:
            response = future.result()
            return response.success, response.message
        else:
            return False, 'Service call failed or timed out'

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

    def validate_position(self, x: float, y: float, z: float,
                         limits: Optional[Dict] = None) -> Tuple[bool, str]:
        """
        Validate that a position is within workspace limits.

        Args:
            x, y, z: Position to validate
            limits: Optional workspace limits dictionary

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

    def run_multi_target_calibration(self, targets: List[Dict], home: Dict,
                                     loops: int, settling_time: float,
                                     movement_timeout: float = 10.0,
                                     workspace_limits: Optional[Dict] = None) -> Dict:
        """
        Run multi-target calibration routine.

        Args:
            targets: List of target position dictionaries with 'name', 'x', 'y', 'z'
            home: Home position dictionary with 'x', 'y', 'z'
            loops: Number of complete cycles through all targets
            settling_time: Time to wait after movement before measuring (seconds)
            movement_timeout: Timeout for each movement (seconds)
            workspace_limits: Optional workspace safety limits

        Returns:
            Dictionary with all calibration results
        """
        home_x, home_y, home_z = home['x'], home['y'], home['z']

        results = {
            'mode': 'multi_target',
            'home': home,
            'num_targets': len(targets),
            'loops': loops,
            'settling_time': settling_time,
            'movement_timeout': movement_timeout,
            'timestamp': datetime.now().isoformat(),
            'targets': [],
            'measurements': []
        }

        # Validate all positions first
        self.get_logger().info('=' * 70)
        self.get_logger().info('VALIDATING TARGET POSITIONS')
        self.get_logger().info('=' * 70)

        valid, msg = self.validate_position(home_x, home_y, home_z, workspace_limits)
        if not valid:
            self.get_logger().error(f'Home position invalid: {msg}')
            results['validation_error'] = f'Home: {msg}'
            return results

        for i, target in enumerate(targets):
            valid, msg = self.validate_position(target['x'], target['y'], target['z'], workspace_limits)
            if not valid:
                self.get_logger().error(f"Target {i+1} ({target.get('name', 'unnamed')}) invalid: {msg}")
                results['validation_error'] = f"Target {i+1}: {msg}"
                return results
            self.get_logger().info(f"✓ Target {i+1} ({target.get('name', 'unnamed')}): Valid")

        # Calculate and display estimated time
        total_movements = loops * len(targets) * 2  # Each target: move to target + return to home
        estimated_time = total_movements * (settling_time + 2.0)  # 2s for movement
        self.get_logger().info(f'\nEstimated calibration time: {estimated_time/60:.1f} minutes')
        self.get_logger().info(f'Total movements: {total_movements} ({loops} loops × {len(targets)} targets × 2 moves)')

        # Display calibration plan
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('CALIBRATION PLAN')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Home position: ({home_x:.4f}, {home_y:.4f}, {home_z:.4f})')
        self.get_logger().info(f'\nTargets to test:')
        for i, target in enumerate(targets):
            self.get_logger().info(f"  {i+1}. {target.get('name', f'target_{i+1}')}: "
                                 f"({target['x']:.4f}, {target['y']:.4f}, {target['z']:.4f})")

        # Store target information
        for target in targets:
            results['targets'].append({
                'name': target.get('name', 'unnamed'),
                'position': {'x': target['x'], 'y': target['y'], 'z': target['z']},
                'description': target.get('description', '')
            })

        # Record initial position
        success, init_x, init_y, init_z = self.get_current_pose()
        if success:
            self.get_logger().info(f'\nInitial position: ({init_x:.4f}, {init_y:.4f}, {init_z:.4f})')
            results['initial_pose'] = {'x': init_x, 'y': init_y, 'z': init_z}

        # Run calibration loops
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('STARTING CALIBRATION')
        self.get_logger().info('=' * 70)

        measurement_count = 0
        for loop in range(loops):
            self.get_logger().info(f'\n{"="*70}')
            self.get_logger().info(f'LOOP {loop+1}/{loops}')
            self.get_logger().info(f'{"="*70}')

            for target_idx, target in enumerate(targets):
                target_name = target.get('name', f'target_{target_idx+1}')
                target_x, target_y, target_z = target['x'], target['y'], target['z']

                self.get_logger().info(f'\n--- Target {target_idx+1}/{len(targets)}: {target_name} (Loop {loop+1}) ---')

                measurement = {
                    'measurement_number': measurement_count + 1,
                    'loop': loop + 1,
                    'target_index': target_idx,
                    'target_name': target_name,
                    'target_position': {'x': target_x, 'y': target_y, 'z': target_z},
                    'target_measurement': {},
                    'home_measurement': {}
                }

                # === MOVE TO TARGET AND MEASURE ===
                self.get_logger().info(f'>> Moving to TARGET: {target_name}')
                self.get_logger().info(f'   Position: ({target_x:.4f}, {target_y:.4f}, {target_z:.4f})')

                move_success, move_msg = self.move_to_position(target_x, target_y, target_z, movement_timeout)
                if not move_success:
                    self.get_logger().error(f'Target movement failed: {move_msg}')
                    measurement['target_measurement'] = {
                        'move_success': False,
                        'error': move_msg
                    }
                else:
                    self.get_logger().info(f'   Movement result: {move_msg}')

                    # Wait for settling
                    time.sleep(settling_time)

                    # Measure actual position
                    pose_success, actual_x, actual_y, actual_z = self.get_current_pose()
                    if not pose_success:
                        self.get_logger().error('   Failed to get current pose at target')
                        measurement['target_measurement'] = {
                            'move_success': True,
                            'pose_success': False
                        }
                    else:
                        # Calculate errors
                        errors = self.calculate_error(
                            (target_x, target_y, target_z),
                            (actual_x, actual_y, actual_z)
                        )

                        self.get_logger().info(f'   Actual: ({actual_x:.4f}, {actual_y:.4f}, {actual_z:.4f})')
                        self.get_logger().info(f'   Errors: X={errors["error_x"]:+.6f}, '
                                             f'Y={errors["error_y"]:+.6f}, '
                                             f'Z={errors["error_z"]:+.6f}, '
                                             f'Euclidean={errors["euclidean_error"]:.6f}')

                        # Store target measurement
                        measurement['target_measurement'] = {
                            'move_success': True,
                            'pose_success': True,
                            'actual': {'x': actual_x, 'y': actual_y, 'z': actual_z},
                            'errors': errors
                        }

                # === MOVE TO HOME AND MEASURE ===
                self.get_logger().info(f'\n>> Returning to HOME')
                move_success, move_msg = self.move_to_position(home_x, home_y, home_z, movement_timeout)
                if not move_success:
                    self.get_logger().error(f'Home movement failed: {move_msg}')
                    measurement['home_measurement'] = {
                        'move_success': False,
                        'error': move_msg
                    }
                else:
                    # Wait for settling
                    time.sleep(settling_time)

                    # Measure actual position
                    pose_success, actual_x, actual_y, actual_z = self.get_current_pose()
                    if not pose_success:
                        self.get_logger().error('   Failed to get current pose at home')
                        measurement['home_measurement'] = {
                            'move_success': True,
                            'pose_success': False
                        }
                    else:
                        # Calculate errors
                        errors = self.calculate_error(
                            (home_x, home_y, home_z),
                            (actual_x, actual_y, actual_z)
                        )

                        self.get_logger().info(f'   Home actual: ({actual_x:.4f}, {actual_y:.4f}, {actual_z:.4f})')
                        self.get_logger().info(f'   Home errors: X={errors["error_x"]:+.6f}, '
                                             f'Y={errors["error_y"]:+.6f}, '
                                             f'Z={errors["error_z"]:+.6f}')

                        # Store home measurement
                        measurement['home_measurement'] = {
                            'move_success': True,
                            'pose_success': True,
                            'actual': {'x': actual_x, 'y': actual_y, 'z': actual_z},
                            'errors': errors
                        }

                # Store complete measurement
                results['measurements'].append(measurement)
                measurement_count += 1

        # Calculate statistics
        if results['measurements']:
            self._calculate_multi_target_statistics(results)

        # Record final position
        success, final_x, final_y, final_z = self.get_current_pose()
        if success:
            self.get_logger().info(f'\nFinal position: ({final_x:.4f}, {final_y:.4f}, {final_z:.4f})')
            results['final_pose'] = {'x': final_x, 'y': final_y, 'z': final_z}

        return results

    def run_single_target_calibration(self, target_x: float, target_y: float, target_z: float,
                                      iterations: int, settling_time: float,
                                      home_x: float, home_y: float, home_z: float) -> Dict:
        """
        Run single-target calibration routine (backward compatibility).

        Args:
            target_x, target_y, target_z: Target position
            iterations: Number of times to repeat the measurement
            settling_time: Time to wait after movement before measuring (seconds)
            home_x, home_y, home_z: Home/base position to return to

        Returns:
            Dictionary with all calibration results
        """
        results = {
            'mode': 'single_target',
            'target': {'x': target_x, 'y': target_y, 'z': target_z},
            'home': {'x': home_x, 'y': home_y, 'z': home_z},
            'iterations': iterations,
            'settling_time': settling_time,
            'timestamp': datetime.now().isoformat(),
            'measurements': []
        }

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Starting single-target calibration routine')
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

            measurement = {
                'iteration': i + 1,
                'target_measurement': {},
                'home_measurement': {}
            }

            # === MOVE TO TARGET AND MEASURE ===
            self.get_logger().info('>> Moving to TARGET position')
            move_success, move_msg = self.move_to_position(target_x, target_y, target_z)
            if not move_success:
                self.get_logger().error(f'Target movement failed: {move_msg}')
                measurement['target_measurement'] = {
                    'move_success': False,
                    'error': move_msg
                }
            else:
                self.get_logger().info(f'Movement result: {move_msg}')

                # Wait for settling
                self.get_logger().info(f'Waiting {settling_time}s for settling...')
                time.sleep(settling_time)

                # Measure actual position
                pose_success, actual_x, actual_y, actual_z = self.get_current_pose()
                if not pose_success:
                    self.get_logger().error('Failed to get current pose at target')
                    measurement['target_measurement'] = {
                        'move_success': True,
                        'pose_success': False
                    }
                else:
                    self.get_logger().info(f'Actual position: ({actual_x:.4f}, {actual_y:.4f}, {actual_z:.4f})')

                    # Calculate errors
                    errors = self.calculate_error(
                        (target_x, target_y, target_z),
                        (actual_x, actual_y, actual_z)
                    )

                    self.get_logger().info(f'Target errors - X: {errors["error_x"]:.6f}, '
                                         f'Y: {errors["error_y"]:.6f}, '
                                         f'Z: {errors["error_z"]:.6f}')
                    self.get_logger().info(f'Target euclidean error: {errors["euclidean_error"]:.6f}')

                    # Store target measurement
                    measurement['target_measurement'] = {
                        'move_success': True,
                        'pose_success': True,
                        'actual': {'x': actual_x, 'y': actual_y, 'z': actual_z},
                        'errors': errors
                    }

            # === MOVE TO HOME AND MEASURE ===
            self.get_logger().info('\n>> Moving to HOME position')
            move_success, move_msg = self.move_to_position(home_x, home_y, home_z)
            if not move_success:
                self.get_logger().error(f'Home movement failed: {move_msg}')
                measurement['home_measurement'] = {
                    'move_success': False,
                    'error': move_msg
                }
            else:
                self.get_logger().info(f'Movement result: {move_msg}')

                # Wait for settling
                self.get_logger().info(f'Waiting {settling_time}s for settling...')
                time.sleep(settling_time)

                # Measure actual position
                pose_success, actual_x, actual_y, actual_z = self.get_current_pose()
                if not pose_success:
                    self.get_logger().error('Failed to get current pose at home')
                    measurement['home_measurement'] = {
                        'move_success': True,
                        'pose_success': False
                    }
                else:
                    self.get_logger().info(f'Actual position: ({actual_x:.4f}, {actual_y:.4f}, {actual_z:.4f})')

                    # Calculate errors
                    errors = self.calculate_error(
                        (home_x, home_y, home_z),
                        (actual_x, actual_y, actual_z)
                    )

                    self.get_logger().info(f'Home errors - X: {errors["error_x"]:.6f}, '
                                         f'Y: {errors["error_y"]:.6f}, '
                                         f'Z: {errors["error_z"]:.6f}')
                    self.get_logger().info(f'Home euclidean error: {errors["euclidean_error"]:.6f}')

                    # Store home measurement
                    measurement['home_measurement'] = {
                        'move_success': True,
                        'pose_success': True,
                        'actual': {'x': actual_x, 'y': actual_y, 'z': actual_z},
                        'errors': errors
                    }

            # Store complete measurement for this iteration
            results['measurements'].append(measurement)

        # Calculate statistics
        if results['measurements']:
            self._calculate_single_target_statistics(results)

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

    def _calculate_multi_target_statistics(self, results: Dict):
        """Calculate statistical metrics from multi-target measurements."""
        measurements = results['measurements']
        num_targets = results['num_targets']
        targets = results['targets']

        # Initialize per-target statistics
        per_target_stats = {}
        for i, target_info in enumerate(targets):
            target_name = target_info['name']
            per_target_stats[target_name] = {
                'index': i,
                'position': target_info['position'],
                'measurements': []
            }

        # Group measurements by target
        for m in measurements:
            if m.get('target_measurement', {}).get('pose_success', False):
                target_name = m['target_name']
                per_target_stats[target_name]['measurements'].append(m['target_measurement'])

        # Calculate statistics for each target
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('PER-TARGET CALIBRATION STATISTICS')
        self.get_logger().info('=' * 70)

        target_summary = []
        for target_name, data in per_target_stats.items():
            if data['measurements']:
                errors_x = [m['errors']['error_x'] for m in data['measurements']]
                errors_y = [m['errors']['error_y'] for m in data['measurements']]
                errors_z = [m['errors']['error_z'] for m in data['measurements']]
                euclidean_errors = [m['errors']['euclidean_error'] for m in data['measurements']]

                stats = {
                    'count': len(data['measurements']),
                    'error_x': self._compute_stats(errors_x),
                    'error_y': self._compute_stats(errors_y),
                    'error_z': self._compute_stats(errors_z),
                    'euclidean_error': self._compute_stats(euclidean_errors)
                }

                data['statistics'] = stats

                self.get_logger().info(f'\n{target_name.upper()} - {stats["count"]} measurements')
                self.get_logger().info(f'  Position: ({data["position"]["x"]:.4f}, {data["position"]["y"]:.4f}, {data["position"]["z"]:.4f})')
                self.get_logger().info(f'  Mean Errors: X={stats["error_x"]["mean"]:+.6f}, '
                                     f'Y={stats["error_y"]["mean"]:+.6f}, '
                                     f'Z={stats["error_z"]["mean"]:+.6f}')
                self.get_logger().info(f'  Mean Euclidean Error: {stats["euclidean_error"]["mean"]:.6f}')
                self.get_logger().info(f'  Std Dev: X={stats["error_x"]["std"]:.6f}, '
                                     f'Y={stats["error_y"]["std"]:.6f}, '
                                     f'Z={stats["error_z"]["std"]:.6f}')

                target_summary.append({
                    'name': target_name,
                    'mean_euclidean_error': stats['euclidean_error']['mean'],
                    'mean_x': stats['error_x']['mean'],
                    'mean_y': stats['error_y']['mean'],
                    'mean_z': stats['error_z']['mean']
                })

        # Calculate global statistics (across all targets)
        all_target_measurements = []
        for data in per_target_stats.values():
            all_target_measurements.extend(data['measurements'])

        if all_target_measurements:
            errors_x = [m['errors']['error_x'] for m in all_target_measurements]
            errors_y = [m['errors']['error_y'] for m in all_target_measurements]
            errors_z = [m['errors']['error_z'] for m in all_target_measurements]
            euclidean_errors = [m['errors']['euclidean_error'] for m in all_target_measurements]

            global_stats = {
                'count': len(all_target_measurements),
                'error_x': self._compute_stats(errors_x),
                'error_y': self._compute_stats(errors_y),
                'error_z': self._compute_stats(errors_z),
                'euclidean_error': self._compute_stats(euclidean_errors)
            }

            self.get_logger().info('\n' + '=' * 70)
            self.get_logger().info('GLOBAL STATISTICS (All Targets Combined)')
            self.get_logger().info('=' * 70)
            self.get_logger().info(f'Total successful measurements: {global_stats["count"]}')
            self.get_logger().info(f'Mean Errors: X={global_stats["error_x"]["mean"]:+.6f}, '
                                 f'Y={global_stats["error_y"]["mean"]:+.6f}, '
                                 f'Z={global_stats["error_z"]["mean"]:+.6f}')
            self.get_logger().info(f'Mean Euclidean Error: {global_stats["euclidean_error"]["mean"]:.6f}')
            self.get_logger().info(f'Std Dev: X={global_stats["error_x"]["std"]:.6f}, '
                                 f'Y={global_stats["error_y"]["std"]:.6f}, '
                                 f'Z={global_stats["error_z"]["std"]:.6f}')
            self.get_logger().info(f'RMS: X={global_stats["error_x"]["rms"]:.6f}, '
                                 f'Y={global_stats["error_y"]["rms"]:.6f}, '
                                 f'Z={global_stats["error_z"]["rms"]:.6f}')

            results['statistics'] = {
                'per_target': per_target_stats,
                'global': global_stats
            }

            # Display target comparison
            if target_summary:
                self.get_logger().info('\n' + '=' * 70)
                self.get_logger().info('TARGET COMPARISON (Sorted by Mean Euclidean Error)')
                self.get_logger().info('=' * 70)
                target_summary.sort(key=lambda x: x['mean_euclidean_error'])
                for i, t in enumerate(target_summary):
                    self.get_logger().info(f'{i+1}. {t["name"]}: {t["mean_euclidean_error"]:.6f}m '
                                         f'(X:{t["mean_x"]:+.6f}, Y:{t["mean_y"]:+.6f}, Z:{t["mean_z"]:+.6f})')

                best = target_summary[0]
                worst = target_summary[-1]
                self.get_logger().info(f'\nBest: {best["name"]} ({best["mean_euclidean_error"]:.6f}m)')
                self.get_logger().info(f'Worst: {worst["name"]} ({worst["mean_euclidean_error"]:.6f}m)')

    def _calculate_single_target_statistics(self, results: Dict):
        """Calculate statistical metrics from single-target measurements."""
        measurements = results['measurements']

        # Separate target and home measurements
        successful_target = [m['target_measurement'] for m in measurements
                            if m.get('target_measurement', {}).get('pose_success', False)]
        successful_home = [m['home_measurement'] for m in measurements
                          if m.get('home_measurement', {}).get('pose_success', False)]

        results['statistics'] = {}

        # Print statistics header
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('CALIBRATION STATISTICS')
        self.get_logger().info('=' * 60)

        # Calculate and display TARGET statistics
        if successful_target:
            errors_x = [m['errors']['error_x'] for m in successful_target]
            errors_y = [m['errors']['error_y'] for m in successful_target]
            errors_z = [m['errors']['error_z'] for m in successful_target]
            euclidean_errors = [m['errors']['euclidean_error'] for m in successful_target]

            target_stats = {
                'count': len(successful_target),
                'error_x': self._compute_stats(errors_x),
                'error_y': self._compute_stats(errors_y),
                'error_z': self._compute_stats(errors_z),
                'euclidean_error': self._compute_stats(euclidean_errors)
            }

            results['statistics']['target'] = target_stats

            self.get_logger().info(f'\nTARGET POSITION - Successful measurements: {target_stats["count"]}')
            for axis in ['error_x', 'error_y', 'error_z', 'euclidean_error']:
                s = target_stats[axis]
                self.get_logger().info(f'\n  {axis.upper()}:')
                self.get_logger().info(f'    Mean: {s["mean"]:.6f}')
                self.get_logger().info(f'    Std Dev: {s["std"]:.6f}')
                self.get_logger().info(f'    Min: {s["min"]:.6f}')
                self.get_logger().info(f'    Max: {s["max"]:.6f}')
                self.get_logger().info(f'    RMS: {s["rms"]:.6f}')
        else:
            self.get_logger().warning('\nNo successful TARGET measurements for statistics')

        # Calculate and display HOME statistics
        if successful_home:
            errors_x = [m['errors']['error_x'] for m in successful_home]
            errors_y = [m['errors']['error_y'] for m in successful_home]
            errors_z = [m['errors']['error_z'] for m in successful_home]
            euclidean_errors = [m['errors']['euclidean_error'] for m in successful_home]

            home_stats = {
                'count': len(successful_home),
                'error_x': self._compute_stats(errors_x),
                'error_y': self._compute_stats(errors_y),
                'error_z': self._compute_stats(errors_z),
                'euclidean_error': self._compute_stats(euclidean_errors)
            }

            results['statistics']['home'] = home_stats

            self.get_logger().info(f'\nHOME POSITION - Successful measurements: {home_stats["count"]}')
            for axis in ['error_x', 'error_y', 'error_z', 'euclidean_error']:
                s = home_stats[axis]
                self.get_logger().info(f'\n  {axis.upper()}:')
                self.get_logger().info(f'    Mean: {s["mean"]:.6f}')
                self.get_logger().info(f'    Std Dev: {s["std"]:.6f}')
                self.get_logger().info(f'    Min: {s["min"]:.6f}')
                self.get_logger().info(f'    Max: {s["max"]:.6f}')
                self.get_logger().info(f'    RMS: {s["rms"]:.6f}')
        else:
            self.get_logger().warning('\nNo successful HOME measurements for statistics')

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


def load_targets_config(config_path: str) -> Tuple[List[Dict], Dict, Dict]:
    """
    Load calibration targets from YAML config file.

    Args:
        config_path: Path to YAML config file

    Returns:
        Tuple of (targets_list, home_dict, settings_dict)
    """
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    targets = config.get('targets', [])
    home = config.get('home_position', {'x': 0.2, 'y': 0.0, 'z': 0.1})
    settings = config.get('calibration_settings', {})
    workspace_limits = config.get('workspace_limits', None)

    return targets, home, settings, workspace_limits


def save_results_json(results: Dict, filename: str):
    """Save calibration results to JSON file."""
    with open(filename, 'w') as f:
        json.dump(results, f, indent=2)
    print(f'Results saved to JSON: {filename}')


def save_results_csv_single(results: Dict, filename: str):
    """Save single-target calibration results to CSV file."""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)

        # Write header
        writer.writerow(['Calibration Results'])
        writer.writerow(['Mode', results.get('mode', 'single_target')])
        writer.writerow(['Timestamp', results['timestamp']])
        writer.writerow(['Target X', results['target']['x']])
        writer.writerow(['Target Y', results['target']['y']])
        writer.writerow(['Target Z', results['target']['z']])
        writer.writerow(['Home X', results['home']['x']])
        writer.writerow(['Home Y', results['home']['y']])
        writer.writerow(['Home Z', results['home']['z']])
        writer.writerow(['Iterations', results['iterations']])
        writer.writerow([])

        # Write TARGET measurements
        writer.writerow(['TARGET POSITION MEASUREMENTS'])
        writer.writerow(['Iteration', 'Move Success', 'Pose Success',
                        'Actual X', 'Actual Y', 'Actual Z',
                        'Error X', 'Error Y', 'Error Z', 'Euclidean Error'])

        for m in results['measurements']:
            target = m.get('target_measurement', {})
            if target.get('pose_success', False):
                writer.writerow([
                    m['iteration'],
                    target.get('move_success', False),
                    target.get('pose_success', False),
                    target['actual']['x'],
                    target['actual']['y'],
                    target['actual']['z'],
                    target['errors']['error_x'],
                    target['errors']['error_y'],
                    target['errors']['error_z'],
                    target['errors']['euclidean_error']
                ])
            else:
                writer.writerow([
                    m['iteration'],
                    target.get('move_success', False),
                    target.get('pose_success', False),
                    '', '', '', '', '', '', ''
                ])

        writer.writerow([])

        # Write HOME measurements
        writer.writerow(['HOME POSITION MEASUREMENTS'])
        writer.writerow(['Iteration', 'Move Success', 'Pose Success',
                        'Actual X', 'Actual Y', 'Actual Z',
                        'Error X', 'Error Y', 'Error Z', 'Euclidean Error'])

        for m in results['measurements']:
            home = m.get('home_measurement', {})
            if home.get('pose_success', False):
                writer.writerow([
                    m['iteration'],
                    home.get('move_success', False),
                    home.get('pose_success', False),
                    home['actual']['x'],
                    home['actual']['y'],
                    home['actual']['z'],
                    home['errors']['error_x'],
                    home['errors']['error_y'],
                    home['errors']['error_z'],
                    home['errors']['euclidean_error']
                ])
            else:
                writer.writerow([
                    m['iteration'],
                    home.get('move_success', False),
                    home.get('pose_success', False),
                    '', '', '', '', '', '', ''
                ])

        # Write statistics
        if 'statistics' in results:
            writer.writerow([])
            writer.writerow(['STATISTICS'])

            # Target statistics
            if 'target' in results['statistics']:
                writer.writerow([])
                writer.writerow(['TARGET POSITION STATISTICS'])
                writer.writerow(['Metric', 'Mean', 'Std Dev', 'Min', 'Max', 'RMS'])

                stats = results['statistics']['target']
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

            # Home statistics
            if 'home' in results['statistics']:
                writer.writerow([])
                writer.writerow(['HOME POSITION STATISTICS'])
                writer.writerow(['Metric', 'Mean', 'Std Dev', 'Min', 'Max', 'RMS'])

                stats = results['statistics']['home']
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


def save_results_csv_multi(results: Dict, filename: str):
    """Save multi-target calibration results to CSV file."""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)

        # Write header
        writer.writerow(['Multi-Target Calibration Results'])
        writer.writerow(['Mode', results.get('mode', 'multi_target')])
        writer.writerow(['Timestamp', results['timestamp']])
        writer.writerow(['Number of Targets', results['num_targets']])
        writer.writerow(['Number of Loops', results['loops']])
        writer.writerow(['Settling Time (s)', results['settling_time']])
        writer.writerow(['Home X', results['home']['x']])
        writer.writerow(['Home Y', results['home']['y']])
        writer.writerow(['Home Z', results['home']['z']])
        writer.writerow([])

        # Write target definitions
        writer.writerow(['TARGET DEFINITIONS'])
        writer.writerow(['Index', 'Name', 'X', 'Y', 'Z', 'Description'])
        for i, target in enumerate(results['targets']):
            pos = target['position']
            writer.writerow([
                i + 1,
                target['name'],
                pos['x'],
                pos['y'],
                pos['z'],
                target.get('description', '')
            ])
        writer.writerow([])

        # Write all measurements
        writer.writerow(['ALL MEASUREMENTS'])
        writer.writerow(['Measurement #', 'Loop', 'Target Index', 'Target Name',
                        'Target X', 'Target Y', 'Target Z',
                        'Actual X', 'Actual Y', 'Actual Z',
                        'Error X', 'Error Y', 'Error Z', 'Euclidean Error'])

        for m in results['measurements']:
            target_meas = m.get('target_measurement', {})
            if target_meas.get('pose_success', False):
                pos = m['target_position']
                actual = target_meas['actual']
                errors = target_meas['errors']
                writer.writerow([
                    m['measurement_number'],
                    m['loop'],
                    m['target_index'] + 1,
                    m['target_name'],
                    pos['x'],
                    pos['y'],
                    pos['z'],
                    actual['x'],
                    actual['y'],
                    actual['z'],
                    errors['error_x'],
                    errors['error_y'],
                    errors['error_z'],
                    errors['euclidean_error']
                ])

        # Write per-target statistics
        if 'statistics' in results and 'per_target' in results['statistics']:
            writer.writerow([])
            writer.writerow(['PER-TARGET STATISTICS'])
            writer.writerow(['Target Name', 'Count', 'Mean Error X', 'Mean Error Y', 'Mean Error Z',
                           'Mean Euclidean', 'Std X', 'Std Y', 'Std Z', 'RMS Euclidean'])

            per_target = results['statistics']['per_target']
            for target_name, data in per_target.items():
                if 'statistics' in data:
                    stats = data['statistics']
                    writer.writerow([
                        target_name,
                        stats['count'],
                        stats['error_x']['mean'],
                        stats['error_y']['mean'],
                        stats['error_z']['mean'],
                        stats['euclidean_error']['mean'],
                        stats['error_x']['std'],
                        stats['error_y']['std'],
                        stats['error_z']['std'],
                        stats['euclidean_error']['rms']
                    ])

        # Write global statistics
        if 'statistics' in results and 'global' in results['statistics']:
            writer.writerow([])
            writer.writerow(['GLOBAL STATISTICS (All Targets Combined)'])
            writer.writerow(['Metric', 'Mean', 'Std Dev', 'Min', 'Max', 'RMS'])

            global_stats = results['statistics']['global']
            for metric in ['error_x', 'error_y', 'error_z', 'euclidean_error']:
                s = global_stats[metric]
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
        description='Calibrate RoArm positioning accuracy and repeatability (single or multi-target)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Single-target mode (backward compatible):
    %(prog)s --target 0.2 -0.1 -0.1 --iterations 20

  Multi-target mode:
    %(prog)s --targets-config config/calibration_targets.yaml --loops 2

  Multi-target with custom settings:
    %(prog)s --targets-config targets.yaml --loops 3 --settling-time 1.5
        """
    )

    # Mode selection
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument('--target', nargs=3, type=float,
                           metavar=('X', 'Y', 'Z'),
                           help='Single target mode: target position (x y z)')
    mode_group.add_argument('--targets-config', type=str,
                           metavar='PATH',
                           help='Multi-target mode: path to YAML config file with target positions')

    # Single-target specific
    parser.add_argument('--iterations', type=int, default=3,
                       help='Single-target mode: number of iterations (default: 3)')

    # Multi-target specific
    parser.add_argument('--loops', type=int, default=None,
                       help='Multi-target mode: number of complete loops through all targets (default: from config or 1)')

    # Common parameters
    parser.add_argument('--home', nargs=3, type=float, default=None,
                       metavar=('X', 'Y', 'Z'),
                       help='Home/base position to return to (default: from config or [0.2, 0.0, 0.1])')
    parser.add_argument('--settling-time', type=float, default=None,
                       help='Time to wait after movement before measuring in seconds (default: from config or 1.0)')
    parser.add_argument('--output-dir', type=str, default='.',
                       help='Output directory for results files (default: current directory)')

    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()

    try:
        # Create calibrator node
        calibrator = RoArmCalibrator()

        if args.target:
            # === SINGLE-TARGET MODE ===
            home = args.home if args.home else [0.2, 0.0, 0.1]
            settling_time = args.settling_time if args.settling_time else 1.0

            results = calibrator.run_single_target_calibration(
                target_x=args.target[0],
                target_y=args.target[1],
                target_z=args.target[2],
                iterations=args.iterations,
                settling_time=settling_time,
                home_x=home[0],
                home_y=home[1],
                home_z=home[2]
            )

            # Generate filenames
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            json_filename = f'{args.output_dir}/calibration_{timestamp}.json'
            csv_filename = f'{args.output_dir}/calibration_{timestamp}.csv'

            # Save results
            save_results_json(results, json_filename)
            save_results_csv_single(results, csv_filename)

        else:
            # === MULTI-TARGET MODE ===
            if not os.path.exists(args.targets_config):
                print(f'ERROR: Config file not found: {args.targets_config}')
                sys.exit(1)

            # Load configuration
            targets, home_config, settings, workspace_limits = load_targets_config(args.targets_config)

            if not targets:
                print('ERROR: No targets defined in config file')
                sys.exit(1)

            # Override with command-line arguments if provided
            home = {'x': args.home[0], 'y': args.home[1], 'z': args.home[2]} if args.home else home_config
            settling_time = args.settling_time if args.settling_time is not None else settings.get('settling_time', 1.0)
            loops = args.loops if args.loops is not None else settings.get('default_loops', 1)
            movement_timeout = settings.get('movement_timeout', 10.0)

            results = calibrator.run_multi_target_calibration(
                targets=targets,
                home=home,
                loops=loops,
                settling_time=settling_time,
                movement_timeout=movement_timeout,
                workspace_limits=workspace_limits
            )

            # Generate filenames
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            json_filename = f'{args.output_dir}/calibration_multi_{timestamp}.json'
            csv_filename = f'{args.output_dir}/calibration_multi_{timestamp}.csv'

            # Save results
            save_results_json(results, json_filename)
            save_results_csv_multi(results, csv_filename)

        print('\n' + '=' * 70)
        print('CALIBRATION COMPLETE')
        print('=' * 70)

    except KeyboardInterrupt:
        print('\nCalibration interrupted by user')
    except Exception as e:
        print(f'\nERROR: {str(e)}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
