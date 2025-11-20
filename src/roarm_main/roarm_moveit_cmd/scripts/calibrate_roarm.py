#!/usr/bin/env python3

"""
RoArm Calibration Script (Refactored)

Main entry point for robot arm calibration. Supports both single-target
and multi-target calibration modes.
"""

import rclpy
from rclpy.node import Node
import argparse
import os
import sys
from datetime import datetime
from typing import Dict, List

# Import calibration modules
from calibration import (
    RobotInterface,
    CalibrationMeasurement,
    StatisticsCalculator,
    ResultsWriter,
    ConfigLoader
)


class CalibrationOrchestrator(Node):
    """
    Main orchestrator for calibration operations.

    Coordinates robot interface, measurements, statistics, and I/O.
    """

    def __init__(self):
        super().__init__('roarm_calibrator')

        # Initialize components
        self.robot = RobotInterface(self)
        self.measurement = CalibrationMeasurement(self.robot, self.get_logger())
        self.statistics = StatisticsCalculator(self.get_logger())
        self.writer = ResultsWriter()

        # Wait for services
        if not self.robot.wait_for_services():
            sys.exit(1)

        self.get_logger().info('Ready to calibrate.')

    def run_single_target(self, target_x: float, target_y: float, target_z: float,
                         iterations: int, settling_time: float,
                         home_x: float, home_y: float, home_z: float) -> Dict:
        """Run single-target calibration."""
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
        self.get_logger().info(f'Starting single-target calibration')
        self.get_logger().info(f'Target: ({target_x:.4f}, {target_y:.4f}, {target_z:.4f})')
        self.get_logger().info(f'Iterations: {iterations}')
        self.get_logger().info('=' * 60)

        # Record initial position
        success, init_x, init_y, init_z = self.robot.get_current_pose()
        if success:
            results['initial_pose'] = {'x': init_x, 'y': init_y, 'z': init_z}
            self.get_logger().info(f'Initial position: ({init_x:.4f}, {init_y:.4f}, {init_z:.4f})')

        # Run iterations
        target = {'x': target_x, 'y': target_y, 'z': target_z}
        home = {'x': home_x, 'y': home_y, 'z': home_z}

        for i in range(iterations):
            self.get_logger().info(f'\n--- Iteration {i+1}/{iterations} ---')

            # Run measurement cycle
            cycle_result = self.measurement.run_measurement_cycle(
                target, home, settling_time
            )

            measurement = {
                'iteration': i + 1,
                'target_measurement': cycle_result['target_measurement'],
                'home_measurement': cycle_result['home_measurement']
            }
            results['measurements'].append(measurement)

        # Calculate statistics
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('CALIBRATION STATISTICS')
        self.get_logger().info('=' * 60)
        results['statistics'] = self.statistics.calculate_single_target_statistics(
            results['measurements']
        )

        # Record final position
        success, final_x, final_y, final_z = self.robot.get_current_pose()
        if success:
            results['final_pose'] = {'x': final_x, 'y': final_y, 'z': final_z}

        return results

    def run_multi_target(self, targets: List[Dict], home: Dict, loops: int,
                        settling_time: float, movement_timeout: float,
                        workspace_limits: Dict = None) -> Dict:
        """Run multi-target calibration."""
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

        # Validate positions
        self.get_logger().info('=' * 70)
        self.get_logger().info('VALIDATING TARGET POSITIONS')
        self.get_logger().info('=' * 70)

        valid, msg = self.measurement.validate_position(
            home['x'], home['y'], home['z'], workspace_limits
        )
        if not valid:
            self.get_logger().error(f'Home position invalid: {msg}')
            results['validation_error'] = f'Home: {msg}'
            return results

        for i, target in enumerate(targets):
            valid, msg = self.measurement.validate_position(
                target['x'], target['y'], target['z'], workspace_limits
            )
            if not valid:
                self.get_logger().error(
                    f"Target {i+1} ({target.get('name', 'unnamed')}) invalid: {msg}"
                )
                results['validation_error'] = f"Target {i+1}: {msg}"
                return results
            self.get_logger().info(f"✓ Target {i+1} ({target.get('name', 'unnamed')}): Valid")

        # Display calibration plan
        total_movements = loops * len(targets) * 2
        estimated_time = total_movements * (settling_time + 2.0)
        self.get_logger().info(f'\nEstimated time: {estimated_time/60:.1f} minutes')
        self.get_logger().info(f'Total movements: {total_movements}')

        # Store target information
        for target in targets:
            results['targets'].append({
                'name': target.get('name', 'unnamed'),
                'position': {'x': target['x'], 'y': target['y'], 'z': target['z']},
                'description': target.get('description', '')
            })

        # Record initial position
        success, init_x, init_y, init_z = self.robot.get_current_pose()
        if success:
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
                self.get_logger().info(
                    f'\n--- Target {target_idx+1}/{len(targets)}: '
                    f'{target.get("name", "unnamed")} (Loop {loop+1}) ---'
                )

                # Run measurement cycle
                cycle_result = self.measurement.run_measurement_cycle(
                    target, home, settling_time, movement_timeout
                )

                measurement = {
                    'measurement_number': measurement_count + 1,
                    'loop': loop + 1,
                    'target_index': target_idx,
                    'target_name': target.get('name', f'target_{target_idx+1}'),
                    'target_position': {'x': target['x'], 'y': target['y'], 'z': target['z']},
                    'target_measurement': cycle_result['target_measurement'],
                    'home_measurement': cycle_result['home_measurement']
                }
                results['measurements'].append(measurement)
                measurement_count += 1

        # Calculate statistics
        results['statistics'] = self.statistics.calculate_multi_target_statistics(
            results['measurements'],
            results['targets']
        )

        # Record final position
        success, final_x, final_y, final_z = self.robot.get_current_pose()
        if success:
            results['final_pose'] = {'x': final_x, 'y': final_y, 'z': final_z}

        return results


def main():
    """Main entry point for calibration."""
    parser = argparse.ArgumentParser(
        description='Calibrate RoArm positioning accuracy (single or multi-target)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Single-target mode:
    %(prog)s --target 0.2 -0.1 -0.1 --iterations 20

  Multi-target mode:
    %(prog)s --targets-config config/calibration_targets.yaml --loops 2
        """
    )

    # Mode selection
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument('--target', nargs=3, type=float,
                           metavar=('X', 'Y', 'Z'),
                           help='Single target mode: target position (x y z)')
    mode_group.add_argument('--targets-config', type=str,
                           metavar='PATH',
                           help='Multi-target mode: path to YAML config file')

    # Single-target parameters
    parser.add_argument('--iterations', type=int, default=3,
                       help='Single-target: number of iterations (default: 3)')

    # Multi-target parameters
    parser.add_argument('--loops', type=int, default=None,
                       help='Multi-target: number of loops through all targets')

    # Common parameters
    parser.add_argument('--home', nargs=3, type=float, default=None,
                       metavar=('X', 'Y', 'Z'),
                       help='Home position (default: from config or [0.2, 0.0, 0.1])')
    parser.add_argument('--settling-time', type=float, default=None,
                       help='Settling time in seconds (default: from config or 1.0)')
    parser.add_argument('--output-dir', type=str, default='.',
                       help='Output directory (default: current directory)')

    args = parser.parse_args()

    # Initialize ROS2
    rclpy.init()

    try:
        # Create calibrator
        calibrator = CalibrationOrchestrator()

        if args.target:
            # === SINGLE-TARGET MODE ===
            home = args.home if args.home else [0.2, 0.0, 0.1]
            settling_time = args.settling_time if args.settling_time else 1.0

            results = calibrator.run_single_target(
                target_x=args.target[0],
                target_y=args.target[1],
                target_z=args.target[2],
                iterations=args.iterations,
                settling_time=settling_time,
                home_x=home[0],
                home_y=home[1],
                home_z=home[2]
            )

            # Save results
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            json_filename = f'{args.output_dir}/calibration_{timestamp}.json'
            csv_filename = f'{args.output_dir}/calibration_{timestamp}.csv'

            calibrator.writer.save_json(results, json_filename)
            calibrator.writer.save_csv(results, csv_filename)

        else:
            # === MULTI-TARGET MODE ===
            if not os.path.exists(args.targets_config):
                print(f'ERROR: Config file not found: {args.targets_config}')
                sys.exit(1)

            # Load configuration
            targets, home_config, settings, workspace_limits = \
                ConfigLoader.load_targets_config(args.targets_config)

            if not targets:
                print('ERROR: No targets defined in config file')
                sys.exit(1)

            # Override with command-line arguments
            home = {'x': args.home[0], 'y': args.home[1], 'z': args.home[2]} \
                   if args.home else home_config
            settling_time = args.settling_time if args.settling_time is not None \
                           else settings.get('settling_time', 1.0)
            loops = args.loops if args.loops is not None \
                   else settings.get('default_loops', 1)
            movement_timeout = settings.get('movement_timeout', 10.0)

            results = calibrator.run_multi_target(
                targets=targets,
                home=home,
                loops=loops,
                settling_time=settling_time,
                movement_timeout=movement_timeout,
                workspace_limits=workspace_limits
            )

            # Save results
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            json_filename = f'{args.output_dir}/calibration_multi_{timestamp}.json'
            csv_filename = f'{args.output_dir}/calibration_multi_{timestamp}.csv'

            calibrator.writer.save_json(results, json_filename)
            calibrator.writer.save_csv(results, csv_filename)

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
