"""
Statistics Module

Handles statistical analysis of calibration measurements.
"""

import math
from typing import List, Dict


class StatisticsCalculator:
    """
    Calculates statistics for calibration measurements.
    """

    def __init__(self, logger):
        """
        Initialize statistics calculator.

        Args:
            logger: Logger instance for output
        """
        self.logger = logger

    def compute_stats(self, values: List[float]) -> Dict[str, float]:
        """
        Compute statistical metrics for a list of values.

        Args:
            values: List of numerical values

        Returns:
            Dictionary with statistics:
                - mean: Average value
                - std: Standard deviation
                - min: Minimum value
                - max: Maximum value
                - rms: Root mean square
        """
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

    def calculate_single_target_statistics(self, measurements: List[Dict]) -> Dict:
        """
        Calculate statistics for single-target calibration.

        Args:
            measurements: List of measurement dictionaries with 'target_measurement'
                         and 'home_measurement' keys

        Returns:
            Dictionary with 'target' and 'home' statistics
        """
        # Separate target and home measurements
        successful_target = [
            m['target_measurement'] for m in measurements
            if m.get('target_measurement', {}).get('pose_success', False)
        ]
        successful_home = [
            m['home_measurement'] for m in measurements
            if m.get('home_measurement', {}).get('pose_success', False)
        ]

        statistics = {}

        # Calculate TARGET statistics
        if successful_target:
            errors_x = [m['errors']['error_x'] for m in successful_target]
            errors_y = [m['errors']['error_y'] for m in successful_target]
            errors_z = [m['errors']['error_z'] for m in successful_target]
            euclidean_errors = [m['errors']['euclidean_error'] for m in successful_target]

            statistics['target'] = {
                'count': len(successful_target),
                'error_x': self.compute_stats(errors_x),
                'error_y': self.compute_stats(errors_y),
                'error_z': self.compute_stats(errors_z),
                'euclidean_error': self.compute_stats(euclidean_errors)
            }

            self._print_position_stats('TARGET', statistics['target'])
        else:
            self.logger.warning('\nNo successful TARGET measurements for statistics')

        # Calculate HOME statistics
        if successful_home:
            errors_x = [m['errors']['error_x'] for m in successful_home]
            errors_y = [m['errors']['error_y'] for m in successful_home]
            errors_z = [m['errors']['error_z'] for m in successful_home]
            euclidean_errors = [m['errors']['euclidean_error'] for m in successful_home]

            statistics['home'] = {
                'count': len(successful_home),
                'error_x': self.compute_stats(errors_x),
                'error_y': self.compute_stats(errors_y),
                'error_z': self.compute_stats(errors_z),
                'euclidean_error': self.compute_stats(euclidean_errors)
            }

            self._print_position_stats('HOME', statistics['home'])
        else:
            self.logger.warning('\nNo successful HOME measurements for statistics')

        return statistics

    def calculate_multi_target_statistics(self, measurements: List[Dict],
                                          targets: List[Dict]) -> Dict:
        """
        Calculate statistics for multi-target calibration.

        Args:
            measurements: List of measurement dictionaries
            targets: List of target information dictionaries

        Returns:
            Dictionary with 'per_target' and 'global' statistics
        """
        # Initialize per-target statistics
        per_target_stats = {}
        for target_info in targets:
            target_name = target_info['name']
            per_target_stats[target_name] = {
                'index': targets.index(target_info),
                'position': target_info['position'],
                'measurements': []
            }

        # Group measurements by target
        for m in measurements:
            if m.get('target_measurement', {}).get('pose_success', False):
                target_name = m['target_name']
                if target_name in per_target_stats:
                    per_target_stats[target_name]['measurements'].append(
                        m['target_measurement']
                    )

        # Calculate statistics for each target
        self.logger.info('\n' + '=' * 70)
        self.logger.info('PER-TARGET CALIBRATION STATISTICS')
        self.logger.info('=' * 70)

        target_summary = []
        for target_name, data in per_target_stats.items():
            if data['measurements']:
                errors_x = [m['errors']['error_x'] for m in data['measurements']]
                errors_y = [m['errors']['error_y'] for m in data['measurements']]
                errors_z = [m['errors']['error_z'] for m in data['measurements']]
                euclidean_errors = [m['errors']['euclidean_error'] for m in data['measurements']]

                stats = {
                    'count': len(data['measurements']),
                    'error_x': self.compute_stats(errors_x),
                    'error_y': self.compute_stats(errors_y),
                    'error_z': self.compute_stats(errors_z),
                    'euclidean_error': self.compute_stats(euclidean_errors)
                }

                data['statistics'] = stats

                self.logger.info(f'\n{target_name.upper()} - {stats["count"]} measurements')
                self.logger.info(f'  Position: ({data["position"]["x"]:.4f}, '
                               f'{data["position"]["y"]:.4f}, '
                               f'{data["position"]["z"]:.4f})')
                self.logger.info(f'  Mean Errors: X={stats["error_x"]["mean"]:+.6f}, '
                               f'Y={stats["error_y"]["mean"]:+.6f}, '
                               f'Z={stats["error_z"]["mean"]:+.6f}')
                self.logger.info(f'  Mean Euclidean Error: {stats["euclidean_error"]["mean"]:.6f}')
                self.logger.info(f'  Std Dev: X={stats["error_x"]["std"]:.6f}, '
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

        global_stats = None
        if all_target_measurements:
            errors_x = [m['errors']['error_x'] for m in all_target_measurements]
            errors_y = [m['errors']['error_y'] for m in all_target_measurements]
            errors_z = [m['errors']['error_z'] for m in all_target_measurements]
            euclidean_errors = [m['errors']['euclidean_error'] for m in all_target_measurements]

            global_stats = {
                'count': len(all_target_measurements),
                'error_x': self.compute_stats(errors_x),
                'error_y': self.compute_stats(errors_y),
                'error_z': self.compute_stats(errors_z),
                'euclidean_error': self.compute_stats(euclidean_errors)
            }

            self.logger.info('\n' + '=' * 70)
            self.logger.info('GLOBAL STATISTICS (All Targets Combined)')
            self.logger.info('=' * 70)
            self.logger.info(f'Total successful measurements: {global_stats["count"]}')
            self.logger.info(f'Mean Errors: X={global_stats["error_x"]["mean"]:+.6f}, '
                           f'Y={global_stats["error_y"]["mean"]:+.6f}, '
                           f'Z={global_stats["error_z"]["mean"]:+.6f}')
            self.logger.info(f'Mean Euclidean Error: {global_stats["euclidean_error"]["mean"]:.6f}')
            self.logger.info(f'Std Dev: X={global_stats["error_x"]["std"]:.6f}, '
                           f'Y={global_stats["error_y"]["std"]:.6f}, '
                           f'Z={global_stats["error_z"]["std"]:.6f}')
            self.logger.info(f'RMS: X={global_stats["error_x"]["rms"]:.6f}, '
                           f'Y={global_stats["error_y"]["rms"]:.6f}, '
                           f'Z={global_stats["error_z"]["rms"]:.6f}')

        # Display target comparison
        if target_summary:
            self._print_target_comparison(target_summary)

        return {
            'per_target': per_target_stats,
            'global': global_stats
        }

    def _print_position_stats(self, position_name: str, stats: Dict):
        """Print statistics for a single position (target or home)."""
        self.logger.info(f'\n{position_name} POSITION - Successful measurements: {stats["count"]}')
        for axis in ['error_x', 'error_y', 'error_z', 'euclidean_error']:
            s = stats[axis]
            self.logger.info(f'\n  {axis.upper()}:')
            self.logger.info(f'    Mean: {s["mean"]:.6f}')
            self.logger.info(f'    Std Dev: {s["std"]:.6f}')
            self.logger.info(f'    Min: {s["min"]:.6f}')
            self.logger.info(f'    Max: {s["max"]:.6f}')
            self.logger.info(f'    RMS: {s["rms"]:.6f}')

    def _print_target_comparison(self, target_summary: List[Dict]):
        """Print comparison of all targets sorted by error."""
        self.logger.info('\n' + '=' * 70)
        self.logger.info('TARGET COMPARISON (Sorted by Mean Euclidean Error)')
        self.logger.info('=' * 70)

        target_summary.sort(key=lambda x: x['mean_euclidean_error'])
        for i, t in enumerate(target_summary):
            self.logger.info(f'{i+1}. {t["name"]}: {t["mean_euclidean_error"]:.6f}m '
                           f'(X:{t["mean_x"]:+.6f}, Y:{t["mean_y"]:+.6f}, '
                           f'Z:{t["mean_z"]:+.6f})')

        best = target_summary[0]
        worst = target_summary[-1]
        self.logger.info(f'\nBest: {best["name"]} ({best["mean_euclidean_error"]:.6f}m)')
        self.logger.info(f'Worst: {worst["name"]} ({worst["mean_euclidean_error"]:.6f}m)')
