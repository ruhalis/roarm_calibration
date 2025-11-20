"""
I/O Handler Module

Handles saving calibration results to JSON and CSV files.
"""

import json
import csv
from typing import Dict


class ResultsWriter:
    """
    Handles writing calibration results to various file formats.
    """

    @staticmethod
    def save_json(results: Dict, filename: str):
        """
        Save calibration results to JSON file.

        Args:
            results: Results dictionary
            filename: Output filename
        """
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        print(f'Results saved to JSON: {filename}')

    @staticmethod
    def save_csv(results: Dict, filename: str):
        """
        Save calibration results to CSV file.
        Automatically detects single-target vs multi-target mode.

        Args:
            results: Results dictionary
            filename: Output filename
        """
        mode = results.get('mode', 'single_target')

        if mode == 'multi_target':
            ResultsWriter._save_csv_multi(results, filename)
        else:
            ResultsWriter._save_csv_single(results, filename)

        print(f'Results saved to CSV: {filename}')

    @staticmethod
    def _save_csv_single(results: Dict, filename: str):
        """Save single-target calibration results to CSV."""
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
            ResultsWriter._write_measurements_table(
                writer, results['measurements'], 'TARGET', 'target_measurement'
            )

            writer.writerow([])

            # Write HOME measurements
            ResultsWriter._write_measurements_table(
                writer, results['measurements'], 'HOME', 'home_measurement'
            )

            # Write statistics
            if 'statistics' in results:
                writer.writerow([])
                writer.writerow(['STATISTICS'])

                if 'target' in results['statistics']:
                    ResultsWriter._write_stats_table(
                        writer, results['statistics']['target'], 'TARGET POSITION'
                    )

                if 'home' in results['statistics']:
                    ResultsWriter._write_stats_table(
                        writer, results['statistics']['home'], 'HOME POSITION'
                    )

    @staticmethod
    def _save_csv_multi(results: Dict, filename: str):
        """Save multi-target calibration results to CSV."""
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
                writer.writerow(['Target Name', 'Count', 'Mean Error X', 'Mean Error Y',
                               'Mean Error Z', 'Mean Euclidean', 'Std X', 'Std Y', 'Std Z',
                               'RMS Euclidean'])

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
                ResultsWriter._write_stats_table(
                    writer, results['statistics']['global'],
                    'GLOBAL STATISTICS (All Targets Combined)'
                )

    @staticmethod
    def _write_measurements_table(writer, measurements: list, position_name: str,
                                  measurement_key: str):
        """Write measurements table to CSV."""
        writer.writerow([f'{position_name} POSITION MEASUREMENTS'])
        writer.writerow(['Iteration', 'Move Success', 'Pose Success',
                        'Actual X', 'Actual Y', 'Actual Z',
                        'Error X', 'Error Y', 'Error Z', 'Euclidean Error'])

        for m in measurements:
            data = m.get(measurement_key, {})
            if data.get('pose_success', False):
                writer.writerow([
                    m.get('iteration', m.get('measurement_number', '')),
                    data.get('move_success', False),
                    data.get('pose_success', False),
                    data['actual']['x'],
                    data['actual']['y'],
                    data['actual']['z'],
                    data['errors']['error_x'],
                    data['errors']['error_y'],
                    data['errors']['error_z'],
                    data['errors']['euclidean_error']
                ])
            else:
                writer.writerow([
                    m.get('iteration', m.get('measurement_number', '')),
                    data.get('move_success', False),
                    data.get('pose_success', False),
                    '', '', '', '', '', '', ''
                ])

    @staticmethod
    def _write_stats_table(writer, stats: Dict, title: str):
        """Write statistics table to CSV."""
        writer.writerow([])
        writer.writerow([f'{title} STATISTICS'])
        writer.writerow(['Metric', 'Mean', 'Std Dev', 'Min', 'Max', 'RMS'])

        for metric in ['error_x', 'error_y', 'error_z', 'euclidean_error']:
            if metric in stats:
                s = stats[metric]
                writer.writerow([
                    metric.upper(),
                    s['mean'],
                    s['std'],
                    s['min'],
                    s['max'],
                    s['rms']
                ])
