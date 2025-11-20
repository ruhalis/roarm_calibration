"""
Configuration Loader Module

Handles loading and validation of calibration configuration files.
"""

import yaml
from typing import List, Dict, Tuple, Optional


class ConfigLoader:
    """
    Loads and validates calibration configuration files.
    """

    @staticmethod
    def load_targets_config(config_path: str) -> Tuple[List[Dict], Dict, Dict, Optional[Dict]]:
        """
        Load calibration targets from YAML config file.

        Args:
            config_path: Path to YAML configuration file

        Returns:
            Tuple of (targets_list, home_dict, settings_dict, workspace_limits)
            where:
                - targets_list: List of target position dictionaries
                - home_dict: Home position dictionary with 'x', 'y', 'z' keys
                - settings_dict: Calibration settings dictionary
                - workspace_limits: Optional workspace limits dictionary
        """
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        targets = config.get('targets', [])
        home = config.get('home_position', {'x': 0.2, 'y': 0.0, 'z': 0.1})
        settings = config.get('calibration_settings', {})
        workspace_limits = config.get('workspace_limits', None)

        return targets, home, settings, workspace_limits

    @staticmethod
    def validate_config(config_path: str) -> Tuple[bool, str]:
        """
        Validate configuration file format and content.

        Args:
            config_path: Path to YAML configuration file

        Returns:
            Tuple of (is_valid, error_message)
        """
        try:
            targets, home, settings, limits = ConfigLoader.load_targets_config(config_path)

            # Validate home position
            if not all(key in home for key in ['x', 'y', 'z']):
                return False, "Home position must have 'x', 'y', 'z' keys"

            # Validate targets
            if not targets:
                return False, "No targets defined in configuration"

            for i, target in enumerate(targets):
                if not all(key in target for key in ['x', 'y', 'z']):
                    return False, f"Target {i+1} must have 'x', 'y', 'z' keys"

            # Validate workspace limits if present
            if limits is not None:
                required_keys = ['x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max']
                if not all(key in limits for key in required_keys):
                    return False, "Workspace limits must have all min/max keys for x, y, z"

            return True, "Configuration valid"

        except Exception as e:
            return False, f"Configuration validation error: {str(e)}"

    @staticmethod
    def get_default_home() -> Dict[str, float]:
        """
        Get default home position.

        Returns:
            Dictionary with default home position
        """
        return {'x': 0.2, 'y': 0.0, 'z': 0.1}

    @staticmethod
    def get_default_workspace_limits() -> Dict[str, float]:
        """
        Get default workspace limits for safety validation.

        Returns:
            Dictionary with default workspace limits
        """
        return {
            'x_min': 0.10, 'x_max': 0.30,
            'y_min': -0.15, 'y_max': 0.15,
            'z_min': -0.15, 'z_max': 0.15
        }

    @staticmethod
    def get_default_settings() -> Dict:
        """
        Get default calibration settings.

        Returns:
            Dictionary with default settings
        """
        return {
            'settling_time': 1.0,
            'default_loops': 1,
            'movement_timeout': 10.0,
            'retry_on_failure': True,
            'max_retries': 2
        }
