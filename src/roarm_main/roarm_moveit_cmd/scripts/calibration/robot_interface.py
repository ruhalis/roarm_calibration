"""
Robot Interface Module

Handles ROS2 communication with the robot arm for calibration.
"""

import sys
import rclpy
from rclpy.node import Node
from roarm_moveit.srv import MovePointCmd, GetPoseCmd
from typing import Tuple


class RobotInterface:
    """
    Interface for communicating with the robot arm via ROS2 services.

    Provides methods for moving the robot and reading its position.
    """

    def __init__(self, node: Node):
        """
        Initialize robot interface.

        Args:
            node: ROS2 node for creating service clients
        """
        self.node = node
        self.logger = node.get_logger()

        # Create service clients
        self.move_client = node.create_client(MovePointCmd, '/move_point_cmd')
        self.get_pose_client = node.create_client(GetPoseCmd, '/get_pose_cmd')

    def wait_for_services(self, timeout_sec: float = 10.0) -> bool:
        """
        Wait for required ROS2 services to become available.

        Args:
            timeout_sec: Maximum time to wait for services

        Returns:
            True if all services are available, False otherwise
        """
        self.logger.info('Waiting for /move_point_cmd service...')
        if not self.move_client.wait_for_service(timeout_sec=timeout_sec):
            self.logger.error('Service /move_point_cmd not available!')
            return False

        self.logger.info('Waiting for /get_pose_cmd service...')
        if not self.get_pose_client.wait_for_service(timeout_sec=timeout_sec):
            self.logger.error('Service /get_pose_cmd not available!')
            return False

        self.logger.info('All services available.')
        return True

    def move_to_position(self, x: float, y: float, z: float,
                        timeout: float = 10.0) -> Tuple[bool, str]:
        """
        Move robot to specified position.

        Args:
            x, y, z: Target coordinates in meters
            timeout: Movement timeout in seconds

        Returns:
            Tuple of (success, message)
        """
        request = MovePointCmd.Request()
        request.x = x
        request.y = y
        request.z = z

        self.logger.info(f'Moving to position: x={x:.4f}, y={y:.4f}, z={z:.4f}')

        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

        if future.result() is not None:
            response = future.result()
            return response.success, response.message
        else:
            return False, 'Service call failed or timed out'

    def get_current_pose(self) -> Tuple[bool, float, float, float]:
        """
        Get current robot end-effector pose.

        Returns:
            Tuple of (success, x, y, z) where x, y, z are in meters
        """
        request = GetPoseCmd.Request()

        future = self.get_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            response = future.result()
            return True, response.x, response.y, response.z
        else:
            return False, 0.0, 0.0, 0.0
