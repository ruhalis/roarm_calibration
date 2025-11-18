#include <memory>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "roarm_moveit/srv/move_point_cmd.hpp"
#include "roarm_moveit_cmd/ik.h"

// Calibration offset parameters (loaded from config file)
struct CalibrationOffsets {
  bool enabled = false;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};
void handle_service(const std::shared_ptr<roarm_moveit::srv::MovePointCmd::Request> request,
                    std::shared_ptr<roarm_moveit::srv::MovePointCmd::Response> response)
{
  // 在这里处理服务请求
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("move_point_cmd_service_node");
  auto logger = node->get_logger();

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

  // 创建 MoveIt MoveGroup 接口
  moveit::planning_interface::MoveGroupInterface move_group(node, "hand");

  // 设置目标位置和姿态
  geometry_msgs::msg::Pose target_pose;

  // Apply calibration offsets if enabled
  double corrected_x = request->x;
  double corrected_y = request->y;
  double corrected_z = request->z;

  if (calib.enabled) {
    corrected_x = request->x - calib.x;
    corrected_y = request->y - calib.y;
    corrected_z = request->z - calib.z;

    RCLCPP_INFO(logger, "Calibration enabled: offsets [%.5f, %.5f, %.5f]", calib.x, calib.y, calib.z);
    RCLCPP_INFO(logger, "Original command: [%.4f, %.4f, %.4f]", request->x, request->y, request->z);
    RCLCPP_INFO(logger, "Corrected target: [%.4f, %.4f, %.4f]", corrected_x, corrected_y, corrected_z);
  }

  target_pose.position.x = corrected_x;
  target_pose.position.y = -1 * corrected_y;
  target_pose.position.z = corrected_z;

  cartesian_to_polar(1000*target_pose.position.x,1000*target_pose.position.y, &base_r, &BASE_point_RAD);
  simpleLinkageIkRad(l2, l3, base_r, 1000*target_pose.position.z);
  
  RCLCPP_INFO(logger, "BASE_point_RAD: %f, SHOULDER_point_RAD: %f, ELBOW_point_RAD: %f", BASE_point_RAD, -SHOULDER_point_RAD, ELBOW_point_RAD);
  RCLCPP_INFO(logger, "x: %f, y: %f, z: %f", request->x, request->y, request->z);
  // 定义目标关节值
  std::vector<double> target = {BASE_point_RAD, -SHOULDER_point_RAD, ELBOW_point_RAD};

  // Set tighter tolerances for improved accuracy (1-2mm)
  move_group.setGoalJointTolerance(0.002);  // 0.002 rad ≈ 0.11° → ~1-2mm error
  move_group.setNumPlanningAttempts(10);
  move_group.setPlanningTime(5.0);

  move_group.setJointValueTarget(target);
  // 规划并执行轨迹
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success) {
    move_group.execute(my_plan);
    response->success = true;
    response->message = "MovePointCmd executed successfully";
    RCLCPP_INFO(logger, "MovePointCmd service executed successfully");
  } else {
    response->success = false;
    response->message = "Planning failed!";
    RCLCPP_ERROR(logger, "Planning failed!");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("move_point_cmd_node");

  // 创建服务
  auto server = node->create_service<roarm_moveit::srv::MovePointCmd>("move_point_cmd", &handle_service);

  RCLCPP_INFO(node->get_logger(), "MovePointCmd service is ready.");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

