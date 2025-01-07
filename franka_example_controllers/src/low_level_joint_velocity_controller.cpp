// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <franka_example_controllers/low_level_joint_velocity_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>



using namespace std::chrono_literals;

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
LowLevelJointVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
LowLevelJointVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type LowLevelJointVelocityController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {

  // execute the subscribed high-level command
  Eigen::VectorXf delta_joint_vel = high_level_command_ - last_joint_vel_command_;
  Eigen::VectorXf last_delta_joint_vel = last_joint_vel_command_ - last_last_joint_vel_command_;

  float max_step = 0.0f; 
  for (int j = 0; j < num_joints; j++){
    float dt = period.seconds();
    float delta_joint_vel_ub = std::min(dt * joint_acc_max_(j), last_delta_joint_vel(j) + 1.0f/2.0f*dt*dt*joint_jerk_max_(j));
    float delta_joint_vel_lb = std::max(-dt * joint_acc_max_(j), last_delta_joint_vel(j) - 1.0f/2.0f*dt*dt*joint_jerk_max_(j));

    // std::cout << "j: " << j << std::endl;
    // std::cout << "dt * joint_acc_max_(j): " << dt * joint_acc_max_(j) << std::endl;
    // std::cout << "last_delta_joint_vel(j): " << last_delta_joint_vel(j) << std::endl;
    // std::cout << "dt*dt*last_delta_joint_vel(j): " << dt*dt*last_delta_joint_vel(j) << std::endl;

    float step = 0.0f;
    if (delta_joint_vel(j) >= 0){
      step = std::abs(delta_joint_vel(j)) / delta_joint_vel_ub;
    }else{
      step = std::abs(delta_joint_vel(j)) / delta_joint_vel_lb;
    }
    max_step = std::max(std::max(1.0f, step), max_step);
  }
  float t_interp = 1.0f / max_step;

  Eigen::VectorXf new_joint_vel_command = (1.0 - t_interp) * last_joint_vel_command_ 
                                          + t_interp * high_level_command_;

  for (int j = 0; j < num_joints; j++){
    command_interfaces_[j].set_value(new_joint_vel_command(j));
  }
  last_last_joint_vel_command_ = last_joint_vel_command_;
  last_joint_vel_command_ = new_joint_vel_command;

  return controller_interface::return_type::OK;
}

void LowLevelJointVelocityController::command_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (static_cast<int>(msg->data.size()) == num_joints) {
    // Process the 7-dimensional vector
    RCLCPP_INFO(get_node()->get_logger(), "Received vector: [%f, %f, %f, %f, %f, %f, %f]",
                msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6]);
    for(int j = 0; j < num_joints; j++)
      high_level_command_(j) = msg->data[j];
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Received vector with unexpected size: %zu", msg->data.size());
  }
}

CallbackReturn LowLevelJointVelocityController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn LowLevelJointVelocityController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(1000ms);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  // initialize the high-level command subscriber
  joint_acc_max_ = Eigen::VectorXf::Zero(num_joints);
  joint_jerk_max_ = Eigen::VectorXf::Zero(num_joints);
  joint_acc_max_ << 15, 7.5, 10, 12.5, 15, 20, 20; // https://frankaemika.github.io/docs/control_parameters.html#limit-table
  joint_jerk_max_ << 7500, 3750, 5000, 6250, 7500, 10000, 10000;
  joint_acc_max_ = joint_acc_max_ * 0.5; // By experiments, 0.5 is a relatively safe value to satisfy the continuity requirements of franka.
  joint_jerk_max_ = joint_jerk_max_ * 0.5;

  high_level_command_ = Eigen::VectorXf::Zero(num_joints);
  last_joint_vel_command_ = Eigen::VectorXf::Zero(num_joints);
  last_last_joint_vel_command_ = Eigen::VectorXf::Zero(num_joints);

  command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "franka/joint_velocity_command", 10, std::bind(&LowLevelJointVelocityController::command_callback, this, std::placeholders::_1));
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn LowLevelJointVelocityController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::LowLevelJointVelocityController,
                       controller_interface::ControllerInterface)
