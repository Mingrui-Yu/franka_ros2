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

#include <franka_example_controllers/low_level_joint_impedance_controller.hpp>
#include <fstream>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>


namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
LowLevelJointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
LowLevelJointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type LowLevelJointImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period.seconds(); // not used
  // save_time_ = save_time_ + period.seconds();

  updateJointStates();

  Vector7d delta_q = q_goal_ - q_desired_l_;
  double dt = period.seconds();
  Vector7d ub_vel = dt * joint_vel_max_;
  Vector7d lb_vel = - dt * joint_vel_max_;
  // Vector7d ub_acc = (q_desired_l_ - q_desired_ll_) + 1.0/2.0 * dt * dt * joint_acc_max_;
  // Vector7d lb_acc = (q_desired_l_ - q_desired_ll_) - 1.0/2.0 * dt * dt * joint_acc_max_;
  // Vector7d ub_jerk = (q_desired_l_ - q_desired_ll_) 
  //           + 1.0/2.0 * (q_desired_l_ - 2*q_desired_ll_ + q_desired_lll_) 
  //           + 1.0/6.0 * std::pow(dt, 3) * joint_jerk_max_;
  // Vector7d lb_jerk = (q_desired_l_ - q_desired_ll_) 
  //           + 1.0/2.0 * (q_desired_l_ - 2*q_desired_ll_ + q_desired_lll_) 
  //           - 1.0/6.0 * std::pow(dt, 3) * joint_jerk_max_;

  // Vector7d ub_acc = 10.0/2.0 * dt * dt * joint_acc_max_;
  // Vector7d lb_acc = -10.0/2.0 * dt * dt * joint_acc_max_;
  // Vector7d ub_jerk = 10.0/6.0 * std::pow(dt, 3) * joint_jerk_max_;
  // Vector7d lb_jerk = -10.0/6.0 * std::pow(dt, 3) * joint_jerk_max_;

  double min_t_interp = 1.0;
  for (int j = 0; j < num_joints; j++){
    // double delta_q_lb = std::max(std::max(lb_vel(j), lb_acc(j)), lb_jerk(j));
    // double delta_q_ub = std::min(std::min(ub_vel(j), ub_acc(j)), ub_jerk(j));
    double delta_q_lb = lb_vel(j);
    double delta_q_ub = ub_vel(j);

    double t_ub;
    if (delta_q(j) >= 0){
      t_ub = delta_q_ub / delta_q(j);
    }else{
      t_ub = delta_q_lb / delta_q(j);
    }
    min_t_interp = std::min(min_t_interp, t_ub);
  }
  double t_interp = std::min(std::max(min_t_interp, 0.0), 1.0); // 0 ~ 1

  Vector7d q_desired = (1.0 - t_interp) * q_desired_l_ + t_interp * q_goal_;
  q_desired = q_desired.cwiseMax(joint_pos_min_).cwiseMin(joint_pos_max_);

  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_desired - q_) + d_gains_.cwiseProduct(-dq_filtered_);
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }

  q_desired_lll_ = q_desired_ll_;
  q_desired_ll_ = q_desired_l_;
  q_desired_l_ = q_desired;

  // // save data
  // Vector7d vec;
  // vec = q_goal_;
  // json_file["q_goal"].push_back(std::vector<double>(vec.data(), vec.data() + vec.size()));
  // vec = q_;
  // json_file["q"].push_back(std::vector<double>(vec.data(), vec.data() + vec.size()));
  // json_file["t_interp"].push_back(t_interp);
  // json_file["elapsed_time_"].push_back(elapsed_time_);


  // if (save_time_ >= 0.2){
  //   std::ofstream outfile("data.json");
  //   outfile << json_file.dump(4);  // 4 是缩进值，用于格式化 JSON 数据
  //   outfile.close();
  //   save_time_ = 0.0;
  // }

  return controller_interface::return_type::OK;
}

void LowLevelJointImpedanceController::command_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (static_cast<int>(msg->data.size()) == num_joints) {
    // Process the 7-dimensional vector
    RCLCPP_INFO(get_node()->get_logger(), "Received vector: [%f, %f, %f, %f, %f, %f, %f]",
                msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6]);
    for(int j = 0; j < num_joints; j++){
      high_level_command_(j) = msg->data[j];
    }
    q_goal_ = high_level_command_;
  }
  else {
    RCLCPP_WARN(get_node()->get_logger(), "Received vector with unexpected size: %zu", msg->data.size());
  }
}

void LowLevelJointImpedanceController::set_stiffness_callback(
      const std::shared_ptr<franka_msgs::srv::SetJointStiffnessDamping::Request> request,
      std::shared_ptr<franka_msgs::srv::SetJointStiffnessDamping::Response>      response)
  {
    for (int i = 0; i < num_joints; ++i) {
      d_gains_(i) = request->joint_damping[i];
      k_gains_(i) = request->joint_stiffness[i];
    }
    response->success = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully reset joint stiffnesss and damping.");
  }

CallbackReturn LowLevelJointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn LowLevelJointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();

  // https://frankaemika.github.io/docs/control_parameters.html#limit-table
  joint_pos_min_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  joint_pos_max_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  joint_vel_max_ << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;
  // joint_acc_max_ << 15, 7.5, 10, 12.5, 15, 20, 20; 
  // joint_jerk_max_ << 7500, 3750, 5000, 6250, 7500, 10000, 10000;
  
  joint_vel_max_ = joint_vel_max_ * 0.8;
  // joint_acc_max_ = joint_acc_max_ * 0.5;
  // joint_jerk_max_ = joint_jerk_max_ * 0.5;

  stiffness_service_ = get_node()->create_service<franka_msgs::srv::SetJointStiffnessDamping>(
    "franka/set_joint_stiffness", std::bind(&LowLevelJointImpedanceController::set_stiffness_callback, this, std::placeholders::_1, std::placeholders::_2));

  command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "franka/joint_impedance_command", 10, std::bind(&LowLevelJointImpedanceController::command_callback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn LowLevelJointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  dq_filtered_.setZero();
  elapsed_time_ = 0.0;
  initial_q_ = q_;

  q_goal_ = initial_q_;
  q_desired_l_ = q_desired_ll_ = q_desired_lll_ = initial_q_;

  return CallbackReturn::SUCCESS;
}

void LowLevelJointImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::LowLevelJointImpedanceController,
                       controller_interface::ControllerInterface)
