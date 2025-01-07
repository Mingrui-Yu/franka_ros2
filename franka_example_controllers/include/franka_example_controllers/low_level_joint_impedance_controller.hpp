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

#pragma once

#include <string>
#include <mutex>
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "motion_generator.hpp"
#include <nlohmann/json.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The joint impedance example controller moves joint 4 and 5 in a very compliant periodic movement.
 */
class LowLevelJointImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  

 private:
  std::mutex mtx;
  std::string arm_id_;
  const int num_joints = 7;
  Vector7d q_;
  Vector7d initial_q_;
  Vector7d q_goal_; // high-level goal join position
  Vector7d q_desired_l_; // last
  Vector7d q_desired_ll_; // last last
  Vector7d q_desired_lll_; // last last last
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  double elapsed_time_{0.0};
//   double save_time_{0.0};

  // for subscribing the high-level commands
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscriber_;
  Vector7d high_level_command_;
  Vector7d joint_pos_min_, joint_pos_max_;
  Vector7d joint_vel_max_; 
//   Vector7d joint_acc_max_, joint_jerk_max_;
  
  // save data
//   nlohmann::json json_file;

  void updateJointStates();
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace franka_example_controllers
