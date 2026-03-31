// Copyright (c) 2026 Franka Robotics GmbH
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

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <franka/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "franka_semantic_components/franka_robot_model.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * Ultrasound Scanning Impedance Controller
 *
 * Cartesian impedance controller for robotic ultrasound scanning with two phases.
 * Both translational and rotational control are defined in the flange/probe frame.
 *
 * Phase 1 – Force Control & Planar Compliance (Translation, flange frame)
 *   • Normal (flange Z): Constant contact force regulation via outer-loop admittance
 *                        (F_normal_des = 5 N pressing into tissue).
 *   • Tangent (flange X/Y): Near-zero stiffness with moderate damping for
 *                           operator hand-guiding across the patient surface.
 *
 * Phase 2 – Selective Rotational Compliance (Orientation, flange frame)
 *   • Roll  (flange X): Free – near-zero stiffness to adapt probe to body curvature.
 *   • Pitch (flange Y): Locked to initial orientation at activation.
 *   • Yaw   (flange Z): Locked to initial orientation to prevent spinning.
 *
 * Stiffness matrix K = diag(K_tx, K_ty, K_tn, K_rx, K_ry, K_rz)  [flange frame]
 *   Defaults: (0, 0, 400, 0, 30, 30)
 *
 * Damping matrix D = diag(D_tx, D_ty, D_tn, D_rx, D_ry, D_rz)  [flange frame]
 *   Defaults: (10, 10, 70, 1, 6, 6)
 *
 * All translational and rotational stiffness/damping are applied in the flange
 * frame and rotated back to the world frame before the J^T torque mapping.
 */
class UltrasoundScanningImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix6x7d = Eigen::Matrix<double, 6, 7>;
  using Matrix7x6d = Eigen::Matrix<double, 7, 6>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;

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
  void update_joint_states();
  bool read_robot_state_pointer();
  void initialize_targets();

  Vector7d saturate_torque_rate(const Vector7d& tau_desired) const;

  std::string robot_type_;
  std::string arm_prefix_;

  static constexpr int kNumJoints = 7;
  static constexpr int kCartesianDim = 6;

  Vector7d q_{};
  Vector7d dq_{};
  Vector7d tau_previous_{};
  Vector7d q_nullspace_target_{};

  Vector3d position_initial_{};
  Vector3d position_desired_{};
  Matrix6x7d jacobian_{};
  Eigen::Matrix3d rotation_initial_{Eigen::Matrix3d::Identity()};

  // Admittance state for normal-axis force control (flange +Z, pressing-positive).
  double normal_pos_initial_{0.0};
  double normal_pos_desired_{0.0};

  // Low-pass filtered external force along flange +Z (pressing-positive).
  double force_normal_filtered_{0.0};

  franka::RobotState* robot_state_ptr_{nullptr};
  std::size_t robot_state_interface_index_{0};

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  // --- Phase 1: Translational gains (flange frame) ---
  // Tangent (flange X/Y): near-zero stiffness for hand-guiding.
  // Normal  (flange Z):   high stiffness for admittance force-tracking loop.
  Vector3d translational_stiffness_{0.0, 0.0, 400.0};
  Vector3d translational_damping_{10.0, 10.0, 70.0};

  // --- Phase 2: Rotational gains (applied in flange frame) ---
  // Roll (X) free, Pitch (Y) and Yaw (Z) locked.
  Vector3d rotational_stiffness_{0.0, 30.0, 30.0};
  Vector3d rotational_damping_{1.0, 6.0, 6.0};

  // --- Force control / admittance parameters ---
  double force_normal_desired_{5.0};   // Desired normal contact force [N] (pressing-positive)
  double force_filter_cutoff_hz_{20.0};
  double admittance_gain_{2e-4};       // m/s per N of force error
  double normal_v_max_{0.02};          // Max admittance velocity [m/s]
  double normal_max_disp_{0.02};       // Max admittance displacement [m]

  // --- Nullspace stabilization ---
  double nullspace_stiffness_{15.0};
  double nullspace_damping_{3.0};

  // --- Safety / saturation ---
  double max_force_xy_{30.0};
  double max_force_z_{80.0};
  double max_torque_xyz_{20.0};
  double max_joint_torque_{80.0};
  double max_delta_tau_{1.0};

  // --- Runtime monitoring publisher ---
  std::string status_topic_{"ultrasound_scanning_impedance/status"};
  double status_publish_rate_hz_{20.0};
  double status_publish_accumulator_{0.0};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_publisher_;

  bool initialized_{false};
};

}  // namespace franka_example_controllers
