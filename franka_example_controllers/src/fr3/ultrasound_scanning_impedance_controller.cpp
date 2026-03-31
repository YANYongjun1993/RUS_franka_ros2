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

#include <franka_example_controllers/fr3/ultrasound_scanning_impedance_controller.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include <franka/model.h>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "pluginlib/class_list_macros.hpp"

namespace {

template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable<From>::value &&
                     std::is_trivially_copyable<To>::value,
                 To>
bit_cast(const From& src) noexcept {
  static_assert(std::is_trivially_constructible<To>::value,
                "This implementation additionally requires "
                "destination type to be trivially constructible");

  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

double clamp_abs(double value, double abs_limit) {
  return std::clamp(value, -abs_limit, abs_limit);
}

}  // namespace

namespace franka_example_controllers {

// ---------------------------------------------------------------------------
// Interface configuration
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
UltrasoundScanningImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration
UltrasoundScanningImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= kNumJoints; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/velocity");
  }

  const auto model_interfaces = franka_robot_model_->get_state_interface_names();
  config.names.insert(config.names.end(), model_interfaces.begin(), model_interfaces.end());

  return config;
}

// ---------------------------------------------------------------------------
// Real-time update loop
// ---------------------------------------------------------------------------

controller_interface::return_type UltrasoundScanningImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  if (!read_robot_state_pointer()) {
    return controller_interface::return_type::ERROR;
  }

  if (!initialized_) {
    initialize_targets();
  }

  update_joint_states();

  // ---- Dynamics ----
  const auto coriolis_array = franka_robot_model_->getCoriolisForceVector();
  const Vector7d coriolis(coriolis_array.data());

  const auto jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kFlange);
  jacobian_ = Eigen::Map<const Matrix6x7d>(jacobian_array.data());

  const auto flange_pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kFlange);
  const Eigen::Map<const Eigen::Matrix4d> flange_pose(flange_pose_array.data());
  const Eigen::Vector3d position = flange_pose.block<3, 1>(0, 3);
  const Eigen::Matrix3d rotation = flange_pose.block<3, 3>(0, 0);

  const Vector6d cartesian_velocity = jacobian_ * dq_;
  const Vector3d linear_velocity = cartesian_velocity.head<3>();
  const Vector3d angular_velocity = cartesian_velocity.tail<3>();

  // ===========================================================================
  // Phase 1: Z-axis force control via admittance + X/Y planar compliance
  // ===========================================================================

  // External wrench in world frame; convert to downward-positive scalar.
  const double force_down_measured = -robot_state_ptr_->O_F_ext_hat_K[2];
  const double dt = period.seconds();

  // First-order low-pass filter on measured force.
  const double tau_filter = 1.0 / (2.0 * M_PI * std::max(1e-3, force_filter_cutoff_hz_));
  const double alpha = dt / (tau_filter + dt);
  force_down_filtered_ += alpha * (force_down_measured - force_down_filtered_);

  // Admittance outer loop: adjust desired Z position to track F_z,des.
  //   x_z_delta = K_z^{-1} * (F_z_meas - F_z_des)   (conceptually)
  // Implemented as velocity-level admittance for smooth transitions:
  const double force_error = force_z_desired_down_ - force_down_filtered_;
  const double down_velocity =
      clamp_abs(admittance_gain_ * force_error, std::max(1e-6, vz_max_));
  down_pos_desired_ += dt * down_velocity;
  down_pos_desired_ =
      std::clamp(down_pos_desired_, down_pos_initial_ - z_max_, down_pos_initial_ + z_max_);

  // Update the desired position (only Z moves via admittance; X/Y track initial).
  position_desired_.z() = -down_pos_desired_;

  // Translational position error (world frame).
  const Vector3d position_error = position_desired_ - position;

  // ===========================================================================
  // Phase 2: Orientation control with selective compliance in flange frame
  // ===========================================================================

  // Compute orientation error in world frame using the standard cross-product form.
  const Vector3d orientation_error_world =
      0.5 * (rotation.col(0).cross(rotation_initial_.col(0)) +
             rotation.col(1).cross(rotation_initial_.col(1)) +
             rotation.col(2).cross(rotation_initial_.col(2)));

  // Transform orientation error and angular velocity into the flange frame
  // so that selective stiffness axes (roll/pitch/yaw) follow the end-effector.
  const Vector3d orientation_error_flange = rotation.transpose() * orientation_error_world;
  const Vector3d angular_velocity_flange = rotation.transpose() * angular_velocity;

  // Apply per-axis stiffness/damping in flange frame:
  //   Roll  (X): K_rx ≈ 0   →  free rotation to adapt to body curvature
  //   Pitch (Y): K_ry = 30  →  locked to initial
  //   Yaw   (Z): K_rz = 30  →  locked to prevent spinning
  const Vector3d torque_flange =
      rotational_stiffness_.cwiseProduct(orientation_error_flange) -
      rotational_damping_.cwiseProduct(angular_velocity_flange);

  // Rotate the orientation torque back into world frame for J^T mapping.
  const Vector3d torque_command_world = rotation * torque_flange;

  // ===========================================================================
  // Assemble Cartesian wrench
  // ===========================================================================

  // Translational force command (world frame).
  // X/Y: K ≈ 0 → only damping acts → operator can drag freely.
  // Z: High K drives the admittance-adjusted reference tracking.
  Vector3d force_command =
      translational_stiffness_.cwiseProduct(position_error) -
      translational_damping_.cwiseProduct(linear_velocity);

  Vector3d torque_command = torque_command_world;

  // Store unsaturated values for monitoring.
  const Vector3d force_command_unsat = force_command;
  const Vector3d torque_command_unsat = torque_command;

  // Saturation.
  force_command.x() = clamp_abs(force_command.x(), max_force_xy_);
  force_command.y() = clamp_abs(force_command.y(), max_force_xy_);
  force_command.z() = clamp_abs(force_command.z(), max_force_z_);

  torque_command.x() = clamp_abs(torque_command.x(), max_torque_xyz_);
  torque_command.y() = clamp_abs(torque_command.y(), max_torque_xyz_);
  torque_command.z() = clamp_abs(torque_command.z(), max_torque_xyz_);

  const bool force_saturation_active =
      (force_command - force_command_unsat).lpNorm<Eigen::Infinity>() > 1e-9;
  const bool torque_saturation_active =
      (torque_command - torque_command_unsat).lpNorm<Eigen::Infinity>() > 1e-9;

  Vector6d desired_wrench;
  desired_wrench << force_command, torque_command;

  // Cartesian wrench → joint torques:  τ_task = J^T · W
  const Vector7d tau_task = jacobian_.transpose() * desired_wrench;

  // ===========================================================================
  // Nullspace control – stabilize joint configuration
  // ===========================================================================

  const Matrix7x6d jacobian_transpose = jacobian_.transpose();
  constexpr double kDampedLeastSquares = 1e-4;
  const Eigen::Matrix<double, kCartesianDim, kCartesianDim> jj_t =
      jacobian_ * jacobian_transpose;
  const Eigen::Matrix<double, kCartesianDim, kCartesianDim> jj_t_damped =
      jj_t + kDampedLeastSquares * Eigen::Matrix<double, kCartesianDim, kCartesianDim>::Identity();
  const Matrix7d nullspace_projector =
      Matrix7d::Identity() - jacobian_transpose * jj_t_damped.inverse() * jacobian_;

  const Vector7d tau_nullspace =
      nullspace_projector *
      (nullspace_stiffness_ * (q_nullspace_target_ - q_) - nullspace_damping_ * dq_);

  // ===========================================================================
  // Total torque = task + Coriolis compensation + nullspace
  // ===========================================================================

  Vector7d tau_desired = tau_task + coriolis + tau_nullspace;

  const Vector7d tau_desired_before_joint_saturation = tau_desired;

  if (max_joint_torque_ > 0.0) {
    for (int i = 0; i < kNumJoints; ++i) {
      tau_desired(i) = clamp_abs(tau_desired(i), max_joint_torque_);
    }
  }

  const bool joint_torque_saturation_active =
      (tau_desired - tau_desired_before_joint_saturation).lpNorm<Eigen::Infinity>() > 1e-9;

  const Vector7d tau_command = saturate_torque_rate(tau_desired);
  const bool torque_rate_saturation_active =
      (tau_command - tau_desired).lpNorm<Eigen::Infinity>() > 1e-9;
  tau_previous_ = tau_command;

  for (int i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(tau_command(i));
  }

  // ===========================================================================
  // Monitoring publisher
  // ===========================================================================

  if (status_publisher_ && status_publish_rate_hz_ > 0.0) {
    status_publish_accumulator_ += dt;
    const double status_period = 1.0 / status_publish_rate_hz_;
    if (status_publish_accumulator_ >= status_period) {
      status_publish_accumulator_ -= status_period;
      std_msgs::msg::Float64MultiArray msg;
      // Layout:
      //  [0]  force_down_filtered        [N, down positive]
      //  [1]  force_z_desired_down       [N]
      //  [2]  force_error                [N]
      //  [3]  admittance_displacement    [m]  (down_pos_desired - down_pos_initial)
      //  [4]  position_error.x           [m]
      //  [5]  position_error.y           [m]
      //  [6]  position_error.z           [m]
      //  [7]  orientation_error_flange.x [rad] (roll – should be ≈ free)
      //  [8]  orientation_error_flange.y [rad] (pitch – locked)
      //  [9]  orientation_error_flange.z [rad] (yaw – locked)
      // [10]  force_command.x            [N]
      // [11]  force_command.y            [N]
      // [12]  force_command.z            [N]
      // [13]  torque_command.x           [Nm]
      // [14]  torque_command.y           [Nm]
      // [15]  torque_command.z           [Nm]
      // [16]  tau_task_norm              [Nm]
      // [17]  tau_nullspace_norm         [Nm]
      // [18]  tau_command_norm           [Nm]
      // [19]  force saturation flag      [0/1]
      // [20]  rotational torque sat flag [0/1]
      // [21]  joint torque sat flag      [0/1]
      // [22]  torque-rate sat flag       [0/1]
      msg.data = {
          force_down_filtered_,
          force_z_desired_down_,
          force_error,
          down_pos_desired_ - down_pos_initial_,
          position_error.x(),
          position_error.y(),
          position_error.z(),
          orientation_error_flange.x(),
          orientation_error_flange.y(),
          orientation_error_flange.z(),
          force_command.x(),
          force_command.y(),
          force_command.z(),
          torque_command.x(),
          torque_command.y(),
          torque_command.z(),
          tau_task.norm(),
          tau_nullspace.norm(),
          tau_command.norm(),
          force_saturation_active ? 1.0 : 0.0,
          torque_saturation_active ? 1.0 : 0.0,
          joint_torque_saturation_active ? 1.0 : 0.0,
          torque_rate_saturation_active ? 1.0 : 0.0,
      };
      status_publisher_->publish(msg);
    }
  }

  return controller_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Lifecycle callbacks
// ---------------------------------------------------------------------------

CallbackReturn UltrasoundScanningImpedanceController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("arm_prefix", "");

    // Phase 1 – translational gains
    auto_declare<double>("translational_stiffness.x", 0.0);
    auto_declare<double>("translational_stiffness.y", 0.0);
    auto_declare<double>("translational_stiffness.z", 400.0);

    auto_declare<double>("translational_damping.x", 10.0);
    auto_declare<double>("translational_damping.y", 10.0);
    auto_declare<double>("translational_damping.z", 70.0);

    // Phase 2 – rotational gains (flange frame: X=roll, Y=pitch, Z=yaw)
    auto_declare<double>("rotational_stiffness.x", 0.0);   // free roll
    auto_declare<double>("rotational_stiffness.y", 30.0);   // locked pitch
    auto_declare<double>("rotational_stiffness.z", 30.0);   // locked yaw

    auto_declare<double>("rotational_damping.x", 1.0);
    auto_declare<double>("rotational_damping.y", 6.0);
    auto_declare<double>("rotational_damping.z", 6.0);

    // Force control / admittance
    auto_declare<double>("force_z_desired_down", 5.0);
    auto_declare<double>("force_filter_cutoff_hz", 20.0);
    auto_declare<double>("admittance_gain", 2e-4);
    auto_declare<double>("vz_max", 0.02);
    auto_declare<double>("z_max", 0.02);

    // Nullspace
    auto_declare<double>("nullspace_stiffness", 15.0);
    auto_declare<double>("nullspace_damping", 3.0);

    // Safety
    auto_declare<double>("max_force_xy", 30.0);
    auto_declare<double>("max_force_z", 80.0);
    auto_declare<double>("max_torque_xyz", 20.0);
    auto_declare<double>("max_joint_torque", 80.0);
    auto_declare<double>("max_delta_tau", 1.0);

    // Monitoring
    auto_declare<std::string>("status_topic", "ultrasound_scanning_impedance/status");
    auto_declare<double>("status_publish_rate_hz", 20.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn UltrasoundScanningImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";

  // Phase 1
  translational_stiffness_.x() =
      get_node()->get_parameter("translational_stiffness.x").as_double();
  translational_stiffness_.y() =
      get_node()->get_parameter("translational_stiffness.y").as_double();
  translational_stiffness_.z() =
      get_node()->get_parameter("translational_stiffness.z").as_double();

  translational_damping_.x() = get_node()->get_parameter("translational_damping.x").as_double();
  translational_damping_.y() = get_node()->get_parameter("translational_damping.y").as_double();
  translational_damping_.z() = get_node()->get_parameter("translational_damping.z").as_double();

  // Phase 2
  rotational_stiffness_.x() = get_node()->get_parameter("rotational_stiffness.x").as_double();
  rotational_stiffness_.y() = get_node()->get_parameter("rotational_stiffness.y").as_double();
  rotational_stiffness_.z() = get_node()->get_parameter("rotational_stiffness.z").as_double();

  rotational_damping_.x() = get_node()->get_parameter("rotational_damping.x").as_double();
  rotational_damping_.y() = get_node()->get_parameter("rotational_damping.y").as_double();
  rotational_damping_.z() = get_node()->get_parameter("rotational_damping.z").as_double();

  // Force control
  force_z_desired_down_ = get_node()->get_parameter("force_z_desired_down").as_double();
  force_filter_cutoff_hz_ = get_node()->get_parameter("force_filter_cutoff_hz").as_double();
  admittance_gain_ = get_node()->get_parameter("admittance_gain").as_double();
  vz_max_ = get_node()->get_parameter("vz_max").as_double();
  z_max_ = get_node()->get_parameter("z_max").as_double();

  // Nullspace
  nullspace_stiffness_ = get_node()->get_parameter("nullspace_stiffness").as_double();
  nullspace_damping_ = get_node()->get_parameter("nullspace_damping").as_double();

  // Safety
  max_force_xy_ = get_node()->get_parameter("max_force_xy").as_double();
  max_force_z_ = get_node()->get_parameter("max_force_z").as_double();
  max_torque_xyz_ = get_node()->get_parameter("max_torque_xyz").as_double();
  max_joint_torque_ = get_node()->get_parameter("max_joint_torque").as_double();
  max_delta_tau_ = get_node()->get_parameter("max_delta_tau").as_double();

  // Monitoring
  status_topic_ = get_node()->get_parameter("status_topic").as_string();
  status_publish_rate_hz_ = get_node()->get_parameter("status_publish_rate_hz").as_double();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_prefix_ + robot_type_ + "/" + k_robot_model_interface_name,
      arm_prefix_ + robot_type_ + "/" + k_robot_state_interface_name);

  return CallbackReturn::SUCCESS;
}

CallbackReturn UltrasoundScanningImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  const std::string robot_state_interface_name =
      arm_prefix_ + robot_type_ + "/" + k_robot_state_interface_name;

  const auto it = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(), [&](const auto& interface) {
        return interface.get_name() == robot_state_interface_name;
      });

  if (it == state_interfaces_.cend()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find robot state interface: %s",
                 robot_state_interface_name.c_str());
    return CallbackReturn::ERROR;
  }

  robot_state_interface_index_ =
      static_cast<std::size_t>(std::distance(state_interfaces_.cbegin(), it));
  if (!read_robot_state_pointer()) {
    return CallbackReturn::ERROR;
  }

  update_joint_states();
  q_nullspace_target_ = q_;
  tau_previous_ = Eigen::Map<const Vector7d>(robot_state_ptr_->tau_J_d.data());
  status_publish_accumulator_ = 0.0;

  status_publisher_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(status_topic_, 10);

  initialized_ = false;
  initialize_targets();
  initialized_ = true;

  return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void UltrasoundScanningImpedanceController::update_joint_states() {
  for (int i = 0; i < kNumJoints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

bool UltrasoundScanningImpedanceController::read_robot_state_pointer() {
  if (robot_state_interface_index_ >= state_interfaces_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Robot state interface index out of range.");
    return false;
  }

  robot_state_ptr_ =
      bit_cast<franka::RobotState*>(state_interfaces_[robot_state_interface_index_].get_value());
  return robot_state_ptr_ != nullptr;
}

void UltrasoundScanningImpedanceController::initialize_targets() {
  const auto flange_pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kFlange);
  const Eigen::Map<const Eigen::Matrix4d> flange_pose(flange_pose_array.data());

  position_initial_ = flange_pose.block<3, 1>(0, 3);
  position_desired_ = position_initial_;
  down_pos_initial_ = -position_initial_.z();
  down_pos_desired_ = down_pos_initial_;

  // Capture the initial orientation of the flange.
  // Roll (flange X) will be free; pitch (flange Y) and yaw (flange Z) will
  // be regulated back to this initial orientation.
  rotation_initial_ = flange_pose.block<3, 3>(0, 0);

  // Initialize force filter with current measured downward force.
  force_down_filtered_ = -robot_state_ptr_->O_F_ext_hat_K[2];
}

UltrasoundScanningImpedanceController::Vector7d
UltrasoundScanningImpedanceController::saturate_torque_rate(const Vector7d& tau_desired) const {
  Vector7d tau_saturated{};
  for (int i = 0; i < kNumJoints; ++i) {
    const double delta =
        std::clamp(tau_desired(i) - tau_previous_(i), -max_delta_tau_, max_delta_tau_);
    tau_saturated(i) = tau_previous_(i) + delta;
  }
  return tau_saturated;
}

}  // namespace franka_example_controllers

// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::UltrasoundScanningImpedanceController,
                       controller_interface::ControllerInterface)
