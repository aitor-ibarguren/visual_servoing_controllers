// Copyright (c) 2026 Aitor Ibarguren
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

#ifndef VISUAL_SERVOING_CONTROLLER__PBVS_CONTROLLER_HPP_
#define VISUAL_SERVOING_CONTROLLER__PBVS_CONTROLLER_HPP_

// ROS2
#include <rclcpp_action/rclcpp_action.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Msgs
#include "lifecycle_msgs/msg/state.hpp"

// ROS2 Control
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// Eigen
#include "Eigen/Geometry"

#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_eigen_kdl/tf2_eigen_kdl.hpp"

// Real Time Tools
#include "realtime_tools/realtime_publisher.hpp"

// Parameters
#include "pbvs_controller/pbvs_controller_params.hpp"

// Kinematics
#include "urdf/model.h"
// KDL
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl_parser/kdl_parser.hpp"

// PID
#include "pbvs_controller/pid_impl.hpp"

// Msgs
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "visual_servoing_controllers_msgs/action/position_based_visual_servoing.hpp"

// Realtime
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace visual_servoing_controller
{

struct PBVSTask
{
  // Control vars
  bool task_active;
  bool initial_target_found;
  bool target_lost;

  // Pose
  bool maintain_pose;
  Eigen::Isometry3d target_destination;

  // Tolerances
  double translation_tolerance;
  double rotation_tolerance;

  // Timeouts
  double task_timeout_ms;
  double target_search_timeout_ms;
  double target_lost_timeout_ms;
  double detection_validity_time_ms;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> task_time_init;
  std::chrono::time_point<std::chrono::high_resolution_clock> target_search_time_init;
  std::chrono::time_point<std::chrono::high_resolution_clock> target_lost_time_init;
};

struct DetectionData
{
  // Control vars
  bool first_detection_received;
  bool last_detection_valid;

  // Pose
  Eigen::Isometry3d last_detection_pose;

  // Timeout
  double detection_timeout_ms;

  // Time
  std::chrono::time_point<std::chrono::high_resolution_clock> last_detection_time;
};

class PBVSController : public controller_interface::ControllerInterface
{
public:
  PBVSController();

  // Command interface
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // State interface
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Update function
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Lifecycle
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // Robot vars
  size_t dof_;
  size_t num_cmd_joints_;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd lower_joint_limits_, upper_joint_limits_;
  Eigen::VectorXd vel_joint_limits_;

  std::string base_link_, tip_link_, camera_link_;

  // Robot state
  std::vector<std::string> command_joint_names_;

  Eigen::VectorXd joint_positions_;
  Eigen::VectorXd joint_velocities_;

  Eigen::VectorXd joint_velocities_prev_;

  Eigen::Isometry3d base_link_H_tip_, tip_H_camera_;

  // Robot commands
  Eigen::VectorXd joint_position_commands_, joint_velocity_commands_;
  // Prev robot commands (open loop)
  Eigen::VectorXd joint_position_commands_prev_, joint_velocity_commands_prev_;

  bool has_position_command_interface_ = false;

  // Parameter handlers
  pbvs_controller::Params params_;
  std::shared_ptr<pbvs_controller::ParamListener> param_listener_;

  // PID
  std::shared_ptr<PID6D> pid_;

  // Max speeds
  double max_translation_speed_;
  double max_rotation_speed_;

  // Control vars
  bool open_loop_;

  // Feedback vars
  bool feedback_active_;
  // adaptive_admittance_controller_msgs::msg::Feedback feedback_msg_;

  // KDL
  KDL::JntArray q_;
  urdf::Model model_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  // Action server
  using PBVSAction = visual_servoing_controllers_msgs::action::PositionBasedVisualServoing;
  using RealtimePBVSGoalHandle = realtime_tools::RealtimeServerGoalHandle<PBVSAction>;
  using RealtimePBVSGoalHandlePtr = std::shared_ptr<RealtimePBVSGoalHandle>;
  using RealtimePBVSGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimePBVSGoalHandlePtr>;

  RealtimePBVSGoalHandleBuffer rt_pbvs_active_goal_;
  rclcpp_action::Server<PBVSAction>::SharedPtr pbvs_action_server_;
  std::atomic<bool> rt_has_pending_goal_{false};
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  rclcpp::Duration action_monitor_period_ = rclcpp::Duration(50ms);

  // PBVS task
  PBVSTask pbvs_task_;

  // Detection
  DetectionData detection_data_;

  std::string detection_topic_name_;
  std::string detection_topic_type_;

  // Action server - Callbacks
  rclcpp_action::GoalResponse goal_received_callback(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PBVSAction::Goal> goal);
  rclcpp_action::CancelResponse goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PBVSAction>> goal_handle);
  void goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<PBVSAction>> goal_handle);

  void send_result(bool succeed, int error_code, const std::string & error_string);
  void preempt_active_goal();

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subs_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_subs_;

  // Callbacks
  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void pose_stamped_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Helper functions
  Eigen::VectorXd to_vector(const Eigen::Isometry3d & h);

  double duration_to_milliseconds(const builtin_interfaces::msg::Duration & d);
  double get_duration_millis(
    const std::chrono::time_point<std::chrono::high_resolution_clock> & init_time);

  bool valid_goal(std::shared_ptr<const PBVSAction::Goal> goal);

  void manage_detection();
  void manage_timeouts();
  void manage_tolerances(const Eigen::VectorXd & vision_error);

  // Helper functions - Kinematics
  bool get_joint_limits(const std::vector<std::string> & joint_names);
  bool get_kinematics(const std::string & base_link, const std::string & tip_link);
  bool getTransform(
    const KDL::Tree & tree, const std::string & from, const std::string & to,
    Eigen::Isometry3d & result);
  void read_joint_state(Eigen::VectorXd & joint_positions, Eigen::VectorXd & joint_velocities);
  Eigen::Isometry3d get_tip_pose(const Eigen::VectorXd & joint_positions);

  // Feedback
  void publish_feedback(
    const std::chrono::time_point<std::chrono::high_resolution_clock> & task_time_init,
    const Eigen::VectorXd & error, const Eigen::VectorXd & camera_twist,
    const Eigen::VectorXd & tip_link_twist);

  // Twist helper functions
  Eigen::VectorXd change_twist_reference(
    const Eigen::VectorXd & twist, const Eigen::Isometry3d & twist_reference);
  Eigen::VectorXd move_twist(const Eigen::VectorXd & twist, const Eigen::Vector3d & q);

  // Kinematics
  Eigen::VectorXd calculate_next_joint_positions(
    const Eigen::VectorXd & joint_positions, const Eigen::VectorXd & twist, double dt);

  template<int Rows, int Cols>
  Eigen::Matrix<double, Cols, Rows>
  dampedPseudoInverse(const Eigen::Matrix<double, Rows, Cols>& mat,
                      double damping)
  {
      Eigen::JacobiSVD<Eigen::Matrix<double, Rows, Cols>> svd(
          mat,
          Eigen::ComputeThinU | Eigen::ComputeThinV);

      const auto& S = svd.singularValues();

      Eigen::VectorXd Sinv =
          S.array() / (S.array().square() + damping * damping);

      return svd.matrixV()
          * Sinv.asDiagonal()
          * svd.matrixU().adjoint();
  }
};

}  // namespace visual_servoing_controller

#endif  // VISUAL_SERVOING_CONTROLLER__PBVS_CONTROLLER_HPP_