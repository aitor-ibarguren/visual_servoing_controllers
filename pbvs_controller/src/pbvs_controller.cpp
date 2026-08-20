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

#include "pbvs_controller/pbvs_controller.hpp"

namespace visual_servoing_controller
{
PBVSController::PBVSController()
: controller_interface::ControllerInterface(), dof_(0), num_cmd_joints_(0)
{
}

PBVSController::CallbackReturn PBVSController::on_init()
{
  // Initialize the parameter handler
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<pbvs_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PBVSController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Check degrees of freedom
  if (dof_ == 0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Degrees of freedom MUST valid positive (actual DOF %lu)", dof_);
    throw std::runtime_error("Invalid degrees of freedom");
  }

  // Reserve space for command interfaces
  conf.names.reserve(dof_ * params_.command_interfaces.size());
  for (const auto & joint_name : command_joint_names_)
  {
    for (const auto & interface_type : params_.command_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration PBVSController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Check degrees of freedom
  if (dof_ == 0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Degrees of freedom MUST valid positive (actual DOF %lu)", dof_);
    throw std::runtime_error("Invalid degrees of freedom");
  }

  // Reserve space for state interfaces
  conf.names.reserve(dof_ * params_.state_interfaces.size());
  for (const auto & joint_name : params_.joints)
  {
    for (const auto & interface_type : params_.state_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::CallbackReturn PBVSController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Prepare the controller for activation.
  RCLCPP_INFO(get_node()->get_logger(), "Configuring PBVSController");

  // Update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // Get parameters from listener
  params_ = param_listener_->get_params();

  // Get DoF
  dof_ = params_.joints.size();

  // Get joint names
  joint_names_ = params_.joints;
  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints were specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get joint limits
  if (!get_joint_limits(joint_names_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error retrieving kinematic info from URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Command joints
  command_joint_names_ = params_.command_joints;

  if (command_joint_names_.empty())
  {
    command_joint_names_ = params_.joints;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "No specific joint names are used for command interfaces. Using 'joints' parameter.");
  }

  // Command interfaces
  has_position_command_interface_ =
    std::find(
      params_.command_interfaces.begin(), params_.command_interfaces.end(),
      hardware_interface::HW_IF_POSITION) != params_.command_interfaces.end();

  // Get open loop
  open_loop_ = params_.open_loop;

  /// Kinematic data
  // Get base, tip, and camera links
  base_link_ = params_.kinematics.base_link;
  tip_link_ = params_.kinematics.tip_link;
  camera_link_ = params_.kinematics.camera_link;

  // Set q
  q_ = KDL::JntArray(joint_names_.size());

  // Get kinematic info
  if (!get_kinematics(base_link_, tip_link_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error generating kinematic solvers from URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get tip to camera transformation
  if (!getTransform(tree_, tip_link_, camera_link_, tip_H_camera_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Error retrieving transformation between tip link and camera link");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Detection
  detection_topic_name_ = params_.detection.topic_name;
  detection_topic_type_ = params_.detection.topic_type;
  detection_data_.detection_timeout_ms = params_.detection.timeout * 1000;

  // Control
  Eigen::VectorXd pid_p(6), pid_i(6), pid_d(6);
  pid_p << params_.control.p, params_.control.p, params_.control.p, params_.control.p,
    params_.control.p, params_.control.p;
  pid_i << params_.control.i, params_.control.i, params_.control.i, params_.control.i,
    params_.control.i, params_.control.i;
  pid_d << params_.control.d, params_.control.d, params_.control.d, params_.control.d,
    params_.control.d, params_.control.d;

  pid_ = std::make_shared<PID6D>(pid_p, pid_i, pid_d);

  max_translation_speed_ = params_.control.max_translation_speed;
  max_rotation_speed_ = params_.control.max_rotation_speed;

  // Feedback
  feedback_active_ = params_.feedback_active;

  // Log
  RCLCPP_INFO(get_node()->get_logger(), "╠═ Open loop: %s", open_loop_ ? "True" : "False");
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    RCLCPP_INFO(get_node()->get_logger(), "╠═ Joint %s limits: ", joint_names_[i].c_str());
    RCLCPP_INFO(
      get_node()->get_logger(), "║  ├─ Lower: %f - Upper: %f", lower_joint_limits_[i],
      upper_joint_limits_[i]);
    RCLCPP_INFO(get_node()->get_logger(), "║  ╰─ Velocity: %f", vel_joint_limits_[i]);
  }
  RCLCPP_INFO(
    get_node()->get_logger(), "╠═ Kinematic chain from '%s' to '%s'", base_link_.c_str(),
    tip_link_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "╠═ Camera link '%s'", camera_link_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "╠═ Detection");
  RCLCPP_INFO(get_node()->get_logger(), "║  ├─ Topic name '%s'", detection_topic_name_.c_str());
  RCLCPP_INFO(get_node()->get_logger(), "║  ├─ Topic type '%s'", detection_topic_type_.c_str());
  RCLCPP_INFO(
    get_node()->get_logger(), "║  ╰─ Timeout: %f", detection_data_.detection_timeout_ms / 1000);
  RCLCPP_INFO(get_node()->get_logger(), "╠═ Control");
  RCLCPP_INFO(get_node()->get_logger(), "║  ├─ P: %f", params_.control.p);
  RCLCPP_INFO(get_node()->get_logger(), "║  ├─ I: %f", params_.control.i);
  RCLCPP_INFO(get_node()->get_logger(), "║  ├─ D: %f", params_.control.d);
  RCLCPP_INFO(
    get_node()->get_logger(), "║  ├─ Max. translation speed: %f",
    params_.control.max_translation_speed);
  RCLCPP_INFO(
    get_node()->get_logger(), "║  ╰─ Max. rotation speed: %f", params_.control.max_rotation_speed);
  RCLCPP_INFO(
    get_node()->get_logger(), "╚═ Feedback: %s", feedback_active_ ? "ACTIVE" : "INACTIVE");

  // Create action server
  pbvs_action_server_ = rclcpp_action::create_server<PBVSAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/position_based_visual_servoing",
    std::bind(&PBVSController::goal_received_callback, this, _1, _2),
    std::bind(&PBVSController::goal_cancelled_callback, this, _1),
    std::bind(&PBVSController::goal_accepted_callback, this, _1));

  // Create subscribers & publishers
  if (detection_topic_type_ == "geometry_msgs/Pose")

  {
    pose_subs_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
      std::string(get_node()->get_name()) + "/" + detection_topic_name_, 1,
      std::bind(&PBVSController::pose_callback, this, std::placeholders::_1));
  }
  else if (detection_topic_type_ == "geometry_msgs/PoseStamped")
  {
    pose_stamped_subs_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      std::string(get_node()->get_name()) + "/" + detection_topic_name_, 1,
      std::bind(&PBVSController::pose_stamped_callback, this, std::placeholders::_1));
  }

  detection_data_.first_detection_received = false;
  detection_data_.last_detection_valid = false;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PBVSController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating PBVSController");

  // Initialize robot joint states
  joint_positions_ = Eigen::VectorXd::Zero(dof_);
  joint_velocities_ = Eigen::VectorXd::Zero(dof_);

  // Initialize joint commands
  joint_position_commands_ = Eigen::VectorXd::Zero(dof_);
  joint_velocity_commands_ = Eigen::VectorXd::Zero(dof_);
  joint_position_commands_prev_ = Eigen::VectorXd::Zero(dof_);
  joint_velocity_commands_prev_ = Eigen::VectorXd::Zero(dof_);

  if (state_interfaces_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces loaded");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Get current joint positions and velocities
  read_joint_state(joint_positions_, joint_velocities_);
  // Set commands to reference joint state
  joint_position_commands_ = joint_positions_;
  joint_velocity_commands_ = joint_velocities_;
  joint_position_commands_prev_ = joint_positions_;
  joint_velocity_commands_prev_ = joint_velocities_;

  // Task
  pbvs_task_.task_active = false;

  /// Log
  // Joint positions
  std::ostringstream oss;
  for (double d : joint_positions_) oss << d << ' ';

  RCLCPP_INFO(get_node()->get_logger(), "Initial joint positions: [ %s]'", oss.str().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PBVSController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating PBVSController");

  // Set all values to zero
  joint_positions_ = Eigen::VectorXd::Zero(dof_);
  joint_velocities_ = Eigen::VectorXd::Zero(dof_);
  joint_position_commands_ = Eigen::VectorXd::Zero(dof_);
  joint_velocity_commands_ = Eigen::VectorXd::Zero(dof_);
  joint_position_commands_prev_ = Eigen::VectorXd::Zero(dof_);
  joint_velocity_commands_prev_ = Eigen::VectorXd::Zero(dof_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PBVSController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating PBVSController");

  // Reset subscribers
  pose_subs_.reset();
  pose_stamped_subs_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse PBVSController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const PBVSAction::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "New PBVS goal received...");

  // Precondition: Running controller
  if (get_lifecycle_id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // ValCheck if valid goal
  if (!valid_goal(goal))
  {
    RCLCPP_INFO(get_node()->get_logger(), "Goal not valid ❌");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Starting new PBVS action goal 🚀");

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PBVSController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<PBVSAction>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  // Manage active goal
  const auto active_goal = *rt_pbvs_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Canceling active action goal: Cancel callback received.");

    // Mark the current goal as canceled
    rt_has_pending_goal_ = false;
    auto action_res = std::make_shared<PBVSAction::Result>();
    // Set result values - ToDo
    active_goal->setCanceled(action_res);
    rt_pbvs_active_goal_.writeFromNonRT(RealtimePBVSGoalHandlePtr());

    pbvs_task_.task_active = false;
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void PBVSController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<PBVSAction>> goal_handle)
{
  // mark a pending goal
  rt_has_pending_goal_ = true;

  // Update new PBVS task

  preempt_active_goal();

  // Set PBVS task values
  pbvs_task_.translation_tolerance = goal_handle->get_goal()->translation_tolerance;
  pbvs_task_.rotation_tolerance = goal_handle->get_goal()->rotation_tolerance;

  pbvs_task_.task_timeout_ms = duration_to_milliseconds(goal_handle->get_goal()->task_timeout);
  pbvs_task_.target_search_timeout_ms =
    duration_to_milliseconds(goal_handle->get_goal()->target_search_timeout);
  pbvs_task_.target_lost_timeout_ms =
    duration_to_milliseconds(goal_handle->get_goal()->target_lost_timeout);
  pbvs_task_.detection_validity_time_ms = params_.detection.timeout * 1000;

  pbvs_task_.task_time_init = std::chrono::high_resolution_clock::now();
  pbvs_task_.target_search_time_init = std::chrono::high_resolution_clock::now();

  pbvs_task_.initial_target_found = false;
  pbvs_task_.target_lost = false;

  // Check if destination provided in goal
  pbvs_task_.mantain_pose = goal_handle->get_goal()->mantain_pose;
  if (!pbvs_task_.mantain_pose)
  {
    tf2::fromMsg(goal_handle->get_goal()->destination, pbvs_task_.target_destination);
  }

  pbvs_task_.task_active = true;

  // Goal info
  if (!pbvs_task_.mantain_pose)
  {
    Eigen::Vector3d t = pbvs_task_.target_destination.translation();
    Eigen::Quaterniond q(pbvs_task_.target_destination.rotation());

    RCLCPP_INFO(
      get_node()->get_logger(),
      "New target destination - Translation XYZ: %f %f %f - Rotation XYZW: %f %f %f %f", t.x(),
      t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
  }

  // Update the active goal
  RealtimePBVSGoalHandlePtr rt_goal = std::make_shared<RealtimePBVSGoalHandle>(goal_handle);
  rt_goal->execute();
  rt_pbvs_active_goal_.writeFromNonRT(rt_goal);

  // Delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimePBVSGoalHandle::runNonRealtime, rt_goal));
}

void PBVSController::send_result(bool succeed, int error_code, const std::string & error_string)
{
  const auto active_goal = *rt_pbvs_active_goal_.readFromNonRT();

  auto action_res = std::make_shared<PBVSAction::Result>();

  action_res->error_code = error_code;
  action_res->error_string = error_string;

  if (succeed)
  {
    active_goal->setSucceeded(action_res);
  }
  else
  {
    active_goal->setAborted(action_res);
  }
}

void PBVSController::preempt_active_goal()
{
  const auto active_goal = *rt_pbvs_active_goal_.readFromNonRT();
  if (active_goal)
  {
    auto action_res = std::make_shared<PBVSAction::Result>();

    action_res->error_code = PBVSAction::Result::NEW_GOAL_RECEIVED;
    action_res->error_string = "Current goal cancelled due to new incoming action.";
    active_goal->setCanceled(action_res);

    rt_pbvs_active_goal_.writeFromNonRT(RealtimePBVSGoalHandlePtr());
  }
}

Eigen::VectorXd PBVSController::change_twist_reference(
  const Eigen::VectorXd & twist, const Eigen::Isometry3d & twist_reference)
{
  // Declare new twist
  Eigen::VectorXd new_twist = Eigen::VectorXd::Zero(6);

  new_twist.head<3>() = twist_reference.rotation() * twist.head<3>();
  new_twist.tail<3>() = twist_reference.rotation() * twist.tail<3>();

  return new_twist;
}

Eigen::VectorXd PBVSController::move_twist(const Eigen::VectorXd & twist, const Eigen::Vector3d & q)
{
  // Declare new twist
  Eigen::VectorXd new_twist = Eigen::VectorXd::Zero(6);

  new_twist.head<3>() = twist.head<3>() + q.cross(twist.tail<3>());
  new_twist.tail<3>() = twist.tail<3>();

  return new_twist;
}

Eigen::VectorXd PBVSController::calculate_next_joint_positions(
  const Eigen::VectorXd & joint_positions, const Eigen::VectorXd & twist, double dt)
{
  Eigen::VectorXd next_joint_position = Eigen::VectorXd::Zero(dof_);

  // To KDL
  KDL::JntArray joint_positions_kdl(joint_positions.size());
  joint_positions_kdl.data = joint_positions;

  // Get Jacobian
  KDL::Jacobian jacobian(dof_);

  jnt_to_jac_solver_->JntToJac(joint_positions_kdl, jacobian);

  // Damped pseudo-inverse
  Eigen::MatrixXd jacobian_pseudo_inverse = dampedPseudoInverse(jacobian.data, 0.1);

  // Compute joint velocities
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(dof_);
  joint_velocities = jacobian_pseudo_inverse * twist;

  // Next joint positions
  next_joint_position = joint_positions + joint_velocities * dt;

  return next_joint_position;
}

controller_interface::return_type PBVSController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Get joint state
  if (open_loop_)
  {
    joint_positions_ = joint_position_commands_prev_;
    joint_velocities_ = joint_velocity_commands_prev_;
  }
  else
  {
    read_joint_state(joint_positions_, joint_velocities_);
  }

  // Manage detection
  manage_detection();

  // Manage timeouts
  manage_timeouts();

  // Check if initial task active & last detection is valid
  if (pbvs_task_.task_active && detection_data_.last_detection_valid)
  {
    // Manage target lost
    pbvs_task_.target_lost = false;

    // Get tip pose
    base_link_H_tip_ = get_tip_pose(joint_positions_);

    // Calculate twist based vision error
    Eigen::VectorXd vision_error =
      to_vector(detection_data_.last_detection_pose * pbvs_task_.target_destination.inverse());

    // PID
    Eigen::VectorXd camera_twist = pid_->calculate(vision_error, period.seconds());

    // Manage max speeds
    auto t_camera_twist = camera_twist.head<3>();
    camera_twist.head<3>() =
      t_camera_twist * std::min(max_translation_speed_ / t_camera_twist.norm(), 1.0);

    auto r_camera_twist = camera_twist.tail<3>();
    camera_twist.tail<3>() =
      r_camera_twist * std::min(max_rotation_speed_ / r_camera_twist.norm(), 1.0);

    // Twist to base link
    Eigen::VectorXd camera_twist_base_link =
      change_twist_reference(camera_twist, base_link_H_tip_ * tip_H_camera_);

    // Transfer twist to tip link
    Eigen::VectorXd twist_base_link_tip = move_twist(
      camera_twist_base_link,
      (base_link_H_tip_ * tip_H_camera_).translation() - base_link_H_tip_.translation());

    // Get joint position commands
    joint_position_commands_ =
      calculate_next_joint_positions(joint_positions_, twist_base_link_tip, period.seconds());

    // Manage destination & tolerances
    manage_tolerances(vision_error);

    // Manage feedback
    if (feedback_active_)
    {
      // Publish
      publish_feedback(pbvs_task_.task_time_init, vision_error, camera_twist, twist_base_link_tip);
    }
  }
  else
  {
    joint_position_commands_ = joint_positions_;
  }

  // Set joint position
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (has_position_command_interface_)
    {
      if (!command_interfaces_[i].set_value(static_cast<double>(joint_position_commands_(i))))
        RCLCPP_WARN(get_node()->get_logger(), "Error setting command for joint %zu", i);
    }
  }

  // Store joint commands
  joint_position_commands_prev_ = joint_position_commands_;
  joint_velocity_commands_prev_ = joint_velocity_commands_;

  return controller_interface::return_type::OK;
}

void PBVSController::pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  // Manage first detection
  if (!detection_data_.first_detection_received) detection_data_.first_detection_received = true;

  // Get pose & time
  tf2::fromMsg(*msg, detection_data_.last_detection_pose);
  detection_data_.last_detection_time = std::chrono::high_resolution_clock::now();
}

void PBVSController::pose_stamped_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Manage first detection
  if (!detection_data_.first_detection_received) detection_data_.first_detection_received = true;

  // Get pose & time
  tf2::fromMsg(msg->pose, detection_data_.last_detection_pose);
  detection_data_.last_detection_time = std::chrono::high_resolution_clock::now();
}

Eigen::VectorXd PBVSController::to_vector(const Eigen::Isometry3d & h)
{
  Eigen::VectorXd v(6);

  // Translation
  v.head<3>() = h.translation();

  // Rotation
  Eigen::AngleAxisd aa(h.rotation());

  v.tail<3>() = aa.axis() * aa.angle();

  return v;
}

double PBVSController::duration_to_milliseconds(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) * 1000.0 + static_cast<double>(d.nanosec) / 1e6;
}

double PBVSController::get_duration_millis(
  const std::chrono::time_point<std::chrono::high_resolution_clock> & init_time)
{
  auto finish_time = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(finish_time - init_time).count();
}

bool PBVSController::valid_goal(std::shared_ptr<const PBVSAction::Goal> goal)
{
  if (goal->translation_tolerance < 0 || goal->rotation_tolerance < 0)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Translation & rotation tolerances must be higher than 0");
    return false;
  }

  return true;
}

void PBVSController::manage_detection()
{
  // Check if last detection is valid
  detection_data_.last_detection_valid =
    (get_duration_millis(detection_data_.last_detection_time) <
     pbvs_task_.detection_validity_time_ms);

  // Manage first detection
  if (
    pbvs_task_.task_active && !pbvs_task_.initial_target_found &&
    detection_data_.first_detection_received && detection_data_.last_detection_valid)
  {
    // Manage mantain position type task
    if (pbvs_task_.mantain_pose)
    {
      pbvs_task_.target_destination = detection_data_.last_detection_pose;

      Eigen::Vector3d t = pbvs_task_.target_destination.translation();
      Eigen::Quaterniond q(pbvs_task_.target_destination.rotation());

      RCLCPP_INFO(
        get_node()->get_logger(),
        "New target destination - Translation XYZ: %f %f %f - Rotation XYZW: %f %f %f %f", t.x(),
        t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
    }

    pbvs_task_.initial_target_found = true;
  }
  // Check if target lost
  else if (
    pbvs_task_.task_active && pbvs_task_.initial_target_found &&
    !detection_data_.last_detection_valid && !pbvs_task_.target_lost)
  {
    pbvs_task_.target_lost = true;
    pbvs_task_.target_lost_time_init = std::chrono::high_resolution_clock::now();
  }
  // Check if target found
  else if (
    pbvs_task_.task_active && pbvs_task_.initial_target_found &&
    detection_data_.last_detection_valid && pbvs_task_.target_lost)
  {
    pbvs_task_.target_lost = false;
  }
}

void PBVSController::manage_timeouts()
{
  // Verify if task timeout reached
  if (
    pbvs_task_.task_active && pbvs_task_.task_timeout_ms > 0 &&
    get_duration_millis(pbvs_task_.task_time_init) > pbvs_task_.task_timeout_ms)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Finishing PBVS task: Task timeout reached ❌");
    send_result(false, PBVSAction::Result::TASK_TIMEOUT, "Task timeout reached");
    pbvs_task_.task_active = false;
  }
  // Verify if target search timeout reached
  else if (
    pbvs_task_.task_active && !pbvs_task_.initial_target_found &&
    pbvs_task_.target_search_timeout_ms > 0 &&
    get_duration_millis(pbvs_task_.target_search_time_init) > pbvs_task_.target_search_timeout_ms)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Finishing PBVS task: Target search timeout reached ❌");
    send_result(false, PBVSAction::Result::TARGET_SEARCH_TIMEOUT, "Target search timeout reached");
    pbvs_task_.task_active = false;
  }
  // Verify if target lost timeout reached
  else if (
    pbvs_task_.task_active && pbvs_task_.target_lost && pbvs_task_.target_lost_timeout_ms > 0 &&
    get_duration_millis(pbvs_task_.target_lost_time_init) > pbvs_task_.target_lost_timeout_ms)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Finishing PBVS task: Target lost timeout reached ❌");
    send_result(false, PBVSAction::Result::TARGET_LOST_TIMEOUT, "Target lost timeout reached");
    pbvs_task_.task_active = false;
  }
}

void PBVSController::manage_tolerances(const Eigen::VectorXd & vision_error)
{
  // Verify if task timeout reached
  if (
    pbvs_task_.task_active && pbvs_task_.task_timeout_ms > 0 &&
    vision_error.head<3>().norm() < pbvs_task_.translation_tolerance &&
    vision_error.tail<3>().norm() < pbvs_task_.rotation_tolerance)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Finishing PBVS task: Tolerances reached 🎯");
    send_result(true, 0, "");
    pbvs_task_.task_active = false;
  }
}

bool PBVSController::get_joint_limits(const std::vector<std::string> & joint_names)
{
  // Get URDF
  const std::string & urdf = get_robot_description();

  // Init model
  urdf::Model model;
  if (!model.initString(urdf))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed parsing URDF");
    return false;
  }

  // Define Eigen::VectorXd size
  lower_joint_limits_.resize(joint_names.size());
  upper_joint_limits_.resize(joint_names.size());
  vel_joint_limits_.resize(joint_names.size());

  // Get limits
  int idx = 0;
  for (auto & joint_name : joint_names)
  {
    // Get joint
    auto joint = model.getJoint(joint_name);
    if (!joint)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found on URDF", joint_name.c_str());
      return false;
    }

    // Get limits
    lower_joint_limits_[idx] = joint->limits->lower;
    upper_joint_limits_[idx] = joint->limits->upper;
    vel_joint_limits_[idx] = joint->limits->velocity;

    idx++;
  }

  return true;
}

bool PBVSController::get_kinematics(const std::string & base_link, const std::string & tip_link)
{
  // Get URDF
  const std::string & urdf = get_robot_description();

  // Init model
  if (!model_.initString(urdf))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed parsing URDF");
    return false;
  }

  // Init tree
  if (!kdl_parser::treeFromUrdfModel(model_, tree_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct KDL tree");
    return false;
  }

  // Get kinematic chain
  if (!tree_.getChain(base_link, tip_link, chain_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to get KDL chain from '%s' to '%s'", base_link.c_str(),
      tip_link.c_str());
    return false;
  }

  // Check joint number
  if (chain_.getNrOfJoints() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Provided joint list (%d) and the number of joints of the KDL chain (%d) are not equal",
      (int)joint_names_.size(), chain_.getNrOfJoints());
    return false;
  }

  // Check if joint names are in the chain order
  int j_number = 0;
  for (size_t i = 0; i < chain_.getNrOfSegments(); i++)
  {
    const KDL::Joint & joint = chain_.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None)
    {
      // Check if same joint name
      if (joint.getName() != joint_names_[j_number])
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "Joint number %d names ('%s' and '%s') are not equal", (int)i,
          joint.getName().c_str(), joint_names_[j_number].c_str());
        return false;
      }

      j_number++;
    }
  }

  // FK solver
  fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
  jnt_to_jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);

  return true;
}

bool PBVSController::getTransform(
  const KDL::Tree & tree, const std::string & from, const std::string & to,
  Eigen::Isometry3d & result)
{
  // Get chain between
  KDL::Chain chain;

  if (!tree.getChain(from, to, chain)) return false;

  // Calculate transformation & verify all are fixed joints
  KDL::Frame frame = KDL::Frame::Identity();

  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
  {
    const KDL::Segment & seg = chain.getSegment(i);

    if (seg.getJoint().getType() != KDL::Joint::None)
    {
      return false;  // non-fixed joint found
    }

    frame = frame * seg.pose(0.0);
  }

  // To Eigen isometry
  tf2::transformKDLToEigen(frame, result);

  return true;
}

void PBVSController::read_joint_state(
  Eigen::VectorXd & joint_positions, Eigen::VectorXd & joint_velocities)
{
  int idx_pos = 0;
  int idx_vel = 0;

  for (auto & state_interface : state_interfaces_)
  {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      // Get position
      const auto joint_position_opt = state_interface.get_optional();
      if (!joint_position_opt.has_value())
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "Unable to retrieve joint state interface value for '%s'",
          state_interface.get_name().c_str());
      }
      else
      {
        joint_positions[idx_pos] = joint_position_opt.value();

        idx_pos++;
      }
    }
    else if (state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      // Get velocity
      const auto joint_vel_opt = state_interface.get_optional();
      if (!joint_vel_opt.has_value())
      {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "Unable to retrieve joint state interface value for '%s'",
          state_interface.get_name().c_str());
      }
      else
      {
        joint_velocities[idx_vel] = joint_vel_opt.value();
        idx_vel++;
      }
    }
  }
}

Eigen::Isometry3d PBVSController::get_tip_pose(const Eigen::VectorXd & joint_positions)
{
  Eigen::Isometry3d tip_pose;

  // Insert joint positions
  for (int i = 0; i < joint_positions.size(); i++) q_(i) = joint_positions[i];

  // Get Cartesian pose
  KDL::Frame kdl_pose;
  fk_solver_->JntToCart(q_, kdl_pose);

  // KDL to Eigen
  // Translation
  tip_pose.translation() = Eigen::Vector3d(kdl_pose.p.x(), kdl_pose.p.y(), kdl_pose.p.z());
  // Rotation
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) tip_pose.linear()(i, j) = kdl_pose.M(i, j);

  return tip_pose;
}

void PBVSController::publish_feedback(
  const std::chrono::time_point<std::chrono::high_resolution_clock> & task_time_init,
  const Eigen::VectorXd & error, const Eigen::VectorXd & camera_twist,
  const Eigen::VectorXd & tip_link_twist)
{
  const auto active_goal = *rt_pbvs_active_goal_.readFromNonRT();

  auto action_feedback = std::make_shared<PBVSAction::Feedback>();

  // Fill data
  action_feedback->elapsed_time =
    rclcpp::Duration(std::chrono::high_resolution_clock::now() - task_time_init);

  Eigen::Isometry3d h = Eigen::Isometry3d::Identity();
  h.linear() = (Eigen::AngleAxisd(error(3), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(error(4), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(error(5), Eigen::Vector3d::UnitX()))
                 .toRotationMatrix();
  h.translation() = Eigen::Vector3d(error(0), error(1), error(2));
  action_feedback->error = tf2::toMsg(h);

  action_feedback->translation_error = error.head<3>().norm();
  action_feedback->rotation_error = error.tail<3>().norm();

  tf2::toMsg(Eigen::Vector3d(camera_twist.head<3>()), action_feedback->camera_twist.linear);
  tf2::toMsg(Eigen::Vector3d(camera_twist.tail<3>()), action_feedback->camera_twist.angular);

  tf2::toMsg(Eigen::Vector3d(tip_link_twist.head<3>()), action_feedback->tip_link_twist.linear);
  tf2::toMsg(Eigen::Vector3d(tip_link_twist.tail<3>()), action_feedback->tip_link_twist.angular);

  // Publish
  active_goal->setFeedback(action_feedback);
}

}  // namespace visual_servoing_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  visual_servoing_controller::PBVSController, controller_interface::ControllerInterface)
