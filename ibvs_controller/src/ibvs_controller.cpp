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

#include "ibvs_controller/ibvs_controller.hpp"

namespace visual_servoing_controller
{
IBVSController::IBVSController()
: controller_interface::ControllerInterface(), dof_(0), num_cmd_joints_(0)
{
}

IBVSController::CallbackReturn IBVSController::on_init()
{
  // Initialize the parameter handler
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ibvs_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration IBVSController::command_interface_configuration() const
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

controller_interface::InterfaceConfiguration IBVSController::state_interface_configuration() const
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

controller_interface::CallbackReturn IBVSController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Prepare the controller for activation.
  RCLCPP_INFO(get_node()->get_logger(), "Configuring IBVSController");

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

  // Camera
  camera_info_topic_ = params_.camera.camera_info_topic_name;

  // Detection
  detection_topic_name_ = params_.detection.topic_name;
  detection_topic_type_ = params_.detection.topic_type;
  detection_data_.detection_timeout_ms = params_.detection.timeout * 1000;

  // Control
  Eigen::VectorXd pid_p(2), pid_i(2), pid_d(2);
  pid_p << params_.control.p, params_.control.p;
  pid_i << params_.control.i, params_.control.i;
  pid_d << params_.control.d, params_.control.d;

  pid_ = std::make_shared<PID2D>(pid_p, pid_i, pid_d);

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
  RCLCPP_INFO(get_node()->get_logger(), "╠═ Camera info topic '%s'", camera_info_topic_.c_str());
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
  ibvs_action_server_ = rclcpp_action::create_server<IBVSAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/image_based_visual_servoing",
    std::bind(&IBVSController::goal_received_callback, this, _1, _2),
    std::bind(&IBVSController::goal_cancelled_callback, this, _1),
    std::bind(&IBVSController::goal_accepted_callback, this, _1));

  // Create subscribers & publishers
  detection_data_.first_detection_received = false;
  detection_data_.last_detection_valid = false;
  camera_info_received_ = false;

  if (detection_topic_type_ == "geometry_msgs/Point")

  {
    point_subs_ = get_node()->create_subscription<geometry_msgs::msg::Point>(
      std::string(get_node()->get_name()) + "/" + detection_topic_name_, 1,
      std::bind(&IBVSController::point_callback, this, std::placeholders::_1));
  }
  else if (detection_topic_type_ == "geometry_msgs/PointStamped")
  {
    point_stamped_subs_ = get_node()->create_subscription<geometry_msgs::msg::PointStamped>(
      std::string(get_node()->get_name()) + "/" + detection_topic_name_, 1,
      std::bind(&IBVSController::point_stamped_callback, this, std::placeholders::_1));
  }

  camera_info_subs_ = get_node()->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, 1,
    std::bind(&IBVSController::camera_info_callback, this, std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IBVSController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating IBVSController");

  // Check if camera info available
  if (!camera_info_received_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Camera info not received yet: Verify camera status and camera info topic");
    return controller_interface::CallbackReturn::ERROR;
  }

  image_height_ = camera_info_.height;
  image_width_ = camera_info_.width;
  cx_ = camera_info_.k[2];
  cy_ = camera_info_.k[5];
  fx_ = camera_info_.k[0];
  fy_ = camera_info_.k[4];

  RCLCPP_INFO(get_node()->get_logger(), "Image size: %d x %d", image_width_, image_height_);
  RCLCPP_INFO(
    get_node()->get_logger(), "Camera parameters: fx %f, fx %f, cx %f, cy %f", fx_, fy_, cx_, cy_);

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
  ibvs_task_.task_active = false;

  /// Log
  // Joint positions
  std::ostringstream oss;
  for (double d : joint_positions_) oss << d << ' ';

  RCLCPP_INFO(get_node()->get_logger(), "Initial joint positions: [ %s]'", oss.str().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IBVSController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating IBVSController");

  // Set all values to zero
  joint_positions_ = Eigen::VectorXd::Zero(dof_);
  joint_velocities_ = Eigen::VectorXd::Zero(dof_);
  joint_position_commands_ = Eigen::VectorXd::Zero(dof_);
  joint_velocity_commands_ = Eigen::VectorXd::Zero(dof_);
  joint_position_commands_prev_ = Eigen::VectorXd::Zero(dof_);
  joint_velocity_commands_prev_ = Eigen::VectorXd::Zero(dof_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IBVSController::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating IBVSController");

  // Reset subscribers
  point_subs_.reset();
  point_stamped_subs_.reset();
  camera_info_subs_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse IBVSController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const IBVSAction::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "New IBVS goal received...");

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

  RCLCPP_INFO(get_node()->get_logger(), "Starting new IBVS action goal 🚀");

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse IBVSController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<IBVSAction>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  // Manage active goal
  const auto active_goal = *rt_ibvs_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Canceling active action goal: Cancel callback received.");

    // Mark the current goal as canceled
    rt_has_pending_goal_ = false;
    auto action_res = std::make_shared<IBVSAction::Result>();
    // Set result values - ToDo
    active_goal->setCanceled(action_res);
    rt_ibvs_active_goal_.writeFromNonRT(RealtimeIBVSGoalHandlePtr());

    ibvs_task_.task_active = false;
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void IBVSController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<IBVSAction>> goal_handle)
{
  // mark a pending goal
  rt_has_pending_goal_ = true;

  // Update new IBVS task

  preempt_active_goal();

  // Set IBVS task values
  ibvs_task_.pixel_tolerance = goal_handle->get_goal()->pixel_tolerance;

  ibvs_task_.task_timeout_ms = duration_to_milliseconds(goal_handle->get_goal()->task_timeout);
  ibvs_task_.target_search_timeout_ms =
    duration_to_milliseconds(goal_handle->get_goal()->target_search_timeout);
  ibvs_task_.target_lost_timeout_ms =
    duration_to_milliseconds(goal_handle->get_goal()->target_lost_timeout);
  ibvs_task_.detection_validity_time_ms = params_.detection.timeout * 1000;

  ibvs_task_.task_time_init = std::chrono::high_resolution_clock::now();
  ibvs_task_.target_search_time_init = std::chrono::high_resolution_clock::now();

  ibvs_task_.initial_target_found = false;
  ibvs_task_.target_lost = false;

  // Check if destination provided in goal
  ibvs_task_.mantain_pixel = goal_handle->get_goal()->mantain_pixel;
  if (!ibvs_task_.mantain_pixel)
  {
    tf2::fromMsg(goal_handle->get_goal()->destination, ibvs_task_.target_destination);
  }

  ibvs_task_.task_active = true;

  // Z distance management
  ibvs_task_.z_distance_in_detection = goal_handle->get_goal()->z_distance_in_detection;

  if (!ibvs_task_.z_distance_in_detection)
    ibvs_task_.predefined_z = goal_handle->get_goal()->predefined_z;

  // Goal info
  if (!ibvs_task_.mantain_pixel)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "New target destination - Pixel X: %f - Pixel Y: %f",
      ibvs_task_.target_destination.x(), ibvs_task_.target_destination.y());
  }

  // Allowed axes
  ibvs_task_.allowed_axes_mask = Eigen::Matrix<double, 6, 6>::Identity();

  if (!goal_handle->get_goal()->all_axes)
  {
    for (size_t i = 0; i < goal_handle->get_goal()->allowed_axes.size(); i++)
      ibvs_task_.allowed_axes_mask(i, i) = goal_handle->get_goal()->allowed_axes[i];
  }

  // Update the active goal
  RealtimeIBVSGoalHandlePtr rt_goal = std::make_shared<RealtimeIBVSGoalHandle>(goal_handle);
  rt_goal->execute();
  rt_ibvs_active_goal_.writeFromNonRT(rt_goal);

  // Delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeIBVSGoalHandle::runNonRealtime, rt_goal));
}

void IBVSController::send_result(bool succeed, int error_code, const std::string & error_string)
{
  const auto active_goal = *rt_ibvs_active_goal_.readFromNonRT();

  auto action_res = std::make_shared<IBVSAction::Result>();

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

void IBVSController::preempt_active_goal()
{
  const auto active_goal = *rt_ibvs_active_goal_.readFromNonRT();
  if (active_goal)
  {
    auto action_res = std::make_shared<IBVSAction::Result>();

    action_res->error_code = IBVSAction::Result::NEW_GOAL_RECEIVED;
    action_res->error_string = "Current goal cancelled due to new incoming action.";
    active_goal->setCanceled(action_res);

    rt_ibvs_active_goal_.writeFromNonRT(RealtimeIBVSGoalHandlePtr());
  }
}

Eigen::VectorXd IBVSController::change_twist_reference(
  const Eigen::VectorXd & twist, const Eigen::Isometry3d & twist_reference)
{
  // Declare new twist
  Eigen::VectorXd new_twist = Eigen::VectorXd::Zero(6);

  new_twist.head<3>() = twist_reference.rotation() * twist.head<3>();
  new_twist.tail<3>() = twist_reference.rotation() * twist.tail<3>();

  return new_twist;
}

Eigen::VectorXd IBVSController::move_twist(const Eigen::VectorXd & twist, const Eigen::Vector3d & q)
{
  // Declare new twist
  Eigen::VectorXd new_twist = Eigen::VectorXd::Zero(6);

  new_twist.head<3>() = twist.head<3>() + q.cross(twist.tail<3>());
  new_twist.tail<3>() = twist.tail<3>();

  return new_twist;
}

Eigen::VectorXd IBVSController::calculate_next_joint_positions(
  const Eigen::VectorXd & joint_positions, const Eigen::VectorXd & twist, double dt)
{
  Eigen::VectorXd next_joint_position = Eigen::VectorXd::Zero(6);

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

controller_interface::return_type IBVSController::update(
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
  if (ibvs_task_.task_active && detection_data_.last_detection_valid)
  {
    // Manage target lost
    ibvs_task_.target_lost = false;

    // Get tip pose
    base_link_H_tip_ = get_tip_pose(joint_positions_);

    // IBVS
    double u = detection_data_.last_detection_point.x() - cx_;
    double v = detection_data_.last_detection_point.y() - cy_;
    double destination_u = ibvs_task_.target_destination.x() - cx_;
    double destination_v = ibvs_task_.target_destination.y() - cy_;

    double z = ibvs_task_.z_distance_in_detection ? detection_data_.last_detection_point.z()
                                                  : ibvs_task_.predefined_z;

    // Calculate vision error
    Eigen::VectorXd vision_error(2);

    vision_error << destination_u - u, destination_v - v;

    // RCLCPP_INFO(
    //   get_node()->get_logger(), "vision_error: %3.3f %3.3f", vision_error(0), vision_error(1));

    // PID
    Eigen::VectorXd pixel_twist = pid_->calculate(vision_error, period.seconds());

    // RCLCPP_INFO(
    //   get_node()->get_logger(), "pixel_twist: %3.3f %3.3f", pixel_twist(0), pixel_twist(1));

    // Get image Jacobian
    Eigen::Matrix<double, 2, 6> j = get_image_jacobian(u, v, z, fx_, fy_);

    // Mute not allowed axis
    j = j * ibvs_task_.allowed_axes_mask;

    // Calculate pseudo-inverse
    Eigen::Matrix<double, 6, 2> j_inv = dampedPseudoInverse(j, 0.1);

    // Camera twist
    Eigen::VectorXd camera_twist = j_inv * pixel_twist;

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

    // RCLCPP_INFO(
    //   get_node()->get_logger(), "camera_twist_base_link: %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f",
    //   camera_twist_base_link(0), camera_twist_base_link(1), camera_twist_base_link(2),
    //   camera_twist_base_link(3), camera_twist_base_link(4), camera_twist_base_link(5));

    // Transfer twist to tip link
    Eigen::VectorXd twist_base_link_tip = move_twist(
      camera_twist_base_link,
      (base_link_H_tip_ * tip_H_camera_).translation() - base_link_H_tip_.translation());

    // RCLCPP_INFO(
    //   get_node()->get_logger(), "camera_twist_base_link: %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f",
    //   camera_twist_base_link(0), camera_twist_base_link(1), camera_twist_base_link(2),
    //   camera_twist_base_link(3), camera_twist_base_link(4), camera_twist_base_link(5));

    // Get joint position commands
    joint_position_commands_ =
      calculate_next_joint_positions(joint_positions_, twist_base_link_tip, period.seconds());

    // Manage destination & tolerances
    manage_tolerances(vision_error);

    // Manage feedback
    if (feedback_active_)
    {
      // Publish
      publish_feedback(ibvs_task_.task_time_init, vision_error, camera_twist, twist_base_link_tip);
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

void IBVSController::point_callback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  // Manage first detection
  if (!detection_data_.first_detection_received) detection_data_.first_detection_received = true;

  // Get pose & time
  tf2::fromMsg(*msg, detection_data_.last_detection_point);
  detection_data_.last_detection_time = std::chrono::high_resolution_clock::now();
}

void IBVSController::point_stamped_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // Manage first detection
  if (!detection_data_.first_detection_received) detection_data_.first_detection_received = true;

  // Get pose & time
  tf2::fromMsg(msg->point, detection_data_.last_detection_point);
  detection_data_.last_detection_time = std::chrono::high_resolution_clock::now();
}

void IBVSController::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  // Store & set flag
  camera_info_ = *msg;
  camera_info_received_ = true;
}

Eigen::VectorXd IBVSController::to_vector(const Eigen::Isometry3d & h)
{
  Eigen::VectorXd v(6);

  // Translation
  v.head<3>() = h.translation();

  // Rotation
  Eigen::AngleAxisd aa(h.rotation());

  v.tail<3>() = aa.axis() * aa.angle();

  return v;
}

double IBVSController::duration_to_milliseconds(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) * 1000.0 + static_cast<double>(d.nanosec) / 1e6;
}

double IBVSController::get_duration_millis(
  const std::chrono::time_point<std::chrono::high_resolution_clock> & init_time)
{
  auto finish_time = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(finish_time - init_time).count();
}

bool IBVSController::valid_goal(std::shared_ptr<const IBVSAction::Goal> goal)
{
  if (goal->pixel_tolerance < 0)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Pixel tolerance must be higher than 0");
    return false;
  }

  if (!goal->all_axes)
  {
    if (goal->allowed_axes.size() != 6)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "If not all axes allowed, 6 boolean values MUST be provided in 'allowed_axes' variable");
      return false;
    }
  }

  return true;
}

void IBVSController::manage_detection()
{
  // Check if last detection is valid
  detection_data_.last_detection_valid =
    (get_duration_millis(detection_data_.last_detection_time) <
     ibvs_task_.detection_validity_time_ms);

  // Manage first detection
  if (
    ibvs_task_.task_active && !ibvs_task_.initial_target_found &&
    detection_data_.first_detection_received && detection_data_.last_detection_valid)
  {
    // Manage mantain position type task
    if (ibvs_task_.mantain_pixel)
    {
      ibvs_task_.target_destination = detection_data_.last_detection_point;

      RCLCPP_INFO(
        get_node()->get_logger(), "New target destination - Pixel X: %f - Pixel Y: %f",
        ibvs_task_.target_destination.x(), ibvs_task_.target_destination.y());

      if (ibvs_task_.z_distance_in_detection)
      {
        RCLCPP_INFO(get_node()->get_logger(), "Z distance expected in received point");
      }
      else
      {
        RCLCPP_INFO(get_node()->get_logger(), "Predefined Z distance: %f", ibvs_task_.predefined_z);
      }
    }

    ibvs_task_.initial_target_found = true;
  }
  // Check if target lost
  else if (
    ibvs_task_.task_active && ibvs_task_.initial_target_found &&
    !detection_data_.last_detection_valid && !ibvs_task_.target_lost)
  {
    ibvs_task_.target_lost = true;
    ibvs_task_.target_lost_time_init = std::chrono::high_resolution_clock::now();
  }
  // Check if target found
  else if (
    ibvs_task_.task_active && ibvs_task_.initial_target_found &&
    detection_data_.last_detection_valid && ibvs_task_.target_lost)
  {
    ibvs_task_.target_lost = false;
  }
}

void IBVSController::manage_timeouts()
{
  // Verify if task timeout reached
  if (
    ibvs_task_.task_active && ibvs_task_.task_timeout_ms > 0 &&
    get_duration_millis(ibvs_task_.task_time_init) > ibvs_task_.task_timeout_ms)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Finishing IBVS task: Task timeout reached ❌");
    send_result(false, IBVSAction::Result::TASK_TIMEOUT, "Task timeout reached");
    ibvs_task_.task_active = false;
  }
  // Verify if target search timeout reached
  else if (
    ibvs_task_.task_active && !ibvs_task_.initial_target_found &&
    ibvs_task_.target_search_timeout_ms > 0 &&
    get_duration_millis(ibvs_task_.target_search_time_init) > ibvs_task_.target_search_timeout_ms)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Finishing IBVS task: Target search timeout reached ❌");
    send_result(false, IBVSAction::Result::TARGET_SEARCH_TIMEOUT, "Target search timeout reached");
    ibvs_task_.task_active = false;
  }
  // Verify if target lost timeout reached
  else if (
    ibvs_task_.task_active && ibvs_task_.target_lost && ibvs_task_.target_lost_timeout_ms > 0 &&
    get_duration_millis(ibvs_task_.target_lost_time_init) > ibvs_task_.target_lost_timeout_ms)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Finishing IBVS task: Target lost timeout reached ❌");
    send_result(false, IBVSAction::Result::TARGET_LOST_TIMEOUT, "Target lost timeout reached");
    ibvs_task_.task_active = false;
  }
}

void IBVSController::manage_tolerances(const Eigen::VectorXd & vision_error)
{
  // Verify if task timeout reached
  if (
    ibvs_task_.task_active && ibvs_task_.task_timeout_ms > 0 &&
    vision_error.head<2>().norm() < ibvs_task_.pixel_tolerance)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Finishing IBVS task: Tolerances reached 🎯");
    send_result(true, 0, "");
    ibvs_task_.task_active = false;
  }
}

bool IBVSController::get_joint_limits(const std::vector<std::string> & joint_names)
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

bool IBVSController::get_kinematics(const std::string & base_link, const std::string & tip_link)
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

bool IBVSController::getTransform(
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

void IBVSController::read_joint_state(
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

Eigen::Isometry3d IBVSController::get_tip_pose(const Eigen::VectorXd & joint_positions)
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

Eigen::Matrix<double, 2, 6> IBVSController::get_image_jacobian(
  double u, double v, double z, double fx, double fy)
{
  Eigen::Matrix<double, 2, 6> m;

  m(0, 0) = -fx / z;
  m(0, 1) = 0.0;
  m(0, 2) = u / z;
  m(0, 3) = (u * v) / fy;
  m(0, 4) = -(fx + (u * u) / fx);
  m(0, 5) = (fx / fy) * v;

  m(1, 0) = 0.0;
  m(1, 1) = -fy / z;
  m(1, 2) = v / z;
  m(1, 3) = fy + (v * v) / fy;
  m(1, 4) = -(u * v) / fx;
  m(1, 5) = -(fy / fx) * u;

  return m;
}

void IBVSController::publish_feedback(
  const std::chrono::time_point<std::chrono::high_resolution_clock> & task_time_init,
  const Eigen::VectorXd & error, const Eigen::VectorXd & camera_twist,
  const Eigen::VectorXd & tip_link_twist)
{
  const auto active_goal = *rt_ibvs_active_goal_.readFromNonRT();

  auto action_feedback = std::make_shared<IBVSAction::Feedback>();

  // Fill data
  action_feedback->elapsed_time =
    rclcpp::Duration(std::chrono::high_resolution_clock::now() - task_time_init);

  action_feedback->error.x = error(0);
  action_feedback->error.y = error(1);
  action_feedback->error.z = 0.0;

  action_feedback->pixel_error = error.norm();

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
  visual_servoing_controller::IBVSController, controller_interface::ControllerInterface)
