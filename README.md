# Visual Servoing Controllers

<p>
  <a href="https://github.com/aitor-ibarguren/visual_servoing_controllers/actions/workflows/ros2_jazzy_ci.yml">
    <img src="https://github.com/aitor-ibarguren/visual_servoing_controllers/actions/workflows/ros2_jazzy_ci.yml/badge.svg" alt="Build">
  </a>
</p>

The package includes ROS2 controllers for Visual Servoing tasks, generating robot movements based on vision detection data. Specifically, the repository contains the next controllers:

* A `visual_servoing_controller/PBVSController` for Position-Based Visual Servoing tasks where the controller generates twist commands based on a 6D pose received through topics.

* A `visual_servoing_controller/IBVSController` for Image-Based Visual Servoing tasks where the controller generates twist commands based on a 2D point (pixel position) received through topics.

> **⚠️ Important:** The controllers only manage the kinematic part of the Visual Servoing tasks, leaving the visual detection outside the controller. The controllers rely on a streaming of 2D/3D poses received through topics to calculate the robot movements, decoupling the target detection and tracking to facilitate the integration of different implementations.

## Position-Based Visual Servoing

### General Features

- ROS2 controller including an action server that executes Position-Based Visual Servoing tasks.
- The PBVS control law calculates a twist value, which is internally computed to generate joint positions using **KDL** to generate the Jacobian matrix and **Eigen** to calculate the pseudo-inverse using *SVD*.
- Allows an **open-loop** mode in which the previously commanded joint positions are used instead of the joint positions from the state interfaces, avoiding the injection of hardware feedback latency and transport delays into the command generation loop.
- The controller includes the option to **enable a feedback topic** to continuously publish the pose error and the twist both in camera and tip link of the kinematic chain.

### Configuration

The next lines show a snippet of the *YAML* file defining the configuration of the `visual_servoing_controller/PBVSController` controller for a UR16e robot:

```yaml
ur_pbvs_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    state_interfaces:
      - position
      - velocity
    command_interfaces:
      - position
    open_loop: true
    feedback_active: true
    kinematics:
      base_link: base_link
      tip_link: tool0
      camera_link: ur_camera
    detection:
      topic_name: detection
      topic_type: geometry_msgs/Pose
      detection_timeout: 0.1
    control:
      p: 2.0
      i: 0.0
      d: 0.0
      max_translation_speed: 0.250
      max_rotation_speed: 0.500
```

Besides the typical *joints*, *command_joints*, and *command_interfaces*, the controller includes the next parameters:

- **open_loop:** Enable an open-loop control (joint position commands from the previous update call are used to calculate the next commands).
- **feedback_active:**: Enable the publishing of the action feedback.
- **kinematics:**
  - **base_link:** Base link of the kinematic chain of the group.
  - **tip_link:** Tip link of the kinematic chain of the group.
  - **camera_link:** Camera link, origin of the vision detections, used to generate the twist commands.
- **detection:**
  - **topic_name:** Topic where the 3D poses of the vision detection module are received as `controller_name/topic_name`.
  - **topic_type:** The topic type, currently supporting `geometry_msgs/Pose` and `geometry_msgs/PoseStamped`.
  - **detection_timeout:** Timeout for the detection, defining the validity period the received 3D pose.
- **control:**
  - **p:** Proportional part P of the PID control system.
  - **i:** Integral part I of the PID control system.
  - **d:** Derivative part D of the PID control system.
  - **max_translation_speed:** Maximum translation speed of the camera frame, used to limit the generated twist.
  - **max_rotation_speed:** Maximum rotation speed of the camera frame, used to limit the generated twist.

These parameters allow defining the general features of the Position-Based Visual Servoing controller. Several other parameters can be set on real time with the action goal, as descrived in the next section.

### PBVS Action Parameters

The controller provides an action server to set PBVS tasks, defining the next parameters:

- **mantain_pose:** Defines if the controller must maintain the pose of the first received 3D detection, describing a *follow the current target* task.
- **destination:** The user can define an specific target pose (setting *mantain_pose* to false).
- **translation_tolerance:** 
The translation tolerance value used to determine if the positioning task is completed.
- **rotation_tolerance:** 
The rotation tolerance value used to determine if the positioning task is completed.
- **task_timeout:** 
The timeout of the complete task before aborting the action. If the timeout value is -1 (or negative), the action will continue until it is cancelled.
- **target_search_timeout:** 
The timeout until the target pose is received for the first time before aborting the action. If the timeout value is -1 (or negative), the action will wait until the target is found for the first time or it is cancelled.
- **target_lost_timeout:** 
The timeout whenever the target is lost (no more detection is received) before aborting the action. If the timeout value is -1 (or negative), the action will wait until the target is found again or it is cancelled.

These action parameters allow parametrizing different PBVS tasks, offering flexibility to create multiple behaviours with the same controller.

### Feedback

In order to enable the introspection of the internal control values when undesired behaviours such as instabilities and oscillations are found, the controller make use of the action feedback (activable through the *feedback_active* parameter). The custom feedback includes the next values:

- **elapsed_time:** Time since the action initialization.
- **error:** The error betweeen the desired pose and the current detection pose as `geometry_msgs::msg::Pose`.
- **received_wrench:** The wrench values received from the sensor.
- **translation_error:** Translation error in meters, extracted from the error as the translation norm.
- **rotation_error:** Rotation error in radians, extracted from the error through the angle-axis representation of the angle.
- **camera_twist:** The twist vector in the camera frame, calculated from the PID control law.
- **tip_link_twist:** The twist vector in the tip link frame, transferred from the camera link, and used to generate the next joint positions.

## Image-Based Visual Servoing

### General Features

- ROS2 controller including an action server that executes Image-Based Visual Servoing tasks.
- The IBVS control law initially calculates a pixel twist value which is transformed in a camera twist using the image Jacobian matrix. Subsequently, this camera twist is internally computed to generate joint positions. In both steps, the controller uses **KDL** to generate the Jacobian matrix and **Eigen** to calculate the pseudo-inverse using *SVD*.
- As the image Jacobian requires depth information, the controller supports the addition of this Z distance on the detection (Z dimension of the `geometry_msgs/Point` or `geometry_msgs/PointStamped` message) or setting a predefined Z distance for the complete task. The rest of the required information is automatically retrieved from the camera information topic.
- Due to kinematic redundancies in Image-Based Visual Servoing (IBVS), this controller includes a feature to enable or disable specific camera motion axes during 2D image-plane error correction (e.g., restricting movement to X/Y translation or pure rotation only). This ensures a predictable, task-tailored motion and prevents undesired robot movements.
- Allows an **open-loop** mode in which the previously commanded joint positions are used instead of the joint positions from the state interfaces, avoiding the injection of hardware feedback latency and transport delays into the command generation loop.
- The controller includes the option to **enable a feedback topic** to continuously publish the pixel error and the twist both in camera and tip link of the kinematic chain.

### Configuration

The next lines show a snippet of the *YAML* file defining the configuration of the `visual_servoing_controller/IBVSController` controller for a UR16e robot:

```yaml
ur_ibvs_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    state_interfaces:
      - position
      - velocity
    command_interfaces:
      - position
    open_loop: true
    feedback_active: true
    kinematics:
      base_link: base_link
      tip_link: tool0
      camera_link: ur_camera
    camera:
      camera_info_topic_name: /realsense/color/camera_info
    detection:
      topic_name: detection
      topic_type: geometry_msgs/Point
      detection_timeout: 0.1
    control:
      p: 2.0
      i: 0.0
      d: 0.0
      max_translation_speed: 0.250
      max_rotation_speed: 0.500

```

Besides the typical *joints*, *command_joints*, and *command_interfaces*, the controller includes the next parameters:

- **open_loop:** Enable an open-loop control (joint position commands from the previous update call are used to calculate the next commands).
- **feedback_active:**: Enable the publishing of the action feedback.
- **kinematics:**
  - **base_link:** Base link of the kinematic chain of the group.
  - **tip_link:** Tip link of the kinematic chain of the group.
  - **camera_link:** Camera link, origin of the vision detections, used to generate the twist commands.
- **camera:**
  - **camera_info_topic_name:** Topic where the camera information is received as `sensor_msgs/CameraInfo`.
- **detection:**
  - **topic_name:** Topic where the pixel position of the vision detection module are received as `controller_name/topic_name`. The subscriber expects a 3D point with optional depth information.
  - **topic_type:** The topic type, currently supporting `geometry_msgs/Point` and `geometry_msgs/PointStamped`.
  - **detection_timeout:** Timeout for the detection, defining the validity period the received 3D pose.
- **control:**
  - **p:** Proportional part P of the PID control system applied to the pixel error.
  - **i:** Integral part I of the PID control system applied to the pixel error.
  - **d:** Derivative part D of the PID control system applied to the pixel error.
  - **max_translation_speed:** Maximum translation speed of the camera frame, used to limit the generated twist.
  - **max_rotation_speed:** Maximum rotation speed of the camera frame, used to limit the generated twist.

These parameters allow defining the general features of the Image-Based Visual Servoing controller. Several other parameters can be set on real time with the action goal, as descrived in the next section.

### IBVS Action Parameters

The controller provides an action server to set IBVS tasks, defining the next parameters:

- **mantain_pixel:** Defines if the controller must maintain the pixel position of the first received 2D detection, describing a *follow the current target* task.
- **destination:** The user can define an specific target pixel (setting *mantain_pixel* to false).
- **z_distance_in_detection:** Defines if the received detection includes the depth information on the Z dimension (true) or there is a predefined depth distance along the complete task.
- **predefined_z:** The predefined depth distance used to calculate the image Jacobian when *z_distance_in_detection* is set to false.
- **Pixel_tolerance:** 
The pixel tolerance value used to determine if the positioning task is completed.
- **all_axes:** Defines if all the camera axes can be used to correct the pixel error (true) or just the axes specified by the user (false).
- **allowed_axes:** Boolean array defining which camera axes can be moved when *all_axes* is set to false.
- **task_timeout:** 
The timeout of the complete task before aborting the action. If the timeout value is -1 (or negative), the action will continue until it is cancelled.
- **target_search_timeout:** 
The timeout until the target pose is received for the first time before aborting the action. If the timeout value is -1 (or negative), the action will wait until the target is found for the first time or it is cancelled.
- **target_lost_timeout:** 
The timeout whenever the target is lost (no more detection is received) before aborting the action. If the timeout value is -1 (or negative), the action will wait until the target is found again or it is cancelled.

These action parameters allow parametrizing different PBVS tasks, offering flexibility to create multiple behaviours with the same controller.

### Feedback

In order to enable the introspection of the internal control values when undesired behaviours such as instabilities and oscillations are found, the controller make use of the action feedback (activable through the *feedback_active* parameter). The custom feedback includes the next values:

- **elapsed_time:** Time since the action initialization.
- **error:** The error betweeen the desired pixel and the current detection pixel as `geometry_msgs::msg::Point`.
- **pixel_error:** Complete pixel error extracted as the error norm.
- **camera_twist:** The twist vector in the camera frame, calculated from the PID control law.
- **tip_link_twist:** The twist vector in the tip link frame, transferred from the camera link, and used to generate the next joint positions.

## License

The *visual_servoing_controllers* repository has an Apache 2.0 license, as found in the [LICENSE](LICENSE) file.
