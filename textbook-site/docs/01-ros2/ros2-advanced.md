# 2.2 ROS 2 Advanced Concepts

## 2.2.1 Launch Files

In ROS 2, `launch` files are used to start multiple ROS 2 nodes simultaneously and to define their configuration (e.g., parameters, remappings). This provides a structured way to manage complex robotic systems, which often involve dozens of interconnected nodes. ROS 2 launch files can be written in either Python or XML, with Python being the more flexible and recommended approach for complex logic.

**Key Features of ROS 2 Launch Files:**
-   **Node Execution:** Start one or more nodes.
-   **Parameter Assignment:** Pass static parameters to nodes.
-   **Remapping:** Change node, topic, or service names to avoid conflicts or customize communication paths.
-   **Conditional Execution:** Use `IfCondition` or `UnlessCondition` to run nodes based on external conditions.
-   **Environment Variables:** Set environment variables for launched processes.
-   **Includes:** Reuse launch file logic from other files or packages.

### Example 2.3: Python Launch File

This example launches the `talker` and `listener` nodes from the previous section with custom names and parameters.

Create `my_ros2_package/launch/my_example_launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_ros2_package',
            executable='talker',
            name='custom_talker', # Custom node name
            parameters=[
                {'publish_frequency': 1.0} # Example parameter
            ],
            remappings=[
                ('/chatter', '/my_custom_chatter') # Remap topic name
            ]
        ),
        Node(
            package='my_ros2_package',
            executable='listener',
            name='custom_listener', # Custom node name
            remappings=[
                ('/chatter', '/my_custom_chatter') # Remap topic name
            ]
        )
    ])
```
To run this launch file:
```bash
ros2 launch my_ros2_package my_example_launch.py
```
*Note: For the parameter `publish_frequency` to work, you would need to modify your `talker.py` to declare and use this parameter.*

## 2.2.2 Robot Motor Control and Joint States

Controlling a humanoid robot's motors and understanding its joint states are fundamental for any physical interaction. ROS 2 provides frameworks like `ros2_control` to standardize hardware interaction.

### `ros2_control`
`ros2_control` is a set of packages designed to provide a generic control architecture for robots within ROS 2. It abstracts away the hardware-specific details, allowing controllers to be written once and deployed on different robot platforms.

**Key Components:**
-   **Hardware Interface:** Communication layer between `ros2_control` and the actual robot hardware (motors, encoders).
-   **Controllers:** Implement the control logic (e.g., PID controllers for joint position, velocity, or effort).
-   **Controller Manager:** Manages the lifecycle and switching of controllers.

### Joint States
`sensor_msgs/msg/JointState` messages are crucial for understanding the current configuration of a robot. These messages contain arrays of `name`, `position`, `velocity`, and `effort` for each joint.

-   **`JointStatePublisher`:** A node that takes joint angles from either GUI or parameters and publishes them as `JointState` messages. Useful for simple visualization.
-   **`RobotStatePublisher`:** A node that reads the robot's URDF/XACRO description and the `JointState` messages, then uses TF2 to broadcast the transforms for all robot links. This allows visualization of the robot model in Rviz2.

### Workflow: Publishing Joint States

1.  **URDF:** Ensure your robot's URDF file correctly defines all joints.
2.  **`JointStatePublisher`:** If you have a real robot or simulation, its hardware interface will publish `JointState` messages. For simple visualization or testing, you can use `joint_state_publisher_gui` (from `joint_state_publisher_gui` package) to manually manipulate joints.
3.  **`RobotStatePublisher`:** Launch this node alongside your `joint_state_publisher` and your robot model in Rviz2 will reflect the joint positions.

```bash
# Example launch (assuming your_robot_description is available)
ros2 launch urdf_tutorial display.launch.py model:=src/my_robot/urdf/my_robot.urdf.xacro
```

## 2.2.3 Sensor Streams

Robots rely heavily on sensor data to perceive their environment. ROS 2 provides standard message types for common sensors, facilitating interoperability.

**Common Sensor Message Types:**
-   **`sensor_msgs/msg/Image`:** Raw or processed camera images.
-   **`sensor_msgs/msg/LaserScan`:** Data from 2D LiDAR scanners.
-   **`sensor_msgs/msg/PointCloud2`:** 3D point cloud data (e.g., from 3D LiDAR, depth cameras).
-   **`sensor_msgs/msg/Imu`:** Inertial Measurement Unit data (acceleration, angular velocity, orientation).
-   **`nav_msgs/msg/Odometry`:** Robot's estimated position and orientation relative to a fixed frame.

Integrating sensor streams involves:
1.  **Driver Node:** A node that interfaces with the physical sensor hardware (or simulation API) and publishes data in the appropriate ROS 2 message type.
2.  **Processing Node(s):** Other nodes subscribe to these sensor topics to perform tasks like filtering, object detection, mapping, or localization.

## 2.2.4 Multi-Node Systems

Real-world humanoid robotics systems are inherently distributed. Designing multi-node systems requires careful consideration of:

-   **Modularity:** Each node should have a single, well-defined responsibility.
-   **Communication Patterns:** Choose appropriate communication mechanisms (topics, services, actions) based on data flow requirements.
-   **Data Consistency:** Ensure that data used by different nodes is synchronized or timestamped correctly to avoid inconsistencies (e.g., TF2 timestamps).
-   **Lifecycle Management:** ROS 2 Lifecycle Nodes provide a way to manage the state of nodes (unconfigured, inactive, active), enabling robust system startups and shutdowns.

**Example: Simple Navigation Stack Data Flow**
-   **LiDAR Node** publishes `sensor_msgs/msg/LaserScan`.
-   **Odometry Node** publishes `nav_msgs/msg/Odometry`.
-   **SLAM Node** subscribes to `LaserScan` and `Odometry`, publishes `nav_msgs/msg/OccupancyGrid` (map) and `/tf` (robot pose).
-   **Path Planning Node** subscribes to `OccupancyGrid` and `/tf`, publishes `nav_msgs/msg/Path`.
-   **Motor Control Node** subscribes to `nav_msgs/msg/Twist` (velocity commands), publishes `sensor_msgs/msg/JointState`.

## 2.2.5 Action Servers

While topics provide streaming data and services offer synchronous request-response, actions are designed for long-running, goal-oriented tasks with periodic feedback. They are ideal for tasks like "move to a location," "pick up an object," or "perform a complex manipulation sequence."

An action consists of three parts:
-   **Goal:** The desired outcome of the action (e.g., target pose, object ID).
-   **Feedback:** Periodic updates on the action's progress (e.g., current position during movement, percentage complete).
-   **Result:** The final outcome of the action (success/failure, final state).


6.  **Update `setup.py` with new entry points.** Build, source, then run the server in one terminal and the client in another.

## 2.2.6 Advanced URDF

Beyond basic links and joints, URDF can specify more detailed physical properties and connect to control interfaces.

### Inertia
The `<inertial>` element within a `<link>` defines its mass and inertia tensor. This is critical for accurate physics simulation.

```xml
<inertial>
  <mass value="1.0"/> <!-- Mass in kg -->
  <origin xyz="0 0 0.05" rpy="0 0 0"/> <!-- Center of mass relative to link frame -->
  <!-- Inertia tensor (values around 0 are common for symmetric shapes) -->
  <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
</inertial>
```

### Joint Limits and Dynamics
The `<limit>` and `<dynamics>` elements within a `<joint>` define its physical constraints and dynamic properties.

```xml
<joint name="revolute_joint" type="revolute">
  ...
  <limit lower="-1.57" upper="1.57" # min/max angle in radians
         effort="100"   # max effort (N*m for revolute, N for prismatic)
         velocity="10"/> # max velocity (rad/s or m/s)
  <dynamics damping="0.1" friction="0.01"/> # viscous damping and static friction
</joint>
```

### Transmissions
The `<transmission>` element, typically defined in a separate XACRO file and included, describes the mechanical coupling between an actuator (motor) and a joint. This is used by `ros2_control` to map controller commands to hardware signals.

```xml
<!-- Example transmission block, typically in a .xacro file -->
<transmission name="joint1_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor1">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## 2.2.7 Example: Controlling Humanoid Joints

Controlling humanoid joints using `ros2_control` typically involves:

1.  **Robot Description (URDF/XACRO):** The robot's URDF includes `<ros2_control>` blocks and `<transmission>` elements for each controllable joint.
2.  **Hardware Interface:** An implementation that communicates with the robot's motors (e.g., a custom `ros2_control` hardware interface for a specific motor driver).
3.  **Controllers:** Often `JointPositionController`, `JointVelocityController`, or `JointTrajectoryController` from `ros2_controllers` are used. These take commands (e.g., desired joint angles) and send them through `ros2_control` to the hardware.

### High-Level Workflow for Joint Control:
1.  **Design URDF:** Define all links, joints, inertias, limits, and transmissions for your humanoid.
2.  **Implement Hardware Interface:** Write (or use an existing) `ros2_control` hardware interface that reads from your robot's sensors (encoders) and writes to its actuators (motors).
3.  **Launch Controller Manager:** Start the `ros2_control` controller manager.
4.  **Load and Start Controllers:** Use `ros2 control load_controller` and `ros2 control set_controller_state` commands (or a launch file) to load and activate appropriate joint controllers (e.g., `joint_state_broadcaster`, `joint_trajectory_controller`).
5.  **Send Commands:** Publish `sensor_msgs/msg/JointTrajectory` messages to the `joint_trajectory_controller`'s command topic (e.g., `/joint_trajectory_controller/joint_trajectory`) from a separate node or script. The controller will then execute the trajectory on the robot.

**Table 2.3: ROS 2 Control Interfaces**

| Interface Type                 | Description                                     | Common Use Case              |
| :----------------------------- | :---------------------------------------------- | :--------------------------- |
| `PositionJointInterface`       | Controls joint position directly.               | Manipulators, arms           |
| `VelocityJointInterface`       | Controls joint velocity directly.               | Wheeled robots, continuous joints |
| `EffortJointInterface`         | Controls joint effort (torque/force).           | Force control, compliant motion |
| `JointStateInterface`          | Reads current joint positions, velocities, efforts. | Feedback, visualization, planning |

---

## 2.2.8 Lab Task: Humanoid Arm Action Server with Advanced URDF

**Objective:** Extend the previous URDF and implement an Action Server to control a simulated humanoid arm to specific joint positions using an advanced URDF.

**Steps:**
1.  **Refine URDF:** Enhance your 3-link arm URDF to include:
    *   Realistic inertial properties for each link.
    *   `lower` and `upper` limits, `effort`, and `velocity` limits for your two revolute joints.
    *   Add a `<ros2_control>` block and `<transmission>` tags for each revolute joint, specifying a `hardware_interface/PositionJointInterface`.
2.  **Create a Simple Hardware Interface (Mock):** Since we don't have physical hardware, create a mock `ros2_control` hardware interface that simply stores commanded positions and publishes current positions (which can just echo the commanded ones, or slowly interpolate). This mock can be a simple Python script or a dummy C++ class.
3.  **Implement a `JointTrajectoryActionServer`:** Create a new ROS 2 Python node (`arm_controller_action_server.py`) that acts as an Action Server for the standard `control_msgs/action/FollowJointTrajectory` action.
    *   This server will receive a `JointTrajectory` goal.
    *   It should parse the trajectory points (desired joint positions, velocities, accelerations over time).
    *   Simulate sending these commands to your mock hardware interface (e.g., by updating internal variables).
    *   Periodically publish `sensor_msgs/msg/JointState` messages representing the arm's current (simulated) position.
    *   Provide feedback (current joint state) and report success/failure based on trajectory completion.
4.  **Create a `JointTrajectoryActionClient`:** Implement a Python node (`arm_controller_action_client.py`) that creates a `FollowJointTrajectory` goal (e.g., move the arm to a predefined sequence of two joint configurations) and sends it to your action server.
5.  **Launch File:** Create a Python launch file to start your `robot_state_publisher`, the mock hardware interface, the `JointTrajectoryActionServer`, and the `arm_controller_action_client`.
6.  **Visualize:** Use Rviz2 to visualize your robot model and its movement as the action client sends goals to the server.

**Deliverables:**
-   Updated `my_simple_robot` package with:
    -   Enhanced URDF.
    -   Mock `ros2_control` hardware interface (e.g., `mock_arm_hardware.py`).
    -   `arm_controller_action_server.py`.
    -   `arm_controller_action_client.py`.
    -   Launch file (`arm_control.launch.py`).
-   Demonstration of the arm moving through a trajectory in Rviz2.

---

## 2.2.9 Multiple Choice Questions

1.  What is the primary advantage of using Python launch files over XML launch files in ROS 2?
    a) Faster execution speed.
    b) Support for more complex logic and conditional execution.
    c) Easier syntax for basic node launches.
    d) Better integration with C++ nodes.
    **Answer: b**

2.  Which `ros2_control` component is responsible for abstracting away hardware-specific communication details?
    a) Controller Manager
    b) Controllers
    c) Hardware Interface
    d) JointStatePublisher
    **Answer: c**

3.  Which of the following is NOT a standard ROS 2 sensor message type?
    a) `sensor_msgs/msg/Image`
    b) `sensor_msgs/msg/LaserScan`
    c) `custom_msgs/msg/MySensorData`
    d) `sensor_msgs/msg/Imu`
    **Answer: c** (While custom messages can be created, this is not a *standard* type)

4.  What is the main benefit of using ROS 2 Actions compared to Services for tasks like "move robot to target location"?
    a) Actions provide periodic feedback on progress.
    b) Actions are non-blocking for the client.
    c) Actions allow for goal cancellation.
    d) All of the above.
    **Answer: d**

5.  The `<transmission>` tag in URDF/XACRO is primarily used by which ROS 2 framework?
    a) TF2
    b) `ros2_control`
    c) Rviz2
    d) `rclpy`
    **Answer: b**

---

## 2.2.10 Exercises

1.  **Launch File Design:** You are tasked with launching a robot system that has two camera nodes, a LiDAR node, and a navigation node. Each camera needs a unique topic name (e.g., `/camera1/image` and `/camera2/image`), and the navigation node needs to receive its input from `/lidar_data`. Write a Python launch file snippet that achieves this using remapping.
2.  **`ros2_control` Concepts:** Explain the relationship between the hardware interface, controller manager, and controllers within the `ros2_control` framework. How do these components work together to enable modular robot control?
3.  **Action vs. Service:** A robot needs to perform a "door opening" routine that involves several steps (approach door, grasp handle, turn handle, pull/push, release). Explain why a ROS 2 Action would be a more suitable communication mechanism for this task than a ROS 2 Service. Detail how feedback and potential cancellation would be handled.
4.  **Advanced URDF:** Consider a joint in a humanoid robot's arm. Explain how specifying `lower` and `upper` limits, `effort`, and `velocity` in the URDF's `<limit>` tag is crucial for both simulation accuracy and safe operation on real hardware. Provide a scenario where ignoring one of these limits could lead to a problem.
5.  **Multi-Node Robustness:** You observe that your robot's navigation stack occasionally fails because the path planning node tries to generate a path before the SLAM node has provided a complete map. Propose two ROS 2 best practices (e.g., using specific communication patterns, lifecycle nodes, or parameter settings) that could mitigate this kind of timing-dependent failure in a multi-node system.