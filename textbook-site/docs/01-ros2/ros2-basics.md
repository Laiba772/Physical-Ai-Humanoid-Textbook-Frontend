# 2.1 ROS 2 Basics

## 2.1.1 Introduction to ROS 2

The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviors across a wide variety of robotic platforms. ROS 2 is the latest iteration, re-engineered to address the demands of real-time performance, multi-robot systems, and embedded hardware, leveraging a Data Distribution Service (DDS) for its communication backbone.

**Why ROS 2?**
-   **Distributed System:** Enables modularity, allowing different components of a robot's software to run on separate processes or even separate machines.
-   **Flexibility:** Supports multiple programming languages (C++, Python).
-   **Robust Communication:** DDS provides reliable, high-performance, and scalable data sharing.
-   **Tooling:** Offers a rich ecosystem of development, debugging, and visualization tools.
-   **Community Support:** Large and active community contributes to extensive resources and packages.

## 2.1.2 Nodes, Topics, Services

ROS 2 organizes robot software into a graph of executable processes called **nodes**. These nodes communicate with each other using various mechanisms:

### Nodes
A node is an executable process that performs computation. A robot system typically comprises many nodes, each responsible for a modular aspect of the robot's functionality (e.g., a node for reading lidar data, another for controlling motors, and another for path planning).

### Topics
Topics are the primary mechanism for asynchronous, many-to-many, publish-subscribe communication in ROS 2. Nodes publish data to topics, and other nodes subscribe to those topics to receive the data. Data is sent as messages, which are structured data types.

**Key Characteristics:**
-   **Unidirectional:** Data flows from publisher to subscriber.
-   **Decoupled:** Publishers and subscribers do not need direct knowledge of each other.
-   **Message Types:** Defined by `.msg` files (e.g., `std_msgs/msg/String`, `sensor_msgs/msg/LaserScan`).

### Services
Services enable synchronous, one-to-one, request-reply communication. A client node sends a request to a service server node, which performs a computation and returns a response. This is suitable for operations that require a direct response and are not typically continuously streamed.

**Key Characteristics:**
-   **Bidirectional:** Request from client, response from server.
-   **Synchronous:** Client waits for a response.
-   **Service Types:** Defined by `.srv` files (e.g., `example_interfaces/srv/AddTwoInts`).

**Table 2.1: Comparison of ROS 2 Communication Mechanisms**

| Feature         | Topics                                | Services                              |
| :-------------- | :------------------------------------ | :------------------------------------ |
| **Communication** | Asynchronous, Pub/Sub                 | Synchronous, Request/Reply            |
| **Direction**   | Unidirectional (Publisher to Subscriber) | Bidirectional (Client to Server)      |
| **Usage**       | Continuous data streams (sensor data, odometry) | Specific actions, queries (map requests, state changes) |
| **Latency**     | Typically low, best effort             | Higher, client waits for response     |
| **Scale**       | One-to-many, many-to-many             | One-to-one                            |

## 2.1.3 rclpy Fundamentals

`rclpy` is the Python client library for ROS 2. It provides an idiomatic Python interface to interact with the underlying ROS 2 C++ core (`rcl`).

**Core Concepts:**
-   **`rclpy.init()`:** Initializes the ROS 2 client library. Must be called before any other `rclpy` function.
-   **`rclpy.shutdown()`:** Cleans up all ROS 2 resources.
-   **`rclpy.create_node()`:** Creates a new ROS 2 node.
-   **`node.create_publisher()`:** Creates a publisher for a specific topic and message type.
-   **`node.create_subscription()`:** Creates a subscriber for a specific topic and message type, associating it with a callback function.
-   **`node.spin_once()`:** Processes a single pending event (e.g., incoming message, timer callback).
-   **`rclpy.spin()`:** Blocks until the node is shut down, continuously processing events.
-   **`node.get_logger()`:** Provides access to the node's logger for printing messages.

## 2.1.4 Simple Publisher/Subscriber

Let's create a basic publisher and subscriber using `rclpy`.

### Step-by-step Workflow

1.  **Create a ROS 2 Package:**
    Open your terminal and navigate to your ROS 2 workspace `src` directory.
    ```bash
    cd <your_ros2_ws>/src
    ros2 pkg create --build-type ament_python my_ros2_package --dependencies rclpy std_msgs
    ```

2.  **Publisher Node (`talker.py`):**
    Create `my_ros2_package/my_ros2_package/talker.py`:
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimplePublisher(Node):
        def __init__(self):
            super().__init__('simple_publisher') # Initialize the node with the name 'simple_publisher'
            self.publisher_ = self.create_publisher(String, 'chatter', 10) # Create a publisher for String messages on 'chatter' topic with queue size 10
            timer_period = 0.5 # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback) # Create a timer to call timer_callback every 0.5 seconds
            self.i = 0 # Counter for messages

        def timer_callback(self):
            msg = String() # Create a new String message
            msg.data = f'Hello ROS 2! Count: {self.i}' # Set message data
            self.publisher_.publish(msg) # Publish the message
            self.get_logger().info(f'Publishing: "{msg.data}"') # Log the published message
            self.i += 1

    def main(args=None):
        rclpy.init(args=args) # Initialize rclpy
        simple_publisher = SimplePublisher() # Create an instance of the publisher node
        rclpy.spin(simple_publisher) # Spin the node, allowing its callbacks to be executed
        simple_publisher.destroy_node() # Destroy the node once rclpy.spin() returns
        rclpy.shutdown() # Shut down rclpy

    if __name__ == '__main__':
        main()
    ```

3.  **Subscriber Node (`listener.py`):**
    Create `my_ros2_package/my_ros2_package/listener.py`:
    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimpleSubscriber(Node):
        def __init__(self):
            super().__init__('simple_subscriber') # Initialize the node with the name 'simple_subscriber'
            # Create a subscriber for String messages on 'chatter' topic.
            # When a message is received, it calls the 'listener_callback' method.
            self.subscription = self.create_subscription(
                String,
                'chatter',
                self.listener_callback,
                10)
            self.subscription # prevent unused variable warning

        def listener_callback(self, msg):
            self.get_logger().info(f'I heard: "{msg.data}"') # Log the received message

    def main(args=None):
        rclpy.init(args=args) # Initialize rclpy
        simple_subscriber = SimpleSubscriber() # Create an instance of the subscriber node
        rclpy.spin(simple_subscriber) # Spin the node
        simple_subscriber.destroy_node() # Destroy the node
        rclpy.shutdown() # Shut down rclpy

    if __name__ == '__main__':
        main()
    ```

4.  **Update `setup.py`:**
    Modify `my_ros2_package/setup.py` to include the entry points for your executable scripts.
    ```python
    from setuptools import find_packages, setup

    package_name = 'my_ros2_package'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/' + package_name, ['package.xml']),
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='your_name',
        maintainer_email='your_email@example.com',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'talker = my_ros2_package.talker:main',
                'listener = my_ros2_package.listener:main',
            ],
        },
    )
    ```

5.  **Build the Package:**
    Navigate to the root of your ROS 2 workspace and build your package.
    ```bash
    cd <your_ros2_ws>
    colcon build --packages-select my_ros2_package
    ```

6.  **Source the Workspace:**
    Before running, you need to source the workspace.
    ```bash
    # On Linux/macOS
    source install/setup.bash
    # On Windows (PowerShell)
    .\install\setup.ps1
    ```

7.  **Run the Nodes:**
    Open two separate terminals, source your workspace in each, and run:
    Terminal 1 (Publisher):
    ```bash
    ros2 run my_ros2_package talker
    ```
    Terminal 2 (Subscriber):
    ```bash
    ros2 run my_ros2_package listener
    ```
    You should see the subscriber printing the messages published by the talker.

**Table 2.2: Essential ROS 2 CLI Commands**

| Command                        | Description                                          | Example                                          |
| :----------------------------- | :--------------------------------------------------- | :----------------------------------------------- |
| `ros2 run <pkg_name> <node_name>` | Executes a node from a package.                      | `ros2 run my_ros2_package talker`                |
| `ros2 topic list`              | Lists all active topics.                             | `ros2 topic list`                                |
| `ros2 topic echo <topic_name>` | Displays messages being published on a topic.        | `ros2 topic echo /chatter`                       |
| `ros2 topic info <topic_name>` | Shows information about a topic (publishers, subscribers, type). | `ros2 topic info /chatter`                       |
| `ros2 interface show <msg_type>` | Displays the definition of a message or service type. | `ros2 interface show std_msgs/msg/String`        |
| `ros2 node list`               | Lists all active nodes.                              | `ros2 node list`                                 |
| `ros2 param list`              | Lists parameters available on a node.                | `ros2 param list /simple_publisher`              |
| `ros2 service list`            | Lists all active services.                           | `ros2 service list`                              |
| `ros2 service call <srv_name> <srv_type> <args>` | Calls a service with arguments.                      | `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"` |

## 2.1.5 URDF Introduction

The Unified Robot Description Format (URDF) is an XML format for describing all elements of a robot. It is commonly used in ROS to describe the kinematic and dynamic properties of a robot, visualize it in tools like Rviz, and generate models for simulation environments.

**Key Elements:**
-   **`<robot>`:** The root element, containing the entire robot description.
-   **`<link>`:** Represents a rigid body part of the robot (e.g., torso, upper arm). Attributes include `name`.
-   **`<joint>`:** Describes the kinematic and dynamic properties of a joint connecting two links. Attributes include `name`, `type` (e.g., `revolute`, `prismatic`, `fixed`), `parent`, `child`.
    -   **`origin`:** Defines the position and orientation of the joint's frame relative to the parent link's frame.
    -   **`axis`:** Specifies the axis of rotation for revolute joints or translation for prismatic joints.
    -   **`limit`:** Defines the upper and lower joint limits, velocity, and effort.

**Example 2.1: Simple Two-Link Arm URDF**

```xml
<?xml version="1.0"?>
<robot name="two_link_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint 1: Connects base_link to link1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/> <!-- Joint is at the top of the base_link -->
    <axis xyz="0 0 1"/> <!-- Rotates around Z-axis -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Link 1 -->
  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/> <!-- Center of cylinder for visual -->
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint 2: Connects link1 to link2 -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/> <!-- Joint is at the top of link1 -->
    <axis xyz="0 1 0"/> <!-- Rotates around Y-axis -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Link 2 (End Effector) -->
  <link name="link2">
    <visual>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

</robot>
```

This URDF defines a simple robot with a base link and two subsequent links connected by revolute joints, allowing rotation around specified axes. Each link has visual, collision, and inertial properties.

## 2.1.6 TF2 Basics

TF2 (Transform Frame 2) is a system for keeping track of multiple coordinate frames and transforming data between them. Robots typically consist of many rigid bodies, each with its own coordinate frame (e.g., base_link, camera_link, end_effector_link). TF2 provides a standardized way to define, broadcast, and listen for these transformations.

**Core Concepts:**
-   **Coordinate Frames:** A 3D coordinate system (origin and orientation).
-   **Transforms:** The relationship (translation and rotation) between two coordinate frames.
-   **`tf2_ros`:** The ROS 2 package providing TF2 functionalities.
-   **`StaticTransformBroadcaster`:** For publishing transforms that do not change over time (e.g., sensor offsets relative to a robot link).
-   **`TransformBroadcaster`:** For publishing dynamic transforms (e.g., robot base_link relative to world frame, or moving joints).
-   **`TransformListener`:** For receiving and buffering transforms, allowing you to query the transformation between any two frames at any point in time.

**Example 2.2: Broadcasting a Static Transform (Python)**

Let's create a node that broadcasts a static transform from a "world" frame to a "base_link" frame.

Create `my_ros2_package/my_ros2_package/static_tf_publisher.py`:
```python
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations # Required for quaternion_from_euler

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Create a TransformStamped message
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'world' # Parent frame
        static_transform_stamped.child_frame_id = 'base_link' # Child frame

        # Set translation (x, y, z)
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.5 # 0.5 meters above world origin

        # Set rotation (roll, pitch, yaw) and convert to quaternion
        quat = tf_transformations.quaternion_from_euler(0, 0, 0) # No rotation
        static_transform_stamped.transform.rotation.x = quat[0]
        static_transform_stamped.transform.rotation.y = quat[1]
        static_transform_stamped.transform.rotation.z = quat[2]
        static_transform_stamped.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(static_transform_stamped)
        self.get_logger().info('Broadcasting static transform from "world" to "base_link"')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Add an entry point in `setup.py`:
```python
        entry_points={
            'console_scripts': [
                'talker = my_ros2_package.talker:main',
                'listener = my_ros2_package.listener:main',
                'static_tf_publisher = my_ros2_package.static_tf_publisher:main', # Add this line
            ],
        },
```
Build and run:
```bash
colcon build --packages-select my_ros2_package
source install/setup.bash
ros2 run my_ros2_package static_tf_publisher
```
You can then visualize this transform using Rviz2 or inspect it with `ros2 run tf2_ros tf2_echo world base_link`.

---

## 2.1.7 Lab Task: Simple ROS 2 Robot

**Objective:** Create a ROS 2 package that simulates a simple robot, broadcasts its coordinate frames, and allows for basic communication.

**Steps:**
1.  **Create a New ROS 2 Package:** Name it `my_simple_robot`. Ensure `rclpy`, `std_msgs`, `sensor_msgs`, and `tf2_ros` are dependencies.
2.  **Define a Simple URDF:** Create a URDF file for a 3-link robotic arm (e.g., base, link1, link2) with two revolute joints. Include visual, collision, and basic inertial properties. Save it in a `urdf` subdirectory within your package.
3.  **Implement a Joint State Publisher Node:** Write a Python node (`joint_state_publisher.py`) that periodically publishes `sensor_msgs/msg/JointState` messages for the two revolute joints defined in your URDF. The joint values can simply cycle between a min and max angle.
4.  **Implement a Static TF Broadcaster Node:** Modify the `static_tf_publisher.py` example to broadcast a static transform from `world` to your robot's `base_link`.
5.  **Visualize in Rviz2:**
    *   Set up an Rviz2 configuration to load your robot model (using `robot_state_publisher` node) and visualize the TF frames.
    *   Ensure your `joint_state_publisher` and `static_tf_publisher` nodes are running.
6.  **Create a Simple Publisher/Subscriber Pair (Optional but Recommended):** Have a node publish a "robot_status" string to a topic, and another node subscribe and print it. This reinforces pub/sub.

**Deliverables:**
-   `my_simple_robot` ROS 2 package with:
    -   `urdf/simple_arm.urdf`
    -   `my_simple_robot/joint_state_publisher.py`
    -   `my_simple_robot/static_tf_publisher.py`
    -   Modified `setup.py` and `package.xml`
-   Brief documentation on how to build and run your package and visualize it in Rviz2.

---

## 2.1.8 Multiple Choice Questions

1.  Which communication mechanism in ROS 2 is primarily used for continuous data streams like sensor readings?
    a) Services
    b) Actions
    c) Topics
    d) Parameters
    **Answer: c**

2.  What is the purpose of the `rclpy.spin()` function?
    a) To initialize the ROS 2 client library.
    b) To process a single pending event.
    c) To continuously process events until the node is shut down.
    d) To create a new ROS 2 node.
    **Answer: c**

3.  Which URDF element is used to define a rigid body part of a robot?
    a) `<joint>`
    b) `<link>`
    c) `<robot>`
    d) `<geometry>`
    **Answer: b**

4.  The `tf2_ros.StaticTransformBroadcaster` is best suited for:
    a) Publishing dynamic joint positions.
    b) Transforming sensor data between moving frames.
    c) Publishing transforms that remain constant over time.
    d) Listening to transformations between arbitrary frames.
    **Answer: c**

5.  What does `colcon build --packages-select my_ros2_package` command do?
    a) Installs the `my_ros2_package` from a remote repository.
    b) Creates a new ROS 2 package named `my_ros2_package`.
    c) Compiles and builds only the `my_ros2_package` in the workspace.
    d) Displays information about the `my_ros2_package`.
    **Answer: c**

---

## 2.1.9 Exercises

1.  **ROS 2 Architecture:** Describe the role of DDS in ROS 2 communication. How does it enhance performance and reliability compared to earlier ROS versions?
2.  **Publisher-Subscriber Design:** You are designing a system where a robot's battery level needs to be monitored by several different nodes (e.g., display, power management, logging). Would you use a ROS 2 topic or service for this communication? Justify your choice, explaining the advantages of your chosen method in this context.
3.  **URDF Modification:** Take the provided simple two-link arm URDF. Modify it to:
    *   Change `joint1` to be a `prismatic` joint.
    *   Add a new fixed link and joint to the end of `link2` that represents a simple gripper (e.g., a small box).
    *   Change the material colors to your preference.
    *   (Self-reflection) How would you visualize this updated robot model in Rviz2?
4.  **TF2 Coordinate Frames:** Explain the concept of a "coordinate frame tree" in TF2. Why is it important to maintain a consistent parent-child relationship in these frames? Provide an example of how an incorrect frame setup could lead to issues in robot navigation.
5.  **Error Handling (Conceptual):** Consider the `SimplePublisher` and `SimpleSubscriber` nodes. What happens if the `chatter` topic is not published to for a long time? What if multiple publishers start publishing to the same `chatter` topic? Discuss how `rclpy` handles these scenarios and what implications they have for robust robot software design.
