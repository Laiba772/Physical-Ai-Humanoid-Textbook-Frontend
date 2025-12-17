# 4.2 Navigation: Sim-to-Real and Bipedal Locomotion

## 4.2.1 Introduction to Nav2

Nav2 (Navigation2) is the ROS 2 navigation framework that enables a robot to autonomously navigate from a starting pose to a goal pose, avoiding obstacles along the way. It is a highly modular and configurable stack of ROS 2 nodes, designed for a wide range of mobile robots, including humanoid platforms.

**Key Components of Nav2:**
-   **State Estimator:** Combines sensor data (IMU, odometry, LiDAR) to provide a robust estimate of the robot's current pose.
-   **Localizer:** Determines the robot's position within a global map (e.g., AMCL for particle filter localization).
-   **Global Planner:** Generates an optimal path from the robot's current location to the goal, considering the static map (e.g., Dijkstra, A*).
-   **Local Planner (Controller):** Executes the global plan while avoiding dynamic obstacles and handling local terrain variations (e.g., DWA, TEB).
-   **Recovery Behaviors:** Strategies to help the robot escape difficult situations (e.g., rotating in place, backing up).
-   **Behavior Tree:** Orchestrates the various components of the navigation stack, allowing for flexible and complex navigation logic.

**Diagram: High-Level Nav2 Architecture**

```
+--------------------+       +--------------------+
|  Robot Sensors     |       |  Global Map        |
| (LiDAR, IMU, etc.) |       |  (OccupancyGrid)   |
+--------+-----------+       +--------+-----------+
         |                              |
         v                              v
+------------------------------------------------+
|          State Estimator & Localizer           |
|  (IMU, Odometry, AMCL, VSLAM for pose fusion)  |
+------------------------+-----------------------+
                         |
                         v
+------------------------------------------------+
|                   Global Planner               |
|            (Path to Goal on Static Map)        |
+------------------------+-----------------------+
                         |
                         v
+------------------------------------------------+
|                   Local Planner                |
|      (Dynamic Obstacle Avoidance, Trajectory)  |
+------------------------+-----------------------+
                         |
                         v
+------------------------------------------------+
|             Recovery Behaviors                 |
|             (e.g., backing up, spinning)       |
+------------------------------------------------+
                         |
                         v
+------------------------------------------------+
|                 Robot Actuators                |
|            (Velocity Commands to Base)         |
+------------------------------------------------+
```

## 4.2.2 Path Planning

Path planning is the process of finding an optimal or feasible path from a start configuration to a goal configuration, avoiding obstacles. In Nav2, this is primarily handled by the global and local planners.

### Global Path Planning
The global planner calculates an initial, collision-free path through the static environment (often represented by an occupancy grid map). It focuses on finding the best route over the entire known area.

**Common Global Planner Algorithms:**
-   **Dijkstra's Algorithm:** Finds the shortest path between nodes in a graph, considering edge weights.
-   **A* (A-star) Search:** An extension of Dijkstra's that uses a heuristic to guide the search, making it more efficient for goal-directed planning.
-   **Theta*:** An any-angle path planning algorithm that produces shorter, "straighter" paths than grid-based planners.

### Local Path Planning
The local planner, or controller, operates in a smaller, dynamic window around the robot. It continuously adjusts the robot's velocity commands to follow the global path, avoid unexpected obstacles (dynamic or unmapped), and handle local terrain variations.

**Common Local Planner Algorithms:**
-   **Dynamic Window Approach (DWA):** Samples possible robot velocities, simulates them forward in time, and chooses the velocity that maximizes a cost function (e.g., closeness to goal, obstacle avoidance, global path progress).
-   **Timed Elastic Band (TEB):** Optimizes the robot's trajectory by minimizing a cost function that considers trajectory length, time, obstacle avoidance, and dynamic constraints.

## 4.2.3 Goal Behavior

A navigation goal in Nav2 is more than just a target pose. It encompasses the desired behavior of the robot upon reaching that pose. The Nav2 Behavior Tree orchestrates these behaviors.

**Example Goal Behavior:**
1.  **Receive Goal:** A `geometry_msgs/msg/PoseStamped` message indicating the target `x, y, z` and `orientation` is sent to the Nav2 stack.
2.  **Global Plan:** The global planner computes a path.
3.  **Local Execution:** The local planner guides the robot along the path.
4.  **Goal Tolerance:** The robot is considered "at goal" when it is within a specified position and orientation tolerance of the target.
5.  **Final Orientation:** Upon reaching the target position, the robot might rotate to achieve a specific final orientation.
6.  **Success/Failure:** The navigation action reports success or failure.

**Behavior Trees in Nav2:**
Nav2 uses behavior trees (BTs) to define and manage complex robot behaviors. A BT is a hierarchical, graphical control flow structure that specifies how the robot should react to its environment and pursue its goals. This allows for flexible and reactive goal behaviors, including recovery behaviors.

## 4.2.4 SLAM → Navigation

The output of a SLAM (Simultaneous Localization and Mapping) system, specifically a consistent 2D or 3D map, is a prerequisite for most navigation stacks, including Nav2.

**Workflow:**
1.  **Map Generation (SLAM):** A robot explores an unknown environment, building a map (e.g., an `OccupancyGrid`) while simultaneously localizing itself within that map. This can be done online or offline.
2.  **Map Saving:** The generated map is saved (e.g., using `ros2 run nav2_map_server map_saver`).
3.  **Navigation Setup:**
    *   The saved map is loaded by the `map_server` node.
    *   A localization node (e.g., AMCL) is initialized with the map and an initial pose estimate.
    *   The robot starts navigating using this map for global planning and the localizer for its current position.

**Table 4.1: Key Nav2 Parameters (examples)**

| Parameter Group     | Parameter Name (Example) | Description                                          | Default (approx.) |
| :------------------ | :----------------------- | :--------------------------------------------------- | :---------------- |
| **Global Planner**  | `plugin`                 | Which global planner to use (e.g., `NavfnPlanner`) | `NavfnPlanner`    |
|                     | `tolerance`              | Goal tolerance for global plan.                      | `0.5` meters      |
| **Local Planner**   | `plugin`                 | Which local planner to use (e.g., `DwbLocalPlanner`) | `DwbLocalPlanner` |
|                     | `max_vel_x`              | Maximum linear velocity in X.                        | `0.2` m/s         |
|                     | `min_vel_x`              | Minimum linear velocity in X.                        | `-0.1` m/s        |
|                     | `max_rot_vel`            | Maximum angular velocity.                            | `0.5` rad/s       |
| **Controller Server** | `controller_frequency`   | Rate at which local planner is called.               | `20.0` Hz         |
| **Behavior Tree**   | `default_nav_tree`       | Path to the default behavior tree XML file.          |                   |
| **AMCL (Localizer)** | `min_particles`          | Minimum number of particles for AMCL.                | `100`             |
|                     | `max_particles`          | Maximum number of particles for AMCL.                | `2000`            |
|                     | `laser_model_type`       | Type of laser model for AMCL.                        | `likelihood_field`|

## 4.2.5 Sim-to-Real Gap

The "sim-to-real gap" refers to the discrepancy between a robot's performance in simulation and its performance in the real world. Despite increasingly sophisticated simulators, perfectly bridging this gap remains a significant challenge.

**Sources of the Sim-to-Real Gap:**
-   **Sensor Models:** Imperfect noise models, unmodeled sensor biases, or subtle real-world phenomena (e.g., lens flare, dust on a camera).
-   **Physics Models:** Inaccurate friction coefficients, simplified collision models, unmodeled mechanical compliance, or slight differences in mass distribution.
-   **Actuator Models:** Differences in motor response, gearbox backlash, or unexpected heating effects.
-   **Environment Fidelity:** Simplifications in lighting, textures, material properties, or dynamic elements that are hard to replicate in simulation.
-   **Control Latency:** Differences in communication and processing delays between simulation and hardware.

**Strategies to Bridge the Gap:**
-   **High-Fidelity Simulation:** Use advanced simulators like Isaac Sim with physically accurate rendering and physics.
-   **Domain Randomization (DR):** Randomize non-critical environmental parameters in simulation (textures, lighting, object positions, robot parameters) to force the learned policy/model to generalize to variations.
-   **System Identification:** Measure physical parameters (mass, friction, motor constants) of the real robot and environment to improve simulation accuracy.
-   **Reinforcement Learning (RL):** Train policies directly in simulation and then fine-tune them on the real robot, leveraging DR to improve transferability.
-   **Hardware-in-the-Loop (HIL):** Use real robot hardware (e.g., motors, sensors) with a simulated environment to test components in a realistic setting.
-   **Progressive Training:** Start with simple simulations and gradually increase fidelity and complexity.

## 4.2.6 Biped Navigation Techniques

Navigating with a bipedal robot introduces unique challenges compared to wheeled or tracked robots. Humanoid robots must maintain dynamic balance while moving, negotiate uneven terrain, and potentially interact with stairs or ladders.

**Key Techniques for Biped Navigation:**
1.  **Gait Generation:** Developing stable and efficient walking patterns (gaits) that consider the robot's kinematics, dynamics, and environmental conditions. This often involves:
    -   **Zero Moment Point (ZMP) Control:** A classic method for ensuring dynamic balance by keeping the ZMP within the support polygon.
    -   **Capture Point (CP) Control:** A more dynamic approach that predicts the point where the robot's center of mass would need to be to come to a stop without falling.
    -   **Model Predictive Control (MPC):** Optimizing future control inputs over a short horizon to achieve desired trajectories while satisfying constraints (balance, joint limits).
2.  **Footstep Planning:** For complex terrain, instead of continuous path planning for the base, bipedal robots often plan discrete footstep locations.
    -   **A*-based Footstep Planners:** Search for sequences of footsteps on a discretized terrain map.
    -   **Sampling-based Planners (e.g., RRTs):** Explore valid footstep placements in a continuous space.
3.  **Perception for Terrain Adaptation:**
    -   **Legged Odometry:** Using IMU and forward kinematics for more accurate pose estimation over rough terrain.
    -   **Vision-based Foothold Detection:** Identifying safe and stable landing spots for feet using depth cameras or LiDAR.
4.  **Whole-Body Control (WBC):** Coordinating the movements of all joints (legs, arms, torso) to achieve desired locomotion while maintaining balance and potentially performing other tasks (e.g., carrying an object).
5.  **Dynamic Walking:** Developing gaits that are not strictly periodic but can adapt quickly to disturbances or changes in terrain.

**Integration with Nav2:**
While Nav2 is designed for generic mobile robots, its modularity allows for integration with bipedal locomotion.
-   **Custom Local Planner:** A specialized local planner can be developed that outputs joint trajectories or footstep commands instead of `cmd_vel`.
-   **Hybrid Approach:** A global planner might still generate a path for the robot's torso, and a bipedal locomotion controller handles the low-level footstep and joint commands to follow this path.
-   **Costmaps for Legged Robots:** Custom cost functions for terrain (e.g., step height, slope) can be incorporated into Nav2's costmaps to make global plans more suitable for bipedal motion.

---

## 4.2.7 Lab Task: Humanoid Navigation in Isaac Sim

**Objective:** Implement a simplified navigation stack for a humanoid robot in Isaac Sim, focusing on demonstrating path planning and obstacle avoidance using a bipedal locomotion model.

**Steps:**
1.  **Isaac Sim Humanoid Setup:**
    *   Spawn a humanoid model in Isaac Sim (e.g., NVIDIA's Carter robot in bipedal mode, or a simplified custom humanoid).
    *   Create a simple environment with a few static obstacles (e.g., walls, boxes) and a target destination.
    *   Ensure your humanoid has a virtual LiDAR and/or depth camera sensors configured to publish ROS 2 topics.
    *   Set up ROS 2 bridge for joint state commands and sensor data.
2.  **ROS 2 Navigation Stack Configuration:**
    *   Create a ROS 2 package for your navigation setup (`humanoid_nav`).
    *   Configure `nav2_bringup` components:
        *   **`map_server`:** Load a pre-built static map of your Isaac Sim environment. (You might need to manually create this map or run a basic SLAM session in Isaac Sim once).
        *   **`amcl`:** For localization within the map.
        *   **`global_planner`:** Use a standard A* or Dijkstra planner.
        *   **`local_planner`:** Here's the critical part: you'll need a *mock* or *simplified* local planner that translates desired `cmd_vel` (from Nav2's output) into a high-level "walk forward," "turn left," etc., instruction that your humanoid's locomotion controller can interpret.
        *   **`robot_localization`:** For fusing odometry and IMU data.
3.  **Bipedal Locomotion Interface (Mock):**
    *   Create a ROS 2 Python node (`bipedal_cmd_vel_to_joint_commands.py`) that subscribes to Nav2's `/cmd_vel` topic.
    *   This node will *simulate* bipedal locomotion. When it receives a forward `cmd_vel`, it could publish a sequence of pre-defined joint commands or trigger a "walk forward" state in your humanoid's `ros2_control` interface (mocked from earlier labs).
    *   For simplicity, turning can be simulated by rotating the base directly in Isaac Sim (if possible through the bridge) or by a simple sequence of foot movements.
4.  **Goal Setting and Visualization:**
    *   Use Rviz2 to visualize the map, robot pose, global plan, and local plan.
    *   Set a navigation goal using the "2D Goal Pose" tool in Rviz2.
5.  **Test Navigation:** Observe if your humanoid can plan a path and *simulate* movement towards the goal, avoiding obstacles. Note the limitations of the "mock" bipedal controller.

**Deliverables:**
-   Isaac Sim environment with a humanoid and obstacles.
-   ROS 2 `humanoid_nav` package with `nav2_bringup` configuration files.
-   `bipedal_cmd_vel_to_joint_commands.py` node.
-   Launch file to bring up Isaac Sim (if possible through a script) and the entire ROS 2 navigation stack.
-   Video or GIF demonstrating the humanoid attempting to navigate to a goal.
-   A brief discussion on the challenges encountered when mapping `cmd_vel` to bipedal locomotion.

---

## 4.2.8 Multiple Choice Questions

1.  Which Nav2 component is responsible for generating an optimal path from the robot's current location to a goal, considering a static map?
    a) Local Planner
    b) Localizer
    c) Global Planner
    d) Recovery Behaviors
    **Answer: c**

2.  Which of the following is a common global path planning algorithm used in Nav2?
    a) Dynamic Window Approach (DWA)
    b) Timed Elastic Band (TEB)
    c) A* (A-star) Search
    d) Pure Pursuit
    **Answer: c**

3.  What does the "sim-to-real gap" refer to in robotics?
    a) The time delay between simulating a robot and deploying it.
    b) The discrepancy between a robot's performance in simulation and in the real world.
    c) The difference in computational power between simulation and real hardware.
    d) The process of transferring simulation assets to a real robot.
    **Answer: b**

4.  Which strategy is NOT typically used to bridge the sim-to-real gap?
    a) Domain Randomization
    b) High-Fidelity Simulation
    c) Exclusive use of real-world data without simulation.
    d) System Identification
    **Answer: c**

5.  For bipedal navigation, what is the primary purpose of Footstep Planning?
    a) To control the robot's joint velocities.
    b) To find discrete, stable landing spots for the robot's feet on complex terrain.
    c) To generate a continuous, smooth path for the robot's center of mass.
    d) To adjust sensor noise models for better localization.
    **Answer: b**

---

## 4.2.9 Exercises

1.  **Nav2 Planner Comparison:** Research and compare two different local planners available in Nav2 (e.g., DWA and TEB). Discuss their strengths, weaknesses, and scenarios where one might be preferred over the other for a humanoid robot navigating in a cluttered indoor environment.
2.  **Role of Behavior Trees:** Explain how Nav2's Behavior Trees contribute to the robustness and flexibility of robot navigation. Provide a simple example of how a behavior tree could manage a "navigation goal" that includes obstacle avoidance and a recovery behavior if stuck.
3.  **Bridging the Gap for Humanoids:** Consider a humanoid robot designed to walk on uneven outdoor terrain. Identify two specific aspects of the sim-to-real gap that would be particularly challenging for this robot, and propose a concrete strategy for each to help bridge that gap, leveraging tools like Isaac Sim.
4.  **Bipedal Navigation Challenges:** Beyond balance, discuss at least three distinct challenges inherent in designing navigation systems specifically for bipedal robots that are less prominent for wheeled mobile robots. How do these challenges impact path planning and control?
5.  **Map Data for Humanoids:** In addition to a standard occupancy grid, what other types of map information (e.g., traversability, semantic labels, elevation) would be highly beneficial for a humanoid robot to navigate effectively in a human-centric environment? How could these map types be integrated into a Nav2-like framework?