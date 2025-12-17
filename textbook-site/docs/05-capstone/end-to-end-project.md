# 6.1 Capstone Project: End-to-End Humanoid Task Execution

## 6.1.1 Introduction: Integrating the Full Stack

This capstone project culminates all the knowledge and skills acquired throughout the course. You will integrate the disparate components of a Physical AI system – from voice command to physical manipulation – into a cohesive, functional humanoid robot system. The goal is to enable a simulated humanoid robot to understand and execute complex, multi-step tasks initiated by natural language.

The project will follow an end-to-end pipeline:
**Voice → Plan → Perception → Navigation → Manipulation**

This integration challenge highlights the complexities and interdependencies of modern robotics, requiring careful consideration of communication protocols, data flows, and error handling across different modules.

## 6.1.2 Step-by-Step Project Workflow

The project will be executed in a simulated environment, leveraging Isaac Sim for high-fidelity physics and rendering, and ROS 2 for inter-module communication.

### Phase 1: Setup and Simulation Environment

1.  **Isaac Sim Scene Creation:**
    *   **Environment:** Design a simple household or office environment in Isaac Sim. Include a few distinct objects (e.g., a "red box," a "blue cup," a "green sphere") and designated "locations" (e.g., "table," "shelf," "charging_dock").
    *   **Humanoid Model:** Import a suitable humanoid robot model into your scene. Ensure it has:
        *   Controllable arms (for manipulation) and legs (for navigation).
        *   Properly configured `ros2_control` interfaces for all controllable joints (at least position control).
        *   Appropriate physics properties (mass, inertia, collision geometry).
    *   **Sensors:** Equip the humanoid with:
        *   A simulated **depth camera** (RGB-D) for object perception.
        *   A simulated **LiDAR** for navigation (mapping and obstacle avoidance).
        *   A simulated **IMU** for pose estimation.
    *   **ROS 2 Bridge:** Ensure the Isaac Sim environment is correctly configured to publish sensor data to ROS 2 topics and receive joint commands from ROS 2.

2.  **ROS 2 Base Packages:**
    *   Create a meta-package (e.g., `humanoid_capstone`) that will house all your project nodes and launch files.
    *   Set up necessary ROS 2 dependencies (e.g., `rclpy`, `std_msgs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `control_msgs`).

### Phase 2: Perception and Navigation Integration

1.  **Object Perception Node (`object_detector_node.py`):**
    *   **Input:** Subscribe to the depth camera's RGB image and depth map topics (from Isaac Sim).
    *   **Processing:**
        *   Implement a simple object detection mechanism. For this project, you can use:
            *   **Color-based segmentation:** Detect objects based on their color (e.g., red box, blue cup).
            *   **Predefined bounding boxes:** If objects are at known locations, a simpler approach is to use the ground truth from Isaac Sim or predefined ROI.
        *   **Pose Estimation:** Estimate the 3D pose (position and orientation) of detected objects relative to the robot's base frame.
    *   **Output:** Publish a custom ROS 2 message (e.g., `DetectedObjects.msg`) containing a list of detected objects, their names, and their 3D poses.
    *   **Visualization:** Use Rviz2 markers to visualize detected object poses.

2.  **Navigation Stack Configuration:**
    *   **Map Generation:** If not pre-built, use a SLAM node (e.g., `slam_toolbox`) to build a 2D occupancy grid map of your Isaac Sim environment using LiDAR data. Save this map.
    *   **Nav2 Bringup:** Configure and launch the Nav2 stack (as discussed in Chapter 4.2).
        *   `map_server`: Load your saved map.
        *   `amcl`: For localization using LiDAR.
        *   `global_planner` & `local_planner`: Standard Nav2 planners.
        *   **Crucially:** You will need to adapt the Nav2 `cmd_vel` output for bipedal locomotion. This involves creating a specialized node that translates `geometry_msgs/msg/Twist` into high-level "walk forward," "turn," "stop" commands for your humanoid's gait controller (which you'll mock or implement simplistically).
        *   **Gait Controller Node (`humanoid_gait_controller.py`):** This node subscribes to `cmd_vel` from Nav2 and publishes joint commands to your `ros2_control` interfaces in Isaac Sim, simulating bipedal walking. For simplicity, this can be a pre-programmed gait that moves in the commanded direction.

### Phase 3: Language and Manipulation Integration

1.  **Speech-to-Text Node (`whisper_node.py`):**
    *   Integrate the Whisper node from Chapter 5.1.
    *   **Input:** Subscribe to a simulated microphone input (or use a pre-recorded audio file published via a `audio_publisher_node.py`).
    *   **Output:** Publish transcribed text to `/speech_to_text`.

2.  **LLM Cognitive Planner Node (`llm_planner_node.py`):**
    *   Integrate the LLM planner node from Chapter 5.2.
    *   **Input:** Subscribe to `/speech_to_text`.
    *   **Tools:** Define a comprehensive set of robot tools (functions) for the LLM to call:
        *   `navigate_to_location(location_name: str)`
        *   `navigate_to_object(object_name: str)`
        *   `grasp_object(object_name: str)`
        *   `place_object(target_name: str)`
        *   `report_object_pose(object_name: str)` (queries perception node)
        *   `report_current_location()` (queries navigation state)
    *   **Context:** Maintain conversation and environmental context.
    *   **Output:** Publish an ordered list of `RobotAction` messages (custom message) to `/robot_action_plan`.

3.  **Manipulation Controller Node (`manipulation_controller_node.py`):**
    *   **Input:** Subscribe to `/robot_action_plan` and `DetectedObjects.msg` (from your perception node).
    *   **Processing:**
        *   For `grasp_object`:
            *   Retrieve the object's pose from the `DetectedObjects` message.
            *   Use inverse kinematics (IK) to calculate target joint angles for the arm to reach the object. (You can use a simple IK solver or a pre-defined reaching motion for the lab).
            *   Publish joint commands to your humanoid's arm via `ros2_control` interface.
            *   Simulate gripper actuation.
        *   For `place_object`:
            *   Calculate target placement pose.
            *   Use IK for arm movement and publish joint commands.
            *   Simulate gripper release.
    *   **Feedback:** Publish feedback on success/failure of manipulation actions.

### Phase 4: Capstone Orchestrator and Testing

1.  **Capstone Orchestrator Node (`capstone_orchestrator.py`):**
    *   This node ties everything together.
    *   **Input:** Subscribes to `/robot_action_plan` from the LLM planner.
    *   **Execution Loop:** Iterates through the action plan. For each `RobotAction`:
        *   If `navigate_to_location`: Send a Nav2 goal. Wait for completion.
        *   If `navigate_to_object`: Query perception node for object pose, then send Nav2 goal.
        *   If `grasp_object`: Send command to `manipulation_controller_node`. Wait for completion.
        *   If `place_object`: Send command to `manipulation_controller_node`. Wait for completion.
    *   **Error Handling:** If any action fails, it can either notify the LLM for replanning (advanced) or simply report failure to the user.
    *   **State Updates:** Updates the LLM's world model/context based on execution results.

2.  **Full ROS 2 Launch File:** Create a comprehensive launch file (`full_capstone.launch.py`) that brings up:
    *   Isaac Sim (if launched via script).
    *   All perception nodes.
    *   All Nav2 nodes.
    *   The `humanoid_gait_controller`.
    *   The `whisper_node` and simulated audio input.
    *   The `llm_planner_node`.
    *   The `manipulation_controller_node`.
    *   The `capstone_orchestrator`.
    *   Rviz2 for visualization.

## 6.1.3 Test Procedure

The following test procedure outlines a multi-step task to validate the integrated system:

1.  **Initial State:** Humanoid robot at its charging station. A "red box" is on the floor in another room. A "blue table" is in the kitchen.
2.  **Voice Command 1:** User says: "Humanoid, please go to the living room."
    *   **Expected:** Robot navigates to the living room.
3.  **Voice Command 2:** User says: "Find the red box."
    *   **Expected:** Robot uses its perception system to locate the red box. LLM confirms its location.
4.  **Voice Command 3:** User says: "Pick up the box."
    *   **Expected:** Robot navigates to the red box, uses its arm to grasp it, and lifts it.
5.  **Voice Command 4:** User says: "Now take it to the blue table."
    *   **Expected:** Robot navigates to the blue table, approaches it, and places the red box on the table surface.
6.  **Voice Command 5:** User says: "Go back to your charging station."
    *   **Expected:** Robot navigates back to its home position.

### Expected Behavior for Each Stage:
-   **Voice:** Accurate transcription of commands.
-   **Plan:** LLM generates a coherent, multi-step action sequence for each complex command.
-   **Perception:** Robot accurately detects and localizes objects when commanded.
-   **Navigation:** Robot plans and executes paths, avoiding obstacles, and moving to specified locations/objects.
-   **Manipulation:** Robot reaches, grasps, and places objects without collision or dropping.

## 6.1.4 Final Grading Rubric

The project will be evaluated based on the successful execution of the end-to-end task and the quality of the implemented modules.

| Category               | Weight | Description                                                                   | Criteria for Excellent (20 pts)                                                                             |
| :--------------------- | :----- | :---------------------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------------- |
| **1. Isaac Sim Setup** | 15%    | Realistic environment, functional humanoid model with sensors and `ros2_control`. | Complex, well-designed environment. Robust humanoid model. All sensors and `ros2_control` interfaces functional. |
| **2. Perception**      | 20%    | Ability to detect and localize objects (e.g., color/depth based).              | Accurate and consistent detection of all specified objects in various lighting/poses. 3D pose estimation.     |
| **3. Navigation**      | 20%    | Mapping, localization, path planning, and basic bipedal locomotion.           | Robot navigates reliably to specified locations and objects, avoiding dynamic obstacles. Basic walking gait implemented. |
| **4. Voice-to-Action** | 15%    | STT and LLM-based cognitive planning and tool use.                            | Accurate transcription. LLM generates correct multi-step plans for complex commands using defined tools.  |
| **5. Manipulation**    | 15%    | Arm control, inverse kinematics (simple), grasping, and placing.              | Smooth, collision-free reaching, grasping, and precise placement. IK (or similar) is stable.            |
| **6. Integration**     | 10%    | Seamless communication and coordination between all modules.                   | All modules communicate effectively. System is robust and handles transitions smoothly.                  |
| **7. Code Quality**    | 5%     | Readability, modularity, comments, adherence to ROS 2 best practices.         | Well-structured, documented, and idiomatic ROS 2 code.                                                    |
| **Total**              | 100%   |                                                                               |                                                                                                           |

**Grading Scale:**
-   **A (90-100%):** All core requirements met, advanced features implemented, excellent code quality, and robust demonstration.
-   **B (80-89%):** Most core requirements met, minor issues or limitations in functionality, good code quality.
-   **C (70-79%):** Core requirements partially met, significant bugs or limitations, acceptable code quality.
-   **D (60-69%):** Minimal functionality, severe issues, poor code quality.
-   **F (Below 60%):** Does not meet minimal project requirements.

This project will provide an unparalleled opportunity to synthesize your learning and contribute to the cutting edge of Physical AI and humanoid robotics.

---

## 6.1.5 Multiple Choice Questions

1.  Which module is responsible for converting spoken commands into written text in the end-to-end project workflow?
    a) LLM Cognitive Planner
    b) Object Perception Node
    c) Speech-to-Text Node
    d) Manipulation Controller Node
    **Answer: c**

2.  What is the primary role of the Capstone Orchestrator Node in this project?
    a) To perform object detection from camera feeds.
    b) To generate high-fidelity physics simulations.
    c) To execute the action plan generated by the LLM planner, coordinating various robot modules.
    d) To control individual joint movements of the humanoid robot.
    **Answer: c**

3.  In Phase 2, adapting Nav2's `cmd_vel` output for bipedal locomotion typically involves:
    a) Directly mapping `cmd_vel` to leg motor torques.
    b) A specialized node that translates `cmd_vel` into high-level gait commands.
    c) Ignoring `cmd_vel` entirely and using only vision-based navigation.
    d) Replacing Nav2 with a custom bipedal-specific navigation stack.
    **Answer: b**

4.  Which sensor is explicitly required on the humanoid for object perception in this project?
    a) IMU
    b) LiDAR
    c) Force/Torque Sensor
    d) Depth Camera
    **Answer: d**

5.  If the humanoid fails to grasp an object due to an inaccurate object pose, which module's performance should be primarily investigated?
    a) Humanoid Gait Controller
    b) Speech-to-Text Node
    c) Object Perception Node
    d) Nav2 Local Planner
    **Answer: c**

---

## 6.1.6 Exercises

1.  **Error Recovery for Navigation:** During the test procedure, imagine the robot gets stuck while navigating to the living room. How could the `Capstone Orchestrator` node detect this (e.g., no progress for a certain time) and trigger a recovery behavior (e.g., activate Nav2's recovery behaviors, or inform the LLM to replan)?
2.  **Advanced Manipulation:** The current manipulation approach is simplified. Propose two ways to make the manipulation more robust for grasping and placing objects in a real-world scenario. Consider factors like object variability, unknown object properties, and precision.
3.  **Human-in-the-Loop:** How could a "human-in-the-loop" mechanism be integrated into this capstone project? Provide an example where the robot would ask the user for clarification or assistance, and how the user's response would influence the LLM's planning or the robot's execution.
4.  **Multi-Object Handling:** Extend the voice command "Pick up the box" to handle multiple objects of the same type (e.g., "Pick up all the red boxes"). Describe the changes needed in the Perception, LLM Planning, and Orchestrator nodes to support this command.
5.  **Performance Optimization:** Given that this is an end-to-end system with many nodes, discuss potential bottlenecks (e.g., high latency, high CPU usage) that might arise during real-time operation. Propose two strategies to optimize the performance of specific modules (e.g., using more efficient algorithms, offloading computation to GPU).