# 5.2 LLM Cognitive Planning for Robotics

## 5.2.1 How LLMs Convert Natural Language to Robot Tasks

Large Language Models (LLMs) have revolutionized the field of natural language processing and are increasingly being applied to robotics for high-level cognitive planning. Their ability to understand context, reason, and generate coherent text allows them to translate abstract natural language commands into actionable, multi-step robot tasks.

The core idea is to leverage the LLM's vast knowledge base and reasoning capabilities to bridge the gap between human-centric instructions and robot-centric execution primitives. Instead of explicit, hard-coded rules for every possible command, LLMs can infer the user's intent and generate a logical sequence of steps.

### Technical Breakdown: LLM Role in Robotics Planning

An LLM typically functions within a broader planning pipeline, often interacting with specialized robot modules.

1.  **Natural Language Input:** The LLM receives a high-level command (e.g., "Make me a cup of coffee").
2.  **Task Decomposition:** The LLM breaks down the complex command into a series of sub-goals or high-level actions (e.g., "Go to kitchen," "Get mug," "Brew coffee," "Add milk"). This is where the LLM's common-sense knowledge about tasks comes into play.
3.  **Action Sequence Generation:** For each sub-goal, the LLM generates a sequence of robot-executable primitives or calls to specialized robot skills (e.g., `navigate(kitchen_location)`, `grasp(mug_object)`, `activate(coffee_machine)`).
4.  **Parameter Grounding:** The LLM identifies and grounds parameters for these actions, often by querying a world model or relying on its contextual understanding (e.g., `object="red box"` in "Pick up the red box").
5.  **Feedback and Refinement:** If a generated plan fails during execution, the LLM can receive feedback (e.g., "Grasping failed") and attempt to replan or suggest alternative actions.

## 5.2.2 Planning Pipelines

A robotic planning pipeline integrating an LLM usually consists of several interconnected modules.

**Diagram: LLM-driven Robot Planning Pipeline**

```
+--------------------+        +--------------------+        +--------------------+        +--------------------+
|  Natural Language  |        |      LLM-based     |        |   Robot Task       |        |   Low-Level        |
|  Command (Voice/Text) |---->|  Cognitive Planner |---->|  Decomposition &   |---->|  Execution         |
|                    |        |  (Task Decomposition, |        |  Action Sequence   |        |  (Perception,      |
|                    |        |  Parameter Grounding)  |        |  Generation        |        |  Navigation,       |
+--------------------+        +----------+---------+        +----------+---------+        |  Manipulation)     |
                                         |                             |                             |
                                         v                             v                             v
                                 +-------+-------+             +-------+-------+             +-------+-------+
                                 |  World Model  |             |  Tool/Skill   |             |  Sensor Feedback|
                                 | (Objects, State)|             |  Library      |             |  (Success/Fail) |
                                 +---------------+             +---------------+             +---------------+
                                                                         ^                             ^
                                                                         |                             |
                                                                         +-----------------------------+
                                                                                  Execution Monitoring &
                                                                                  Error Handling
```

**Modules in the Pipeline:**
-   **Front-end (STT/NLU):** Converts user input into a structured intent (as discussed in 5.1).
-   **LLM Cognitive Planner:** Takes the structured intent and generates a high-level plan.
-   **World Model:** Provides the LLM with information about the environment, robot state, and available objects. This can be a simple database or a complex knowledge graph.
-   **Tool/Skill Library:** A set of robot-executable functions or "tools" (e.g., `move_to_object(object_id)`, `grasp_object(object_id, grasp_type)`, `open_door()`). The LLM "chooses" and "calls" these tools.
-   **Execution Monitor:** Oversees the execution of the robot's actions, providing feedback to the LLM on success or failure.
-   **Low-Level Execution Modules:** Traditional robotics modules responsible for actual perception (object detection), navigation (path planning, motor control), and manipulation (inverse kinematics, gripper control).

## 5.2.3 Multi-Step Reasoning

LLMs excel at multi-step reasoning, which is crucial for handling complex, multi-stage robot tasks. This involves breaking down a problem into smaller, manageable steps and understanding the dependencies between them.

**How LLMs Facilitate Multi-Step Reasoning:**
-   **Chain of Thought (CoT) Prompting:** By explicitly asking the LLM to "think step-by-step," it can generate intermediate reasoning steps, which improves the quality and explainability of the final plan.
-   **Planning as Search:** The LLM can be prompted to explore different sequences of actions (a search space) to find an optimal or feasible plan.
-   **State Tracking:** With appropriate prompting, an LLM can maintain a mental model of the robot's current state and the environment, updating it after each action.

### Example 5.3: "Pick the red box" → Robot Plan (using conceptual LLM interaction)

Consider the command: "Pick the red box and place it on the blue table."

**LLM Internal Reasoning (Chain of Thought):**
1.  **Identify Goal:** User wants to move a specific object (red box) to a specific location (blue table).
2.  **Decompose Task:** This involves multiple sub-tasks:
    *   Find the red box.
    *   Navigate to the red box.
    *   Grasp the red box.
    *   Find the blue table.
    *   Navigate to the blue table.
    *   Place the red box on the blue table.
3.  **Check Prerequisites/Dependencies:**
    *   To grasp the red box, the robot needs to be near it and know its pose.
    *   To place the red box, the robot needs to be near the blue table and know its pose.
    *   The robot needs to be holding the red box before it can place it.
4.  **Map to Robot Skills/Tools:**
    *   Find: `perceive_object(color="red", type="box")` -> returns `object_id_red_box`, `pose_red_box`
    *   Navigate: `navigate_to_pose(pose)`
    *   Grasp: `grasp_object(object_id)`
    *   Place: `place_object(object_id, target_pose)`

**LLM-Generated Action Sequence (JSON format, conceptual):**

```json
[
  {"action": "perceive_object", "params": {"color": "red", "type": "box"}, "id_output": "red_box_id"},
  {"action": "navigate_to_object", "params": {"object_id": "red_box_id"}},
  {"action": "grasp_object", "params": {"object_id": "red_box_id"}},
  {"action": "perceive_object", "params": {"color": "blue", "type": "table"}, "id_output": "blue_table_id"},
  {"action": "navigate_to_object", "params": {"object_id": "blue_table_id"}},
  {"action": "place_object", "params": {"object_id": "red_box_id", "target_id": "blue_table_id"}}
]
```

This sequence can then be executed by a robotics framework (like ROS 2), where each `action` corresponds to a call to a specific robot skill or a sequence of low-level commands.

## 5.2.4 Tool Use and Function Calling

A powerful paradigm for LLM-driven robotics is "tool use" or "function calling." Here, the LLM acts as a high-level orchestrator, deciding which robot skills (tools) to call and with what parameters, rather than directly generating low-level motor commands.

**Workflow with Tool Use:**
1.  **Define Tools:** Provide the LLM with a description of available robot skills or functions (e.g., `navigate(location: str)`, `pick_up(object_name: str)`, `report_status()`). These descriptions include the function name, parameters, and a natural language explanation of what the tool does.
2.  **LLM Decision:** The LLM receives the user's command and, based on its internal reasoning and the provided tool definitions, decides which tool(s) to call.
3.  **Generate Function Call:** The LLM outputs a structured function call (e.g., `pick_up(object_name='red box')`).
4.  **Execute Function:** The robot system parses this function call and executes the corresponding robot skill.
5.  **Observe Result:** The result of the skill (success, failure, sensor data) is fed back to the LLM, allowing it to continue planning or replan.

**Example 5.4: LLM Tool Definition for a Robot (Conceptual)**

```python
robot_tools = [
    {
        "name": "navigate_to_location",
        "description": "Moves the robot to a specified named location (e.g., 'kitchen', 'bedroom').",
        "parameters": {
            "type": "object",
            "properties": {
                "location": {"type": "string", "description": "The name of the location to navigate to."}
            },
            "required": ["location"]
        }
    },
    {
        "name": "grasp_object",
        "description": "Instructs the robot to pick up an object from its current perceived location.",
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {"type": "string", "description": "The name or description of the object to grasp (e.g., 'red box', 'coffee cup')."}
            },
            "required": ["object_name"]
        }
    },
    {
        "name": "place_object",
        "description": "Instructs the robot to place the currently held object onto a specified target (e.g., a table, shelf).",
        "parameters": {
            "type": "object",
            "properties": {
                "target_name": {"type": "string", "description": "The name or description of the target to place the object on."}
            },
            "required": ["target_name"]
        }
    }
]

# When user says "Go to the kitchen and grab the cereal box"
# LLM might first output: navigate_to_location(location="kitchen")
# After navigation, user says (or LLM infers): "Now pick up the cereal box"
# LLM then outputs: grasp_object(object_name="cereal box")
```

This approach separates the high-level decision-making (LLM) from the low-level execution (robot skills), creating a more robust and modular system.

---

## 5.2.5 Lab Task: LLM-Driven Task Planner

**Objective:** Implement a ROS 2 node that uses an LLM (mocked or real) to convert natural language commands into a sequence of structured robot actions, simulating a cognitive planning pipeline.

**Steps:**
1.  **Define Robot Actions (Tools):**
    *   Create a Python dictionary `robot_skills` that defines available robot actions (tools) with their names, descriptions, and expected parameters. Include `navigate_to_location`, `grasp_object`, `place_object` as in Example 5.4.
2.  **LLM Interface (Mock):**
    *   Since direct LLM API calls might incur cost/latency for a lab, create a Python function `mock_llm_planner(command_text, robot_skills)` that simulates an LLM.
    *   This function should take the user's natural language command and the `robot_skills` definitions.
    *   For simple commands, it can return a hardcoded JSON action sequence. For more complex ones, it should demonstrate multi-step reasoning by returning multiple sequential actions.
    *   For example:
        *   "Go to kitchen" -> `[{"action": "navigate_to_location", "params": {"location": "kitchen"}}]`
        *   "Get the red apple" -> `[{"action": "grasp_object", "params": {"object_name": "red apple"}}]`
        *   "Move the blue box to the shelf" -> `[{"action": "grasp_object", "params": {"object_name": "blue box"}}, {"action": "place_object", "params": {"target_name": "shelf"}}]`
    *   Consider adding a simple error case for "unclear" commands.
3.  **ROS 2 LLM Planner Node:**
    *   Create a ROS 2 Python node (`llm_planner_node.py`).
    *   Subscribe to the `/speech_to_text` topic (from previous lab, publishing `std_msgs/msg/String`).
    *   When a new text command arrives, pass it to your `mock_llm_planner` function along with the `robot_skills`.
    *   Publish the resulting structured action plan (e.g., as a custom ROS 2 message `RobotActionPlan.msg` containing a list of structured actions) to a new topic `/robot_action_plan`.
    *   Log the received natural language command and the generated action plan.
4.  **ROS 2 Action Executor Node (Mock):**
    *   Create a ROS 2 Python node (`action_executor_node.py`).
    *   Subscribe to the `/robot_action_plan` topic.
    *   When an action plan arrives, iterate through each action in the plan.
    *   For each action, print a message indicating which action is being "executed" and with what parameters (e.g., "Executing navigate_to_location to kitchen").
    *   Simulate a delay (e.g., `time.sleep(1)`) to represent execution time.
    *   Publish a simple `std_msgs/msg/Bool` message to a `/action_feedback` topic indicating success (`True`) after each simulated action.
5.  **Launch File:** Create a launch file to run `whisper_node` (from previous lab, if using recorded audio), `llm_planner_node`, and `action_executor_node`.
6.  **Test:** Provide input commands (via recorded audio or direct `ros2 topic pub`). Observe the LLM planning node outputting action plans and the action executor node "executing" them sequentially.

**Deliverables:**
-   ROS 2 package (`llm_robot_planner`) containing:
    -   `llm_planner_node.py` (with `robot_skills` and `mock_llm_planner`).
    -   `action_executor_node.py`.
    -   `RobotActionPlan.msg` (custom message for action plans).
    -   Launch file (`llm_planning.launch.py`).
-   Console output demonstrating the conversion from natural language to multi-step robot action plans and their simulated execution.
-   A brief discussion on how a real LLM integration would differ from the mock and its potential benefits and challenges.

---

## 5.2.6 Multiple Choice Questions

1.  What is the primary role of Large Language Models (LLMs) in robot cognitive planning?
    a) To directly control robot motor joint angles.
    b) To convert abstract natural language commands into actionable, multi-step robot tasks.
    c) To perform low-level sensor data processing.
    d) To manage the robot's battery level.
    **Answer: b**

2.  Which concept helps an LLM break down a complex command into a series of sub-goals and improve the quality of its generated plan?
    a) Random Forest
    b) Chain of Thought (CoT) Prompting
    c) Support Vector Machines
    d) Principal Component Analysis
    **Answer: b**

3.  In an LLM-driven planning pipeline, what is the purpose of a "World Model"?
    a) To perform speech-to-text conversion.
    b) To store and provide information about the environment, robot state, and available objects.
    c) To execute low-level manipulation commands.
    d) To render photorealistic simulations.
    **Answer: b**

4.  When an LLM outputs a structured call like `grasp_object(object_name='red box')`, this is an example of what paradigm in LLM-driven robotics?
    a) Direct motor control
    b) Reinforcement learning from scratch
    c) Tool use / Function calling
    d) Purely reactive behavior
    **Answer: c**

5.  Which of the following is a potential disadvantage of using an LLM for direct, real-time robot planning compared to traditional hard-coded or rule-based systems?
    a) Inability to handle new commands.
    b) Lower flexibility.
    c) Higher latency and computational cost.
    d) Lack of contextual understanding.
    **Answer: c**

---

## 5.2.7 Exercises

1.  **LLM Hallucinations in Robotics:** LLMs are known to "hallucinate" (generate plausible but incorrect information). Discuss the potential dangers of LLM hallucinations when used for robot cognitive planning. How can developers design robust systems to mitigate these risks (e.g., through verification, feedback loops, or constrained output)?
2.  **Prompt Engineering for Robotics:** Draft a prompt for an LLM that asks it to generate a plan for the command "Prepare breakfast for me." Assume the robot has access to tools like `find_item(item_name)`, `pick_up(item_name)`, `heat_food(item_name)`, `serve_food(item_name)`. The prompt should encourage multi-step reasoning.
3.  **Dynamic World Modeling:** In a real-world scenario, the robot's environment is dynamic. Discuss how the "World Model" component of an LLM planning pipeline would need to be updated and maintained in real-time based on sensor feedback (e.g., new objects appearing, objects moving) to ensure the LLM generates valid plans.
4.  **Failure Recovery with LLMs:** A robot is executing a plan generated by an LLM to "clean the table." It attempts `grasp_object("dirty plate")` but fails (e.g., due to slippery surface). Describe how an LLM could leverage this failure feedback to generate a recovery plan (e.g., "try again with different grasp", "report problem", "ask for help").
5.  **Multi-Modal LLM Inputs:** Future LLMs are likely to accept multi-modal inputs (e.g., text, images, video). How could a multi-modal LLM enhance robot cognitive planning beyond just text commands? Provide an example of a robot task that would significantly benefit from such an input (e.g., "Pick up *that* object" with an image of the object).