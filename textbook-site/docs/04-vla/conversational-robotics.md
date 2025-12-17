# 5.3 Conversational Robotics

## 5.3.1 Introduction to Robot Dialogue

Conversational robotics focuses on enabling robots to engage in natural, meaningful dialogue with humans. Beyond simply understanding commands (Voice-to-Action) or planning tasks (LLM Cognitive Planning), conversational robots can maintain context, respond appropriately to questions, offer clarification, and engage in more fluid, human-like interactions. This is particularly important for humanoid robots designed to co-exist and collaborate with humans in complex environments.

**Key Aspects of Robot Dialogue:**
-   **Natural Language Understanding (NLU):** Interpreting user utterances.
-   **Dialogue State Tracking:** Maintaining a record of the conversation's progress and relevant information.
-   **Dialogue Policy:** Deciding the robot's next action (e.g., ask a question, provide information, execute a command).
-   **Natural Language Generation (NLG):** Formulating the robot's responses in human-readable language.
-   **Context Management:** Remembering past interactions and relevant environmental details.

## 5.3.2 Reactive vs. Deliberative Conversation

Robot conversational systems can broadly be categorized into reactive and deliberative approaches.

### Reactive Conversation
-   **Definition:** The robot responds immediately and directly to the user's last utterance, often without deep understanding of the broader context or future implications.
-   **Mechanism:** Typically rule-based, pattern matching, or simple slot-filling models.
-   **Characteristics:**
    -   Fast response times.
    -   Limited scope and depth of understanding.
    -   Can feel unnatural or repetitive if the conversation deviates from expected patterns.
    -   Primarily used for simple command-and-control interfaces or FAQ-like interactions.
-   **Example:** User: "Move forward." Robot: "Moving forward." User: "Stop." Robot: "Stopping."

### Deliberative Conversation
-   **Definition:** The robot considers the entire conversation history, its internal state, its goals, and potentially its understanding of the world before formulating a response or action. It aims for more coherent, goal-oriented, and human-like dialogue.
-   **Mechanism:** Often employs more advanced AI techniques, including dialogue state trackers, dialogue managers, and deep learning models (especially LLMs).
-   **Characteristics:**
    -   Slower response times compared to purely reactive systems due to increased processing.
    -   Deep understanding of context and intent.
    -   Can handle ambiguity, ask clarifying questions, and recover from misunderstandings.
    -   Enables more complex, multi-turn interactions for task completion or social engagement.
-   **Example:** User: "I need to clean the table." Robot: "Okay. Which table would you like me to clean? The one in the living room or the kitchen?"

**Table 5.1: Reactive vs. Deliberative Conversation**

| Feature           | Reactive Conversation                         | Deliberative Conversation                     |
| :---------------- | :-------------------------------------------- | :-------------------------------------------- |
| **Response Time** | Fast                                          | Can be slower                                 |
| **Contextual Awareness** | Low (focus on last utterance)                 | High (considers history, world state)         |
| **Complexity Handled** | Simple commands, direct questions             | Complex tasks, multi-turn dialogue, ambiguity |
| **Primary Goal**  | Immediate response/action                     | Goal-oriented interaction, deeper understanding |
| **Example Use Case** | Command execution, simple Q&A                 | Task-oriented dialogue, social robotics         |

## 5.3.3 Context Management

Effective conversation requires the robot to remember and utilize information from the ongoing dialogue and its environment. This is known as context management.

**Types of Context:**
1.  **Dialogue History:** Previous turns in the conversation.
    -   _Example:_ User: "Pick up the block." Robot: "Okay." User: "Now move it there." (Robot must remember "the block" from previous turn).
2.  **User Preferences/Profile:** Information about the user.
    -   _Example:_ "Remember I like my coffee black."
3.  **Environmental State:** The robot's knowledge of its surroundings (e.g., object locations, room layout).
    -   _Example:_ User: "Is the red ball still on the floor?" (Robot needs to check its internal map/perception data).
4.  **Task State:** Progress of an ongoing task.
    -   _Example:_ Robot: "I've grasped the red box. Where would you like me to place it?"

**Technical Approaches for Context Management:**
-   **Slot Filling:** Identifying and storing key pieces of information (slots) extracted from utterances.
-   **Dialogue State Trackers (DSTs):** Models that maintain a representation of the current state of the dialogue, including filled slots, user intent, and current task progress.
-   **Knowledge Graphs:** Structured representations of facts and relationships about the world, which the dialogue system can query.
-   **LLMs:** Modern LLMs, especially with their large context windows, can inherently manage conversational history and often infer context without explicit state tracking. They can be prompted to store and retrieve information about the conversation.

## 5.3.4 Safety Filters

When robots, especially humanoids, engage in conversation, safety is paramount. This involves not only physical safety but also preventing the robot from generating harmful, inappropriate, or biased responses. Safety filters are crucial for mitigating these risks.

**Types of Safety Filters:**
1.  **Content Moderation:** Filtering out toxic, hateful, sexually explicit, violent, or self-harm-related content in both user inputs and robot outputs.
2.  **Guardrails/Bounds Checking:** Ensuring robot commands remain within safe operational limits.
    -   _Example:_ User: "Move at maximum speed." Robot: (Internal check) "I will move at a safe operational speed, not maximum, to avoid collision."
3.  **Ethical/Moral Constraints:** Preventing the robot from engaging in actions that violate ethical guidelines or societal norms. This is particularly challenging for LLM-driven systems.
4.  **Misinterpretation Detection:** Identifying when the robot might have misunderstood a command, prompting for clarification.
5.  **Emergency Stop Integration:** Allowing users to issue immediate stop commands that bypass the conversational system.

### Technical Breakdown: Implementing Safety Filters

-   **Keyword/Phrase Blacklists:** Simple, reactive filters.
-   **ML-based Classifiers:** Trained models to detect problematic content in text.
-   **LLM-based Moderation:** Using a separate LLM or a moderation API to evaluate the safety of prompts and responses.
-   **Pre-defined Policies:** Hard-coded rules and constraints that override LLM-generated actions if they violate safety protocols.
-   **Reinforcement Learning from Human Feedback (RLHF):** Training LLMs to align with human values and safety preferences.

**Diagram: Safety Filter Integration**

```
+--------------------+        +---------------------+
|    User Input      |        |   Robot Response    |
| (Spoken/Text)      |------->|   (Text/Spoken)     |
+--------------------+        +---------------------+
          |                               ^
          v                               |
+---------------------+        +---------------------+
|   Speech-to-Text    |        |   Natural Language  |
|                     |        |   Generation        |
+---------------------+        +---------------------+
          |                               ^
          v                               |
+---------------------+      +---------------------+
|  Content Moderation |------>|                     |
|  (User Input)       |      |  LLM/Dialogue Mgr.  |
+---------------------+      |                     |
          |                    |  Safety Check &   |
          v                    |  Guardrails       |
+---------------------+      |  (Robot Action)     |
|   NLU & Context     |------>|                     |
|                     |      +---------------------+
+---------------------+                 ^
          |                             |
          v                             |
+---------------------+                 |
| LLM Cognitive Planner |-----------------+
|   (Action Sequence) |
+---------------------+
```

---

## 5.3.5 Lab Task: Conversational Robot with Context and Safety

**Objective:** Extend the previous LLM planning system to include basic conversational elements, context management, and a safety filter.

**Steps:**
1.  **Context Store:**
    *   Create a simple in-memory Python dictionary (`robot_context`) in your `llm_planner_node.py` to store key-value pairs (e.g., `{"last_object_mentioned": "block", "current_location": "kitchen"}`).
    *   Modify your `mock_llm_planner` function to receive this `robot_context`.
2.  **LLM with Contextual Awareness (Mock):**
    *   Enhance `mock_llm_planner` to demonstrate deliberative conversation:
        *   If the command refers to "it" or "that," make it query `robot_context["last_object_mentioned"]`.
        *   If the user asks "Where am I?", the mock LLM should respond by querying `robot_context["current_location"]` (instead of generating an action).
        *   If the user asks a follow-up question related to a previous command, the LLM should show awareness.
3.  **Safety Filter (Rule-Based):**
    *   Implement a simple rule-based safety filter in `llm_planner_node.py` *before* the `mock_llm_planner` is called.
    *   Example: If the input text contains "destroy" or "harm," the node should print a safety warning and prevent any action plan generation.
    *   Another example: If a generated `move_to_location` action attempts to move to a "forbidden_zone," the filter should intercept and prompt for an alternative or refuse.
4.  **Dialogue Management (Simple NLG):**
    *   In `llm_planner_node.py`, instead of always publishing an action plan, sometimes publish a `std_msgs/msg/String` to a new topic `/robot_response` for conversational replies (e.g., "I am currently in the kitchen.").
5.  **Action Executor with Feedback:**
    *   Modify `action_executor_node.py` to send back a `std_msgs/msg/String` feedback to the `llm_planner_node.py` (e.g., "Successfully moved to kitchen," "Grasping failed"). The `llm_planner_node.py` should update its `robot_context` based on this feedback.
6.  **Test:**
    *   Use a sequence of commands to demonstrate context: "Pick up the red ball." -> "Now put it on the table."
    *   Test safety filters: "Destroy everything." -> (Expected: safety warning).
    *   Test context query: "Where am I?" -> (Expected: conversational response from robot based on context).

**Deliverables:**
-   Updated `llm_planner_node.py` with `robot_context`, enhanced `mock_llm_planner` for context and safety.
-   Updated `action_executor_node.py` to provide feedback.
-   New ROS 2 message for `RobotActionPlan` (if needed) and `std_msgs/msg/String` for `/robot_response`.
-   Launch file running `whisper_node`, `llm_planner_node`, and `action_executor_node`.
-   Console output demonstrating:
    *   Multi-turn conversation with context.
    *   Successful triggering of safety filter.
    *   Robot providing conversational responses.
-   Discussion on the limitations of rule-based safety filters and challenges of ethical AI in conversational robotics.

---

## 5.3.6 Multiple Choice Questions

1.  What is the primary difference between a "reactive" and a "deliberative" conversational robot?
    a) Reactive robots only respond to voice, while deliberative robots respond to text.
    b) Reactive robots consider only the last utterance, while deliberative robots consider broader context and goals.
    c) Deliberative robots are always faster than reactive robots.
    d) Reactive robots can handle ambiguity, while deliberative robots cannot.
    **Answer: b**

2.  Which component is most responsible for maintaining a record of the conversation's progress and relevant information for a robot?
    a) Speech-to-Text (STT) module
    b) Natural Language Generation (NLG) module
    c) Dialogue State Tracking / Context Management
    d) Low-level motor controllers
    **Answer: c**

3.  Which of the following would NOT typically be stored in a robot's conversational context?
    a) The user's preferred coffee order.
    b) The robot's current battery level.
    c) The raw audio waveform of the last spoken command.
    d) The location of an object mentioned two turns ago.
    **Answer: c**

4.  Why are "safety filters" particularly crucial in conversational robotics, especially for humanoid robots?
    a) To enhance the robot's speech recognition accuracy.
    b) To ensure the robot always sounds polite.
    c) To prevent the robot from generating harmful/inappropriate responses or executing unsafe commands.
    d) To speed up the robot's response time.
    **Answer: c**

5.  Which technique is used to train Large Language Models to align with human values and safety preferences?
    a) Rule-based blacklisting
    b) Reinforcement Learning from Human Feedback (RLHF)
    c) Keyword spotting
    d) Manual dictionary creation
    **Answer: b**

---

## 5.3.7 Exercises

1.  **Designing a Clarification Strategy:** A user tells a robot, "Go get the book." The robot perceives several books in the room. Design a deliberative conversational strategy for the robot to ask for clarification, identifying what information it needs (e.g., color, size, location) and how it would phrase its questions.
2.  **Context Evolution:** Describe how the `robot_context` would need to evolve over a conversation where the user first says "Find the red box," then "What's in it?", and finally "Pick it up." Detail the updates to the context and how the robot would leverage it.
3.  **Proactive Conversation:** Beyond just responding to user commands, how could a humanoid robot proactively engage in conversation to be more helpful? Give an example of a proactive conversational turn and explain what information (from its sensors, internal state, or context) the robot would need to initiate it.
4.  **LLM and Ethical Dilemmas:** Imagine a scenario where a user asks a humanoid robot to perform an action that is technically feasible but morally ambiguous or potentially harmful (e.g., "Hide this sensitive document from my boss"). How might an LLM-driven conversational system, equipped with safety filters and ethical guidelines, handle such a request? Discuss the challenges and possible responses.
5.  **Multi-Modal Dialogue:** How can visual cues (e.g., a robot's gaze direction, head nods, facial expressions) be integrated into conversational robotics to make interactions more natural and effective? Provide an example of how a robot's visual behavior could complement its spoken dialogue in a complex task.