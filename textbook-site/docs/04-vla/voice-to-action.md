# 5.1 Voice-to-Action for Humanoid Robotics

## 5.1.1 Introduction to Voice Commands

Integrating natural language processing into robotics enables more intuitive and accessible human-robot interaction. Instead of programming robots with complex code or operating them with joysticks, users can issue high-level commands using their voice. This "Voice-to-Action" paradigm is critical for making humanoid robots truly useful and adaptive companions in human environments.

The journey from a spoken command to a robot's physical action involves several stages:
1.  **Speech-to-Text (STT):** Converting spoken audio into written text.
2.  **Natural Language Understanding (NLU):** Parsing the text to extract intent, entities, and context.
3.  **Action Planning:** Translating the understood intent into a sequence of executable robot tasks.
4.  **Robot Execution:** Commanding the robot's hardware to perform the planned actions.

This chapter focuses on the initial steps: speech recognition and parsing instructions.

## 5.1.2 Whisper: State-of-the-Art Speech-to-Text

OpenAI's Whisper is a powerful, open-source automatic speech recognition (ASR) system. It is trained on a massive dataset of diverse audio and performs exceptionally well across various languages, accents, and noisy environments. Whisper's robust performance makes it an excellent choice for converting spoken commands into reliable text inputs for robotic systems.

**Key Features of Whisper:**
-   **Multilingual:** Supports transcription in numerous languages and translation into English.
-   **Robustness:** Handles background noise, speech impediments, and different speaking styles effectively.
-   **Accuracy:** Achieves state-of-the-art results on many benchmark datasets.
-   **Open Source:** Accessible models and code, allowing for local deployment and fine-tuning.

### Technical Breakdown: How Whisper Works
Whisper is an encoder-decoder Transformer model.
-   **Encoder:** Takes raw audio input, processes it, and extracts high-level features.
-   **Decoder:** Takes these features and generates the corresponding text token by token.

It uses a sequence-to-sequence architecture, similar to those used in neural machine translation. It's trained to predict the next token in a sequence given the previous tokens and the audio features, allowing it to generate full sentences rather than just isolated words.

### Workflow: Using Whisper for Speech-to-Text

1.  **Audio Capture:** Use a microphone to record the user's voice command.
2.  **Audio Preprocessing:** Convert the captured audio into a format suitable for Whisper (e.g., 16kHz sample rate, mono channel).
3.  **Whisper Model Inference:** Pass the preprocessed audio to the Whisper model.
4.  **Text Output:** Receive the transcribed text.

**Example 5.1: Python Snippet for Whisper Integration (using `whisper` library)**

First, install the library: `pip install openai-whisper`

```python
import whisper
import pyaudio # For audio capture (optional, can use pre-recorded files)
import wave
import numpy as np

# Load a tiny model for demonstration, or a larger one for better accuracy
# Options: 'tiny', 'base', 'small', 'medium', 'large'
model = whisper.load_model("base")

def record_audio(filename="output.wav", duration=5):
    """Records audio from microphone for a given duration."""
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000 # Whisper expects 16kHz audio

    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)

    print(f"Recording for {duration} seconds...")
    frames = []
    for _ in range(0, int(RATE / CHUNK * duration)):
        data = stream.read(CHUNK)
        frames.append(data)

    print("Finished recording.")

    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()
    return filename

def transcribe_audio(audio_path):
    """Transcribes an audio file using the Whisper model."""
    print(f"Transcribing {audio_path}...")
    result = model.transcribe(audio_path)
    return result["text"]

if __name__ == "__main__":
    # Option 1: Transcribe a pre-recorded file
    # audio_file = "path/to/your/audio.mp3"
    # text = transcribe_audio(audio_file)
    # print(f"Transcribed Text (from file): {text}")

    # Option 2: Record from microphone and transcribe
    recorded_file = record_audio(duration=5) # Record for 5 seconds
    text = transcribe_audio(recorded_file)
    print(f"Transcribed Text (from mic): {text}")
```

## 5.1.3 Speech-to-Text in ROS 2

Integrating Whisper into a ROS 2 system allows robots to directly receive and process spoken commands.

### Technical Breakdown: ROS 2 Whisper Node

A typical ROS 2 `whisper_node` would:
1.  **Subscribe to Audio Input:** Listen to a ROS 2 topic (e.g., `/audio_in`) that provides raw audio data from a microphone. This audio might come from a `audio_driver` node.
2.  **Buffer Audio:** Collect incoming audio chunks until a certain duration or a silence detection threshold is met (indicating end of speech).
3.  **Process with Whisper:** Feed the buffered audio to the Whisper model.
4.  **Publish Text Output:** Publish the transcribed text to another ROS 2 topic (e.g., `/speech_to_text`) as a `std_msgs/msg/String` message.
5.  **Service Interface (Optional):** Offer a ROS 2 service to transcribe a given audio file or buffer on demand, useful for specific applications.

**Example 5.2: ROS 2 `whisper_node` Structure (Conceptual Python)**

```python
# my_ros2_package/my_ros2_package/whisper_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData # Assuming audio_common_msgs for audio input
import whisper
import numpy as np

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.declare_parameter('model_name', 'base')
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.get_logger().info(f"Loading Whisper model: {model_name}")
        self.model = whisper.load_model(model_name)
        self.get_logger().info("Whisper model loaded.")

        self.audio_subscription = self.create_subscription(
            AudioData,
            '/audio_in',
            self.audio_callback,
            10
        )
        self.text_publisher = self.create_publisher(String, '/speech_to_text', 10)

        self.audio_buffer = []
        self.buffer_duration_seconds = 5 # Process audio every 5 seconds or on silence detection
        self.buffer_max_size = self.buffer_duration_seconds * 16000 # 16kHz mono audio

        # A timer to periodically check buffer or more advanced VAD (Voice Activity Detection) could be used
        # For simplicity, we'll process on buffer full here.

    def audio_callback(self, msg: AudioData):
        # Assuming msg.data is raw audio bytes (e.g., int16)
        # Convert to numpy array of float32 for Whisper
        audio_chunk = np.frombuffer(bytes(msg.data), dtype=np.int16).astype(np.float32) / 32768.0
        self.audio_buffer.extend(audio_chunk)

        if len(self.audio_buffer) >= self.buffer_max_size:
            self.process_audio_buffer()

    def process_audio_buffer(self):
        if not self.audio_buffer:
            return

        audio_segment = np.array(self.audio_buffer, dtype=np.float32)
        self.audio_buffer = [] # Clear buffer after processing

        try:
            result = self.model.transcribe(audio_segment, fp16=False) # fp16=False for CPU inference
            transcribed_text = result["text"].strip()
            if transcribed_text:
                self.get_logger().info(f"Transcribed: '{transcribed_text}'")
                text_msg = String()
                text_msg.data = transcribed_text
                self.text_publisher.publish(text_msg)
        except Exception as e:
            self.get_logger().error(f"Error during transcription: {e}")

def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperNode()
    rclpy.spin(whisper_node)
    whisper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.1.4 Parsing Instructions into Actions

Once spoken commands are converted into text, the next critical step is Natural Language Understanding (NLU) to extract the user's **intent** and any relevant **entities**. This allows the robot to understand *what* the user wants and *on what* it wants the action performed.

### Intent Recognition
Determining the primary goal or command in the user's utterance.
**Examples:**
-   "Move forward" -> `intent: move`
-   "Pick up the red ball" -> `intent: pick_up`
-   "Turn left by 90 degrees" -> `intent: turn`

### Entity Extraction
Identifying key pieces of information (objects, locations, quantities, directions) that are crucial for executing the command.
**Examples:**
-   "Pick up the **red ball**" -> `entity: object = "red ball"`
-   "Move **forward**" -> `entity: direction = "forward"`
-   "Turn **left** by **90 degrees**" -> `entity: direction = "left", angle = "90 degrees"`

### Technical Breakdown: Parsing Methods

1.  **Rule-Based Parsing:**
    -   Using regular expressions or predefined grammar rules to match patterns in the text.
    -   **Pros:** Highly accurate for specific, well-defined commands. Easy to implement for simple tasks.
    -   **Cons:** Not scalable for complex, varied natural language. Brittle to variations in phrasing.

2.  **Machine Learning (ML)-Based NLU:**
    -   Training models (e.g., using libraries like SpaCy, NLTK, or dedicated NLU platforms like Rasa, Google Dialogflow, etc.) on annotated examples to learn intent and entity patterns.
    -   **Pros:** More robust to variations in phrasing. Scalable to complex language.
    -   **Cons:** Requires annotated training data. Can be computationally more intensive.

3.  **Large Language Models (LLMs) for NLU:**
    -   Using advanced LLMs (like Gemini, GPT) to directly interpret commands and extract structured actions. LLMs can perform zero-shot or few-shot NLU with remarkable accuracy due to their vast pre-training.
    -   **Pros:** Highly flexible, handles complex and novel commands, requires minimal explicit training data.
    -   **Cons:** Can be slow (latency), computationally expensive, and may hallucinate or misinterpret in unusual contexts. Requires careful prompting.

### Workflow: Parsing with an LLM (Example using Gemini API - Conceptual)

```python
# Conceptual Python snippet for LLM-based parsing
import json
from your_llm_api_client import Gemini

llm_client = Gemini(api_key="YOUR_API_KEY")

def parse_robot_command_with_llm(command_text):
    prompt = f"""
    You are a robot command parser. Your task is to extract the user's intent and any relevant entities from a natural language command and output them in a JSON format.

    If the command is unclear or cannot be parsed into a robot action, respond with "unclear".

    Examples:
    Command: "Pick up the red box"
    Output: {{"intent": "pick_up", "entities": {{"object": "red box"}}}}

    Command: "Go to the kitchen"
    Output: {{"intent": "navigate", "entities": {{"location": "kitchen"}}}}

    Command: "Stop moving"
    Output: {{"intent": "stop", "entities": {{}}}}

    Command: "What's the weather like?"
    Output: "unclear"

    Command: "{command_text}"
    Output:
    """
    response = llm_client.generate_content(prompt, temperature=0.0) # Low temperature for factual extraction
    try:
        parsed_output = response.text.strip()
        if parsed_output == "unclear":
            return {"intent": "unclear", "entities": {}}
        return json.loads(parsed_output)
    except json.JSONDecodeError:
        return {"intent": "parsing_error", "entities": {"raw_output": response.text}}

if __name__ == "__main__":
    commands = [
        "Please fetch the blue book from the shelf",
        "Move forward five steps",
        "Can you turn off the light?",
        "Tell me a joke",
        "Rotate clockwise 90 degrees",
        "Find my keys"
    ]
    for cmd in commands:
        parsed = parse_robot_command_with_llm(cmd)
        print(f"Command: '{cmd}' -> Parsed: {parsed}")
```

The output from the parsing stage (e.g., `{"intent": "pick_up", "entities": {"object": "red box"}}`) can then be fed into a robot's action planning module, which translates this structured intent into a series of low-level robot movements and commands.

---

## 5.1.5 Lab Task: ROS 2 Voice Command Parser

**Objective:** Create a ROS 2 system where a user can speak a command, Whisper transcribes it, and a simple parser extracts a robot action.

**Steps:**
1.  **Audio Source Node:** Set up a simple ROS 2 `audio_publisher_node.py` that simulates audio input. Instead of a real microphone, you can:
    *   Load a pre-recorded WAV file (e.g., containing "move forward" or "pick up the cup").
    *   Publish chunks of this audio as `audio_common_msgs/msg/AudioData` messages to `/audio_in`.
2.  **Whisper ROS 2 Node:**
    *   Implement the `whisper_node.py` as described in Example 5.2.
    *   Ensure it subscribes to `/audio_in` and publishes to `/speech_to_text`.
    *   Configure it to use a small Whisper model (e.g., `tiny` or `base`) to avoid excessive memory/CPU usage.
3.  **Command Parser Node:**
    *   Create a ROS 2 Python node (`command_parser_node.py`).
    *   Subscribe to the `/speech_to_text` topic (output from Whisper).
    *   Implement a simple rule-based parser (e.g., using `if/else` statements and keyword checking) to detect intents and entities:
        *   "move forward/backward/left/right" -> `intent: move, direction: [forward/backward/left/right]`
        *   "turn left/right" -> `intent: turn, direction: [left/right]`
        *   "pick up [object_name]" -> `intent: pick_up, object: [object_name]`
    *   Publish the parsed action (e.g., as a custom ROS 2 message `RobotAction.msg` containing `string intent` and `string[] params`) to a new topic `/robot_action`.
    *   Log the parsed intent and entities to the console.
4.  **Launch File:** Create a Python launch file to bring up all three nodes (`audio_publisher_node`, `whisper_node`, `command_parser_node`).
5.  **Test:** Run the launch file. The `audio_publisher_node` will "play" its command, Whisper will transcribe it, and the `command_parser_node` will log the extracted action.

**Deliverables:**
-   ROS 2 package (`voice_command_interface`) containing:
    -   `audio_publisher_node.py` (simulated audio source).
    -   `whisper_node.py` (Whisper integration).
    -   `command_parser_node.py` (Rule-based parser).
    -   `RobotAction.msg` (custom message for parsed actions).
    -   Launch file (`voice_command.launch.py`).
-   Console output demonstrating successful speech-to-text and parsing of at least three distinct commands.
-   A brief discussion on the limitations of rule-based parsing and when ML/LLM-based parsing would be necessary.

---

## 5.1.6 Multiple Choice Questions

1.  What is the primary function of the Speech-to-Text (STT) component in a Voice-to-Action system?
    a) To understand the emotional tone of the speaker.
    b) To convert spoken audio into written text.
    c) To translate text into a different language.
    d) To generate a sequence of robot tasks from text.
    **Answer: b**

2.  Which of the following is a key advantage of OpenAI's Whisper model for STT in robotics?
    a) It only works for a single language.
    b) It is highly sensitive to background noise.
    c) It offers robust performance across diverse audio conditions and languages.
    d) It requires extensive custom training for each new application.
    **Answer: c**

3.  In the context of Natural Language Understanding (NLU), what does "intent recognition" aim to extract from a user's command?
    a) The grammatical structure of the sentence.
    b) The primary goal or command the user wants the robot to perform.
    c) The raw audio features of the spoken command.
    d) The sentiment (positive/negative) of the user's speech.
    **Answer: b**

4.  Which parsing method is generally most robust to variations in natural language phrasing but typically requires annotated training data?
    a) Rule-based parsing
    b) Regular expressions
    c) Machine Learning (ML)-based NLU
    d) Hardcoded keyword matching
    **Answer: c**

5.  When integrating Whisper into a ROS 2 system, which ROS 2 message type would most likely be used to publish the transcribed text?
    a) `sensor_msgs/msg/AudioData`
    b) `std_msgs/msg/Int32`
    c) `std_msgs/msg/String`
    d) `geometry_msgs/msg/PoseStamped`
    **Answer: c**

---

## 5.1.7 Exercises

1.  **Microphone Selection:** You are choosing a microphone for a humanoid robot that will take voice commands in a noisy factory environment. Research and discuss the features you would prioritize in microphone selection (e.g., directional vs. omnidirectional, noise cancellation, frequency response) and justify your choices.
2.  **Voice Activity Detection (VAD):** Explain why a Voice Activity Detection (VAD) component is often critical before feeding audio to an ASR system like Whisper. How would VAD improve the efficiency and accuracy of a ROS 2 `whisper_node`?
3.  **Improving Rule-Based Parser:** Take the conceptual rule-based parser from the lab task. Propose two specific improvements or extensions to make it more flexible, without switching to a full ML/LLM model. For example, how would you handle "Go forward ten meters" versus "Go forward"?
4.  **LLM vs. Traditional NLU for Robotics:** Compare and contrast the use of a pre-trained Large Language Model (LLM) for parsing robot commands versus a traditional ML-based NLU system (e.g., using SpaCy). Discuss the advantages and disadvantages of each, considering aspects like development effort, robustness to new commands, and computational cost.
5.  **Multimodal Commands:** Imagine a scenario where a user says "Pick up *that* object" while pointing to an object. How would a Voice-to-Action system need to be extended to handle such multimodal commands (voice + gesture)? Discuss the additional sensing and processing steps required.
