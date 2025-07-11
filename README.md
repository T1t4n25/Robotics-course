# Robotics Curriculum – Full Roadmap

---

## Phase 1 - Programming & Circuits Basics

### أساسيات برمجة (4 أسابيع مع القليل من الـ problem solving)

1. **Session 1**
    - Introduction to the curriculum and tools to install
    - Hello world example in C++
    - Using g++ to build your code
    - PlatformIO on VSCode for beginner-friendly environment

2. **Session 2**
    - Variables, memory, operations (% / * - +)
    - Control statements (if, switch)
    - Loops
    - Tiny challenge: FizzBuzz
    - Homework: simple pattern printing

3. **Session 3**
    - Arrays
    - Functions
    - Basic pointers and pointer arithmetic

4. **Session 4**
    - Struct
    - STL (math, data structures)
    - Basic algorithms: searching, sorting

5. **Session 5**
    - Object-Oriented Programming basics
    - Problem solving with real-life inspired examples

---

### أساسيات دوائر كهربية مع STM32 وبرمجتها

6. **Session 6**
    - How electricity flows, electrons, and general safety
    - Voltage, current, resistance, power – concepts & calculations
    - Homework: explore resistors, multimeter use

7. **Session 7**
    - Batteries, charging/discharging
    - BMS, voltage regulators
    - Powering Arduino/STM32/ESP32 from USB or batteries
    - Homework: build simple working circuit

8. **Session 8**
    - Microcontroller power limitations
    - LED connection types: sinking vs sourcing
    - Powering motors, sensors, and servos (overview)
    - DC motor drivers (overview)

9. **Session 9**
    - GPIO and pin modes
    - PWM signals and their uses
    - ADC and analog sensors
    - Using temperature/humidity sensors, photoresistors, and map() function

10. **Session 10**
    - 7-segment displays and multiplexing
    - LCDs, OLEDs, and I2C basics
    - Project assignment: build a smart home demo

11. **Session 11**
    - Project support and debugging
    - Q&A session

12. **Session 12**
    - Reviewing finished projects
    - Project upgrades and integration ideas
    - Helping remaining students finish

---

## Phase 2 - ESP32, Wireless Control, and IoT Projects

1. **Session 1**
    - Intro to communication using microcontrollers
    - ESP32 overview and pin layout
    - Connect ESP32 to Wi-Fi
    - Control LED via HTTP GET `/on` and `/off`

2. **Session 2**
    - Bluetooth vs BLE (high-level explanation)
    - Connect via mobile app
    - Build a password-protected LED control terminal

3. **Session 3**
    - ESP32-CAM overview, limitations, pinout
    - Stream video over local Wi-Fi
    - Capture and serve photos over the network

4. **Session 4**
    - ESP32-CAM face detection / face recognition demo
    - Trigger actions (buzzer/message) on face detection
    - Save photos to SD card (logging)

5. **Session 5 - Communication Protocols**
    - Serial vs I2C vs SPI
    - Connect ESP32 to STM32
    - Send data across protocols

6. **Session 6**
    - Log sensor data to the cloud
    - Use Firebase as a backend for temp/humidity
    - Build charts to visualize data over time

7. **Session 7 - Blynk Integration**
    - Overview of Blynk platform
    - Build dashboard + controls
    - Connect ESP32 to Blynk
    - Send/receive data and commands

8. **Session 8 - DroidPad Controller**
    - Use DroidPad to build virtual gamepad
    - Send control data over Bluetooth/Wi-Fi
    - Learn basic JSON syntax

9. **Session 9 - Smart Car Project**
    - Control car via Bluetooth/Wi-Fi
    - Use ESP32-CAM for video feed
    - Integrate sensors:  temp/humidity
    - Add safety features using IR
    - Optional: LED headlights, buzzer, LCD screen
    - Optional: Use RC car chassis for better results
    - Optional: (gyroscope, accelerometer) for a hand glove controller

---

## Phase 3 - Build a Robot Arm with ROS 2

### Goal: Full pipeline from ROS 2 → microcontroller → real robot arm

1. **Session 1 - What is ROS 2 and How It Works**
    - ROS 2 uses in industry (robot arms, autonomous cars)
    - Node-topic-message architecture
    - Install Ubuntu + ROS 2 Jazzy
    - Run basic publisher/subscriber (`talker`, `listener`)
    - Use `ros2 topic list`, `ros2 topic echo`

2. **Session 2 - Create Your First ROS 2 Workspace and Node**
    - Create new workspace
    - Build a Python package
    - Write simple publisher node (joint angles simulator)
    - Publish dummy values

3. **Session 3 - Simulate a Robot Arm (URDF + RViz)**
    - Load basic robot arm in RViz using `joint_state_publisher_gui`
    - Move joints manually
    - Activity: set poses (left, center, right)

4. **Session 4 - Control Arm in Simulation**
    - Write node to send joint angles programmatically
    - Animate arm poses step-by-step
    - Activity: wave the robot arm

5. **Session 5 - Services and Parameters**
    - Create ROS 2 service: move to "home", "grab"
    - Add parameters and YAML config
    - Activity: trigger poses via terminal

6. **Session 6 - Custom Messages**
    - Define `MoveArm.msg` with joint1 to joint4
    - Publish poses using custom message
    - Activity: cycle through stored poses

7. **Session 7 - Serial Bridge to ESP32/STM32**
    - ESP32/Arduino reads joint angles from serial
    - ROS 2 Python node sends formatted serial string
    - Activity: move real robot arm using ROS 2

8. **Session 8 - Build the Real Arm**
    - Assemble SG90 servo arm with gripper
    - Use 5V external power safely
    - Activity: test 2 real-world poses

9. **Session 9 - Optional Additions**
    - Add IR sensor or ultrasonic sensor
    - Build GUI with `rqt` or Python
    - Activity: allow movement only if object is close

10. **Session 10 - Combine Everything**
    - Final code: ROS 2 node → serial → physical robot
    - Safety checks and limits
    - Activity: complete motion test with 3+ poses

11–12. **Final Project**
- Pick and place robot with camera mounted on a big tank like chassis

    
