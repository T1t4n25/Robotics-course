# Robotics Curriculum – Full Roadmap

---

## Phase 1 - Programming & Circuits Basics
- **C++ Programming:** Master core programming concepts in C++ including data types, control flow, functions, and basic **Object-Oriented Programming (OOP)**.
    
- **Electronic Circuits:** Understand fundamental electrical principles like voltage, current, and resistance to safely build and troubleshoot circuits.
    
- **Microcontroller Interfacing:** Learn to use an **STM32** to control and read from hardware using **GPIO**, **PWM**, and **Analog-to-Digital Converters (ADC)**.
    
- **Data Display:** Utilize various components like **7-segment displays** and **LCDs/OLEDs** with protocols like **I2C** to visualize data.
    
- **Sensor Integration:** Connect and read data from common sensors (e.g., temperature, photoresistor) to make your projects interactive.
    
- **Project-Based Learning:** Apply all skills to complete a comprehensive **smart home project**, integrating code and electronics.
### **أساسيات البرمجة (4 أسابيع مع القليل من الـ problem solving)**

1. **Session 1: C++ Basics & Manual Compilation** 🖥️
    
    - Introduction to the curriculum and essential tools to install.
        
    - The "Hello, World!" example in C++.
        
    - Using the `g++` compiler to build your code from the command line.
        
    - **Outcome:** Students will be able to write, compile, and run a simple C++ program using the `g++` compiler.
        
    - **Task:** Compile and run a basic "Hello, World!" program using g++.
        
2. **Session 2: Variables & Control Flow** 🧠
    
    - Variables, memory, and arithmetic operations (`%`, `/`, `*`, `-`, `+`).
        
    - Control statements (`if`, `switch`).
        
    - Loops.
        
    - Tiny challenge: FizzBuzz.
        
    - **Outcome:** Students will be able to use C++ variables, control statements (`if`, `switch`), and loops to solve simple logic problems.
        
    - **Task:** Write a program that prints a right-angled triangle pattern of `*` characters, where the number of rows is a user-defined variable.
        
3. **Session 3: Arrays & Functions** ✍️
    
    - Arrays.
        
    - Functions.
        
    - Basic pointers and pointer arithmetic.
        
    - **Outcome:** Students will be able to work with arrays, define custom functions, and use basic pointers to manipulate data.
        
    - **Task:** Create a function that takes an integer array and its size, then reverses the array in-place using pointers.
        
4. **Session 4: Structs & STL** 📚
    
    - `struct` data types.
        
    - Standard Template Library (**STL**) for math and data structures.
        
    - Basic algorithms: searching and sorting.
        
    - **Outcome:** Students will be able to create custom data structures with `structs` and use the Standard Template Library (STL) for common tasks like sorting.
        
    - **Task:** Define a `struct` named `Student` with members for name, ID, and GPA. Use `std::vector` to store multiple student records and sort them by GPA.
        
5. **Session 5: OOP & Problem Solving** 🚀
    
    - Object-Oriented Programming (**OOP**) basics.
        
    - Problem solving with real-life inspired examples.
        
    - **Outcome:** Students will be able to apply the fundamental principles of Object-Oriented Programming (OOP) to solve problems using classes and objects.
        
    - **Task:** Create a class named `BankAccount` with private members for balance and account number, and public methods for `deposit`, `withdraw`, and `get_balance`.
        

---

### **أساسيات دوائر كهربية مع STM32 وبرمجتها**

6. **Session 6: Electricity Basics & Introducing the Microcontroller IDE** ⚡
    
    - How electricity flows, electrons, and general safety.
        
    - Voltage, current, resistance, and power – concepts and calculations.
        
    - Introduction to PlatformIO on VSCode: Why we need a specialized IDE for embedded systems, and how to set up a basic project.
        
    - **Outcome:** Students will understand fundamental electrical concepts and be able to set up their development environment with **PlatformIO** in VSCode.
        
    - **Task:** Use a breadboard to build a simple series circuit with two resistors and an LED with a button and calculate the resistance.
        
    - **Problem-Solving Task:** Write a C++ program that calculates the current and power given voltage and resistance, handling potential errors.
        
7. **Session 7: Powering Microcontrollers** 🔋
    
    - Batteries, charging, discharging.
        
    - BMS and voltage regulators.
        
    - Powering Arduino/STM32/ESP32 from USB or batteries.
        
    - **Outcome:** Students will be able to safely power an **STM32** using a battery and a simple voltage regulator circuit.
        
    - **Task:** Power an STM32 using a 9V battery and a simple voltage regulator circuit.
        
    - **Problem-Solving Task:** Write a C++ program that simulates a battery management system (BMS) with charging/discharging logic.
        
8. **Session 8: Microcontroller Power & LEDs** ⚙️
    
    - Microcontroller power limitations.
        
    - LED connection types: sinking vs sourcing.
        
    - Powering motors, sensors, and servos (high-level overview).
        
    - **Outcome:** Students will be able to connect and program an LED to an **STM32** pin, understanding the difference between sinking and sourcing current.
        
    - **Task:** Connect an LED to an STM32 pin and program it to turn on/off by controlling the pin in both sinking and sourcing modes.
        
    - **Problem-Solving Task:** Write a C++ function that takes a motor speed and direction and outputs the correct signals.
        
9. **Session 9: GPIO, PWM & ADC** 📈
    
    - **GPIO** and pin modes.
        
    - **PWM** signals and their uses.
        
    - **ADC** and analog sensors.
        
    - Using temperature/humidity sensors and photoresistors.
        
    - **Outcome:** Students will be able to read data from an analog sensor and use **PWM** to control an output based on the sensor's values.
        
    - **Task:** Connect a temperature sensor to an analog pin of the STM32. Read the sensor data, convert it to Celsius, and display the result on the serial monitor.
        
    - **Problem-Solving Task:** Write a C++ function that uses a temperature value to calculate a PWM duty cycle for a cooling fan.
        
10. **Session 10: Displaying Data** 📊
    
    - **7-segment displays** and multiplexing.
        
    - **LCDs**, **OLEDs**, and **I2C** protocol basics.
        
    - Project assignment: build a smart home demo.
        
    - **Outcome:** Students will be able to display dynamic data on both a **7-segment display** and an **LCD/OLED** screen.
        
    - **Task:** Connect a 4-digit 7-segment display to the STM32 and program it to display a countdown from 9999 to 0.
        
    - **Problem-Solving Task:** Create a C++ program that converts an integer (e.g., a sensor reading) into an array of four digits for display on a 7-segment.
        
11. **Session 11: Project Support** 🤝
    
    - Project support and debugging.
        
    - Q&A session.
        
    - **Outcome:** Students will have debugged their smart home project and implemented at least one additional feature.
        
    - **Task:** Implement at least one new feature in your smart home project (e.g., adding an LCD screen to display the status or a button to manually toggle a device).
        
    - **Problem-Solving Task:** Write a C++ program that simulates the logic of your project's main control system.
        
12. **Session 12: Project Review** ✅
    
    - Reviewing finished projects.
        
    - Project upgrades and integration ideas.
        
    - Helping remaining students finish.
        
    - **Outcome:** Students will have a completed, working smart home project and will be able to present its code, hardware, and functionality.
        
    - **Task:** Present your finished smart home project.
        
    - **Problem-Solving Task:** Review another student's project code and suggest a specific optimization for either logic or memory usage.
---

## Phase 2 - ESP32, Wireless Control, and IoT Projects

1. **Session 1: WiFi Communication** 
    
    - Intro to online communication using microcontrollers
        
    - ESP32 overview, capabilities, pins layout
        
    - Connect ESP32 to wifi
        
    - Use HTTP GET `/on` `/off` to control an LED
        
	- **Task:** Write code to connect the ESP32 to a local Wi-Fi network. Then, create a simple web server that listens for a URL with `/on` or `/off` to toggle an LED.
		
    - **Outcome:** Students will be able to connect an **ESP32** to a Wi-Fi network and control an LED via simple web browser commands.
        
2. **Session 2: Bluetooth & Mobile Control** 📱
    
    - Connect to ESP32 using bluetooth
        
    - Bluetooth vs BLE, a high-level explanation
        
    - Using a mobile app to control leds and motors
        
    - Build a password-locked LED controller via Bluetooth Terminal as an activity
		
	- **Task:** Program the ESP32 to receive commands over Bluetooth. Use a mobile app like a Bluetooth Terminal to send a password. If the password is correct, toggle an LED. 
        
    - **Outcome:** Students will be able to establish a **Bluetooth** connection with an **ESP32** and control its outputs from a mobile app.
        
3. **Session 3: Video Streaming** 
    
    - ESP32-CAM, limitations, and pinout
        
    - Streaming video over WLAN
        
    - Capturing photos and serving them over LAN
		
    - **Task:** Configure the ESP32-CAM to stream a live video feed that can be viewed on a web browser on the same network. Add a button on the webpage to capture a still image.
        
    - **Outcome:** Students will be able to set up an **ESP32-CAM** to stream video and capture photos over a local network.
        
4. **Session 4: Image Processing & QR Code Detection** 
    
    - ESP32-CAM QR Code Scanner demo
        
    - Trigger an action when QR is seen
        
    - Basic SD card photo logging
	    
	- **Task:** Program the ESP32-CAM to read a QR code. If the decoded message is "turn_on," toggle an LED. If the message is "turn_off," turn the LED off. Display the decoded message on the serial monitor.
        
    - **Outcome:** Students will be able to capture an image with the **ESP32-CAM**, process it to find a QR code, and perform a specific action based on the code's content and be able to save photos to SD Card.
        
5. **Session 5: Communication Protocols (Serial, I2C, SPI)** 
    
    - Serial vs I2C vs SPI
        
    - Connect two microcontrollers (**ESP32** <--> **STM32**)
        
    - Send data over serial or I2C
	    
    - **Task:** Connect an **ESP32** and an **STM32**. Program the ESP32 to read a sensor value and send it to the STM32 via **I2C**. The STM32 should then display the received value on the serial monitor.
        
    - **Outcome:** Students will be able to understand and use **Serial**, **I2C**, and **SPI** protocols to enable communication between different microcontrollers.
        
6. **Session 6: Cloud Logging** 
    
    - Logging data to the cloud
        
    - Firebase as a cloud storage for sensor data (temp, humidity)
        
    - Charts to visualize changes overtime
	    
    - **Task:** Connect a temperature and humidity sensor to the ESP32. Write a program that sends the sensor data to a Firebase database every few seconds.
        
    - **Outcome:** Students will be able to send sensor data from an **ESP32** to a cloud platform like **Firebase** for storage and visualization.
        
7. **Session 7: Blynk Dashboard** 
    
    - Overview on Blynk platform
        
    - Creating a dashboard and controls
        
    - Connecting esp32 with blynk
        
    - Taking commands and sending data
	    
    - **Task:** Create a Blynk dashboard with a gauge to display temperature readings and a button to remotely control an LED connected to the ESP32.
        
    - **Outcome:** Students will be able to create a custom **Blynk** dashboard to remotely monitor and control a microcontroller project.
        
8. **Session 8: Mobile App Integration** 
    
    - Learn how to make controllers with Droid Pad
        
    - How to communicate data
        
    - What is JSON
	    
    - **Task:** Use Droid Pad to build a simple controller interface with a joystick. Program the ESP32 to receive the joystick data in **JSON** format and use it to control two servos.
        
    - **Outcome:** Students will be able to use a mobile application to send and receive structured data (**JSON**) for advanced control.
        
9. **Session 9: Smart Car Project** 
    
    - Control car using bluetooth or wifi
        
    - Use esp-cam to send video feed to phone
        
    - Use different sensors to get data about the place you're at like gyroscope, accelerometer, temp and humidity
	    
    - **Task:** Assemble the smart car chassis and connect the motor drivers. Program the ESP32 to receive control commands (forward, backward, left, right) over Bluetooth or Wi-Fi to drive the car.
        
    - **Outcome:** Students will have successfully implemented wireless control and camera integration for the **smart car project**.
        
10. **Session 10: Smart Car Project (Advanced Features)** 
    
    - Create a safety mechanism to stop on objects with IR sensor
        
    - Optional: Put led lights in front, a buzzer, and an LCD with messages from your phone or animations
        
    - Optional: Use RC car chassis instead of 2W /4W car
	    
    - **Task:** Add an **IR sensor** to the front of the car. Program a safety mechanism that automatically stops the car if an obstacle is detected.
        
    - **Outcome:** Students will have integrated multiple sensors (e.g., gyroscope, IR) and additional features into the smart car to collect environmental data and perform automated actions.
        
11. **Session 11: Project Support** 
    
    - Support for the project, QA
        
    - **Outcome:** Students will have a fully functional smart car prototype with all planned features implemented and debugged.
        
12. **Session 12: Project Review** 
    
    - Reviewing the project for finished students
        
    - Giving ideas for improvements and more integration
        
    - Helping the rest of the students to finish
	    
    - **Task:** Work on debugging and finalizing your smart car project. Ensure that all components (camera, sensors, motors, etc.) are working together smoothly and are ready for a final demo.
        
    - **Outcome:** Students will be able to present a completed **smart car project** and explain its functionality and design choices to their peers.

---

### **Phase 3 - Build a Robot Arm with ROS 2**

### Goal: Create a full robotics pipeline in simulation, then optionally extend to a physical robot arm.

---

### **Module 1: Linux Fundamentals (4 Sessions)**

  

1. **Session 1: Introduction to the Linux Command Line** 🐧

- Understanding the terminal and shell.

- Navigating the filesystem with `pwd`, `cd`, and `ls` (including common flags like `-a` and `-l`).

- Basic file and directory management with `mkdir`, `rm`, `mv`, and `touch`.

- Using the `man` command to understand other commands.

- **Outcome:** Students will be able to navigate the Ubuntu filesystem and perform basic file management using the command line.

- **Task:** Navigate to a designated home directory. Create a project folder and inside it, create several subdirectories. Create, move, and rename multiple files within this structure using only terminal commands.

2. **Session 2: Permissions, Users, and the `sudo` Command** 🔐

- Understanding file permissions (read, write, execute) and how they apply to users, groups, and others.

- The `chmod` and `chown` commands.

- The concept of the root user and the importance of using `sudo` for administrative tasks.

- **Outcome:** Students will be able to manage file access permissions, understand user privileges, and safely use the `sudo` command.

- **Task:** Create a new file and change its permissions to be read-only for all users. Then, create a "shared" directory and set permissions so that all users can read and write to it, but not delete its contents.

3. **Session 3: Text Editing and Process Management** ✍️

- Introduction to command-line text editors with a focus on `nano` for beginners.

- Basic `vim` commands as an alternative.

- Viewing running processes with `ps`, `top`, and `htop`.

- Managing processes by bringing them to the background (`&`) and terminating them with the `kill` command.

- **Outcome:** Students will be able to edit configuration files directly from the terminal and effectively manage running applications and background processes.

- **Task:** Use `nano` to create a simple text file. Run a long-running process in the background and then use `ps` and `kill` to safely terminate it.

4. **Session 4: Package Management and Networking** 🌐

- The role of a package manager.

- Using `apt` to install, update, upgrade, and remove software.

- Finding packages with `apt-cache search`.

- Basic networking commands like `ping`, `ifconfig`, and a high-level overview of `ssh`.

- **Outcome:** Students will be able to install and manage software on Ubuntu and perform basic network diagnostics.

- **Task:** Install a new command-line tool using `apt` (e.g., `htop`). Check the local IP address with `ifconfig` and then use `ping` to verify network connectivity to an external server.

  

---

  

### **Module 2: Simulation with Gazebo & ROS 2 (5 Sessions)**

  

5. **Session 5: The ROS 2 CLI and Architecture** 🐢

- Installing **ROS 2** Jazzy.

- The core architecture of ROS 2: nodes, topics, and messages.

- The `ros2` command-line interface (`CLI`) to inspect the running network.

- The `talker/listener` demo as a practical example.

- **Outcome:** Students will understand the fundamental concepts of the ROS 2 network and will be able to use the `ros2 CLI` to interact with nodes and topics.

- **Task:** Install ROS 2. Run the `talker` and `listener` nodes and use `ros2 node list` and `ros2 topic echo` to inspect the communication between them.

6. **Session 6: Introduction to Gazebo Simulation** 🤖

- What is **Gazebo** and why it's used in robotics.

- Exploring the Gazebo interface, its physics engine, and its role as a virtual environment.

- Spawning simple objects from the command line.

- **Outcome:** Students will be able to launch the Gazebo simulator and spawn basic objects to understand the physics and coordinate systems of the virtual world.

- **Task:** Launch a blank Gazebo world. Use the `gz` CLI to spawn a cube and a sphere. Then, use the GUI to apply a force to the objects and observe their behavior.

7. **Session 7: Robot Modeling with URDF** 🧱

- The **Unified Robot Description Format (URDF)**.

- Defining robot components as links (rigid bodies) and joints (connections).

- Creating a simple four-link robot arm model with multiple joints.

- **Outcome:** Students will be able to write a simple URDF file from scratch to define a robot's physical structure and joints.

- **Task:** Write a URDF file for a four-link robot arm. Use `rviz2` to load and visualize the model, then use the `joint_state_publisher_gui` to manually move each joint.

8. **Session 8: Advanced Gazebo Simulation & Sensors** 👁️

- Integrating the URDF model into a Gazebo world.

- Adding simulated sensors to the robot arm's model (e.g., a camera, distance sensor).

- Viewing sensor data as a ROS 2 topic.

- **Outcome:** Students will be able to add simulated sensors to their robot arm model and view the sensor data as a ROS 2 topic.

- **Task:** Modify the URDF file to include a simulated camera and a distance sensor on the robot arm's end-effector. Launch the Gazebo simulation and use `ros2 topic echo` to see the sensor data.

9. **Session 9: Programming a Simulated Robot** 👨‍💻

- Creating a ROS 2 Python node.

- Publishing messages to control the robot's joints based on a plan.

- Creating a subscriber node to read data from the simulated sensors.

- **Outcome:** Students will be able to write a basic ROS 2 Python node to programmatically move the simulated robot arm and read sensor feedback.

- **Task:** Write a Python publisher node that sends a sequence of joint angles to the simulated arm, making it perform a simple "go-home" routine.

---


### **Module 3: Final Project: From Simulation to Reality (5 Sessions)**


> **Note:** The core project for this module is to create a complete **"pick and place"** pipeline in simulation. Hardware integration is an optional extension for students who wish to apply their completed simulation code to a physical robot arm.

10. **Session 10: ROS 2 Services and Parameters** ⚙️

- Introduction to **ROS 2 services** for request/response communication.

- Creating a service client and server.

- Storing configuration data in a **YAML** file and accessing it with ROS 2 parameters.

- **Outcome:** Students will be able to create and use ROS 2 services and parameters for specific, reusable robot actions within their simulation.

- **Task:** Create a service called `set_pose`. Store the joint angles for two poses ("home" and "grab") in a YAML file. Write a service client that requests the simulated arm to move to these poses by name.

11. **Session 11: Custom Messages and Packages** ✉️

- The need for custom message types.

- Defining a `.msg` file with custom data structures.

- Building a custom message package with `colcon build`.

- **Outcome:** Students will be able to define and use a custom message type to send complex data to the simulated robot arm.

- **Task:** Create a custom message that can hold all joint angles as an array. Modify the publisher node to use this custom message to cycle through stored poses in the simulation.

12. **Session 12: Final Project in Simulation** ✅

- Combining all the concepts learned so far: nodes, topics, services, and custom messages.

- Programming a complete **"pick and place"** logic for the simulated robot arm.

- Implementing safety checks and joint limits within the simulation code.

- **Outcome:** Students will have a fully functional robot arm controlled by ROS 2 in a simulated environment, capable of completing a complex task.

- **Task:** Finalize all simulation code. Demonstrate a complete "pick and place" operation where the simulated arm moves to a specific location, interacts with a virtual object, and moves it to another location.

13. **Session 13: (Optional) Assembling the Real Robot Arm** 🔩

- Physical assembly of the servo-based robot arm.

- Safely using an external 5V power supply to prevent damage to the microcontroller.

- Calibrating the servos and testing individual joint movements.

- **Outcome:** Students who choose this path will have a physical robot arm assembled and ready for integration.

- **Task:** Assemble the arm with SG90 servos and a gripper. Connect the servos to the microcontroller with an external power source. Test 2 distinct poses using a simple script.

13. **Session 14: (Optional) Bridging to Hardware** 🌉
- How to read and write serial data in ROS 2.

- The importance of a serial bridge.

- Programming a microcontroller (**ESP32** or **Arduino**) to receive a formatted string of joint angles (e.g., "J1:90,J2:45,J3:0") and control its servos.

- **Outcome:** Students will be able to establish a serial bridge between a ROS 2 node and their physical robot arm to control it with the same code used in the simulation.

- **Task:** Write a ROS 2 node that sends formatted joint angle commands over a serial port. Program the microcontroller to parse these commands and move the corresponding servos on the physical arm.
