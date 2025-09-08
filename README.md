# **Phase 1 – Programming & Circuits Basics with Arduino**

From zero background → building a **line-following robot**.

---

## **Part A: Foundations of Circuits & Programming (6 Sessions)**

### **Session 1: Electricity Basics & First Arduino Program**

**Topics:**

- Voltage, current, resistance
    
- Ohm’s Law (conceptual only)
    
- Series vs. parallel resistors
    
- Arduino IDE setup & “Hello World” with Serial Monitor
    
- Blink LED sketch
    

**Outcome:** Students can wire an LED safely and run their first Arduino program.  
**Task:** Blink an LED and print “LED ON” / “LED OFF” to the Serial Monitor.

---

### **Session 2: Buttons & Variables**

**Topics:**

- Breadboards and button wiring with pull-down resistors
    
- Variables & math operators in C++
    
- Digital input (`digitalRead`)
    

**Outcome:** Students understand how inputs change program behavior.  
**Task:** Control an LED with a button (press = ON, release = OFF).

---

### **Session 3: Conditions & Interaction**

**Topics:**

- `if/else` statements
    
- Logical operators (`&&`, `||`)
    
- Button-controlled LED with conditions
    

**Outcome:** Students apply conditions to make circuits interactive.  
**Task:** LED ON when button pressed, OFF otherwise. Extend: if two buttons are pressed at once, blink LED.  Make a button toggle led state between ON and OFF.
**Mini-Milestone 1:** _Interactive Light_.

---

### **Session 4: Loops & PWM**

**Topics:**

- `for`, `while`, `do-while` loops
    
- PWM basics for dimming LEDs
    
- `analogWrite` function
    

**Outcome:** Students can create repetitive behavior and control intensity.  
**Task:** Fade an LED brightness up and down smoothly with loops.

---

### **Session 5: Sensors & Thresholds**

**Topics:**

- Analog input with `analogRead`
    
- LDR (light-dependent resistor)
    
- Threshold decision-making
    

**Outcome:** Students build reactive programs based on sensor input.  
**Task:** Build a night lamp: LED turns ON in dark, OFF in light.  
**Mini-Milestone 2:** _Smart Night Lamp_.

---

### **Session 6: Functions & Motor Basics**

**Topics:**

- Functions for reusability (`void myFunction()`)
    
- Motor drivers (L293D/H-bridge)
    
- Controlling motor speed with PWM
    

**Outcome:** Students modularize code and control motors safely.  
**Task:** Write a function `runMotor(speed)` and control motor speed via Serial input.

---

## **Part B: Expanding Arduino Control (5 Sessions)**

### **Session 7: Libraries, Servos & Memory Basics**

**Topics:**

- `#include` and Arduino libraries
    
- Servo motor basics
    
- Pointer overview and how it interacts with memory
    

**Outcome:** Students extend Arduino with libraries.  
**Task:** Sweep a servo 0–180° automatically, then control it via button presses.
Use a pointer with a function to change a value by reference.

---

### **Session 8: LCD Displays**

**Topics:**

- 16x2 LCD in 4-bit mode
    
- I2C LCD for simpler wiring
    
- Printing sensor values
    

**Outcome:** Students can visualize data on displays.  
**Task:** Show live LDR + temperature sensor values on an LCD.

---

### **Session 9: Temperature & Environment Sensors**

**Topics:**

- LM35 temperature sensor
    
- DHT11 temperature + humidity sensor
    
- Reading & displaying values
    

**Outcome:** Students integrate environmental sensors into projects.  
**Task:** Display “Too Hot” on LCD if LM35 > 30°C, else show live reading.

---

### **Session 10: Distance Sensing & Decisions**

**Topics:**

- Ultrasonic sensor (HC-SR04)
    
- Measuring distance in cm
    
- Using distance to control motors
    

**Outcome:** Students build obstacle-aware systems.  
**Task:** Motor runs forward unless obstacle < 10 cm, then stops.  
**Mini-Milestone 3:** _Parking Assistant_.

---

### **Session 11: Automation & Bluetooth Control**

**Topics:**

- Relays as electronic switches
    
- Driving larger loads (lamps, fans)
    
- Using Bluetooth to control from phone
    

**Outcome:** Students can design small “automation” systems.  
**Task:** Build a fan system: DC motor (fan) ON if temp > 30°C, OFF otherwise, with status shown on LCD, Extend it with Bluetooth control for multiple lamps, a fan, and display temperature on the phone.

---

## **Part C: Robotics & Final Project (4 Sessions)**

### **Session 12: IR Sensors & Line Detection**

**Topics:**

- IR line sensors for black/white detection
    
- Mapping sensor states to robot movement
    
- Basic logic for tracking lines
    

**Outcome:** Students understand robotic sensing for navigation.  
**Task:** Robot moves forward if both sensors detect line, turns if only one sensor detects line.

---

### **Session 13: Stepper Motors & Precision Control**

**Topics:**

- Stepper motor basics
    
- ULN2003/A4988 driver usage
    
- Comparing stepper vs. servo vs. DC motor
    

**Outcome:** Students gain exposure to precision motor control.  
**Task:** Rotate a stepper motor exactly 180° and back with button control.

---

### **Session 14: Robot Integration & Testing**

**Topics:**

- Combining motors + sensors + logic
    
- Tuning thresholds for reliable detection
    
- Debugging with Serial Monitor
    

**Outcome:** Students integrate subsystems into a robot.  
**Task:** Build a two-motor robot that follows a straight black line.

---

### **Session 15: Final Project – Line Following Robot**

**Topics:**

- Refining line-following algorithm
    
- Handling curves & recovery if robot drifts
    
- Project presentation & peer feedback
    

**Outcome:** Students complete a full robotic system showcasing all skills.  
**Task:** Build and program a two-motor robot that follows a curved track and recovers when off-line.  
**Final Milestone Project:** _Line Following Robot_.


# Bonus/Recorded Sessions (1 is mandatory the others are optional)

### **Error types and the tool-chain** (More detail for the seekers of knowledge)

**Topics:**

- Knowing different types of errors 
	
- gain knowledge of the tool-chain used to make an executable

**Outcome:** Students will be able to diagnose errors and where it originates from, gaining more skills to analyse errors and how to evade them.

**Task:** Try to replicate all the error types intentionally and mention where in the tool-chain it came from.

### VSCode Installation and g++ (Must be seen before programming)
**Topics:**
- Installing g++ and making sure it works
	
- Installing VSCode 
	
- Explain VSCode and it's customization options
	
- Explain Extensions System and install *code runner* extension

**Outcome:** Having the ability to use a better tools that is VSCode in programming, saving time in writing code and debugging.

**Task:** None.


### PlatformIO for programming Arduino (Optional tool)
**Topics:**

- Installing and Explanation of PlatformIO Extension 
	
- How to make an Arduino project, choosing the board and framework
	
- Folders explanation 
	
- Explaining the config file

**Outcome:** Coding faster in VSCode with auto completion and AI when needed and having more control with a more fast, advanced tool than Arduino IDE.

**Task:** Try to replicate a task inside of VSCode instead of Arduino IDE.

