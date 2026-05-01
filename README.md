# Autonomous-Pick-and-Place-with-Theta-Star-for-the-Kuka-youBot

## Abstract
This project presents the design and simulation of an autonomous robotic system for industrial material handling. The system, referred to as the **Factory Automation Robot**, performs pick-and-place operations by identifying, grasping, and relocating objects within a structured factory environment. The implementation demonstrates the integration of perception, decision-making, and robotic manipulation in a simulated industrial workflow.

**Autonomous Pick-and-Place Robotic System using KUKA robot in Webots**

**1. Introduction**

Automation in modern manufacturing environments plays a critical role in improving efficiency, accuracy, and safety. This project focuses on developing a robotic solution capable of autonomously handling objects in a factory-like setting. The system is designed to replicate real-world industrial processes using a simulated environment.

**2. Objective**

To develop an autonomous robotic system capable of detecting, identifying, and manipulating objects from a storage area and placing them accurately onto designated shelves using intelligent control strategies.

**3. System Architecture**

The system consists of the following key components:

Robotic Platform: KUKA robotic arm with gripper
Perception System:
Camera for vision-based object detection
LiDAR for spatial awareness and environment sensing
Control System:
Python-based control algorithms
Predefined coordinate mapping for structured environments
Simulation Platform:
Webots robotic simulation environment

**4. Methodology**

The system operates using a hybrid perception and control approach:

Object detection using camera-based vision system
Position estimation using sensor data and predefined coordinates
Motion planning for robotic arm movement
Grasping using robotic gripper
Object placement on designated shelves
This structured workflow enables accurate and repeatable industrial operations.

**5. Key Features**
   
Fully autonomous operation
Vision-based object detection
Sensor-assisted spatial awareness (LiDAR)
High precision pick-and-place mechanism
Structured industrial workflow simulation

**6. Simulation Environment**

The system is implemented in an intermediate-level industrial environment featuring:

Factory layout with defined storage zones
Multiple box-type objects
Shelving units for placement
Realistic robotic manipulation setup

**7. Tools and Technologies**

Programming Language: Python
Simulation Software: Webots
Robotic System: KUKA robotic arm
Sensors: Camera and LiDAR

**8. Results and Discussion**

The system successfully demonstrates autonomous pick-and-place functionality within a structured environment. The integration of vision and sensor-based perception enables reliable object detection and positioning.

Note: Quantitative performance metrics such as accuracy and execution time will be included in future updates.

**9. Future Work**

Integration of advanced computer vision algorithms (e.g., deep learning)
Real-time obstacle avoidance
Dynamic path planning
Multi-robot coordination for large-scale automation

**10. Repository Structure**

/code → Python control scripts
/worlds → Webots simulation files
/images → Project visuals and screenshots
/docs → Additional documentation

**11. Authors**

Manish Kondurkar
Master’s in Electrical Engineering
University of South Alabama

Gabriel Goncalves
Master’s in Computer Engineering
University of South Alabama

Kyle Moore
Master’s in Electrical Engineering
University of South Alabama

Paxton cooper
Bachelors in Electrical Engineering
University of South Alabama

**12. Acknowledgments**

This project was developed as part of an academic robotics course, emphasizing practical implementation of autonomous systems in industrial environments.

