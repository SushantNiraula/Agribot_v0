# 🌱 Agribot — Autonomous Agricultural Mobile Robot

**Agribot** is a six-wheel autonomous agricultural rover developed as my **major project in Electronics, Communication and Information Engineering**.

The project explores how robotics, embedded systems, computer vision, edge AI, control, sensing, and ROS 2 can be integrated into a single mobile platform for agricultural environments.

Agribot was originally developed as a real physical robot and is now being further reconstructed, organized, simulated, and extended as a **ROS 2 research platform**.

This repository contains the main software workspace for the robot, including:

- mobile robot modeling,
- differential-drive control,
- wheel odometry,
- sensor integration,
- EKF-based state estimation,
- Gazebo simulation,
- crop-row navigation,
- computer vision,
- obstacle avoidance,
- agricultural inspection,
- robotic arm coordination,
- ROS 2 interfaces,
- hardware/simulation bringup,
- and higher-level system integration.

The project is currently also serving as the **base experimental platform for research work on perception models, inference performance, and closed-loop agricultural robot navigation**.

---

# Project Motivation

Agricultural automation is challenging because farms are not highly structured environments.

A robot operating between crop rows must deal with:

- irregular crop geometry,
- changing illumination,
- missing plants,
- weeds,
- soil and terrain variations,
- sensor noise,
- limited computational resources,
- localization drift,
- obstacles,
- and the need to make decisions in real time.

Agribot was built as an attempt to study these problems through a complete robotic system rather than treating perception, control, or AI as isolated components.

The basic operating philosophy of the robot is:

```text
Sense
  ↓
Understand Environment
  ↓
Navigate
  ↓
Inspect / Probe
  ↓
Decide
  ↓
Actuate
  ↓
Continue
```

---

# System Overview

Agribot is designed as a layered robotic system.

```text
┌──────────────────────────────────────────┐
│           User / Monitoring Layer        │
│                                          │
│      Remote Monitoring • Commands        │
└────────────────────┬─────────────────────┘
                     │
                     ↓
┌──────────────────────────────────────────┐
│             ROS 2 Autonomy Layer         │
│                                          │
│ Navigation • Perception • State Machine  │
│ Inspection • Sensor Fusion • Planning    │
└────────────────────┬─────────────────────┘
                     │
                     ↓
┌──────────────────────────────────────────┐
│              Perception Layer            │
│                                          │
│ Camera • Crop Rows • LiDAR • AI / CV     │
└────────────────────┬─────────────────────┘
                     │
                     ↓
┌──────────────────────────────────────────┐
│          State Estimation / Control      │
│                                          │
│ Encoder Odom • IMU • EKF • cmd_vel       │
└────────────────────┬─────────────────────┘
                     │
                     ↓
┌──────────────────────────────────────────┐
│          Embedded Control Layer          │
│                                          │
│ MCU • PID • Encoders • Motor Control     │
└────────────────────┬─────────────────────┘
                     │
                     ↓
┌──────────────────────────────────────────┐
│               Robot Hardware             │
│                                          │
│ 6-Wheel Base • Arm • Sensors • Pump      │
└──────────────────────────────────────────┘
```

---

# 🤖 Physical Agribot

Agribot originated as a **real six-wheel agricultural rover** built during my undergraduate engineering project.

The six wheels are operated as a left and right drive group, giving the robot differential-drive / skid-steer behavior.

Conceptually:

```text
       LEFT DRIVE                     RIGHT DRIVE

     ┌───────────┐                  ┌───────────┐
     │   Wheel   │                  │   Wheel   │
     ├───────────┤                  ├───────────┤
     │   Wheel   │      ROBOT       │   Wheel   │
     ├───────────┤                  ├───────────┤
     │   Wheel   │                  │   Wheel   │
     └───────────┘                  └───────────┘
```

The physical project integrates or has explored:

- six-wheel mobile base,
- wheel encoders,
- motor drivers,
- embedded motor controller,
- Raspberry Pi-based computing,
- IMU,
- camera perception,
- LiDAR,
- ROS 2,
- micro-ROS,
- robotic arm,
- soil sensing,
- irrigation,
- crop inspection,
- edge AI,
- and remote monitoring.

---

# 🧠 Computing Architecture

Agribot separates high-level computation from low-level deterministic hardware control.

A simplified architecture is:

```text
                 Raspberry Pi / ROS 2
                         │
      ┌──────────────────┼──────────────────┐
      │                  │                  │
      ↓                  ↓                  ↓
 Perception         Navigation        State Estimation
      │                  │                  │
      └──────────────────┼──────────────────┘
                         │
                      /cmd_vel
                         │
                         ↓
              Embedded Controller
                         │
                ┌────────┴────────┐
                │                 │
                ↓                 ↓
           Left Motors       Right Motors
                ↑                 ↑
                │                 │
              Encoder Feedback
```

The high-level computer handles computationally heavier tasks such as perception and autonomous decision-making.

The embedded controller is responsible for time-sensitive operations such as:

- encoder counting,
- wheel-speed measurement,
- PID control,
- motor actuation,
- low-level odometry,
- and communication with the ROS 2 system.

---

# ROS 2 Architecture

The current repository organizes Agribot as a collection of ROS 2 packages rather than a single monolithic program.

This allows individual subsystems to be developed, tested, replaced, and debugged independently.

Representative packages include:

```text
Agribot_v0/
│
├── agribot_base/
│
├── agribot_bringup/
│
├── agribot_controller/
│
├── agribot_control_bridge/
│
├── agribot_flask_bridge/
│
├── agribot_interfaces/
│
├── agribot_vision/
│
├── crop_row_nav/
│
├── arm_control/
│
├── cmd_vel_bridge/
│
├── firmware/pico2/        (Pico 2 micro-ROS firmware, see its README)
│
├── hardware/arm_pi/       (Raspberry Pi arm scripts, see its README)
│
└── ...
```

The workspace contains both:

```text
SIMULATION
     +
REAL-HARDWARE INTEGRATION
```

which is important because the same higher-level architecture can progressively move from simulation toward physical deployment.

---

# 🌍 Simulation Environment

The repository contains a Gazebo-based simulation environment for testing the robot without requiring continuous access to the physical platform.

The simulation stack includes:

- robot spawning,
- robot state publishing,
- controllers,
- ROS–Gazebo bridging,
- simulated sensors,
- RViz visualization,
- differential-drive behavior,
- and a custom environment.

Conceptually:

```text
                 Gazebo
                   │
           Simulated Agribot
                   │
       ┌───────────┼───────────┐
       │           │           │
     Camera      LiDAR        Odom
       │           │           │
       └───────────┼───────────┘
                   │
                  ROS 2
                   │
            Agribot Nodes
```

Simulation provides a controlled environment for:

- debugging,
- navigation development,
- perception experiments,
- controller testing,
- and repeatable research experiments.

---

# 🚜 Differential-Drive Control

Although Agribot physically contains six wheels, the wheels are grouped into:

```text
Left Side
Right Side
```

and modeled using differential-drive kinematics.

Given commanded robot velocities:

\[
v
\]

for linear velocity and:

\[
\omega
\]

for angular velocity, the required wheel velocities are determined from:

\[
v_L = v-\frac{\omega L}{2}
\]

\[
v_R = v+\frac{\omega L}{2}
\]

where \(L\) represents wheel separation.

This abstraction allows higher-level ROS 2 nodes to simply publish velocity commands without directly controlling individual motors.

---

# `/cmd_vel` Control Pipeline

A major interface throughout the robot is:

```text
geometry_msgs/Twist
```

published through:

```text
/cmd_vel
```

The control pipeline can be represented as:

```text
Autonomy / Navigation
        ↓
     /cmd_vel
        ↓
Control Bridge
        ↓
Embedded Controller
        ↓
Differential Kinematics
        ↓
PID Motor Control
        ↓
Six-Wheel Drivetrain
```

This creates a clean separation between:

```text
"What motion should the robot perform?"
```

and:

```text
"How should the motors physically achieve it?"
```

---

# 📏 Wheel Odometry

Agribot uses encoder measurements to estimate robot motion.

For each wheel group:

\[
d = \Delta ticks
\left(
\frac{2\pi r}{N}
\right)
\]

where:

- \(r\) = wheel radius
- \(N\) = encoder ticks per revolution

The left and right wheel distances are combined to estimate:

\[
x,\ y,\ \theta
\]

for the robot pose.

Conceptually:

```text
Wheel Encoders
      ↓
Encoder Ticks
      ↓
Wheel Distance
      ↓
Differential-Drive Odometry
      ↓
     x,y,θ
      ↓
ROS Odometry
```

The repository contains a dedicated base package for handling the conversion from low-level wheel information into robot odometry.

---

# 🧭 State Estimation

Wheel odometry alone is imperfect.

Real robots experience:

- wheel slip,
- encoder noise,
- uneven terrain,
- mechanical inaccuracies,
- and integration drift.

Agribot therefore includes an **Extended Kalman Filter (EKF)** configuration to combine available state-estimation information.

Conceptually:

```text
Wheel Odometry ──────┐
                     │
IMU ─────────────────┼──→ EKF ──→ Filtered Robot State
                     │
Other Measurements ──┘
```

This provides a better state estimate for higher-level navigation than relying on wheel odometry alone.

---

# 🗺️ SLAM and LiDAR

The project also explores LiDAR-based environmental perception and mapping.

The ROS 2 workspace contains configuration for SLAM and sensor integration.

The overall idea is:

```text
LiDAR
  ↓
LaserScan
  ↓
SLAM / Mapping
  ↓
Map + Robot Pose
  ↓
Navigation
```

LiDAR is also useful as an independent obstacle-sensing modality alongside camera-based crop-row navigation.

---

# 🌿 Crop-Row Navigation

One of the most important parts of Agribot is autonomous navigation through agricultural crop rows.

The current implementation contains a lightweight computer-vision navigation pipeline designed with edge-computing constraints in mind.

The camera image is reduced to:

```text
320 × 240
```

before processing to reduce computational cost.

The present pipeline uses:

```text
Camera Image
     ↓
Resize
     ↓
HSV Conversion
     ↓
Green Color Segmentation
     ↓
Morphological Filtering
     ↓
Region Analysis
     ↓
Crop Centroids
     ↓
Line Fitting
     ↓
Navigation Error
     ↓
Steering Command
```

---

# Lightweight Crop Detection

The current navigator uses HSV color segmentation.

Conceptually:

\[
I_{RGB}
\rightarrow
I_{HSV}
\rightarrow
Binary\ Crop\ Mask
\]

Pixels within calibrated green thresholds are retained while the remaining pixels are rejected.

Morphological filtering is then used to reduce small segmentation artifacts.

---

# Strip-Based Crop Analysis

Instead of treating the entire image as a single region, the lower part of the image is divided into horizontal strips.

```text
Camera Image

┌─────────────────────────────┐
│                             │
│         ignored / far       │
│                             │
├─────────────────────────────┤
│          strip 1            │
├─────────────────────────────┤
│          strip 2            │
├─────────────────────────────┤
│          strip 3            │
├─────────────────────────────┤
│          strip 4            │
└─────────────────────────────┘
```

For each strip:

1. crop pixels are segmented,
2. contours are detected,
3. small noise regions are rejected,
4. the dominant crop region is selected,
5. its centroid is calculated.

This produces a series of crop-row observations:

\[
(x_1,y_1),
(x_2,y_2),
...,
(x_n,y_n)
\]

---

# Crop-Row Line Estimation

A line is fitted through detected crop centroids.

Conceptually:

```text
       •
         •
           •
             •
               ↓
         Estimated Row
```

The line is extrapolated toward the lower portion of the image to obtain a navigation anchor.

The error between the anchor and the image center becomes the steering error:

\[
e = x_{center}-x_{row}
\]

and angular velocity is generated proportionally:

\[
\omega = K_p e
\]

while the robot continues moving forward.

---

# Closed-Loop Visual Navigation

This creates a closed perception-control loop:

```text
Camera
   ↓
Crop Detection
   ↓
Row Geometry
   ↓
Steering Error
   ↓
cmd_vel
   ↓
Robot Motion
   ↓
New Camera Image
   └──────────────→ repeat
```

This distinction is important.

The objective is not only:

```text
"Can the model detect crops?"
```

but:

```text
"Can the perception system provide information
fast and reliably enough for the robot to navigate?"
```

That question becomes especially important in the research work built on top of this project.

---

# End-of-Row Detection

Agricultural navigation also requires recognizing when a crop row has ended.

Agribot maintains a small state machine such as:

```text
NAVIGATING
     ↓
End of row detected
     ↓
EOL_TURN
     ↓
Search for crop row
     ↓
Row detected
     ↓
NAVIGATING
```

The current implementation uses multiple consecutive frames before accepting an end-of-row condition.

This prevents a single failed detection from immediately triggering a turn.

---

# Navigation State Machine

The crop navigation system currently contains states such as:

```text
          ┌──────────────┐
          │  NAVIGATING  │
          └──────┬───────┘
                 │
       ┌─────────┴─────────┐
       │                   │
 travelled inspection   end of row
     distance              │
       │                   ↓
       ↓               ┌──────────┐
┌──────────────┐       │ EOL_TURN │
│  INSPECTING  │       └────┬─────┘
└──────┬───────┘            │
       │                 row found
       └──────────┬──────────┘
                  ↓
             NAVIGATING
```

This allows perception, navigation, and agricultural inspection to operate as one coordinated system.

---

# 🦾 Crop Inspection Arm

Agribot includes a robotic manipulator for interacting with crops and soil.

Rather than tightly coupling the arm and navigation code, the repository uses a ROS 2 action interface.

The navigation system can request:

```text
/arm/inspect_crop
```

through a custom:

```text
CropInspection
```

action.

This architecture allows the mobile base to:

1. navigate,
2. reach an inspection interval,
3. stop,
4. request an arm operation,
5. wait for completion,
6. continue navigating.

Conceptually:

```text
Navigate
   ↓
Inspection Point Reached
   ↓
Stop Robot
   ↓
Send CropInspection Goal
   ↓
Arm / Sensor Operation
   ↓
Result
   ↓
Resume Navigation
```

---

# Custom ROS 2 Interfaces

Agribot contains a dedicated interface package for project-specific ROS communication.

For example:

```text
CropInspection.action
```

allows the navigation system and robotic arm controller to interact through a defined ROS 2 contract.

This keeps subsystems loosely coupled and makes the robot easier to extend.

---

# 🌱 Soil Sensing and Irrigation

The physical Agribot project also explored localized agricultural interaction.

This includes:

```text
Soil Sensor
     ↓
Soil Condition
     ↓
Robot Decision
     ↓
Water Pump
```

The arm-control subsystem contains interfaces for soil sensing and pump actuation.

This extends the robot beyond:

```text
observe + navigate
```

toward:

```text
observe + navigate + interact
```

which is one of the broader goals of field robotics.

---

# 📷 Agricultural Perception

Agribot has explored multiple forms of visual perception.

These include:

- crop-row detection,
- classical image processing,
- plant inspection,
- disease-related visual analysis,
- and edge-AI-based perception.

The original major-project architecture also explored learned models for agricultural perception alongside lightweight methods suitable for embedded deployment.

This repository now gives me a framework in which those approaches can be compared under the **same robotic task**.

---

# 🚧 Obstacle Avoidance

Crop following alone is not sufficient for autonomous navigation.

The repository contains obstacle-avoidance logic that combines:

```text
Camera Navigation Command
          +
      LiDAR Scan
          ↓
 Safety / Obstacle Layer
          ↓
Final Motion Command
```

This allows perception-based row-following commands to be modified or overridden when an obstacle is detected.

This layered architecture is useful because:

```text
crop-row navigation
```

answers:

```text
"Where should I drive?"
```

while obstacle avoidance answers:

```text
"Is it currently safe to execute that motion?"
```

---

# 🔄 Robot Operating Modes

Agribot supports separating autonomous and manual operation.

Conceptually:

```text
             CONTROL MODE
                /    \
               /      \
          MANUAL      AUTO
             ↓          ↓
       User Control   Autonomy
```

The autonomy nodes can stop the robot when manual or stop mode is selected.

This is particularly important during development and testing of a physical robot.

---

# 🌐 Monitoring and Software Integration

Agribot is not limited to ROS nodes running locally.

The project has also explored bridges for transmitting robot state and commands to higher-level monitoring interfaces.

The repository includes software such as:

```text
agribot_flask_bridge
agribot_control_bridge
```

which connect ROS-side information with external monitoring/control components.

This allows data such as:

- robot status,
- sensor events,
- camera information,
- soil condition,
- odometry,
- and commands

to move between the robotics stack and user-facing software.

---

# 🧪 Hardware and Simulation Bringup

One design goal of the reconstructed repository is to maintain separate launch paths for:

```text
REAL ROBOT
```

and:

```text
SIMULATION
```

Representative launch files include:

```text
agribot.launch.py
display.launch.py
gazebo.launch.py
hardware.launch.py
pi_autonomy.launch.py
```

This is useful because a robotics system should ideally allow individual components to be tested without rewriting the complete architecture.

---

# System Data Flow

A simplified view of the complete Agribot system is:

```text
                        CAMERA
                          │
                          ↓
                  Crop-Row Perception
                          │
                          ↓
                   Steering Command
                          │
                          │
LiDAR ──→ Obstacle Layer ─┤
                          │
                          ↓
                       cmd_vel
                          │
                          ↓
                 Embedded Controller
                          │
                 ┌────────┴────────┐
                 ↓                 ↓
             Left Drive        Right Drive
                 ↑                 ↑
                 └──── Encoders ───┘
                          │
                          ↓
                    Wheel Odometry
                          │
              ┌───────────┴──────────┐
              │                      │
             IMU                    EKF
              │                      │
              └──────────────→ Robot State
                                      │
                                      ↓
                               Navigation Logic
                                      │
                                      ↓
                              Inspection Trigger
                                      │
                                      ↓
                                Robotic Arm
                                      │
                           ┌──────────┴─────────┐
                           ↓                    ↓
                     Soil Sensing        Crop Inspection
                           │
                           ↓
                       Irrigation
```

---

# Major Project Scope

Agribot should be viewed as a **system-integration robotics project**.

The engineering challenge was not simply building:

```text
a classifier
```

or:

```text
a mobile robot
```

or:

```text
a robotic arm
```

individually.

The challenge was integrating multiple engineering domains:

```text
Mechanical Platform
        +
Embedded Electronics
        +
Motor Control
        +
Sensors
        +
Computer Vision
        +
ROS 2
        +
Localization
        +
Navigation
        +
AI
        +
Robot Manipulation
        +
Monitoring
```

into one robotic system.

---

# What I Learned From Agribot

Agribot became one of the projects through which many concepts from my Electronics, Communication and Information Engineering degree started connecting together.

### Embedded systems

I worked with:

```text
GPIO
PWM
interrupts
microcontrollers
motor drivers
encoders
sensor interfaces
```

### Control systems

Concepts such as:

\[
PID
\]

and closed-loop motor velocity control became physical rather than purely mathematical.

### Robotics

I worked with:

```text
differential-drive kinematics
odometry
coordinate frames
robot state
mobile navigation
```

### Signal and sensor processing

The robot requires noisy physical sensor information to be turned into usable state estimates.

### Computer vision

Camera images must become meaningful geometric information that can control robot motion.

### Artificial intelligence

Learned perception models can provide richer semantic information, but they introduce computational cost and deployment constraints.

### Software engineering

A robotic system benefits from modular packages, clearly defined interfaces, reusable nodes, launch systems, and separation between hardware-dependent and hardware-independent components.

---

# Development Philosophy

As the project has evolved, I have increasingly moved away from thinking:

```text
"Does this algorithm work?"
```

toward:

```text
"Does this algorithm work inside the complete robot?"
```

For robotics, these are very different questions.

A perception model may achieve excellent accuracy but still perform poorly in the real control loop if:

- inference is too slow,
- frame rate becomes inconsistent,
- latency is high,
- computational load delays other nodes,
- detections fluctuate,
- or the robot reacts too late.

Likewise, a simpler method with lower standalone accuracy may sometimes produce better robotic behavior if it provides:

```text
faster
+
more predictable
+
sufficiently accurate
```

perception.

That system-level perspective has become one of the most important lessons from Agribot.

---

# Current Development

The repository is currently being actively improved as I reconstruct and refine the original major-project system.

Current areas of work include:

- ROS 2 Jazzy integration
- Gazebo simulation
- hardware/simulation consistency
- drivetrain control
- encoder odometry
- EKF configuration
- crop-row navigation
- perception optimization
- obstacle avoidance
- arm coordination
- system reliability
- reproducible experimentation
- and research benchmarking

---

# Future Improvements

Planned technical improvements include:

- [ ] more accurate drivetrain calibration
- [ ] improved wheel-slip handling
- [ ] stronger sensor fusion
- [ ] improved crop-row datasets
- [ ] learned crop-row segmentation
- [ ] optimized edge inference
- [ ] improved end-of-row maneuvers
- [ ] more robust obstacle avoidance
- [ ] Nav2 integration and evaluation
- [ ] improved simulation-to-real consistency
- [ ] better hardware diagnostics
- [ ] improved arm manipulation
- [ ] structured experiment logging
- [ ] automated performance benchmarking
- [ ] reproducible research configurations
- [ ] quantitative closed-loop navigation evaluation

---

# Tech Stack

### Robotics

- ROS 2
- ROS 2 Actions
- rclpy
- robot_localization / EKF
- SLAM
- Gazebo
- RViz
- differential-drive control
- odometry

### Computer Vision

- OpenCV
- NumPy
- cv_bridge
- HSV segmentation
- contour processing
- line fitting
- camera-based navigation

### Embedded Robotics

- micro-ROS
- wheel encoders
- motor drivers
- PID velocity control
- microcontroller firmware

### Software

- Python
- C++
- Linux
- ROS launch
- YAML configuration
- Flask / communication bridges

---

# Running the Workspace

The repository is structured as a ROS 2 workspace containing multiple packages.

Clone it into a workspace source directory:

```bash
mkdir -p ~/agribot_ws/src
cd ~/agribot_ws/src

git clone https://github.com/SushantNiraula/Agribot_v0.git
```

Return to the workspace:

```bash
cd ~/agribot_ws
```

Install ROS dependencies where applicable:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build:

```bash
colcon build --symlink-install
```

Source:

```bash
source install/setup.bash
```

---

# Simulation

The repository contains a dedicated Gazebo bringup path.

After building and sourcing the workspace, simulation development centers around the launch files inside:

```text
agribot_bringup/launch/
```

including the Gazebo-specific launch configuration.

---

# Hardware

Real-robot operation uses a separate hardware bringup path.

This separation is intentional:

```text
Simulation
     │
same ROS architecture
     │
Hardware
```

Low-level firmware for the drive base is in [`firmware/pico2/`](firmware/pico2/README.md) and the Raspberry Pi arm scripts are in [`hardware/arm_pi/`](hardware/arm_pi/README.md). Copy `.env.example` to `.env` before running the Flask bridge.

The exact hardware configuration is still being refined as the original physical Agribot system is reconstructed into a cleaner and more reproducible software architecture.

---

# Repository Status

> 🚧 **Active Development / Research**

This repository represents both:

1. the continuing development and reconstruction of my undergraduate **Agribot major project**, and
2. the experimental foundation for research derived from observations made while developing the robot.

The project should therefore not yet be treated as a polished production robotics framework.

It is an evolving engineering and research workspace.

---

# 👨‍💻 Author

**Sushant Niraula**

Electronics, Communication and Information Engineering

Interests:

- Robotics
- Autonomous Systems
- Embedded Systems
- ROS 2
- Computer Vision
- Edge AI
- Robot Perception
- Control Systems
- Artificial Intelligence

GitHub: [SushantNiraula](https://github.com/SushantNiraula)

---

# 🔬 Research Extension — From Major Project to Research Platform

Agribot is now also being used as the **base repository for my ongoing research paper**.

The full Agribot system is the engineering origin and experimental platform for the research, rather than the complete robot itself being presented as the research contribution.

The research direction focuses more narrowly on a question that emerged naturally while developing the robot:

> **How do different perception models and inference configurations affect the actual closed-loop navigation performance of an agricultural mobile robot?**

Instead of evaluating computer-vision models only through conventional metrics such as accuracy, IoU, precision, or recall, I am interested in examining their behavior when the predictions directly influence robot motion.

The experimental idea can be summarized as:

```text
                     SAME AGRIBOT
                         │
                         ↓
                  SAME NAVIGATION TASK
                         │
             ┌───────────┼───────────┐
             │           │           │
             ↓           ↓           ↓
         Method A     Model B     Model C
             │           │           │
             ↓           ↓           ↓
          Inference   Inference   Inference
             │           │           │
             └───────────┼───────────┘
                         ↓
                 Steering Commands
                         ↓
                 Closed-Loop Robot
                         ↓
              Navigation Performance
```

The comparison is intended to consider both **model characteristics** and **deployment/inference characteristics**, including factors such as:

```text
Perception Accuracy
        ↕
Inference Latency
        ↕
Update Frequency / FPS
        ↕
Compute Requirement
        ↕
Model Size
        ↕
Deployment Precision
        ↕
Closed-Loop Navigation Quality
```

This creates a more robotics-centered evaluation than simply asking:

```text
Which model is the most accurate?
```

The more useful question becomes:

```text
Which perception + inference configuration
produces the best behavior for the robot
under realistic edge-computing constraints?
```

Possible experiments can therefore compare different perception approaches or model families while also evaluating different inference configurations such as optimized and quantized execution.

For example, the research framework can study trade-offs between:

```text
heavier / potentially more accurate model
                 VS
lighter / faster perception system
```

and between inference configurations such as:

```text
FP32
FP16
INT8
```

or different execution backends where appropriate.

The main idea is to connect:

\[
\text{Perception Model}
\]

with:

\[
\text{Inference Performance}
\]

with:

\[
\text{Control Loop Performance}
\]

and finally:

\[
\text{Robot Navigation Performance}
\]

rather than evaluating each component independently.

Agribot therefore now serves two related purposes:

```text
                     AGRIBOT
                        │
        ┌───────────────┴───────────────┐
        │                               │
        ↓                               ↓
 Major Engineering Project       Research Test Platform
        │                               │
 Complete Robot System          Controlled Experiments
        │                               │
 Hardware + ROS + AI        Models + Inference + Navigation
        │                               │
        └───────────────┬───────────────┘
                        ↓
                Robotics Research
```

The original major project provided the real engineering problem.

The current research uses that experience to ask a narrower and measurable scientific question about the interaction between **perception, inference latency, computational constraints, and closed-loop autonomous navigation**.

---

# License

The code in this repository is released under the Apache License 2.0. See [`LICENSE`](LICENSE). Check the licence of any third-party model or dataset separately (see the note on `crop_row_nav/models/`).
