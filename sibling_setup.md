# Current ALOHA System Documentation (ROS2-Based)

**This Document Location:** `/home/aloha/gemini-live-dynamix/sibling_setup.md`
**Sibling Repository (ROS2 System):** `/home/aloha/gemini-live/`
**System:** Mobile ALOHA with Gemini 2.5 Flash Live API
**Stack:** ROS2 Humble + Interbotix SDK
**Date:** 2025-01-13

This document describes the **current working ROS2-based system** in the sibling repository (`/home/aloha/gemini-live/`), which uses ROS2 Humble and the Interbotix SDK for robot control.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Current Architecture](#current-architecture)
3. [Repository Structure](#repository-structure)
4. [Dependencies](#dependencies)
5. [Installation](#installation)
6. [Running the System](#running-the-system)
7. [How It Works](#how-it-works)
8. [Configuration Files](#configuration-files)
9. [Troubleshooting](#troubleshooting)
10. [Future Migration](#future-migration)

---

## System Overview

This system enables voice control of a Mobile ALOHA robot through Google's Gemini 2.5 Flash Live API. The robot control is implemented using **ROS2 Humble** and the **Interbotix SDK**, which provides a tested and validated interface to the Dynamixel motors.

### Key Components

- **Gemini 2.5 Flash Live API:** Real-time voice conversation and tool calling
- **React Frontend:** Web-based UI for interaction
- **Python Bridge:** HTTP server that routes tool calls to robot
- **ROS2 + Interbotix:** Motor control middleware
- **VX300S Robot:** 6-DOF arm with gripper
- **Dual Cameras:** RealSense D405 cameras for visual feedback

### Performance Metrics

| Metric | Current System (ROS2) |
|--------|-----------------------|
| **Startup Time** | ~5 seconds |
| **Command Latency** | 20-40ms |
| **Running Processes** | ~10 |
| **Dependencies** | ROS2 Humble + Interbotix (~2GB) |

---

## Current Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│              Google Gemini 2.5 Flash Live API                   │
│  • Real-time voice conversation                                 │
│  • Tool calling for robot control                               │
│  • Visual scene understanding (camera feeds)                    │
│  • Native audio (30 HD voices, 24 languages)                    │
└───────────────────────────────┬─────────────────────────────────┘
                                │ WebSocket
┌───────────────────────────────▼─────────────────────────────────┐
│           React Frontend (TypeScript, Port 3000)                │
│  • ALOHAControl.tsx - UI and tool definitions                   │
│  • genai-live-client.ts - WebSocket to Gemini                   │
│  • audio-recorder.ts - Microphone capture                       │
│  • Camera frame merging                                         │
└───────────────────────────────┬─────────────────────────────────┘
                                │ HTTP/WebSocket (localhost)
┌───────────────────────────────▼─────────────────────────────────┐
│        Python Bridge (bridge_aloha_real.py, Port 8081)          │
│  • aiohttp HTTP server                                          │
│  • Receives tool calls from Gemini                              │
│  • Launches minimal_launch.sh subprocess                        │
│  • Routes to ArmController / GripperController                  │
└─────────┬────────────────┬──────────────────────────┬───────────┘
          │                │                          │
┌─────────▼────────┐ ┌────▼───────────┐ ┌───────────▼──────────┐
│ ArmController    │ │ GripperControl │ │ CameraController     │
│                  │ │                │ │                      │
│ • Move joints    │ │ • Open/close   │ │ • RealSense D405    │
│ • Move cartesian │ │ • State track  │ │ • pyrealsense2 SDK  │
│ • Trajectory     │ │ • 300mA limit  │ │ • Independent       │
│ • Safety checks  │ │ • 10Hz monitor │ │                      │
└─────────┬────────┘ └────┬───────────┘ └─────────────────────┘
          │                │
┌─────────▼────────────────▼─────────────────────┐
│   InterbotixManipulatorXS (Python wrapper)     │
│ • High-level robot interface                   │
│ • InterbotixArmXSInterface                     │
│ • InterbotixGripperXSInterface                 │
│ • Modern Robotics IK/FK                        │
└─────────┬──────────────────────────────────────┘
          │
┌─────────▼──────────────────────────────────────┐
│   InterbotixRobotXSCore (ROS2 Client Layer)    │
│ • rclpy.Node (Python ROS2 node)                │
│ • Service clients (torque, registers, etc.)    │
│ • Publishers (joint commands)                  │
│ • Subscribers (joint states)                   │
└─────────┬──────────────────────────────────────┘
          │ ROS2 Topics/Services
┌─────────▼──────────────────────────────────────┐
│   xs_sdk Node (C++, interbotix_xs_sdk pkg)     │
│ • InterbotixDriverXS                           │
│ • DynamixelWorkbench library                   │
│ • Sync Read/Write operations                   │
└─────────┬──────────────────────────────────────┘
          │ Serial USB
┌─────────▼──────────────────────────────────────┐
│   Dynamixel Motors (VX300S Hardware)           │
│ • 9 motors (6 arm + 2 shadow + 1 gripper)      │
└────────────────────────────────────────────────┘
```

---

## Repository Structure

```
/home/aloha/gemini-live/
├── gemini-live-api-control/
│   ├── bridges/
│   │   └── bridge_aloha_real.py          # Main bridge (ROS2 version)
│   ├── live-api-console/                 # React frontend
│   │   ├── src/
│   │   │   ├── components/
│   │   │   │   ├── aloha-control/
│   │   │   │   │   └── ALOHAControl.tsx  # Robot control UI
│   │   │   │   └── control-tray/
│   │   │   │       └── ControlTray.tsx   # Connection UI
│   │   │   └── lib/
│   │   │       ├── genai-live-client.ts  # Gemini WebSocket
│   │   │       └── audio-recorder.ts     # Audio capture
│   │   ├── package.json
│   │   └── .env                          # Gemini API key
│   └── run_bridge.sh                     # Launch script
├── arm_controller.py                     # Arm control (ROS2 version)
├── gripper_controller.py                 # Gripper control (ROS2 version)
├── camera_controller.py                  # Camera control (independent)
├── minimal_launch.sh                     # Launch ROS2 driver
├── minimal_arm_control.py                # Direct arm control test
├── robot_utils.py                        # Trajectory utilities
└── safety_validator.py                   # Safety checking

/home/aloha/interbotix_ws/
├── src/
│   ├── interbotix_ros_core/              # Core ROS2 packages
│   ├── interbotix_ros_manipulators/      # Arm-specific packages
│   ├── interbotix_ros_toolboxes/         # Python modules
│   └── aloha/                            # ALOHA-specific code
├── build/                                # Colcon build output
└── install/                              # Installed packages
```

---

## Dependencies

### System Level

- **Ubuntu 22.04** (Jammy)
- **ROS2 Humble**
- **Python 3.10**
- **Node.js 16+** and npm

### ROS2 Packages

Installed in `/home/aloha/interbotix_ws/`:
- `interbotix_ros_core` - Core driver packages
- `interbotix_ros_manipulators` - Arm-specific packages
- `interbotix_ros_toolboxes` - Python modules
- `interbotix_xs_sdk` - C++ SDK node
- `interbotix_xs_driver` - C++ driver with DynamixelWorkbench
- `dynamixel_workbench_toolbox` - Motor control library
- `aloha` - Custom ALOHA utilities

### Python Packages

```bash
pip3 list | grep -E "(rclpy|interbotix|modern|numpy|aiohttp|pyrealsense)"
```

- `rclpy` - ROS2 Python client
- `modern_robotics` - Kinematics library
- `numpy` - Numerical computation
- `interbotix_xs_modules` - Interbotix Python API
- `aiohttp` - Web server
- `aiohttp-cors` - CORS support
- `pyrealsense2` - RealSense cameras

---

## Installation

### Step 1: Install ROS2 Humble

```bash
# Add ROS2 apt repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Build Interbotix Workspace

```bash
# Create workspace
mkdir -p ~/interbotix_ws/src
cd ~/interbotix_ws/src

# Clone Interbotix repositories
git clone https://github.com/Interbotix/interbotix_ros_core.git
git clone https://github.com/Interbotix/interbotix_ros_manipulators.git
git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git

# Install dependencies
cd ~/interbotix_ws
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install

# Source workspace
echo "source ~/interbotix_ws/install/setup.bash" >> ~/.bashrc
source ~/interbotix_ws/install/setup.bash
```

### Step 3: Configure udev Rules

```bash
# Copy udev rules for USB permissions
sudo cp ~/interbotix_ws/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/99-interbotix-udev.rules /etc/udev/rules.d/

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group
sudo usermod -aG dialout $USER

# Log out and back in for changes to take effect
```

### Step 4: Install Python Dependencies

```bash
# Install Interbotix modules
pip3 install interbotix_xs_modules

# Install other dependencies
pip3 install modern_robotics numpy
pip3 install aiohttp aiohttp-cors
pip3 install pyrealsense2
```

### Step 5: Set Up Frontend

```bash
cd /home/aloha/gemini-live/gemini-live-api-control/live-api-console

# Install npm dependencies
npm install

# Create .env file with Gemini API key
echo "REACT_APP_GEMINI_API_KEY=your_api_key_here" > .env
```

---

## Running the System

### Terminal 1: Start Robot Bridge

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash

# Navigate to project
cd /home/aloha/gemini-live/gemini-live-api-control

# Run bridge (launches ROS2 driver automatically)
./run_bridge.sh
```

**Alternative (manual):**
```bash
# Terminal 1a: Launch ROS2 driver
cd /home/aloha/gemini-live
./minimal_launch.sh

# Terminal 1b: Run bridge
cd /home/aloha/gemini-live/gemini-live-api-control
python3 bridges/bridge_aloha_real.py
```

**Expected Output:**
```
[Bridge] Starting robot driver...
[Bridge] Robot driver launched (PID: 12345)
[Bridge] Initializing gripper controller...
[Bridge] ✓ Gripper controller initialized successfully
[Bridge] Initializing arm controller (sharing robot interface)...
[Bridge] ✓ Arm controller initialized successfully
[Bridge] Initializing camera controller...
[Bridge] ✓ Camera controller initialized successfully

Starting bridge server on http://localhost:8081
```

### Terminal 2: Start React Frontend

```bash
cd /home/aloha/gemini-live/gemini-live-api-control/live-api-console
npm start
```

**Opens browser at:** `http://localhost:3000`

### Using the System

1. **Connect to Gemini:**
   - Click "Connect" button in UI
   - Allow microphone access

2. **Test Voice Commands:**
   - "Move the arm to home position"
   - "Open the gripper"
   - "Move to position x=0.3, y=0, z=0.2"

3. **Monitor Status:**
   - Check terminal output
   - Use: `curl http://localhost:8081/status`

---

## How It Works

### Control Flow Example

**User says:** "Move the arm to position x=0.3, y=0.1, z=0.2"

```
1. Browser captures audio → sends to Gemini Live API
2. Gemini transcribes + reasons → decides to call move_arm tool
3. React frontend receives tool call → POSTs to http://localhost:8081/aloha-tool-call
4. Python bridge receives request → calls arm_controller.move_to_position()
5. ArmController validates workspace → calls InterbotixManipulatorXS
6. InterbotixManipulatorXS runs IK → publishes to ROS2 topic
7. xs_sdk C++ node receives trajectory → writes to Dynamixel motors
8. Motors execute movement → publish state back via ROS2
9. ArmController monitors state → updates internal tracking
10. Bridge returns success → Gemini receives empty response
```

### ROS2 Topics Used

```bash
# View active topics
ros2 topic list

# Key topics:
/follower_left/joint_states              # Published by xs_sdk
/follower_left/commands/joint_group      # Subscribed by xs_sdk
/follower_left/commands/joint_single     # For gripper
/follower_left/commands/joint_trajectory # For smooth motion
```

### ROS2 Services Used

```bash
# View available services
ros2 service list

# Key services:
/follower_left/torque_enable             # Enable/disable motors
/follower_left/set_operating_modes       # Configure control modes
/follower_left/get_motor_registers       # Read motor parameters
/follower_left/set_motor_registers       # Write motor parameters
```

---

## Configuration Files

### Motor Configuration

**Location:** `/home/aloha/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/config/vx300s.yaml`

Contains:
- Motor IDs and models
- Position limits
- Velocity limits
- Drive modes (normal/reversed)
- Shadow motor configuration
- Sleep positions

### Operating Modes

**Location:** `/home/aloha/interbotix_ws/src/aloha/config/follower_modes_left.yaml`

Contains:
- Operating modes (position, velocity, current-based)
- Profile settings
- Torque enable settings

### Kinematics

**Location:** `/home/aloha/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/mr_descriptions.py`

Contains:
- Screw axes (Slist)
- Home configuration (M matrix)
- For IK/FK computation

---

## Troubleshooting

### Issue: "ROS2 node not found"

**Solution:**
```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash

# Verify ROS2 daemon is running
ros2 daemon status

# Restart if needed
ros2 daemon stop
ros2 daemon start
```

### Issue: "Cannot find /dev/ttyDXL"

**Solution:**
```bash
# Check available devices
ls -l /dev/ttyUSB*
ls -l /dev/ttyDXL*

# Verify permissions
groups  # Should include 'dialout'

# If not, add user to dialout
sudo usermod -aG dialout $USER
# Log out and back in
```

### Issue: "xs_sdk node failed to launch"

**Solution:**
```bash
# Test manual launch
cd ~/interbotix_ws
source install/setup.bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300s robot_name:=follower_left

# Check for errors in output
```

### Issue: "Gripper not moving"

**Solution:**
```bash
# Check gripper configuration
ros2 service call /follower_left/get_motor_registers interbotix_xs_msgs/srv/RegisterValues "{cmd_type: 'group', name: 'gripper', reg: 'Operating_Mode'}"

# Should return 5 (current-based position mode)

# Manually test gripper
python3 -c "
from gripper_controller import GripperController
gc = GripperController()
gc.initialize()
gc.open_gripper()
"
```

### Issue: "Camera not detected"

**Solution:**
```bash
# List RealSense devices
rs-enumerate-devices

# Expected output shows 2 cameras:
# Device 0: D405 (serial: 130322273632)
# Device 1: D405 (serial: 130322273629)

# If not found, check USB connections
# Try different USB ports
```

---

## Future Migration

The sibling repository (`/home/aloha/gemini-live/`) uses **ROS2 Humble + Interbotix SDK** for robot control. While this provides a tested and validated solution, there is an alternative approach using **direct Dynamixel SDK control** that eliminates the ROS2 dependency.

This new repository (`/home/aloha/gemini-live-dynamix/`) is being created to implement that alternative approach.

### Why Consider Migration?

**Benefits of Direct Dynamixel SDK:**
- ✅ Simpler installation (3 pip packages vs ROS2 + workspace)
- ✅ Faster startup (~1s vs ~5s)
- ✅ Lower latency (5-15ms vs 20-40ms)
- ✅ Easier deployment (no environment sourcing)
- ✅ Single Python process vs multiple processes

**Tradeoffs:**
- ❌ Custom motor control code to write (~1500 lines)
- ❌ More testing burden
- ❌ Loss of Interbotix support and validated configs
- ❌ Loss of ROS2 ecosystem tools (RViz, rosbag)

### Migration Path

For detailed instructions on implementing a Dynamixel SDK-based system, see:

**[`dynamixel_setup.md`](dynamixel_setup.md)** (in this repository: `/home/aloha/gemini-live-dynamix/`)

That document provides:
- Complete system architecture (with Gemini + React)
- What stays the same (frontend, cameras, bridge structure)
- What changes (controllers, initialization)
- Full implementation of DynamixelController
- Step-by-step migration guide

The migration preserves all high-level functionality (voice control, Gemini integration, safety checks) while replacing only the ROS2 motor control layer.

---

## Summary

The sibling repository (`/home/aloha/gemini-live/`) contains a **fully functional** voice-controlled robot system using:
- Gemini 2.5 Flash Live API for natural language control
- React frontend for user interaction
- Python bridge for tool call routing
- **ROS2 + Interbotix** for reliable motor control
- Dual cameras for visual feedback

The system is production-ready and uses battle-tested Interbotix configurations. For an alternative approach without ROS2, see [`dynamixel_setup.md`](dynamixel_setup.md) in this repository (`/home/aloha/gemini-live-dynamix/`).

---

**Sibling Repository (ROS2 System):** `/home/aloha/gemini-live/`
**This Repository (Dynamixel SDK):** `/home/aloha/gemini-live-dynamix/`
**Documentation Date:** 2025-01-13
**System Type:** ROS2-based (sibling), Dynamixel SDK-based (this repo)
**Status:** Production-ready (sibling), In Development (this repo)
