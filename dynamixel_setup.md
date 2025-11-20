# ALOHA with Dynamixel SDK - System Documentation

**System:** Mobile ALOHA with Gemini Robotics ER API
**Stack:** Direct Dynamixel SDK + Python (No ROS2)
**Date:** 2025-11-20

This document describes the vision-guided Mobile ALOHA system using Google's Gemini Robotics ER API with direct Dynamixel SDK motor control (no ROS2).

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Architecture](#architecture)
3. [How It Works](#how-it-works)
4. [Components](#components)
5. [Installation](#installation)
6. [Running the System](#running-the-system)
7. [Configuration](#configuration)
8. [Troubleshooting](#troubleshooting)

---

## System Overview

This system enables vision-guided robot control using Google's Gemini Robotics ER (Embodied Reasoning) API. The robot executes manipulation tasks by analyzing camera images, planning actions step-by-step, and verifying results visually after each movement.

### Key Features

- **Vision-Guided Control:** Gemini ER analyzes camera images and plans manipulation tasks
- **Iterative Execution:** Execute one action at a time with visual verification between steps
- **Dual Camera System:** Gripper camera for grasp verification + overhead camera for spatial planning
- **Direct Motor Control:** Sub-10ms latency via Dynamixel SDK (no ROS2)
- **Safety Systems:** Workspace limits, joint constraints, collision avoidance
- **Robot Ceremonies:** Smooth connect/disconnect workflows with sleep positions

### Performance Metrics

| Metric | Performance |
|--------|-------------|
| **Startup Time** | ~1 second |
| **Command Latency** | 5-15ms |
| **State Monitoring** | 10Hz |
| **Dependencies** | 7 packages (~60MB) |
| **Installation Steps** | 4 commands |

---

## Architecture

### System Flow

```
┌─────────────────────────────────────────────────────────────────┐
│         Google Gemini Robotics ER API (google-genai SDK)        │
│  • Model: gemini-robotics-er-1.5-preview                        │
│  • Multimodal input: Text prompts + Camera images               │
│  • Iterative execution: One action → Visual verification        │
│  • Conversation state maintained across steps                   │
└───────────────────────────────┬─────────────────────────────────┘
                                │ REST API (JSON)
┌───────────────────────────────▼─────────────────────────────────┐
│    Python ER Bridge (er-setup/bridge_simple_er.py, Port 8082)   │
│  • aiohttp HTTP server with CORS                                │
│  • Endpoint: POST /robotics-er-request                          │
│  • Captures camera images automatically                         │
│  • Sends images + robot state to Gemini ER                      │
│  • Executes ONE function at a time                              │
│  • Returns execution result + NEW images for verification       │
│  • Manages conversation state across steps                      │
└─────────┬────────────────┬──────────────────────────┬───────────┘
          │                │                          │
┌─────────▼────────┐ ┌────▼───────────┐ ┌───────────▼──────────┐
│ ArmController    │ │ GripperControl │ │ CameraController     │
│                  │ │                │ │                      │
│ • Move joints    │ │ • Open/close   │ │ • RealSense D405    │
│ • Move cartesian │ │ • State track  │ │ • RGBD capture      │
│ • Trajectories   │ │ • 300mA limit  │ │ • Base64 JPEG       │
│ • Safety checks  │ │ • Grasp verify │ │ • Dual cameras      │
│ • IK via MR      │ │                │ │   - gripper_cam     │
│ • Ceremonies     │ │                │ │   - top_cam         │
└─────────┬────────┘ └────┬───────────┘ └──────────────────────┘
          │                │
┌─────────▼────────────────▼─────────────────────┐
│   Modern Robotics IK/FK (Python library)       │
│ • modern_robotics.IKinSpace()                  │
│ • VX300S model (Slist, M matrix)               │
│ • Pure NumPy, no external dependencies         │
└─────────┬──────────────────────────────────────┘
          │
┌─────────▼──────────────────────────────────────┐
│   DynamixelController (Python)                 │
│ • PortHandler - Serial port management         │
│ • PacketHandler - Protocol 2.0 packets         │
│ • Motor initialization from config             │
│ • Sync read/write operations                   │
│ • Shadow motor coordination (2↔3, 4↔5)         │
│ • State monitoring thread (10Hz)               │
└─────────┬──────────────────────────────────────┘
          │ dynamixel_sdk Python package
┌─────────▼──────────────────────────────────────┐
│   Dynamixel SDK (Python bindings)              │
│ • Serial I/O via PortHandler                   │
│ • Protocol 2.0 packet encoding                 │
│ • GroupSyncRead/Write for efficiency          │
└─────────┬──────────────────────────────────────┘
          │ Serial USB
┌─────────▼──────────────────────────────────────┐
│   U2D2 USB Adapter                             │
│ • TTL Half-Duplex conversion                   │
│ • Device: /dev/ttyDXL_follower_right           │
│ • Baud: 1Mbps                                  │
└─────────┬──────────────────────────────────────┘
          │
┌─────────▼──────────────────────────────────────┐
│   Dynamixel Motors (VX300S Hardware)           │
│ • 9 motors (6 arm + 2 shadow + 1 gripper)      │
│ • Protocol 2.0, 12-bit resolution              │
│ • Position control with internal PID           │
└────────────────────────────────────────────────┘
```

---

## How It Works

### Iterative Execution Workflow

The system operates in a step-by-step loop with visual verification:

**Step 1: User submits task**
```
"Pick up the red cube"
```

**Step 2: Bridge captures current state**
- Takes photos from both cameras (gripper_cam + top_cam)
- Reads current joint positions and end-effector position
- Packages as context for Gemini ER

**Step 3: Gemini ER analyzes and plans**
- Receives: Task description + 2 camera images + robot state
- Analyzes: Object locations, robot position, workspace
- Returns: ONE function call (e.g., `move_arm` to approach position)

**Step 4: Bridge executes action**
- Calls the robot function (e.g., `arm_controller.move_to_position()`)
- Waits for completion
- Captures NEW camera images showing result

**Step 5: Visual verification loop**
- Sends execution result + NEW images back to Gemini ER
- Gemini verifies: "Did the robot reach the target?"
- If yes → Returns next action (e.g., `control_gripper("close")`)
- If no → Adjusts and retries

**Step 6: Repeat until task complete**
- Each action gets visual verification
- Critical steps (like grasping) are verified in gripper camera
- Task completes when Gemini ER returns `task_complete: true`

### Example Task Execution

```
User: "Pick up the red cube"

Step 1: Gemini → move_arm([0.3, 0.1, 0.3])  # Approach cube
        Bridge → Executes, captures images
        Gemini → Verifies position in new images

Step 2: Gemini → move_arm([0.3, 0.1, 0.05])  # Descend to grasp height
        Bridge → Executes, captures images
        Gemini → Confirms alignment

Step 3: Gemini → control_gripper("open")  # Prepare to grasp
        Bridge → Executes, captures images
        Gemini → Sees gripper is open

Step 4: Gemini → control_gripper("close")  # Attempt grasp
        Bridge → Executes, captures images
        Gemini → CHECKS GRIPPER CAMERA - "Cube is secured in gripper!"

Step 5: Gemini → move_arm([0.3, 0.1, 0.25])  # Lift object
        Bridge → Executes, captures images
        Gemini → Confirms lift successful

... continues until task complete
```

### Available Robot Functions

The bridge exposes these functions to Gemini ER:

```python
def move_arm(position: list[float] = None, pose: str = None, moving_time: float = 1.5):
    """Move robot end effector to target position or named pose.

    Args:
        position: Target [x, y, z] in meters (relative to robot base)
        pose: Named pose - "home", "ready", or "sleep"
        moving_time: Time to complete movement in seconds
    """

def control_gripper(action: str):
    """Open or close the robot gripper.

    Args:
        action: "open" or "close"
    """

def get_arm_status():
    """Get current arm state (joints, position, pose)."""

def get_gripper_status():
    """Get current gripper state."""

def capture_camera_frame(reason: str):
    """Capture fresh camera frames.

    Args:
        reason: Why frames are needed (e.g., "verify_grasp")
    """
```

---

## Components

### Active Components (Connected and In Use)

**1. ER Bridge** - `er-setup/bridge_simple_er.py`
- HTTP REST server on port 8082
- Integrates with Gemini Robotics ER API (google-genai SDK)
- Handles iterative execution with visual verification
- Manages conversation state across multiple steps
- Performs robot connect/disconnect ceremonies

**2. DynamixelController** - `dynamixel_controller.py`
- Direct motor control via Dynamixel SDK
- Sync read/write operations for efficiency
- Shadow motor coordination (motors 2↔3, 4↔5)
- State monitoring thread (10Hz)
- Register access for configuration

**3. VX300S Model** - `models/vx300s_model.py`
- Robot kinematics (Product of Exponentials)
- Screw axes (Slist) and home configuration (M matrix)
- Joint limits and workspace bounds
- Conversion helpers (radians ↔ Dynamixel units)

**4. ArmController** - `controllers/arm_controller.py`
- Joint movement with safety checks
- Cartesian movement with IK (Modern Robotics)
- Named poses (home, ready, sleep)
- Trajectory execution
- Opening/closing ceremonies

**5. GripperController** - `controllers/gripper_controller.py`
- Open/close operations
- Current limiting (300mA for safe grasping)
- State tracking
- Position monitoring

**6. CameraController** - `controllers/camera_controller.py`
- RealSense D405 camera interface
- RGBD capture (color + depth)
- Base64 JPEG encoding for API transmission
- Dual camera support:
  - `gripper_cam` - Mounted on end-effector (grasp verification)
  - `top_cam` - Overhead view (spatial planning)

### Inactive Components (Not Connected to Current Bridge)

**Vision System** - `vision/` directory
*Purpose:* Advanced object detection and 3D localization
*Status:* Developed but not integrated into ER bridge

Components:
- `vision/gemini_vision_detector.py` - Object detection via Gemini Vision API
- `vision/depth_projector.py` - 2D pixel → 3D coordinates conversion
- `vision/camera_calibration.py` - Camera-to-robot transformations
- `vision/object_localizer.py` - Combined detection + 3D localization
- `controllers/vision_controller.py` - High-level vision API

**Why not connected:** The current ER bridge relies on Gemini ER API to perform all vision reasoning directly. The vision modules could be integrated in the future for more precise localization.

---

## Installation

### Prerequisites

- Ubuntu 20.04+ or macOS
- Python 3.8+
- U2D2 USB adapter connected to `/dev/ttyDXL_follower_right`
- VX300S robot hardware (right arm)
- 2x Intel RealSense D405 cameras
- Google API key for Gemini

### Step 1: Install Python Dependencies

```bash
# Install Dynamixel SDK
pip3 install dynamixel-sdk

# Install kinematics library
pip3 install modern-robotics

# Install camera SDK
pip3 install pyrealsense2

# Install Gemini SDK
pip3 install google-genai

# Install web server dependencies
pip3 install aiohttp aiohttp-cors

# Install other dependencies
pip3 install numpy pyyaml python-dotenv
```

### Step 2: Set Up Serial Port Permissions

```bash
# Add your user to dialout group
sudo usermod -aG dialout $USER

# Log out and log back in for changes to take effect

# Verify device exists
ls -l /dev/ttyDXL_follower_right
```

### Step 3: Clone Repository

```bash
git clone <your-repo-url>
cd gemini-live-dynamix

# Config directory should already exist with vx300s.yaml
```

### Step 4: Configure API Key

```bash
# Create .env file in project root
echo "GEMINI_API_KEY=your_api_key_here" > .env

# Or set environment variable
export GEMINI_API_KEY=your_api_key_here
```

### Step 5: Verify Installation

```bash
# Test Dynamixel connection
python3 -c "
from dynamixel_controller import DynamixelController
dxl = DynamixelController('/dev/ttyDXL_follower_right', 1000000, 'config/vx300s.yaml')
dxl.initialize_motors()
positions = dxl.sync_read_positions()
print('Motor positions:', positions)
print('Motor count:', len(positions))
dxl.close()
"
```

**Expected output:**
```
Motor positions: {1: 2048, 2: 1854, 3: 1854, 4: 2100, 5: 2100, 6: 2048, 7: 2048, 8: 2048, 9: 2500}
Motor count: 9
```

---

## Running the System

### Start ER Bridge

```bash
cd /home/aloha/gemini-live-dynamix
python3 er-setup/bridge_simple_er.py
```

**Command-line options:**
```bash
python3 er-setup/bridge_simple_er.py --port 8082 \
                                      --dxl-port /dev/ttyDXL_follower_right \
                                      --baudrate 1000000
```

**Expected output:**
```
================================================================
Gemini Robotics ER Bridge - Direct Dynamixel Control
================================================================

Features:
  - Direct hardware control via Dynamixel SDK
  - Automatic camera capture (gripper_cam + top_cam)
  - Iterative visual verification
  - Conversation state maintained

============================================================
[ER Bridge] Initializing Robot Hardware
============================================================

[1/5] Loading robot model...
  ✓ VX300S model loaded (6-DOF, 0.75m reach)

[2/5] Connecting to Dynamixel bus at /dev/ttyDXL_follower_right...
  ✓ Dynamixel controller connected, torque enabled, monitoring at 10Hz

[3/5] Initializing arm controller...
  ✓ Arm controller ready (awaiting connection for opening ceremony)

[4/5] Initializing gripper controller...
  ✓ Gripper controller ready (awaiting connection)

[5/5] Initializing camera controller...
  ✓ Camera controller ready (gripper_cam + top_cam)

============================================================
[ER Bridge] ✓ Robot hardware initialized!
[ER Bridge] ℹ️  Use /robot/connect to perform opening ceremony
============================================================

Starting server on http://localhost:8082
Endpoints:
  - POST /robotics-er-request
  - GET /status
  - POST /robot/connect
  - POST /robot/disconnect
  - GET  /robot/status
```

### Using the System

**1. Connect Robot (Opening Ceremony)**

This slowly moves the arm to ready position and opens the gripper:

```bash
curl -X POST http://localhost:8082/robot/connect
```

**2. Submit Task**

```bash
curl -X POST http://localhost:8082/robotics-er-request \
  -H "Content-Type: application/json" \
  -d '{
    "prompt": "Pick up the red cube and move it to the blue cube"
  }'
```

**Response format:**
```json
{
  "success": true,
  "conversation_id": "uuid-string",
  "step": 1,
  "reasoning": "I see a red cube at position [0.3, 0.1]. Moving arm to approach...",
  "next_action": {
    "function": "move_arm",
    "args": {"position": [0.3, 0.1, 0.3]}
  },
  "execution_result": {
    "success": true,
    "function": "move_arm",
    "message": "Moved to position [0.300, 0.100, 0.300]"
  },
  "images": ["base64_gripper_cam", "base64_top_cam"],
  "verification_check": "After execution, gripper should be 30cm above cube",
  "task_complete": false
}
```

**3. Continue Conversation**

The frontend automatically sends feedback requests:

```bash
curl -X POST http://localhost:8082/robotics-er-request \
  -H "Content-Type: application/json" \
  -d '{
    "conversation_id": "uuid-from-previous-response",
    "execution_result": {...}
  }'
```

Fresh images are captured automatically and sent with each feedback request.

**4. Disconnect Robot (Closing Ceremony)**

This closes the gripper and slowly moves to sleep position:

```bash
curl -X POST http://localhost:8082/robot/disconnect
```

**5. Monitor Status**

```bash
# Bridge status
curl http://localhost:8082/status

# Robot connection state
curl http://localhost:8082/robot/status

# Individual camera frame
curl http://localhost:8082/camera/gripper_cam/frame
```

---

## Configuration

### Motor Configuration (`config/vx300s.yaml`)

```yaml
motors:
  1:  # Waist
    ID: 1
    Model: "XM430-W350"
    Drive_Mode: 0  # Normal
    Min_Position: 0
    Max_Position: 4095
    Velocity_Limit: 131

  2:  # Shoulder (Primary)
    ID: 2
    Model: "XM540-W270"
    Drive_Mode: 1  # REVERSED
    Min_Position: 841
    Max_Position: 2867
    Velocity_Limit: 131
    Secondary_ID: 3  # Shadow motor

  3:  # Shoulder Shadow
    ID: 3
    Model: "XM540-W270"
    Drive_Mode: 0  # Normal
    Min_Position: 841
    Max_Position: 2867
    Velocity_Limit: 131
    Secondary_ID: 2  # Mirrors motor 2

  4:  # Elbow (Primary)
    ID: 4
    Model: "XM540-W270"
    Drive_Mode: 1  # REVERSED
    Min_Position: 898
    Max_Position: 3094
    Velocity_Limit: 131
    Secondary_ID: 5  # Shadow motor

  5:  # Elbow Shadow
    ID: 5
    Model: "XM540-W270"
    Drive_Mode: 0  # Normal
    Min_Position: 898
    Max_Position: 3094
    Velocity_Limit: 131
    Secondary_ID: 4  # Mirrors motor 4

  6:  # Forearm Roll
    ID: 6
    Model: "XM430-W350"
    Drive_Mode: 0
    Min_Position: 0
    Max_Position: 4095
    Velocity_Limit: 131

  7:  # Wrist Angle
    ID: 7
    Model: "XM430-W350"
    Drive_Mode: 1  # REVERSED
    Min_Position: 830
    Max_Position: 3504
    Velocity_Limit: 131

  8:  # Wrist Rotate
    ID: 8
    Model: "XM430-W350"
    Drive_Mode: 0
    Min_Position: 0
    Max_Position: 4095
    Velocity_Limit: 131

  9:  # Gripper
    ID: 9
    Model: "XM430-W350"
    Drive_Mode: 0
    Min_Position: 0
    Max_Position: 4095
    Velocity_Limit: 131

groups:
  arm:
    motor_ids: [1, 2, 4, 6, 7, 8]  # Primary motors only

grippers:
  gripper:
    motor: 9
    horn_radius: 0.022  # meters
    arm_length: 0.036   # meters

poses:
  home: [0.0, -0.3, 0.6, 0.0, -0.3, 0.0]
  sleep: [0.0, -1.85, 1.55, 0.0, 0.8, 0.0]
  ready: [0.0, -0.96, 1.16, 0.0, -0.3, 0.0]
```

### Critical Configuration Notes

**Shadow Motor Coordination:**
- Motors 2 ↔ 3 (shoulder) MUST move together
- Motors 4 ↔ 5 (elbow) MUST move together
- `DynamixelController.sync_write_positions()` handles this automatically
- Never command shadow motors independently

**Drive Mode:**
- `0` = Normal direction
- `1` = Reversed direction (motors 2, 4, 7)
- Ensures consistent joint angle directions

**Position Limits:**
- Prevent mechanical damage
- Enforced by DynamixelController on every write
- Values in Dynamixel units (0-4095 for 12-bit resolution)

---

## Troubleshooting

### Issue: "Failed to open port /dev/ttyDXL_follower_right"

**Solution:**
```bash
# Check if device exists
ls -l /dev/ttyDXL*
ls -l /dev/ttyUSB*

# Check permissions
groups  # Should include 'dialout'

# If not in dialout group:
sudo usermod -aG dialout $USER
# Log out and back in

# Try alternative device name
python3 er-setup/bridge_simple_er.py --dxl-port /dev/ttyUSB0
```

### Issue: "Motor not responding" or "Communication error"

**Solution:**
```bash
# Check motor power supply
# Verify U2D2 LED is on
# Check USB connection

# Test with single motor
python3 -c "
from dynamixel_controller import DynamixelController
dxl = DynamixelController('/dev/ttyDXL_follower_right', 1000000, 'config/vx300s.yaml')
pos = dxl.read_register(1, 132, 4)  # Read position of motor 1
print('Motor 1 position:', pos)
"
```

### Issue: "IK solution not found"

**Causes:**
- Target position out of reach
- Target violates joint limits
- Singularity in robot configuration

**Solution:**
```python
# Check workspace limits
workspace = {
    'x': (0.15, 0.50),
    'y': (-0.30, 0.30),
    'z': (0.05, 0.40)
}

# Try position closer to current pose
# Verify robot model (Slist, M) is correct
```

### Issue: "Shadow motors out of sync"

**Solution:**
```python
# Read positions
dxl.sync_read_positions()
positions = dxl.current_positions
print(f"Motor 2: {positions[2]}, Motor 3: {positions[3]}")
print(f"Motor 4: {positions[4]}, Motor 5: {positions[5]}")

# If difference > 100 units, manually sync:
dxl.sync_write_positions({2: positions[2], 3: positions[2]})
dxl.sync_write_positions({4: positions[4], 5: positions[4]})
```

### Issue: "Camera not detected"

**Solution:**
```bash
# List RealSense devices
rs-enumerate-devices

# Check permissions
sudo usermod -aG video $USER

# Update camera serial numbers in camera_controller.py
```

### Issue: "GEMINI_API_KEY not set"

**Solution:**
```bash
# Check if .env exists
cat .env

# Create if missing
echo "GEMINI_API_KEY=your_key_here" > .env

# Or set environment variable
export GEMINI_API_KEY=your_key_here
```

---

## Safety Considerations

### Workspace Limits

Position limits relative to robot base (meters):
- **X:** 0.15 to 0.50 (forward/backward)
- **Y:** -0.30 to 0.30 (left/right)
- **Z:** 0.05 to 0.40 (up/down)

### Joint Limits (radians)

1. **Waist:** [-π, π] (-180° to 180°)
2. **Shoulder:** [-1.97, 1.75] (-113° to 100°)
3. **Elbow:** [-1.52, 1.80] (-87° to 103°)
4. **Forearm Roll:** [-π, π] (-180° to 180°)
5. **Wrist Angle:** [-1.74, 2.23] (-100° to 128°)
6. **Wrist Rotate:** [-π, π] (-180° to 180°)

### Safety Rules

1. **Always check limits** before commanding motion
2. **Shadow motors must sync** (2↔3, 4↔5)
3. **Gripper current limited** to 300mA
4. **Emergency stop** disables all torque immediately
5. **Validate IK solutions** before execution
6. **Use ceremonies** for connect/disconnect (smooth movements)

---

## Resources

- [Dynamixel SDK Documentation](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [Modern Robotics Library](https://github.com/NxRLab/ModernRobotics)
- [Dynamixel Protocol 2.0](https://emanual.robotis.com/docs/en/dxl/protocol2/)
- [XM430 Motor Manual](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)
- [XM540 Motor Manual](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/)
- [Gemini Robotics ER API](https://ai.google.dev/gemini-api/docs/robotics-overview)
- [google-genai Python SDK](https://googleapis.github.io/python-genai/)

---

**Last Updated:** 2025-11-20
**Active Bridge:** `er-setup/bridge_simple_er.py` on port 8082
**Architecture:** Direct Dynamixel SDK (No ROS2)
