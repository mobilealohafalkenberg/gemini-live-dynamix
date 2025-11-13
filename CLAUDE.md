# Gemini Live Dynamixel Bridge

**Project:** Voice-Controlled ALOHA Mobile Robot via Gemini 2.5 Flash
**Architecture:** Direct Dynamixel SDK Control (No ROS2)
**Status:** In Development
**Date:** 2025-01-13

---

## Project Overview

This project implements a voice-controlled Mobile ALOHA robot system using Google's Gemini 2.5 Flash Live API. The key innovation is **direct Dynamixel SDK motor control**, eliminating the ROS2 middleware layer for simpler deployment, lower latency, and minimal dependencies.

### Core Principle

**Simplicity with Robustness:** Every component does exactly what it needs to do, nothing more. The system provides essential features for optimal functionality while maintaining a minimal, maintainable codebase.

### What This Project Does

1. **Voice Control:** Natural language commands processed by Gemini 2.5 Flash
2. **Direct Hardware Control:** Python controllers communicate directly with Dynamixel motors via the Dynamixel SDK
3. **Tool Call Bridge:** HTTP server routes Gemini tool calls to robot controllers
4. **Visual Feedback:** Dual RealSense cameras provide visual context to Gemini

### What This Project Does NOT Do

- No ROS2 (this is the whole point)
- No complex middleware layers
- No unnecessary abstraction
- No features beyond core manipulation and voice control

---

## AI Assistant Guidelines

**IMPORTANT RULES FOR AI ASSISTANTS WORKING ON THIS PROJECT:**

### File Creation Policy

**DO NOT create markdown (.md) or text files unless explicitly instructed by the user.**

This includes but is not limited to:
- Documentation files (README.md, DOCS.md, etc.)
- Notes or planning files
- Log files
- Configuration documentation
- Tutorial files
- Any other .md or .txt files

**Exceptions:**
- The user explicitly asks you to create a specific file
- Python source files (.py), YAML configs (.yaml), JSON files (.json), and other code files are allowed when needed for implementation

**Rationale:** This project values minimal documentation overhead. All necessary documentation is contained in CLAUDE.md, TODO.md, and the reference files (sibling_setup.md, dynamixel_setup.md).

### Code Over Documentation

- Always prefer working code over additional documentation
- Modify existing files rather than creating new ones
- Comments in code are acceptable and encouraged
- Focus on implementation, not meta-documentation

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│              Google Gemini 2.5 Flash Live API                   │
│  • Real-time voice conversation & transcription                 │
│  • Tool calling for robot control                               │
│  • Visual scene understanding (camera feeds)                    │
│  • Native audio (30 HD voices, 24 languages)                    │
└───────────────────────────────┬─────────────────────────────────┘
                                │ WebSocket
┌───────────────────────────────▼─────────────────────────────────┐
│           React Frontend (TypeScript, Port 3000)                │
│  • Microphone capture (16kHz PCM16)                             │
│  • WebSocket client for Gemini Live API                         │
│  • Tool definitions (control_gripper, move_arm, etc.)           │
│  • Camera frame merging and display                             │
└───────────────────────────────┬─────────────────────────────────┘
                                │ HTTP (localhost:8081)
┌───────────────────────────────▼─────────────────────────────────┐
│        Python Bridge (bridge_aloha_real.py, Port 8081)          │
│  • aiohttp HTTP server with CORS                                │
│  • Routes Gemini tool calls to robot controllers                │
│  • Fire-and-forget response pattern                             │
│  • Single Python process                                        │
└─────────┬────────────────┬──────────────────────────┬───────────┘
          │                │                          │
┌─────────▼────────┐ ┌────▼───────────┐ ┌───────────▼──────────┐
│ ArmController    │ │ GripperControl │ │ CameraController     │
│ • Joint control  │ │ • Open/close   │ │ • RealSense D405    │
│ • IK/FK (MR)     │ │ • State track  │ │ • Thread-based      │
│ • Safety checks  │ │ • Current lim  │ │ • pyrealsense2      │
│ • Trajectories   │ │                │ │                      │
└─────────┬────────┘ └────┬───────────┘ └──────────────────────┘
          │                │
┌─────────▼────────────────▼─────────────────────┐
│   DynamixelController (Python)                 │
│  • Direct Dynamixel SDK control                │
│  • Sync read/write operations                  │
│  • Shadow motor coordination                   │
│  • State monitoring (10Hz thread)              │
└─────────┬──────────────────────────────────────┘
          │ Serial USB (/dev/ttyDXL)
┌─────────▼──────────────────────────────────────┐
│   VX300S Hardware (9 Dynamixel Motors)         │
│  • 6 arm joints + 2 shadow + 1 gripper         │
│  • Protocol 2.0, 1Mbps baud                    │
└────────────────────────────────────────────────┘
```

---

## Communication Layers

### 1. Gemini ↔ Frontend Communication
- **Protocol:** WebSocket (Gemini Live API)
- **Audio:** 16kHz PCM16 streaming
- **Video:** 1 FPS JPEG frames (dual camera view)
- **Tool Calls:** JSON-RPC style function calling

### 2. Frontend ↔ Bridge Communication
- **Protocol:** HTTP REST
- **Endpoint:** `POST http://localhost:8081/aloha-tool-call`
- **Pattern:** Fire-and-forget (immediate empty response)
- **Payload:** `{name: string, args: object}`

### 3. Bridge ↔ Hardware Communication
- **Method:** Direct Python function calls
- **Controllers:** ArmController, GripperController, CameraController
- **Backend:** DynamixelController (Dynamixel SDK)
- **Serial:** /dev/ttyDXL at 1Mbps

---

## Components from Sibling Project

### Unchanged (Copy As-Is)

These components are brought from `/home/aloha/gemini-live/` without modification:

1. **React Frontend** (`live-api-console/`)
   - `src/components/aloha-control/ALOHAControl.tsx`
   - `src/lib/genai-live-client.ts`
   - `src/lib/audio-recorder.ts`
   - `src/components/control-tray/ControlTray.tsx`
   - All other frontend files

2. **Camera Controller** (`camera_controller.py`)
   - Already ROS-independent
   - Uses pyrealsense2 directly
   - No changes needed

3. **Utility Modules**
   - `robot_utils.py` (trajectory planning)
   - `safety_validator.py` (workspace/joint limits)

### Modified (Refactor Backend Only)

These keep the same **external API** but change **internal implementation**:

1. **Bridge Server** (`bridges/bridge_aloha_real.py`)
   - **Keep:** HTTP endpoints, tool routing logic, CORS config
   - **Change:** Initialization (no ROS2 launch, use DynamixelController)

2. **Arm Controller** (`arm_controller.py`)
   - **Keep:** Method signatures, safety checks, trajectory logic
   - **Change:** Backend (use DynamixelController + Modern Robotics IK)

3. **Gripper Controller** (`gripper_controller.py`)
   - **Keep:** open/close/set_position API
   - **Change:** Backend (use DynamixelController)

### New Components

These are created from scratch:

1. **DynamixelController** (`dynamixel_controller.py`)
   - Direct motor control via Dynamixel SDK
   - Sync read/write operations
   - Shadow motor coordination
   - State monitoring thread

2. **VX300S Model** (`vx300s_model.py`)
   - Robot kinematics (Slist, M matrix)
   - For Modern Robotics IK/FK
   - Joint limits and workspace

3. **Configuration** (`config/vx300s.yaml`)
   - Motor IDs and models
   - Position/velocity limits
   - Shadow motor mappings
   - Named poses

---

## Implementation Focus

### Priority 1: Communication Between Bridge and Tool Calls

**Objective:** Gemini tool calls must reliably trigger robot actions.

**Key Files:**
- `bridges/bridge_aloha_real.py` - HTTP server
- Frontend tool definitions in `ALOHAControl.tsx`

**Requirements:**
- Tool call routing must be robust
- Fire-and-forget pattern (don't block Gemini)
- Clear error handling and logging
- Status endpoint for monitoring

**Tools to Support:**
```typescript
1. control_gripper(action: "open" | "close")
2. move_arm(position?: [x,y,z], joints?: [...], pose?: "home|sleep|ready")
3. move_arm_trajectory(trajectory: [...], speed: "slow|medium|fast")
4. get_robot_state()
```

### Priority 2: Communication with Hardware

**Objective:** Reliable, low-latency control of Dynamixel motors.

**Key Files:**
- `dynamixel_controller.py` - Core motor interface
- `arm_controller.py` - High-level arm control
- `gripper_controller.py` - Gripper control

**Requirements:**
- Shadow motor synchronization (critical!)
- Sub-10ms command latency
- State monitoring at 10Hz
- Graceful error recovery
- Emergency stop capability

**Critical Implementation Details:**
1. **Shadow Motors:** Motors 2↔3 (shoulder) and 4↔5 (elbow) must move together
2. **Sync Operations:** Use GroupSyncRead/Write for efficiency
3. **Current Limiting:** Gripper limited to 300mA for safe grasping
4. **Position Limits:** Enforce motor-specific min/max values

---

## Key Design Decisions

### 1. Fire-and-Forget Tool Response

**Why:** Gemini Live API requires fast tool responses. Robot movements take seconds.

**Implementation:**
```python
async def handle_tool_call(request):
    data = await request.json()
    # Execute robot command (non-blocking)
    result = arm_controller.move_to_position(..., blocking=False)
    # Return immediately
    return web.json_response({'success': True})
```

### 2. Shared DynamixelController Instance

**Why:** Only one serial connection allowed. All controllers share one instance.

**Implementation:**
```python
# Initialize once in bridge
dxl_controller = DynamixelController(...)
dxl_controller.initialize_motors()

# Share with controllers
arm_controller = ArmController(dynamixel_controller=dxl_controller, ...)
gripper_controller = GripperController(dynamixel_controller=dxl_controller)
```

### 3. Direct IK via Modern Robotics

**Why:** Eliminate ROS2 dependency. Modern Robotics is pure Python/NumPy.

**Implementation:**
```python
import modern_robotics as mr

joint_solution, success = mr.IKinSpace(
    robot_model.Slist,
    robot_model.M,
    T_target,
    current_joints,
    eomg=0.01,
    ev=0.001
)
```

### 4. Simplistic Feature Set

**Why:** Minimize complexity, maximize reliability.

**What We Include:**
- Joint control
- Cartesian control (with IK)
- Named poses (home, sleep, ready)
- Multi-waypoint trajectories
- Gripper open/close
- Safety limits
- Visual feedback

**What We Exclude:**
- Compliance control
- Force sensing
- Advanced trajectory planning
- Self-collision checking
- Task-level planning
- Recording/replay (teleoperation)

---

## File Structure

```
/home/aloha/gemini-live-dynamix/
├── CLAUDE.md                          # This file
├── TODO.md                            # Development tasks
├── sibling_setup.md                   # Reference: ROS2 system docs
├── dynamixel_setup.md                 # Reference: Implementation guide
│
├── config/
│   └── vx300s.yaml                    # Motor configuration
│
├── bridges/
│   └── bridge_aloha_real.py           # HTTP server (Modified)
│
├── controllers/
│   ├── dynamixel_controller.py        # NEW: Direct motor control
│   ├── arm_controller.py              # Modified: Use Dynamixel SDK
│   ├── gripper_controller.py          # Modified: Use Dynamixel SDK
│   └── camera_controller.py           # Unchanged from sibling
│
├── models/
│   └── vx300s_model.py                # NEW: Robot kinematics
│
├── utils/
│   ├── robot_utils.py                 # Unchanged: Trajectory planning
│   └── safety_validator.py            # Unchanged: Safety checks
│
└── live-api-console/                  # Unchanged: React frontend
    ├── src/
    │   ├── components/
    │   │   ├── aloha-control/
    │   │   │   └── ALOHAControl.tsx
    │   │   └── control-tray/
    │   │       └── ControlTray.tsx
    │   └── lib/
    │       ├── genai-live-client.ts
    │       └── audio-recorder.ts
    ├── package.json
    └── .env                           # REACT_APP_GEMINI_API_KEY
```

---

## Development Workflow

### Phase 1: Core Infrastructure
1. Implement `DynamixelController` class
2. Create `vx300s_model.py` with kinematics
3. Set up `config/vx300s.yaml` with motor params
4. Test motor connection and sync read/write

### Phase 2: Controllers
1. Refactor `arm_controller.py` to use DynamixelController
2. Refactor `gripper_controller.py` to use DynamixelController
3. Copy `camera_controller.py` unchanged
4. Test individual controller functionality

### Phase 3: Bridge Integration
1. Modify `bridge_aloha_real.py` initialization
2. Copy frontend from sibling project
3. Test tool call routing
4. Test full integration with Gemini

### Phase 4: Testing & Validation
1. Unit tests for each controller
2. Integration tests for tool calls
3. Safety limit testing
4. Latency benchmarking
5. End-to-end voice control testing

---

## Dependencies

### Python Packages (5 total)
```bash
pip3 install dynamixel-sdk      # Dynamixel motor control
pip3 install modern-robotics    # IK/FK computation
pip3 install pyrealsense2       # RealSense cameras
pip3 install aiohttp aiohttp-cors  # HTTP server
pip3 install numpy pyyaml       # Utilities
```

### System Requirements
- Python 3.8+
- USB serial port (/dev/ttyDXL or /dev/ttyUSB0)
- Permissions: user in `dialout` group
- Node.js 16+ and npm (for frontend)

### Hardware
- VX300S robot arm (Trossen Robotics ALOHA Mobile)
- U2D2 USB adapter
- 2x Intel RealSense D405 cameras

---

## Performance Targets

| Metric | Target | Current ROS2 |
|--------|--------|--------------|
| **Startup Time** | < 2 seconds | ~5 seconds |
| **Command Latency** | < 10ms | 20-40ms |
| **State Monitoring** | 10Hz | 10Hz |
| **Running Processes** | 1 (+ 2 threads) | ~10 |
| **Dependencies** | 5 packages (~50MB) | ROS2 + workspace (~2GB) |

---

## Safety Considerations

### Workspace Limits
- X: 0.15m to 0.50m
- Y: -0.30m to 0.30m
- Z: 0.05m to 0.40m

### Joint Limits (radians)
1. Waist: [-π, π]
2. Shoulder: [-1.97, 1.75]
3. Elbow: [-1.52, 1.80]
4. Forearm Roll: [-π, π]
5. Wrist Angle: [-1.74, 2.23]
6. Wrist Rotate: [-π, π]

### Critical Safety Rules
1. **Always check limits** before commanding motion
2. **Shadow motors must sync** (2↔3, 4↔5)
3. **Gripper current limited** to 300mA
4. **Emergency stop** disables all torque immediately
5. **Validate IK solutions** before execution

---

## Common Pitfalls to Avoid

### 1. Shadow Motor Desync
**Problem:** Commanding motor 2 without motor 3 causes mechanical stress.
**Solution:** Always write to both shadow motors together in sync_write_positions().

### 2. Blocking Tool Calls
**Problem:** Waiting for robot movement blocks Gemini conversation.
**Solution:** Use `blocking=False` in controllers, return immediately.

### 3. Serial Port Contention
**Problem:** Multiple DynamixelController instances fight for port.
**Solution:** Initialize once, share instance across all controllers.

### 4. IK Failures
**Problem:** Target position out of reach, IK doesn't converge.
**Solution:** Check workspace limits first, handle IK failure gracefully.

### 5. Missing Camera Permissions
**Problem:** pyrealsense2 can't access cameras.
**Solution:** Add user to `video` group: `sudo usermod -aG video $USER`

---

## Testing Strategy

### Unit Tests
- `test_dynamixel_controller.py` - Motor operations
- `test_arm_controller.py` - Arm movements and IK
- `test_gripper_controller.py` - Gripper operations
- `test_camera_controller.py` - Camera capture

### Integration Tests
- `test_bridge_endpoints.py` - HTTP API
- `test_tool_routing.py` - Tool call handling
- `test_shadow_sync.py` - Shadow motor coordination

### Hardware Tests
- Joint movement accuracy
- Cartesian positioning accuracy
- Trajectory smoothness
- Emergency stop response time
- Camera frame rate and quality

### Safety Tests
- Workspace limit enforcement
- Joint limit enforcement
- Collision detection (if implemented)
- Emergency stop reliability

---

## Troubleshooting

### Port Not Found
```bash
ls -l /dev/ttyDXL /dev/ttyUSB*
sudo usermod -aG dialout $USER
# Log out and back in
```

### Motor Communication Error
```bash
# Check power supply
# Verify U2D2 connection
# Test with Dynamixel Wizard
# Check baudrate (should be 1000000)
```

### Shadow Motors Out of Sync
```python
# Check current positions
positions = dxl_controller.sync_read_positions()
print(f"Motor 2: {positions[2]}, Motor 3: {positions[3]}")

# Manually sync if needed
dxl_controller.sync_write_positions({2: target, 3: target})
```

### IK Solution Not Found
```python
# Target may be out of reach
# Try reducing distance from current position
# Check orientation constraints
# Verify robot model (Slist, M) is correct
```

---

## Reference Documentation

### Internal Docs
- `sibling_setup.md` - Complete ROS2 system documentation (reference)
- `dynamixel_setup.md` - Detailed implementation guide (reference)
- `TODO.md` - Current development tasks

### External Resources
- [Dynamixel SDK Manual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [Modern Robotics Library](https://github.com/NxRLab/ModernRobotics)
- [Dynamixel Protocol 2.0](https://emanual.robotis.com/docs/en/dxl/protocol2/)
- [XM430 Motor Manual](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)
- [XM540 Motor Manual](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/)
- [Gemini 2.5 Flash Live API (Vertex AI)](https://docs.cloud.google.com/vertex-ai/generative-ai/docs/models/gemini/2-5-flash-live-api)
- [Gemini Live API Guide](https://ai.google.dev/gemini-api/docs/live)

---

## Project Philosophy

### Simplicity
- Minimal dependencies
- No unnecessary abstraction
- Direct control paths
- Clear, readable code

### Robustness
- Comprehensive error handling
- Safety checks at every level
- Graceful degradation
- Clear error messages

### Maintainability
- Well-documented code
- Consistent naming conventions
- Modular architecture
- Comprehensive testing

### Performance
- Low latency (< 10ms commands)
- Efficient sync operations
- Minimal memory footprint
- Fast startup (< 2s)

---

## Success Criteria

A successful implementation will:

1. Start in < 2 seconds (vs 5s for ROS2)
2. Command latency < 10ms (vs 20-40ms)
3. Reliable shadow motor synchronization
4. Smooth trajectory execution
5. Robust tool call handling
6. Clear error messages and recovery
7. Safe operation within all limits
8. Natural voice control via Gemini

---

**Last Updated:** 2025-01-13
**Repository:** `/home/aloha/gemini-live-dynamix/`
**Sibling (ROS2):** `/home/aloha/gemini-live/`
**Status:** Ready for Implementation
