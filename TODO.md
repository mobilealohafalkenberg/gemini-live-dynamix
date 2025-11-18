# Gemini Live Dynamixel Bridge - Implementation TODO

**Project:** Voice-Controlled ALOHA Mobile Robot via Gemini 2.5 Flash
**Architecture:** Direct Dynamixel SDK Control (No ROS2)
**Status:** In Development

---

## Phase 1: Core Infrastructure

### DynamixelController Implementation
- [ ] Create `dynamixel_controller.py` in root directory
- [ ] Implement PortHandler and PacketHandler initialization
- [ ] Implement motor initialization from config file
- [ ] Implement sync read operations (GroupSyncRead)
- [ ] Implement sync write operations (GroupSyncWrite)
- [ ] Implement shadow motor coordination (motors 2”3, 4”5)
- [ ] Implement position conversion methods (radians ” Dynamixel units)
- [ ] Implement state monitoring thread (10Hz)
- [ ] Implement torque enable/disable methods
- [ ] Implement emergency stop functionality
- [ ] Implement cleanup and port close methods

### VX300S Kinematics Model
- [ ] Create `models/` directory
- [ ] Create `models/vx300s_model.py`
- [ ] Define Slist (screw axes in space frame)
- [ ] Define M matrix (home configuration)
- [ ] Define joint limits for all 6 joints
- [ ] Define workspace limits (X, Y, Z bounds)
- [ ] Add named poses (home, sleep, ready)

### Motor Configuration
- [ ] Create `config/vx300s.yaml`
- [ ] Configure all 9 motors (IDs, models, drive modes)
- [ ] Set position limits for each motor
- [ ] Set velocity limits for each motor
- [ ] Configure shadow motor mappings (2”3, 4”5)
- [ ] Define motor groups (arm, gripper)
- [ ] Define sleep/home positions

---

## Phase 2: Controller Refactoring

### ArmController Refactoring
- [ ] Update imports (remove ROS2, add DynamixelController and Modern Robotics)
- [ ] Refactor `__init__()` to accept DynamixelController and VX300S model
- [ ] Refactor `initialize()` to use DynamixelController instead of ROS2
- [ ] Refactor `move_joints()` to use DynamixelController.set_joint_positions_radians()
- [ ] Refactor `move_to_position()` to use Modern Robotics IK directly
- [ ] Implement `_build_transformation_matrix()` helper for IK
- [ ] Refactor `move_to_pose()` to use DynamixelController
- [ ] Refactor `execute_trajectory()` to use DynamixelController
- [ ] Update `get_arm_state()` to use DynamixelController.get_joint_positions_radians()
- [ ] Verify safety checks and workspace limits remain intact
- [ ] Verify non-blocking execution support (blocking=False parameter)

### GripperController Refactoring
- [ ] Update imports (remove ROS2, add DynamixelController)
- [ ] Refactor `__init__()` to accept DynamixelController
- [ ] Refactor `initialize()` to set current limit (300mA) via DynamixelController
- [ ] Refactor `open_gripper()` to use DynamixelController.sync_write_positions()
- [ ] Refactor `close_gripper()` to use DynamixelController.sync_write_positions()
- [ ] Refactor `set_gripper_position()` to use DynamixelController
- [ ] Implement `_radians_to_dynamixel()` conversion helper
- [ ] Update `get_gripper_state()` to use DynamixelController

### CameraController Verification
- [ ] Verify camera_controller.py has no ROS2 dependencies
- [ ] Verify it uses pyrealsense2 directly
- [ ] Test camera initialization independently
- [ ] Verify thread-based capture works

---

## Phase 3: Bridge Integration

### Bridge Initialization Update
- [ ] Update `updated_bridge_aloha.py` imports
- [ ] Remove ROS2 subprocess launch code
- [ ] Add DynamixelController initialization in `initialize_robot()`
- [ ] Add VX300S model initialization
- [ ] Share DynamixelController instance with arm and gripper controllers
- [ ] Update gripper controller initialization
- [ ] Update arm controller initialization
- [ ] Add proper error handling and logging
- [ ] Verify fire-and-forget response pattern remains intact

### Tool Call Routing Verification
- [ ] Verify `control_gripper` tool routing works
- [ ] Verify `move_arm` tool routing works (position, joints, pose modes)
- [ ] Verify `move_arm_trajectory` tool routing works
- [ ] Verify `get_robot_state` tool routing works
- [ ] Verify camera endpoints work (`/camera/gripper`, `/camera/top`)
- [ ] Verify `/status` endpoint works

---

## Phase 4: Deployment & Setup

### Python Dependencies
- [ ] Create `requirements.txt` with all dependencies
  - dynamixel-sdk
  - modern-robotics
  - pyrealsense2
  - aiohttp
  - aiohttp-cors
  - numpy
  - pyyaml

### Serial Port Setup
- [ ] Document serial port permissions (dialout group)
- [ ] Create udev rules if needed for /dev/ttyDXL
- [ ] Verify U2D2 connection and baud rate (1Mbps)

### Frontend Setup
- [ ] Verify `live-api-console/.env` has REACT_APP_GEMINI_API_KEY
- [ ] Verify all npm dependencies are installed
- [ ] Verify tool definitions match bridge endpoints

### System Verification
- [ ] Test DynamixelController standalone (read positions)
- [ ] Test arm movement (joint and Cartesian)
- [ ] Test gripper open/close
- [ ] Test camera capture
- [ ] Test bridge server startup
- [ ] Test frontend connection to Gemini
- [ ] Test end-to-end voice command execution

---

## Critical Implementation Notes

### Shadow Motor Synchronization
- **MUST** always command motors 2 and 3 together (shoulder)
- **MUST** always command motors 4 and 5 together (elbow)
- Motor 3 mirrors motor 2, motor 5 mirrors motor 4
- Failure to sync causes mechanical stress and damage

### Safety Constraints
- Workspace limits: X[0.15-0.50m], Y[-0.30-0.30m], Z[0.05-0.40m]
- Joint limits must be enforced before every movement
- IK solutions must be validated before execution
- Emergency stop must disable all torque immediately

### Fire-and-Forget Pattern
- Bridge must return immediately, not wait for robot movement
- Use `blocking=False` in all controller methods called from tool handlers
- Only status checks should block

---

<!--
## Testing (To Be Implemented After Core Components)

### Unit Tests
- [ ] test_dynamixel_controller.py - Motor operations and sync
- [ ] test_arm_controller.py - Joint/Cartesian movement, IK
- [ ] test_gripper_controller.py - Open/close operations
- [ ] test_shadow_sync.py - Verify motors 2”3 and 4”5 stay synchronized

### Integration Tests
- [ ] test_bridge_endpoints.py - HTTP API functionality
- [ ] test_tool_routing.py - Gemini tool call handling
- [ ] test_full_stack.py - Frontend ’ Bridge ’ Hardware

### Safety Tests
- [ ] Workspace limit enforcement
- [ ] Joint limit enforcement
- [ ] Emergency stop response time
- [ ] IK failure handling

### Performance Tests
- [ ] Command latency (target: <10ms)
- [ ] State monitoring frequency (target: 10Hz)
- [ ] Startup time (target: <2s)
- [ ] Trajectory smoothness
-->

---

**Last Updated:** 2025-01-13
**Repository:** `/home/aloha/gemini-live-dynamix/`
