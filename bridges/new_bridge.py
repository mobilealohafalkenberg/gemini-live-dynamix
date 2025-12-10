import os
import sys
import json
import asyncio
import base64
import time
import logging
from typing import List, Dict, Any, Optional
from pathlib import Path

from aiohttp import web, WSMsgType
from aiohttp_cors import setup, ResourceOptions
from dotenv import load_dotenv

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

# Google GenAI SDK (v1.0+)
from google import genai
from google.genai import types

# --- HARDWARE IMPORTS (Keep your existing setup) ---
try:
    from dynamixel_controller import DynamixelController
    from models.vx300s_model import VX300S
    from controllers.arm_controller import ArmController
    from controllers.gripper_controller import GripperController
    from controllers.camera_controller import CameraController
    ROBOT_HARDWARE_AVAILABLE = True
except ImportError:
    ROBOT_HARDWARE_AVAILABLE = False

# Load .env file from project root
env_path = Path(__file__).parent.parent / '.env'
load_dotenv(dotenv_path=env_path)

# Configure logging
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

# --- STATE ---
arm_controllers: Dict[str, Dict] = {}  # arm_id -> {dxl, arm, gripper, port, model}
camera_controller: Optional[Any] = None
robot_connected: bool = False  # True after opening ceremony
ws_clients: set = set()  # Connected WebSocket clients
ws_sequence: int = 0  # Message sequence number

# --- WEBSOCKET HELPERS ---

async def send_ws(ws: web.WebSocketResponse, msg_type: str, payload: dict):
    """Send a typed message to a single WebSocket client."""
    global ws_sequence
    ws_sequence += 1
    try:
        await ws.send_json({
            'type': msg_type,
            'timestamp': int(time.time() * 1000),
            'sequence': ws_sequence,
            'payload': payload
        })
    except Exception as e:
        logging.error(f"WebSocket send error: {e}")
        ws_clients.discard(ws)


async def broadcast_ws(msg_type: str, payload: dict):
    """Broadcast a message to all connected WebSocket clients."""
    global ws_sequence
    if not ws_clients:
        return

    ws_sequence += 1
    message = {
        'type': msg_type,
        'timestamp': int(time.time() * 1000),
        'sequence': ws_sequence,
        'payload': payload
    }

    disconnected = set()
    for ws in ws_clients:
        try:
            await ws.send_json(message)
        except Exception as e:
            logging.error(f"WebSocket broadcast error: {e}")
            disconnected.add(ws)

    ws_clients.difference_update(disconnected)


# --- HARDWARE INITIALIZATION ---

def initialize_hardware() -> bool:
    """
    Initialize robot hardware - detect and set up all follower arms and cameras.

    Returns:
        True if at least one arm was successfully initialized
    """
    global arm_controllers, camera_controller

    if not ROBOT_HARDWARE_AVAILABLE:
        logging.warning("Robot hardware modules not available - running in mock mode")
        return False

    print("\n" + "=" * 60)
    print("[Bridge] Initializing Robot Hardware")
    print("=" * 60)

    # Detect follower arms
    detected_arms = DynamixelController.detect_follower_ports()
    print(f"\n[Detection] Found {len(detected_arms)} follower arm(s)")

    if not detected_arms:
        logging.warning("No follower arms detected. Check USB connections and udev rules.")
        return False

    # Initialize each detected arm
    config_path = Path(__file__).parent.parent / 'config' / 'vx300s.yaml'
    arm_controllers = {}

    for arm_info in detected_arms:
        port = arm_info['port']
        arm_id = arm_info['arm_id']
        print(f"\n[{arm_id}] Initializing on {port}...")

        try:
            # Create robot model
            robot_model = VX300S()

            # Initialize DynamixelController
            dxl_controller = DynamixelController(
                port=port,
                baudrate=1000000,
                config_file=str(config_path)
            )

            if not dxl_controller.initialize_motors():
                logging.error(f"Failed to initialize motors for {arm_id}")
                continue

            dxl_controller.enable_torque()
            dxl_controller.start_monitoring(frequency=10)

            # Initialize ArmController
            arm_ctrl = ArmController(
                dynamixel_controller=dxl_controller,
                robot_model=robot_model,
                enable_safety=True,
                dry_run=False
            )

            if not arm_ctrl.initialize_without_movement():
                logging.error(f"Failed to initialize arm controller for {arm_id}")
                continue

            # Initialize GripperController with arm_id for per-arm calibration
            gripper_ctrl = GripperController(
                dynamixel_controller=dxl_controller,
                arm_id=arm_id,
                dry_run=False
            )

            if not gripper_ctrl.initialize():
                logging.error(f"Failed to initialize gripper controller for {arm_id}")
                continue

            # Store controller stack
            arm_controllers[arm_id] = {
                'dxl': dxl_controller,
                'arm': arm_ctrl,
                'gripper': gripper_ctrl,
                'port': port,
                'model': robot_model
            }

            print(f"  ✓ {arm_id} fully initialized")

        except Exception as e:
            logging.error(f"Failed to initialize {arm_id}: {e}")
            import traceback
            traceback.print_exc()

    # Initialize cameras
    print(f"\n[Cameras] Initializing...")
    try:
        camera_controller = CameraController()
        if camera_controller.initialize():
            print(f"  ✓ Cameras: {camera_controller.get_camera_names()}")
        else:
            logging.warning("Camera initialization failed")
    except Exception as e:
        logging.error(f"Camera initialization error: {e}")

    print("\n" + "=" * 60)
    print(f"[Bridge] ✓ Initialized {len(arm_controllers)} arm(s)")
    print(f"[Bridge] Connected: {list(arm_controllers.keys())}")
    print("=" * 60 + "\n")

    return len(arm_controllers) > 0


def shutdown_hardware():
    """Safely shutdown all robot controllers."""
    global arm_controllers, camera_controller

    print("\n[Bridge] Shutting down robot...")

    ALL_MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9]

    for arm_id, ctrl in arm_controllers.items():
        try:
            print(f"  [{arm_id}] Disabling torque...")
            ctrl['dxl'].disable_torque(ALL_MOTOR_IDS)
            ctrl['dxl'].stop_monitoring()
            ctrl['dxl'].close()
        except Exception as e:
            logging.error(f"Error shutting down {arm_id}: {e}")

    arm_controllers = {}
    print("[Bridge] ✓ Shutdown complete")


# --- HARDWARE FUNCTIONS ---

def capture_encoded_images() -> Dict[str, str]:
    """Captures images and returns Dict[camera_name, base64_string]"""
    if not camera_controller or not camera_controller.initialized:
        return {}
    return camera_controller.get_all_frames_base64()


def get_camera_description(cam_name: str) -> str:
    """Get human-readable description of camera view for Gemini context."""
    descriptions = {
        'overhead_camera': 'Top-down view of workspace',
        'right_gripper': 'View from right arm gripper, looking forward',
        'left_gripper': 'View from left arm gripper, looking forward',
    }
    return descriptions.get(cam_name, f'Camera view: {cam_name}')


# --- TOOLS DEFINITIONS ---
# SDK-native FunctionDeclarations with proper Schema definitions

def build_tool_declarations(connected_arms: List[str]) -> types.Tool:
    """
    Build SDK-native tool declarations with proper Schema definitions.

    Uses types.FunctionDeclaration which the SDK converts to the
    correct API format automatically.
    """
    arm_enum_desc = f"Arm ID. Must be one of: {', '.join(connected_arms)}"

    return types.Tool(
        function_declarations=[
            # move_arm - Cartesian position control
            types.FunctionDeclaration(
                name="move_arm",
                description="Move robot arm end-effector to a 3D Cartesian position. Use this for precise positioning.",
                    parameters=types.Schema(
                    type=types.Type.OBJECT,
                    properties={
                        "arm_id": types.Schema(
                            type=types.Type.STRING,
                            description=arm_enum_desc
                        ),
                        "position": types.Schema(
                            type=types.Type.ARRAY,
                            items=types.Schema(type=types.Type.NUMBER),
                            description="Target [x, y, z] position in meters. Robot base is origin. +X=forward, +Y=left, +Z=up."
                        ),
                        "orientation": types.Schema(
                            type=types.Type.ARRAY,
                            items=types.Schema(type=types.Type.NUMBER),
                            description="Optional [roll, pitch, yaw] in radians. Overrides pitch_degrees if provided."
                        ),
                        "pitch_degrees": types.Schema(
                            type=types.Type.NUMBER,
                            description="Gripper pitch in degrees. Negative = down (e.g., -45 for steep down, -15 for nearly horizontal). Default is -30°. Common values: -60 (very steep), -45 (steep), -30 (moderate), -15 (gentle), 0 (horizontal)."
                        ),
                        "moving_time": types.Schema(
                            type=types.Type.NUMBER,
                            description="Movement duration in seconds (default: 1.5)"
                        )
                    },
                    required=["arm_id", "position"]
                )
            ),

            # control_gripper - Position-based gripper control
            types.FunctionDeclaration(
                name="control_gripper",
                description="Control gripper position. Use 0.0 for fully closed, 1.0 for fully open, or intermediate values for partial grip.",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    properties={
                        "arm_id": types.Schema(
                            type=types.Type.STRING,
                            description=arm_enum_desc
                        ),
                        "position": types.Schema(
                            type=types.Type.NUMBER,
                            description="Gripper position: 0.0 (closed) to 1.0 (open). Use ~0.3 for holding small objects."
                        )
                    },
                    required=["arm_id", "position"]
                )
            ),

            # finish_task - Signal completion
            types.FunctionDeclaration(
                name="finish_task",
                description="Call this when the task is complete or impossible to continue.",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    properties={
                        "success": types.Schema(
                            type=types.Type.BOOLEAN,
                            description="Whether the task was completed successfully"
                        ),
                        "summary": types.Schema(
                            type=types.Type.STRING,
                            description="Brief summary of what was accomplished or why task failed"
                        )
                    },
                    required=["success", "summary"]
                )
            ),

            # resume_after_stop - Recovery from error state
            types.FunctionDeclaration(
                name="resume_after_stop",
                description="Resume arm operations after an error. Call this when you see 'System in ERROR state' to clear the error and retry commands.",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    properties={
                        "arm_id": types.Schema(
                            type=types.Type.STRING,
                            description=arm_enum_desc
                        )
                    },
                    required=["arm_id"]
                )
            ),

            # get_arm_state - Query current arm position
            types.FunctionDeclaration(
                name="get_arm_state",
                description="Get the current position and state of an arm. Returns end-effector position (x,y,z), joint angles, and arm state. Use this to check current position before planning movements.",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    properties={
                        "arm_id": types.Schema(
                            type=types.Type.STRING,
                            description=arm_enum_desc
                        )
                    },
                    required=["arm_id"]
                )
            ),

            # execute_trajectory - Multi-waypoint manipulation sequences with checkpoints
            types.FunctionDeclaration(
                name="execute_trajectory",
                description="Execute a multi-waypoint trajectory with gripper coordination and visual checkpoints. Set checkpoint=true on waypoints where you need to see camera images before continuing. At checkpoints, execution stops and returns current position - review images and call execute_trajectory again with next waypoints.",
                parameters=types.Schema(
                    type=types.Type.OBJECT,
                    properties={
                        "arm_id": types.Schema(
                            type=types.Type.STRING,
                            description=arm_enum_desc
                        ),
                        "waypoints": types.Schema(
                            type=types.Type.ARRAY,
                            items=types.Schema(
                                type=types.Type.OBJECT,
                                properties={
                                    "point": types.Schema(
                                        type=types.Type.ARRAY,
                                        items=types.Schema(type=types.Type.NUMBER),
                                        description="Target [x, y, z] position in meters"
                                    ),
                                    "label": types.Schema(
                                        type=types.Type.STRING,
                                        description="Descriptive name (e.g., 'approach', 'pre-grasp', 'grasp', 'lift')"
                                    ),
                                    "gripper_position": types.Schema(
                                        type=types.Type.NUMBER,
                                        description="Gripper position: 0.0 (closed) to 1.0 (open). Use ~0.3 for grasping, ~0.8 for approach."
                                    ),
                                    "checkpoint": types.Schema(
                                        type=types.Type.BOOLEAN,
                                        description="If true, stop at this waypoint and return camera images. Review images, then call execute_trajectory again with remaining waypoints."
                                    )
                                },
                                required=["point", "label"]
                            ),
                            description="Ordered list of waypoints. Set checkpoint=true before critical actions (e.g., before grasping) to verify position."
                        ),
                    },
                    required=["arm_id", "waypoints"]
                )
            )
        ]
    )


# --- TOOL EXECUTION FUNCTIONS ---

def execute_move_arm(arm_id: str, position: List[float], orientation: Optional[List[float]] = None, pitch_degrees: Optional[float] = None, moving_time: float = 1.5) -> Dict[str, Any]:
    """Execute move_arm tool.

    Args:
        arm_id: Which arm (follower_right, follower_left)
        position: [x, y, z] in meters
        orientation: Optional [roll, pitch, yaw] in radians. If None, uses pitch_degrees.
        pitch_degrees: Gripper pitch in degrees (negative = down). Default is -30°.
        moving_time: Movement duration in seconds
    """
    if orientation:
        print(f"[Robot] Moving {arm_id} to {position} with orientation {orientation}")
    elif pitch_degrees is not None:
        print(f"[Robot] Moving {arm_id} to {position} (pitch: {pitch_degrees}°)")
    else:
        print(f"[Robot] Moving {arm_id} to {position} (default pitch: -30°)")

    if arm_id not in arm_controllers:
        return {
            "status": "error",
            "error": f"Arm '{arm_id}' not found",
            "valid_arms": list(arm_controllers.keys()),
            "recovery": "Use a valid arm_id from valid_arms"
        }

    try:
        arm = arm_controllers[arm_id]['arm']

        # Get starting position
        start_state = arm.get_arm_state()
        start_ee = start_state.get('ee_position', {})
        start_position = [start_ee.get('x', 0), start_ee.get('y', 0), start_ee.get('z', 0)]

        result = arm.move_to_position(position, orientation=orientation, pitch_degrees=pitch_degrees, moving_time=moving_time, blocking=True)
        if result.get('success'):
            # Get final position
            final_state = arm.get_arm_state()
            final_ee = final_state.get('ee_position', {})
            final_position = [final_ee.get('x', 0), final_ee.get('y', 0), final_ee.get('z', 0)]

            response = {
                "status": "success",
                "start_position": start_position,
                "final_position": final_position,
                "target_position": position,
                "message": "Movement complete. Verify alignment in camera images."
            }

            # Include orientation feedback if relaxation occurred
            if result.get('orientation_relaxed'):
                import math
                requested_pitch_deg = math.degrees(result.get('requested_pitch', 0))
                achieved_pitch_deg = math.degrees(result.get('achieved_pitch', 0))
                response['orientation_note'] = (
                    f"Note: Requested pitch ({requested_pitch_deg:.0f}°) was not achievable. "
                    f"Used pitch={achieved_pitch_deg:.0f}° instead."
                )
                response['achieved_orientation'] = result.get('achieved_orientation')

            return response
        else:
            return {
                "status": "error",
                "error": result.get('error', 'Move failed'),
                "recovery": "Check position is within workspace (X: 0.15-0.50m, Y: -0.30-0.30m, Z: 0.02-0.40m)"
            }
    except Exception as e:
        return {
            "status": "error",
            "error": str(e),
            "recovery": "Try resume_after_stop if arm is in error state"
        }


def execute_control_gripper(arm_id: str, position: float) -> Dict[str, Any]:
    """Execute control_gripper tool with position-based control.

    Args:
        arm_id: Which arm (follower_right, follower_left)
        position: 0.0 (closed) to 1.0 (open)
    """
    print(f"[Robot] Gripper {arm_id} -> position {position:.2f}")

    if arm_id not in arm_controllers:
        return {
            "status": "error",
            "error": f"Arm '{arm_id}' not found",
            "valid_arms": list(arm_controllers.keys()),
            "recovery": "Use a valid arm_id from valid_arms"
        }

    # Validate position range
    if not 0.0 <= position <= 1.0:
        return {
            "status": "error",
            "error": f"Position {position} out of range",
            "recovery": "Use position between 0.0 (closed) and 1.0 (open)"
        }

    try:
        gripper = arm_controllers[arm_id]['gripper']
        result = gripper.set_gripper_position(position, blocking=True)

        if result.get('success'):
            return {
                "status": "success",
                "target_position": position,
                "state": result.get('state'),
                "position": result.get('position'),
                "position_normalized": result.get('position_normalized'),
                "message": f"Gripper moved to position {position:.2f}"
            }
        else:
            return {
                "status": "error",
                "target_position": position,
                "error": result.get('error', 'Gripper position failed'),
                "recovery": "Check gripper is not obstructed"
            }
    except Exception as e:
        return {
            "status": "error",
            "error": str(e),
            "recovery": "Try resume_after_stop if arm is in error state"
        }


def execute_finish_task(success: bool, summary: str) -> Dict[str, Any]:
    """Execute finish_task tool."""
    return {"status": "task_ended", "success": success, "summary": summary}


def execute_resume_after_stop(arm_id: str) -> Dict[str, Any]:
    """Resume arm operations after error state."""
    print(f"[Robot] Resuming {arm_id} after error/stop")

    if arm_id not in arm_controllers:
        return {
            "status": "error",
            "error": f"Arm '{arm_id}' not found",
            "valid_arms": list(arm_controllers.keys())
        }

    try:
        arm = arm_controllers[arm_id]['arm']
        result = arm.resume_after_stop()
        if result.get('success'):
            return {
                "status": "success",
                "state": result.get('state', 'resumed'),
                "message": result.get('message', 'Arm resumed. Ready for commands.')
            }
        else:
            return {"status": "error", "error": result.get('error', 'Resume failed')}
    except Exception as e:
        return {"status": "error", "error": str(e)}


def execute_get_arm_state(arm_id: str) -> Dict[str, Any]:
    """Get current arm state and position."""
    print(f"[Robot] Getting state for {arm_id}")

    if arm_id not in arm_controllers:
        return {
            "status": "error",
            "error": f"Arm '{arm_id}' not found",
            "valid_arms": list(arm_controllers.keys())
        }

    try:
        arm = arm_controllers[arm_id]['arm']
        state = arm.get_arm_state()

        if state.get('success'):
            ee_pos = state.get('ee_position', {})
            return {
                "status": "success",
                "arm_id": arm_id,
                "position": [ee_pos.get('x', 0), ee_pos.get('y', 0), ee_pos.get('z', 0)],
                "position_dict": ee_pos,
                "orientation": state.get('ee_orientation', {}),
                "joints_degrees": state.get('joints_degrees', []),
                "state": state.get('state', 'unknown'),
                "pose": state.get('pose')  # Named pose if applicable (home, sleep, ready)
            }
        else:
            return {"status": "error", "error": state.get('error', 'Failed to get arm state')}
    except Exception as e:
        return {"status": "error", "error": str(e)}


def execute_trajectory(arm_id: str, waypoints: List[Dict]) -> Dict[str, Any]:
    """Execute multi-waypoint trajectory with gripper coordination and checkpoint support."""
    print(f"[Robot] Executing trajectory on {arm_id}: {len(waypoints)} waypoints")

    if arm_id not in arm_controllers:
        return {
            "status": "error",
            "error": f"Arm '{arm_id}' not found",
            "valid_arms": list(arm_controllers.keys())
        }

    if not waypoints:
        return {
            "status": "error",
            "error": "No waypoints provided",
            "recovery": "Provide at least one waypoint with {point: [x,y,z], label: 'name'}"
        }

    # Validate waypoints
    for i, wp in enumerate(waypoints):
        if isinstance(wp, str):
            return {
                "status": "error",
                "error": f"Waypoint {i} is a string, expected object",
                "recovery": "Each waypoint must be: {point: [x,y,z], label: 'name'}"
            }
        if not isinstance(wp, dict):
            return {
                "status": "error",
                "error": f"Waypoint {i} is {type(wp).__name__}, expected object"
            }
        if 'point' not in wp:
            return {
                "status": "error",
                "error": f"Waypoint {i} missing 'point'",
                "recovery": "Add point: [x, y, z] to waypoint"
            }
        if not isinstance(wp['point'], list) or len(wp['point']) != 3:
            return {
                "status": "error",
                "error": f"Waypoint {i} point must be [x, y, z]"
            }

    try:
        arm = arm_controllers[arm_id]['arm']
        gripper = arm_controllers[arm_id]['gripper']

        result = arm.execute_trajectory(
            waypoints=waypoints,
            coordinate_with_gripper=gripper
        )

        # Check if we hit a checkpoint
        if result.get('status') == 'checkpoint':
            return {
                "status": "checkpoint",
                "label": result.get('label'),
                "current_position": result.get('current_position'),
                "waypoints_completed": len(result.get('waypoints_completed', [])),
                "waypoints_remaining": result.get('remaining_waypoints', 0),
                "message": "Checkpoint reached. Review images and plan next trajectory from current position."
            }

        # Normal completion
        if result.get('success') or result.get('status') == 'completed':
            return {
                "status": "success",
                "waypoints_completed": result.get('total_waypoints', len(waypoints)),
                "message": "Trajectory complete. Verify in camera images."
            }
        else:
            return {
                "status": "partial",
                "waypoints_completed": len(result.get('waypoints_completed', [])),
                "error": result.get('error', 'Trajectory incomplete')
            }
    except Exception as e:
        return {"status": "error", "error": str(e)}


def build_system_instruction(connected_arms: List[str]) -> str:
    """Build lean system instruction - technical details are in tool schemas."""
    arm_list = ', '.join(connected_arms)

    return f"""You are a robotic manipulation agent controlling ViperX 300s 6-DOF arm(s).

AVAILABLE ARMS: {arm_list}

EXECUTION PROTOCOL:
1. Use get_arm_state to check current position before planning movements
2. Use execute_trajectory with checkpoint=true before critical actions (grasping)
3. At checkpoints, review camera images and adjust coordinates if needed
4. After actions, verify results in camera images
5. Call finish_task when complete or impossible

VISUAL FEEDBACK LOOP:
- Camera images are provided after every action
- Small adjustments (1-2cm) are often needed based on visual feedback
- If position looks off, plan a new trajectory with corrected coordinates

Be precise and verify visually."""


# --- GEMINI SESSION MANAGER ---
ion 
class RobotSession:
    """
    Manages a single autonomous task execution session.

    Uses models.generate_content() with explicit history management
    instead of chats.create() to avoid SDK state confusion.
    """

    MAX_STEPS = 20  # Maximum steps before forced termination

    def __init__(self, prompt: str, ws: web.WebSocketResponse):
        self.ws = ws
        self.client = genai.Client(api_key=os.getenv("GEMINI_API_KEY"))
        self.task_prompt = prompt
        self.active = True
        self.step = 0

        # Explicit conversation history (not SDK-managed)
        self.history: List[types.Content] = []

        # Build dynamic system instruction and tools
        connected_arms = list(arm_controllers.keys())
        self.system_instruction = build_system_instruction(connected_arms)
        self.tools = build_tool_declarations(connected_arms)

        # Config for all API calls - with SDK native tool calling
        self.config = types.GenerateContentConfig(
            system_instruction=self.system_instruction,
            tools=[self.tools],  # Pass Tool object with FunctionDeclarations
            temperature=0.5,
            thinking_config=types.ThinkingConfig(thinking_budget=-1)  # -1 for unlimited thinking
        )

    async def start(self):
        """Initialize and run the task execution loop."""
        print(f"\n[Session] Starting task: {self.task_prompt}")

        # 1. Capture initial camera state
        images = capture_encoded_images()
        await self._send_ws("camera_frame", images)

        # 2. Build initial message with task + images
        initial_parts = [types.Part.from_text(text=f"TASK: {self.task_prompt}")]
        for cam_name, b64 in images.items():
            if b64:
                img_bytes = base64.b64decode(b64)
                initial_parts.append(types.Part.from_bytes(data=img_bytes, mime_type="image/jpeg"))

        # 3. Add to history
        self.history.append(types.Content(role='user', parts=initial_parts))

        # 4. Run the execution loop
        await self._execute_loop()

    async def _execute_loop(self):
        """
        Main execution loop with SDK native tool calling.

        Pattern:
        1. Call Gemini (stateless API with tools)
        2. Process Part.function_call from response
        3. Execute function
        4. Capture new images
        5. Send Part.from_function_response + images back
        6. Repeat until finish_task is called
        """
        while self.step < self.MAX_STEPS and self.active:
            self.step += 1

            # Notify frontend of step progress
            await self._send_ws("step_update", {
                "step": self.step,
                "max_steps": self.MAX_STEPS
            })

            print(f"[Gemini] Step {self.step}/{self.MAX_STEPS} - Calling model...")

            try:
                # 1. Call Gemini (stateless - pass full history each time)
                response = await asyncio.to_thread(
                    self.client.models.generate_content,
                    model='gemini-robotics-er-1.5-preview',
                    contents=self.history,
                    config=self.config
                )
            except Exception as e:
                logging.error(f"Gemini API error: {e}")
                await self._send_ws("error", {"message": str(e)})
                self.active = False
                break

            # 2. Check for valid response content
            if not response.candidates or len(response.candidates) == 0:
                print("[Session] No candidates in response")
                await self._send_ws("error", {"message": "No candidates in API response"})
                self.active = False
                break

            candidate = response.candidates[0]
            finish_reason = getattr(candidate, 'finish_reason', None)

            # Check finish_reason for issues
            if finish_reason in ('SAFETY', 'RECITATION', 'BLOCKLIST'):
                print(f"[Session] Response blocked: {finish_reason}")
                await self._send_ws("error", {"message": f"Response blocked by safety filter: {finish_reason}"})
                self.active = False
                break

            if candidate.content is None:
                print(f"[Session] Response content is None (finish_reason: {finish_reason})")
                await self._send_ws("error", {"message": f"Empty response from model (reason: {finish_reason})"})
                self.active = False
                break

            # Store response in history
            self.history.append(candidate.content)

            # 3. Process response parts - look for function_call and text
            function_call = None
            reasoning_text = ""

            for part in candidate.content.parts:
                # Text part = reasoning/thinking
                if hasattr(part, 'text') and part.text:
                    reasoning_text += part.text
                # Function call part = action to execute
                if hasattr(part, 'function_call') and part.function_call:
                    function_call = part.function_call

            # 4. Broadcast reasoning to frontend
            if reasoning_text:
                print(f"[Gemini] Reasoning: {reasoning_text[:100]}...")
                await self._send_ws("reasoning", {"text": reasoning_text})

            # 5. No function call = task might be complete or model is confused
            if not function_call:
                print("[Session] No function call in response")
                await self._send_ws("task_complete", {
                    "success": False,
                    "summary": "Model did not provide a function call"
                })
                self.active = False
                break

            func_name = function_call.name
            func_args = dict(function_call.args) if function_call.args else {}

            print(f"[Gemini] Action: {func_name}({func_args})")

            # Notify frontend
            await self._send_ws("next_action", {
                "function": func_name,
                "args": func_args,
                "step": self.step
            })

            # 6. Check for finish_task (signals task completion)
            if func_name == "finish_task":
                success = func_args.get('success', False)
                summary = func_args.get('summary', 'Task finished')
                self.active = False
                await self._send_ws("task_complete", {
                    "success": success,
                    "summary": summary
                })
                break

            # 7. Execute the function
            exec_result = await self._execute_tool(func_name, func_args)

            # Notify frontend of result
            await self._send_ws("execution_result", {
                "function": func_name,
                "result": exec_result,
                "step": self.step,
                "success": exec_result.get("status") != "error"
            })

            # 8. Capture NEW images after execution
            new_images = capture_encoded_images()
            await self._send_ws("camera_frame", new_images)

            # 9. Build function response + images for next turn
            response_parts = self._build_function_response(func_name, exec_result, new_images)
            self.history.append(types.Content(role='user', parts=response_parts))

        # Loop ended - check why
        if self.step >= self.MAX_STEPS and self.active:
            logging.warning(f"Task exceeded max steps ({self.MAX_STEPS})")
            await self._send_ws("task_complete", {
                "success": False,
                "reason": "max_steps_exceeded",
                "summary": f"Task terminated after {self.MAX_STEPS} steps"
            })

    def _build_function_response(self, func_name: str, result: dict, images: dict) -> list:
        """Build SDK function response + images for next model turn."""
        parts = []

        # Part 1: SDK function response (tells model what happened)
        parts.append(types.Part.from_function_response(
            name=func_name,
            response={"result": result}
        ))

        # Part 2: New camera images (visual verification) with identification
        for cam_name, b64 in images.items():
            if b64:
                img_bytes = base64.b64decode(b64)
                parts.append(types.Part.from_bytes(data=img_bytes, mime_type="image/jpeg"))
                # Add camera identification text after each image
                cam_desc = get_camera_description(cam_name)
                parts.append(types.Part.from_text(text=f"[{cam_name}] {cam_desc}"))

        # Part 3: Context-aware prompt
        if result.get('status') == 'checkpoint':
            label = result.get('label', 'checkpoint')
            current_pos = result.get('current_position', [])
            parts.append(types.Part.from_text(
                text=f"CHECKPOINT at '{label}'. Current position: {current_pos}. "
                     f"Review images and plan next trajectory from this position."
            ))
        else:
            parts.append(types.Part.from_text(
                text="Action complete. Examine the new camera images to verify the result."
            ))

        return parts

    async def _execute_tool(self, name: str, args: dict) -> dict:
        """Execute a tool function."""
        try:
            if name == "move_arm":
                return await asyncio.to_thread(
                    execute_move_arm,
                    args.get('arm_id'),
                    args.get('position'),
                    args.get('orientation'),    # None if not provided
                    args.get('pitch_degrees'),  # None if not provided (defaults to -30°)
                    args.get('moving_time', 1.5)
                )
            elif name == "control_gripper":
                return await asyncio.to_thread(
                    execute_control_gripper,
                    args.get('arm_id'),
                    args.get('position', 0.5)  # Default to half-open if not specified
                )
            elif name == "finish_task":
                return execute_finish_task(args.get('success', False), args.get('summary', ''))
            elif name == "resume_after_stop":
                return await asyncio.to_thread(
                    execute_resume_after_stop,
                    args.get('arm_id')
                )
            elif name == "get_arm_state":
                return await asyncio.to_thread(
                    execute_get_arm_state,
                    args.get('arm_id')
                )
            elif name == "execute_trajectory":
                return await asyncio.to_thread(
                    execute_trajectory,
                    args.get('arm_id'),
                    args.get('waypoints', [])
                )
            else:
                return {"status": "error", "error": f"Unknown tool: {name}"}
        except Exception as e:
            logging.error(f"Tool execution error: {e}")
            return {"status": "error", "error": str(e)}

    async def _send_ws(self, msg_type: str, payload: dict):
        """Send a message to the WebSocket client."""
        if self.ws.closed:
            return
        try:
            await self.ws.send_json({
                "type": msg_type,
                "timestamp": int(time.time() * 1000),
                "payload": payload
            })
        except Exception as e:
            logging.error(f"WebSocket send error: {e}")

# --- WEBSOCKET MESSAGE HANDLERS ---

async def handle_ws_robot_connect(ws: web.WebSocketResponse):
    """Handle robot connect - execute opening ceremony on all arms."""
    global robot_connected

    if not arm_controllers:
        await send_ws(ws, 'error', {'code': 'NO_ROBOT', 'message': 'Robot not initialized'})
        return

    if robot_connected:
        await send_ws(ws, 'robot_status', {'connected': True, 'message': 'Already connected'})
        return

    try:
        print("\n[WebSocket] Opening Ceremony")

        for arm_id, arm_stack in arm_controllers.items():
            arm_ctrl = arm_stack['arm']
            gripper_ctrl = arm_stack['gripper']

            print(f"  [{arm_id}] Moving to ready position...")
            ceremony_result = await asyncio.get_event_loop().run_in_executor(
                None,
                lambda ac=arm_ctrl: ac.opening_ceremony(moving_time=4.0, blocking=True)
            )

            if not ceremony_result.get('success'):
                await send_ws(ws, 'error', {
                    'code': 'CEREMONY_FAILED',
                    'message': f'{arm_id}: {ceremony_result.get("error", "Opening ceremony failed")}'
                })
                return

            # Open gripper
            print(f"  [{arm_id}] Opening gripper...")
            gripper_result = await asyncio.get_event_loop().run_in_executor(
                None,
                lambda gc=gripper_ctrl: gc.open_gripper()
            )

            if not gripper_result.get('success'):
                await send_ws(ws, 'error', {
                    'code': 'GRIPPER_FAILED',
                    'message': f'{arm_id}: {gripper_result.get("error", "Gripper open failed")}'
                })
                return

        robot_connected = True

        # Broadcast to all clients
        await broadcast_ws('robot_status', {
            'connected': True,
            'connected_arms': list(arm_controllers.keys()),
            'message': 'Robot connected and ready'
        })

        print("[WebSocket] ✓ Opening ceremony complete")

    except Exception as e:
        logging.error(f"Robot connect error: {e}")
        await send_ws(ws, 'error', {'code': 'CONNECT_ERROR', 'message': str(e)})


async def handle_ws_robot_disconnect(ws: web.WebSocketResponse):
    """Handle robot disconnect - execute closing ceremony on all arms."""
    global robot_connected

    if not arm_controllers:
        await send_ws(ws, 'error', {'code': 'NO_ROBOT', 'message': 'Robot not initialized'})
        return

    if not robot_connected:
        await send_ws(ws, 'robot_status', {'connected': False, 'message': 'Already disconnected'})
        return

    try:
        print("\n[WebSocket] Closing Ceremony")

        for arm_id, arm_stack in arm_controllers.items():
            arm_ctrl = arm_stack['arm']
            gripper_ctrl = arm_stack['gripper']

            # Close gripper first
            print(f"  [{arm_id}] Closing gripper...")
            gripper_result = await asyncio.get_event_loop().run_in_executor(
                None,
                lambda gc=gripper_ctrl: gc.close_gripper()
            )

            if not gripper_result.get('success'):
                logging.warning(f"{arm_id}: Gripper close failed - {gripper_result.get('error', 'unknown')}")
                # Continue with shutdown anyway

            print(f"  [{arm_id}] Moving to sleep position...")
            await asyncio.get_event_loop().run_in_executor(
                None,
                lambda ac=arm_ctrl: ac.closing_ceremony(moving_time=4.0, blocking=True)
            )

        robot_connected = False

        # Broadcast to all clients
        await broadcast_ws('robot_status', {
            'connected': False,
            'message': 'Robot disconnected - arm in sleep position'
        })

        print("[WebSocket] ✓ Closing ceremony complete")

    except Exception as e:
        logging.error(f"Robot disconnect error: {e}")
        await send_ws(ws, 'error', {'code': 'DISCONNECT_ERROR', 'message': str(e)})


async def handle_ws_status_request(ws: web.WebSocketResponse):
    """Handle status request via WebSocket."""
    cameras = []
    if camera_controller and camera_controller.initialized:
        cameras = camera_controller.get_camera_names()

    await send_ws(ws, 'robot_status', {
        'connected': robot_connected,
        'connected_arms': list(arm_controllers.keys()),
        'cameras': cameras
    })


# --- SERVER SETUP ---

async def websocket_handler(request: web.Request):
    """Main WebSocket handler with message routing."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    ws_clients.add(ws)

    # Send connection info immediately
    cameras = []
    if camera_controller and camera_controller.initialized:
        cameras = camera_controller.get_camera_names()

    await send_ws(ws, 'connection_info', {
        'connected_arms': list(arm_controllers.keys()),
        'robot_connected': robot_connected,
        'cameras': cameras
    })

    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                    msg_type = data.get("type")

                    if msg_type == "task_request":
                        prompt = data.get("payload", {}).get("prompt")
                        if prompt and robot_connected:
                            session = RobotSession(prompt, ws)
                            asyncio.create_task(session.start())
                        elif not robot_connected:
                            await send_ws(ws, 'error', {
                                'code': 'NOT_CONNECTED',
                                'message': 'Robot not connected. Send robot_connect first.'
                            })

                    elif msg_type == "robot_connect":
                        await handle_ws_robot_connect(ws)

                    elif msg_type == "robot_disconnect":
                        await handle_ws_robot_disconnect(ws)

                    elif msg_type == "status_request":
                        await handle_ws_status_request(ws)

                except json.JSONDecodeError:
                    await send_ws(ws, 'error', {'code': 'INVALID_JSON', 'message': 'Invalid JSON'})

            elif msg.type == WSMsgType.ERROR:
                logging.error(f"WebSocket error: {ws.exception()}")

    finally:
        ws_clients.discard(ws)

    return ws


def make_app():
    """Create and configure the aiohttp application."""
    app = web.Application()

    # Setup CORS
    cors = setup(app, defaults={
        '*': ResourceOptions(
            allow_credentials=True,
            expose_headers='*',
            allow_headers='*'
        )
    })

    # WebSocket endpoint
    resource = app.router.add_resource('/ws')
    route = resource.add_route('GET', websocket_handler)
    cors.add(route)

    return app


async def on_shutdown(app):
    """Cleanup on server shutdown."""
    # Close all WebSocket connections
    for ws in ws_clients:
        await ws.close()

    # Shutdown hardware
    shutdown_hardware()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Gemini Robotics ER Bridge')
    parser.add_argument('--port', type=int, default=8082, help='Server port (default: 8082)')
    parser.add_argument('--no-robot', action='store_true', help='Run without robot hardware (mock mode)')
    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("  Gemini Robotics ER Bridge")
    print("  SDK Native Tool Calling | WebSocket API")
    print("=" * 60)
    print()
    print("Features:")
    print("  - SDK native FunctionDeclaration tool definitions")
    print("  - Proper types.Tool and types.Schema usage")
    print("  - WebSocket real-time streaming")
    print("  - Automatic image injection between steps")
    print()

    if not os.getenv('GEMINI_API_KEY'):
        print("WARNING: GEMINI_API_KEY environment variable not set!")
        print("Set it in .env or export GEMINI_API_KEY=your_key")
        print()

    # Initialize hardware at startup
    if not args.no_robot:
        initialize_hardware()
    else:
        print("[Bridge] Running in mock mode (--no-robot)")

    # Create and run app
    app = make_app()
    app.on_shutdown.append(on_shutdown)

    print(f"\n[Bridge] Starting WebSocket server on port {args.port}...")
    print(f"[Bridge] Connect via: ws://localhost:{args.port}/ws")
    print("[Bridge] Messages: task_request, robot_connect, robot_disconnect, status_request\n")

    web.run_app(app, port=args.port, print=None)