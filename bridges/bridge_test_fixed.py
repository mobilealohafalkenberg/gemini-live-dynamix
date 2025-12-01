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

            # Initialize GripperController
            gripper_ctrl = GripperController(
                dynamixel_controller=dxl_controller,
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
        # Return mock if no camera for testing
        return {"mock_cam": "R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7"}
    return camera_controller.get_all_frames_base64()


# --- TOOLS DEFINITIONS ---
# Native Python functions that the SDK will convert to Schemas

def move_arm(arm_id: str, position: List[float], moving_time: float = 1.5) -> Dict[str, Any]:
    """
    Moves the robot arm end-effector to a specific 3D Cartesian coordinate.

    Args:
        arm_id: The ID of the arm (e.g., 'follower_right').
        position: Target [x, y, z] coordinates in meters.
        moving_time: Duration of movement in seconds.
    """
    print(f"[Robot] Moving {arm_id} to {position}")
    if not ROBOT_HARDWARE_AVAILABLE:
        return {"status": "success", "simulated": True, "new_position": position}

    # Validate arm_id exists
    if arm_id not in arm_controllers:
        return {"status": "error", "error": f"Unknown arm_id: {arm_id}. Available: {list(arm_controllers.keys())}"}

    try:
        arm = arm_controllers[arm_id]['arm']
        # Blocking call to ensure movement completes before we return
        arm.move_to_position(position, moving_time=moving_time, blocking=True)
        state = arm.get_arm_state()
        return {
            "status": "success",
            "final_position": state['end_effector_position'],
            "message": "Movement complete. Analyze the accompanying image to verify alignment."
        }
    except Exception as e:
        return {"status": "error", "error": str(e)}

def control_gripper(arm_id: str, action: str) -> Dict[str, Any]:
    """
    Opens or closes the robot gripper.

    Args:
        arm_id: The ID of the arm.
        action: 'open' or 'close'.
    """
    print(f"[Robot] Gripper {arm_id} -> {action}")
    if not ROBOT_HARDWARE_AVAILABLE:
        return {"status": "success", "simulated": True, "action": action}

    # Validate arm_id exists
    if arm_id not in arm_controllers:
        return {"status": "error", "error": f"Unknown arm_id: {arm_id}. Available: {list(arm_controllers.keys())}"}

    try:
        gripper = arm_controllers[arm_id]['gripper']
        if action == 'open': gripper.open_gripper()
        elif action == 'close': gripper.close_gripper()
        return {"status": "success", "action": action, "message": "Gripper actuated."}
    except Exception as e:
        return {"status": "error", "error": str(e)}

def adjust_position(arm_id: str, delta_x: float, delta_y: float, delta_z: float) -> Dict[str, Any]:
    """
    Makes a small relative adjustment to the current end-effector position.
    Useful for fine-tuning alignment based on visual feedback.

    Args:
        arm_id: The ID of the arm (e.g., 'follower_right')
        delta_x: Change in X coordinate in meters (typically -0.05 to +0.05)
        delta_y: Change in Y coordinate in meters (typically -0.05 to +0.05)
        delta_z: Change in Z coordinate in meters (typically -0.05 to +0.05)

    Returns:
        Dict with status, old position, new position, and delta applied
    """
    print(f"[Robot] Adjusting {arm_id} by Δ[{delta_x:.3f}, {delta_y:.3f}, {delta_z:.3f}]")

    if not ROBOT_HARDWARE_AVAILABLE:
        return {
            "status": "success",
            "simulated": True,
            "delta": [delta_x, delta_y, delta_z]
        }

    # Validate arm_id exists
    if arm_id not in arm_controllers:
        return {"status": "error", "error": f"Unknown arm_id: {arm_id}. Available: {list(arm_controllers.keys())}"}

    try:
        arm = arm_controllers[arm_id]['arm']

        # Get current position
        current_state = arm.get_arm_state()
        current_pos = current_state['end_effector_position']

        # Calculate new position
        new_pos = [
            current_pos[0] + delta_x,
            current_pos[1] + delta_y,
            current_pos[2] + delta_z
        ]

        print(f"  Current: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
        print(f"  New:     [{new_pos[0]:.3f}, {new_pos[1]:.3f}, {new_pos[2]:.3f}]")

        # Execute movement
        arm.move_to_position(new_pos, moving_time=1.0, blocking=True)

        # Get final state
        final_state = arm.get_arm_state()
        final_pos = final_state['end_effector_position']

        return {
            "status": "success",
            "old_position": current_pos,
            "new_position": final_pos,
            "delta_applied": [delta_x, delta_y, delta_z],
            "message": "Adjustment complete. Check new image to verify alignment."
        }

    except Exception as e:
        logging.error(f"Adjustment failed: {e}")
        return {"status": "error", "error": str(e)}


def finish_task(success: bool, summary: str):
    """Call this when the task is fully complete or if it is impossible."""
    return {"status": "task_ended", "success": success, "summary": summary}

# Tool list for the model
ROBOT_TOOLS = [move_arm, adjust_position, control_gripper, finish_task]

def build_system_instruction(connected_arms: List[str]) -> str:
    """Build dynamic system instruction with connected arm info."""
    arm_list = ', '.join(f'"{a}"' for a in connected_arms) if connected_arms else '"mock_arm"'
    first_arm = connected_arms[0] if connected_arms else 'mock_arm'

    return f"""You are an advanced robotic manipulation agent controlling a ViperX 300s 6-DOF arm.

CONNECTED ARMS: [{arm_list}]

COORDINATE SYSTEM:
- Robot base is at [0, 0, 0]
- +X is FORWARD (toward workspace)
- +Y is LEFT
- +Z is UP
- Typical workspace: X: 0.15-0.50m, Y: -0.30 to 0.30m, Z: 0.05-0.40m

⚠️ CRITICAL LIMITATION - CAMERA CALIBRATION:
The camera position and viewing angle are NOT precisely calibrated. This means:
- You CANNOT accurately determine exact 3D positions from images alone on first attempt
- Your initial position estimates will typically have 5-15cm error
- You MUST use an iterative visual servoing approach with multiple corrections

MANDATORY ITERATIVE APPROACH PROTOCOL:
1. Make a conservative initial estimate from the image
2. Move to a SAFE HEIGHT (Z = 0.20m) above the estimated target position
3. Capture new image and carefully assess horizontal alignment (X, Y offset)
4. Use adjust_position() to make SMALL corrections (max 0.05m per adjustment)
5. Repeat steps 3-4 until visually confirmed centered above target (2-4 iterations typical)
6. Only then descend slowly to the actual grasp height
7. Verify gripper alignment in image before closing
8. Expect to use 6-10 total movements per successful object manipulation

VISUAL OFFSET ESTIMATION GUIDELINES:
When analyzing images to estimate positioning errors:
- Object at edge of camera view: approximately 0.08-0.12m offset
- Object at 1/3 from center: approximately 0.04-0.06m offset
- Object near center but not aligned: approximately 0.02-0.03m offset
- Always make conservative (smaller) adjustments and iterate

TOOL USAGE STRATEGY:
- Use move_arm() for large movements (>0.10m) and initial approaches
- Use adjust_position() for fine corrections (<0.05m) during alignment phase
- Use control_gripper() only after visual confirmation of alignment
- Never attempt to grasp without at least 2-3 verification steps

SAFETY RULES (NEVER VIOLATE):
1. NEVER command Z < 0.05m (table collision hazard)
2. Keep Z > 0.15m during all horizontal travel movements
3. Maximum reach is approximately 0.50m from robot base
4. NEVER make movements larger than 0.15m without intermediate verification

GRIPPER PROTOCOL:
- ALWAYS open gripper BEFORE approaching any object
- Close gripper only after visual confirmation of proper alignment
- After closing, verify object is grasped before attempting to lift
- If grip fails, open gripper, reassess position, and retry

FUNCTION CALLING:
- Use arm_id="{first_arm}" for all function calls
- Call finish_task(success=True, summary="Detailed description") when objective completed
- Call finish_task(success=False, summary="Reason for failure") if task becomes impossible

You are methodical, patient, and precise. Rushing leads to failures. Visual verification after every movement is mandatory. Small iterative corrections are better than large blind movements."""

# --- GEMINI SESSION MANAGER ---

class RobotSession:
    """Manages a single autonomous task execution session."""

    MAX_STEPS = 15  # Maximum steps before forced termination

    def __init__(self, prompt: str, ws: web.WebSocketResponse):
        self.ws = ws
        self.client = genai.Client(api_key=os.getenv("GEMINI_API_KEY"))
        self.chat = None
        self.task_prompt = prompt
        self.active = True
        self.step = 0

        # Build dynamic system instruction
        connected_arms = list(arm_controllers.keys())
        self.system_instruction = build_system_instruction(connected_arms)

    async def start(self):
        """Initializes the chat and kicks off the loop."""

        # 1. Initialize Chat with specific ER configurations
        self.chat = self.client.chats.create(
            model='gemini-robotics-er-1.5-preview', # The specific Robotics ER model
            config=types.GenerateContentConfig(
                system_instruction=self.system_instruction,
                tools=ROBOT_TOOLS,
                temperature=0.1, # Low temp for precision
                automatic_function_calling=False, # MANDATORY: We handle the loop manually to inject images
                thinking_config=types.ThinkingConfig(thinking_budget=1024) # Enable "Thinking" for spatial reasoning
            )
        )

        # 2. Capture Initial State
        images = capture_encoded_images()
        await self._send_ws("camera_frame", images)

        # 3. Build First Message (Text + Initial Images)
        content_parts = [types.Part.from_text(text=f"TASK: {self.task_prompt}")]
        for name, b64 in images.items():
            img_bytes = base64.b64decode(b64)
            content_parts.append(types.Part.from_bytes(data=img_bytes, mime_type="image/jpeg"))

        # 4. Start the Loop
        await self._run_turn(content_parts)

    async def _run_turn(self, message_parts):
        """
        The Core Loop:
        Send Message -> Get Prediction -> Execute Tool -> Bundle (Result + NEW IMAGE) -> Recurse
        """
        if not self.active:
            return

        # Step tracking
        self.step += 1
        await self._send_ws("step_update", {
            "step": self.step,
            "max_steps": self.MAX_STEPS
        })

        # Check max steps
        if self.step > self.MAX_STEPS:
            logging.warning(f"Task exceeded max steps ({self.MAX_STEPS})")
            self.active = False
            await self._send_ws("task_complete", {
                "success": False,
                "reason": "max_steps_exceeded",
                "summary": f"Task terminated after {self.MAX_STEPS} steps without completion"
            })
            return

        print(f"[Gemini] Step {self.step}/{self.MAX_STEPS} - Sending to model...")
        try:
            # A. Send to Model
            response = await asyncio.to_thread(
                self.chat.send_message,
                message=message_parts
            )
        except Exception as e:
            print(f"[Gemini] Error: {e}")
            await self._send_ws("error", {"message": str(e)})
            return

        # B. Handle Model Output
        # The model might "Think" first (generate text), then "Act" (function call)
        # Or just "Think" and ask for clarification.

        for part in response.candidates[0].content.parts:
            # 1. Handle Thinking/Reasoning (Text)
            if part.text:
                print(f"[Gemini Thinking] {part.text[:100]}...")
                await self._send_ws("reasoning", {"text": part.text})

            # 2. Handle Tool Call (Action)
            if part.function_call:
                fc = part.function_call

                # Enhanced logging with position tracking
                print("\n" + "="*70)
                print(f"[STEP {self.step}/{self.MAX_STEPS}] GEMINI ACTION")
                print("="*70)
                print(f"Function: {fc.name}")
                print(f"Arguments: {json.dumps(fc.args, indent=2)}")

                # If it's a movement command, show current vs target
                if fc.name in ["move_arm", "adjust_position"] and ROBOT_HARDWARE_AVAILABLE:
                    try:
                        arm_id = fc.args.get('arm_id')
                        if arm_id and arm_id in arm_controllers:
                            arm = arm_controllers[arm_id]['arm']
                            current_state = arm.get_arm_state()
                            current_pos = current_state['end_effector_position']

                            print(f"\nCurrent Position: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")

                            if fc.name == "move_arm":
                                target_pos = fc.args.get('position', [0,0,0])
                                delta = [target_pos[i] - current_pos[i] for i in range(3)]
                                print(f"Target Position:  [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                                print(f"Delta Required:   [{delta[0]:.3f}, {delta[1]:.3f}, {delta[2]:.3f}]")
                                print(f"Distance:         {(sum(d**2 for d in delta)**0.5):.3f}m")

                            elif fc.name == "adjust_position":
                                dx = fc.args.get('delta_x', 0)
                                dy = fc.args.get('delta_y', 0)
                                dz = fc.args.get('delta_z', 0)
                                new_pos = [current_pos[0]+dx, current_pos[1]+dy, current_pos[2]+dz]
                                print(f"Delta Request:    [{dx:.3f}, {dy:.3f}, {dz:.3f}]")
                                print(f"Target Position:  [{new_pos[0]:.3f}, {new_pos[1]:.3f}, {new_pos[2]:.3f}]")

                    except Exception as e:
                        logging.debug(f"Could not log position info: {e}")

                print("="*70 + "\n")

                # Execute the tool (FIX: was commented out, causing NameError)
                result = await self._execute_tool(fc.name, fc.args)

                # Notify Frontend of Result
                await self._send_ws("next_action", {
                    "function": fc.name,
                    "args": fc.args,
                    "step": self.step,
                    "result": result
                })

                # Check for termination
                if fc.name == "finish_task":
                    self.active = False
                    await self._send_ws("task_complete", result)
                    return

                # C. PREPARE NEXT INPUT (Crucial Step)
                # Capture NEW images immediately after execution
                new_images = capture_encoded_images()
                await self._send_ws("camera_frame", new_images)

                # Construct the Response Message
                # It MUST contain the FunctionResponse AND the New Images
                next_parts = []

                # Part 1: The Code Result
                next_parts.append(types.Part.from_function_response(
                    name=fc.name,
                    response={"result": result}
                ))

                # Part 2: The Visual Result (The "Verification" Image)
                for cam_name, b64 in new_images.items():
                    img_bytes = base64.b64decode(b64)
                    next_parts.append(types.Part.from_bytes(data=img_bytes, mime_type="image/jpeg"))

                # Part 3: Optional prompt reinforcement (usually not needed for ER model, but helpful)
                # next_parts.append(types.Part.from_text(text="Action complete. View image to verify."))

                # Recursive call to start next turn
                await self._run_turn(next_parts)
                return

    async def _execute_tool(self, name: str, args: dict) -> dict:
        if name == "move_arm":
            return await asyncio.to_thread(move_arm, args.get('arm_id'), args.get('position'), args.get('moving_time', 1.5))
        elif name == "adjust_position":
            return await asyncio.to_thread(adjust_position, args.get('arm_id'), args.get('delta_x', 0.0), args.get('delta_y', 0.0), args.get('delta_z', 0.0))
        elif name == "control_gripper":
            return await asyncio.to_thread(control_gripper, args.get('arm_id'), args.get('action'))
        elif name == "finish_task":
            return finish_task(args.get('success'), args.get('summary'))
        return {"error": "Unknown tool"}

    async def _send_ws(self, msg_type: str, payload: dict):
        if self.ws.closed: return
        await self.ws.send_json({
            "type": msg_type,
            "timestamp": int(time.time() * 1000),
            "payload": payload
        })

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

        loop = asyncio.get_running_loop()

        for arm_id, arm_stack in arm_controllers.items():
            arm_ctrl = arm_stack['arm']
            gripper_ctrl = arm_stack['gripper']

            print(f"  [{arm_id}] Moving to ready position...")
            ceremony_result = await loop.run_in_executor(
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
            await loop.run_in_executor(
                None,
                lambda gc=gripper_ctrl: gc.open_gripper()
            )

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

        loop = asyncio.get_running_loop()

        for arm_id, arm_stack in arm_controllers.items():
            arm_ctrl = arm_stack['arm']
            gripper_ctrl = arm_stack['gripper']

            # Close gripper first
            await loop.run_in_executor(
                None,
                lambda gc=gripper_ctrl: gc.close_gripper()
            )

            print(f"  [{arm_id}] Moving to sleep position...")
            await loop.run_in_executor(
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


# --- HTTP ENDPOINT HANDLERS ---

async def handle_robot_status(request: web.Request) -> web.Response:
    """
    GET /robot/status - Get current robot status for all connected arms.
    """
    if not arm_controllers:
        return web.json_response({
            'success': False,
            'error': 'Robot not initialized',
            'connected_arms': [],
            'arm_count': 0,
            'robot_connected': False
        })

    try:
        arms_status = {}

        for arm_id, stack in arm_controllers.items():
            try:
                arm_ctrl = stack['arm']
                state = arm_ctrl.get_arm_state()
                arms_status[arm_id] = {
                    'port': stack['port'],
                    'initialized': True,
                    'state': state
                }
            except Exception as e:
                arms_status[arm_id] = {
                    'port': stack['port'],
                    'initialized': False,
                    'error': str(e)
                }

        return web.json_response({
            'success': True,
            'connected_arms': list(arm_controllers.keys()),
            'arm_count': len(arm_controllers),
            'robot_connected': robot_connected,
            'arms': arms_status
        })

    except Exception as e:
        return web.json_response({
            'success': False,
            'error': str(e),
            'connected_arms': [],
            'arm_count': 0
        }, status=500)


async def handle_camera_info(request: web.Request) -> web.Response:
    """GET /camera/info - Get camera initialization status."""
    if not camera_controller:
        return web.json_response({
            'success': False,
            'initialized': False,
            'error': 'Camera controller not created'
        })

    return web.json_response({
        'success': True,
        'initialized': camera_controller.initialized,
        'cameras': camera_controller.get_camera_names() if camera_controller.initialized else []
    })


async def handle_camera_frame(request: web.Request) -> web.Response:
    """GET /camera/{camera_name}/frame - Get a frame from a specific camera."""
    camera_names = camera_controller.get_camera_names() if camera_controller and camera_controller.initialized else []
    default_camera = camera_names[0] if camera_names else 'camera_0'
    camera_name = request.match_info.get('camera_name', default_camera)

    if not camera_controller or not camera_controller.initialized:
        return web.json_response({
            'success': False,
            'error': 'Camera controller not initialized'
        }, status=503)

    frame_b64 = camera_controller.get_frame_base64(camera_name)

    if frame_b64:
        return web.json_response({
            'success': True,
            'camera': camera_name,
            'frame': frame_b64,
            'format': 'jpeg_base64'
        })
    else:
        return web.json_response({
            'success': False,
            'error': f'No frame available from {camera_name}'
        }, status=404)


async def handle_robot_connect(request: web.Request) -> web.Response:
    """POST /robot/connect - Execute opening ceremony on all arms."""
    global robot_connected

    if not arm_controllers:
        return web.json_response({
            'success': False,
            'error': 'Robot not initialized'
        }, status=400)

    if robot_connected:
        return web.json_response({
            'success': True,
            'message': 'Already connected',
            'connected_arms': list(arm_controllers.keys())
        })

    try:
        print("\n[HTTP] Opening Ceremony")
        loop = asyncio.get_running_loop()

        for arm_id, arm_stack in arm_controllers.items():
            arm_ctrl = arm_stack['arm']
            gripper_ctrl = arm_stack['gripper']

            print(f"  [{arm_id}] Moving to ready position...")
            ceremony_result = await loop.run_in_executor(
                None,
                lambda ac=arm_ctrl: ac.opening_ceremony(moving_time=4.0, blocking=True)
            )

            if not ceremony_result.get('success'):
                return web.json_response({
                    'success': False,
                    'error': f'{arm_id}: {ceremony_result.get("error", "Opening ceremony failed")}'
                }, status=500)

            await loop.run_in_executor(
                None,
                lambda gc=gripper_ctrl: gc.open_gripper()
            )

        robot_connected = True

        # Broadcast to WebSocket clients
        await broadcast_ws('robot_status', {
            'connected': True,
            'connected_arms': list(arm_controllers.keys()),
            'message': 'Robot connected and ready'
        })

        print("[HTTP] ✓ Opening ceremony complete")

        return web.json_response({
            'success': True,
            'message': 'Robot connected and ready',
            'connected_arms': list(arm_controllers.keys())
        })

    except Exception as e:
        logging.error(f"Robot connect error: {e}")
        return web.json_response({
            'success': False,
            'error': str(e)
        }, status=500)


async def handle_robot_disconnect(request: web.Request) -> web.Response:
    """POST /robot/disconnect - Execute closing ceremony on all arms."""
    global robot_connected

    if not arm_controllers:
        return web.json_response({
            'success': False,
            'error': 'Robot not initialized'
        }, status=400)

    if not robot_connected:
        return web.json_response({
            'success': True,
            'message': 'Already disconnected'
        })

    try:
        print("\n[HTTP] Closing Ceremony")
        loop = asyncio.get_running_loop()

        for arm_id, arm_stack in arm_controllers.items():
            arm_ctrl = arm_stack['arm']
            gripper_ctrl = arm_stack['gripper']

            await loop.run_in_executor(
                None,
                lambda gc=gripper_ctrl: gc.close_gripper()
            )

            print(f"  [{arm_id}] Moving to sleep position...")
            await loop.run_in_executor(
                None,
                lambda ac=arm_ctrl: ac.closing_ceremony(moving_time=4.0, blocking=True)
            )

        robot_connected = False

        # Broadcast to WebSocket clients
        await broadcast_ws('robot_status', {
            'connected': False,
            'message': 'Robot disconnected - arm in sleep position'
        })

        print("[HTTP] ✓ Closing ceremony complete")

        return web.json_response({
            'success': True,
            'message': 'Robot disconnected - arm in sleep position'
        })

    except Exception as e:
        logging.error(f"Robot disconnect error: {e}")
        return web.json_response({
            'success': False,
            'error': str(e)
        }, status=500)


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
            allow_headers='*',
            allow_methods='*'
        )
    })

    # HTTP endpoints
    app.router.add_get('/robot/status', handle_robot_status)
    app.router.add_post('/robot/connect', handle_robot_connect)
    app.router.add_post('/robot/disconnect', handle_robot_disconnect)
    app.router.add_get('/camera/info', handle_camera_info)
    app.router.add_get('/camera/{camera_name}/frame', handle_camera_frame)

    # WebSocket endpoint
    app.router.add_get('/ws', websocket_handler)

    # Add CORS to all routes
    for route in list(app.router.routes()):
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
    print("\n" + "=" * 60)
    print("  Gemini Robotics ER Bridge (Chat API)")
    print("  HTTP + WebSocket | Port 8082")
    print("=" * 60)

    # Initialize hardware at startup
    initialize_hardware()

    # Create and run app
    app = make_app()
    app.on_shutdown.append(on_shutdown)

    print("\n[Bridge] Starting server on port 8082...")
    print()
    print("HTTP Endpoints:")
    print("  GET  /robot/status              - Get robot status")
    print("  POST /robot/connect             - Opening ceremony")
    print("  POST /robot/disconnect          - Closing ceremony")
    print("  GET  /camera/info               - Camera info")
    print("  GET  /camera/{name}/frame       - Get camera frame")
    print()
    print("WebSocket Endpoint:")
    print("  ws://localhost:8082/ws")
    print("  Messages: task_request, robot_connect, robot_disconnect, status_request")
    print()

    web.run_app(app, port=8082, print=None)
