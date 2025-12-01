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

    try:
        gripper = arm_controllers[arm_id]['gripper']
        if action == 'open': gripper.open_gripper()
        elif action == 'close': gripper.close_gripper()
        return {"status": "success", "action": action, "message": "Gripper actuated."}
    except Exception as e:
        return {"status": "error", "error": str(e)}

def finish_task(success: bool, summary: str):
    """Call this when the task is fully complete or if it is impossible."""
    return {"status": "task_ended", "success": success, "summary": summary}

# Tool list for the model
ROBOT_TOOLS = [move_arm, control_gripper, finish_task]


def build_system_instruction(connected_arms: List[str]) -> str:
    """Build dynamic system instruction with connected arm info (balanced ~100 lines)."""
    arm_list = ', '.join(f'"{a}"' for a in connected_arms) if connected_arms else '"mock_arm"'
    first_arm = connected_arms[0] if connected_arms else 'mock_arm'

    return f"""You are a robotic manipulation agent controlling a ViperX 300s 6-DOF arm.

CONNECTED ARMS: [{arm_list}]

COORDINATE SYSTEM:
- Robot base at [0, 0, 0]
- +X: FORWARD (toward workspace), +Y: LEFT, +Z: UP
- Units: meters
- Workspace: X: 0.15-0.50m, Y: -0.30 to 0.30m, Z: 0.05-0.40m

SAFETY RULES:
1. NEVER command Z < 0.05m (table collision)
2. Keep Z > 0.15m for travel movements
3. Max reach ~0.50m from base
4. ALWAYS open gripper before approaching object
5. ALWAYS verify grasp in camera before lifting

VISUAL-SERVOING PROTOCOL:
After EVERY action, you receive NEW camera images showing current state.
- Verify the action succeeded by examining the new image
- If gripper missed object, estimate offset and retry
- Use visual feedback to refine position estimates

EXAMPLE WORKFLOW (Pick and Place):
1. Analyze overhead camera -> detect object position
2. move_arm to position above object (Z + 0.15m clearance)
3. Open gripper (control_gripper action="open")
4. Descend to grasp height (Z ~ 0.05m above table)
5. Close gripper (control_gripper action="close")
6. VERIFY in camera: Is object between gripper jaws?
   - If YES: Lift and transport
   - If NO: Open gripper, adjust position, retry
7. Transport to destination
8. Lower and release
9. Return to home, call finish_task

AVAILABLE FUNCTIONS:
- move_arm(arm_id="{first_arm}", position=[x, y, z], moving_time=1.5)
- control_gripper(arm_id="{first_arm}", action="open" or "close")
- finish_task(success=True/False, summary="what was accomplished")

RESPONSE FORMAT (Return valid JSON only):
{{
    "reasoning": "What I observe in current images and why this action is needed",
    "next_action": {{
        "function": "move_arm",
        "args": {{"arm_id": "{first_arm}", "position": [0.3, 0.1, 0.2]}}
    }},
    "task_complete": false
}}

When done:
{{
    "reasoning": "Task complete - verified in camera",
    "next_action": null,
    "task_complete": true,
    "summary": "What was accomplished"
}}

Be precise, methodical, verify each action visually before proceeding."""


# --- GEMINI SESSION MANAGER ---

class RobotSession:
    """
    Manages a single autonomous task execution session.

    Uses models.generate_content() with explicit history management
    instead of chats.create() to avoid SDK state confusion.
    """

    MAX_STEPS = 10  # Maximum steps before forced termination

    def __init__(self, prompt: str, ws: web.WebSocketResponse):
        self.ws = ws
        self.client = genai.Client(api_key=os.getenv("GEMINI_API_KEY"))
        self.task_prompt = prompt
        self.active = True
        self.step = 0

        # Explicit conversation history (not SDK-managed)
        self.history: List[types.Content] = []

        # Build dynamic system instruction
        connected_arms = list(arm_controllers.keys())
        self.system_instruction = build_system_instruction(connected_arms)

        # Config for all API calls
        self.config = types.GenerateContentConfig(
            system_instruction=self.system_instruction,
            temperature=0.1,
            thinking_config=types.ThinkingConfig(thinking_budget=1024)
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
        Main execution loop - simple while loop with explicit state.

        Pattern:
        1. Call Gemini (stateless API)
        2. Parse JSON response
        3. Execute function
        4. Capture new images
        5. Add feedback to history
        6. Repeat
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
                    model='gemini-2.5-flash-preview-05-20',
                    contents=self.history,
                    config=self.config
                )
            except Exception as e:
                logging.error(f"Gemini API error: {e}")
                await self._send_ws("error", {"message": str(e)})
                self.active = False
                break

            # 2. Store response in history
            if response.candidates and len(response.candidates) > 0:
                self.history.append(response.candidates[0].content)

            # 3. Parse JSON response
            result = self._parse_response(response)

            # 4. Broadcast reasoning to frontend
            if result.get('reasoning'):
                print(f"[Gemini] Reasoning: {result['reasoning'][:100]}...")
                await self._send_ws("reasoning", {"text": result['reasoning']})

            # 5. Check for task completion
            if result.get('task_complete'):
                print(f"[Session] Task complete: {result.get('summary', 'No summary')}")
                self.active = False
                await self._send_ws("task_complete", {
                    "success": True,
                    "summary": result.get('summary', 'Task completed')
                })
                break

            # 6. Get and execute next action
            next_action = result.get('next_action')
            if not next_action:
                print("[Session] No next action - ending loop")
                await self._send_ws("task_complete", {
                    "success": False,
                    "summary": "Model did not provide a next action"
                })
                self.active = False
                break

            func_name = next_action.get('function', 'unknown')
            func_args = next_action.get('args', {})

            print(f"[Gemini] Action: {func_name}({func_args})")

            # Notify frontend
            await self._send_ws("next_action", {
                "function": func_name,
                "args": func_args,
                "step": self.step
            })

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

            # 9. Build feedback and add to history
            feedback_parts = self._build_feedback(next_action, exec_result, new_images)
            self.history.append(types.Content(role='user', parts=feedback_parts))

        # Loop ended - check why
        if self.step >= self.MAX_STEPS and self.active:
            logging.warning(f"Task exceeded max steps ({self.MAX_STEPS})")
            await self._send_ws("task_complete", {
                "success": False,
                "reason": "max_steps_exceeded",
                "summary": f"Task terminated after {self.MAX_STEPS} steps"
            })

    def _parse_response(self, response) -> dict:
        """Parse JSON response from model."""
        result = {
            'reasoning': '',
            'next_action': None,
            'task_complete': False,
            'summary': ''
        }

        try:
            text = response.text if response.text else ''

            if not text:
                logging.warning("Empty response from model")
                return result

            # Handle markdown fencing
            if "```json" in text:
                start = text.find("```json") + 7
                end = text.find("```", start)
                if end > start:
                    text = text[start:end].strip()
            elif "```" in text:
                start = text.find("```") + 3
                end = text.find("```", start)
                if end > start:
                    text = text[start:end].strip()

            # Parse JSON
            parsed = json.loads(text)

            result['reasoning'] = parsed.get('reasoning', '')
            result['next_action'] = parsed.get('next_action')
            result['task_complete'] = parsed.get('task_complete', False)
            result['summary'] = parsed.get('summary', '')

        except (json.JSONDecodeError, ValueError) as e:
            logging.error(f"JSON parse error: {e}")
            logging.error(f"Raw response: {response.text[:500] if response.text else 'None'}...")
            result['reasoning'] = f"Parse error: {e}"
            result['task_complete'] = True  # Fail safe

        return result

    def _build_feedback(self, action: dict, result: dict, images: dict) -> list:
        """Build feedback content parts for next iteration."""
        parts = []

        # Text feedback
        func_name = action.get('function', 'unknown')
        func_args = action.get('args', {})
        success = result.get('status') != 'error'

        feedback_text = f"""EXECUTION RESULT:
Function: {func_name}
Arguments: {json.dumps(func_args)}
Success: {success}
"""
        if result.get('message'):
            feedback_text += f"Message: {result['message']}\n"
        if result.get('status') == 'error':
            feedback_text += f"Error: {result.get('error', 'Unknown')}\n"
        if result.get('final_position'):
            feedback_text += f"Final position: {result['final_position']}\n"

        feedback_text += "\nNEW CAMERA IMAGES show current state. Analyze and decide next action."

        parts.append(types.Part.from_text(text=feedback_text))

        # Add images
        for cam_name, b64 in images.items():
            if b64:
                img_bytes = base64.b64decode(b64)
                parts.append(types.Part.from_bytes(data=img_bytes, mime_type="image/jpeg"))

        return parts

    async def _execute_tool(self, name: str, args: dict) -> dict:
        """Execute a tool function."""
        try:
            if name == "move_arm":
                return await asyncio.to_thread(
                    move_arm,
                    args.get('arm_id'),
                    args.get('position'),
                    args.get('moving_time', 1.5)
                )
            elif name == "control_gripper":
                return await asyncio.to_thread(
                    control_gripper,
                    args.get('arm_id'),
                    args.get('action')
                )
            elif name == "finish_task":
                return finish_task(args.get('success'), args.get('summary'))
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
            await asyncio.get_event_loop().run_in_executor(
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

        for arm_id, arm_stack in arm_controllers.items():
            arm_ctrl = arm_stack['arm']
            gripper_ctrl = arm_stack['gripper']

            # Close gripper first
            await asyncio.get_event_loop().run_in_executor(
                None,
                lambda gc=gripper_ctrl: gc.close_gripper()
            )

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
    print("\n" + "=" * 60)
    print("  Gemini Robotics ER Bridge (Chat API)")
    print("  WebSocket-only | Port 8082")
    print("=" * 60)

    # Initialize hardware at startup
    initialize_hardware()

    # Create and run app
    app = make_app()
    app.on_shutdown.append(on_shutdown)

    print("\n[Bridge] Starting WebSocket server on port 8082...")
    print("[Bridge] Connect via: ws://localhost:8082/ws")
    print("[Bridge] Messages: task_request, robot_connect, robot_disconnect, status_request\n")

    web.run_app(app, port=8082, print=None)