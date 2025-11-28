#!/usr/bin/env python3
"""
Simple REST bridge for Gemini Robotics ER testing with iterative visual verification.

Implements Google's 4-step function calling pattern:
1. Define function declarations (in prompt)
2. Call LLM with current camera images
3. Execute ONE function at a time (frontend responsibility)
4. Return execution result + NEW images back to model

Key features:
- Conversation state maintained across API calls
- Visual verification after each execution step
- Model can verify, adjust, and retry based on camera feedback
- Returns ONE function call at a time for step-by-step execution

Documentation: https://ai.google.dev/gemini-api/docs/robotics-overview
"""

import os
import sys
import json
import logging
import time
import base64
import uuid
import asyncio
from pathlib import Path
from aiohttp import web, WSMsgType
from aiohttp_cors import setup, ResourceOptions
from dotenv import load_dotenv

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

# Robot hardware imports - optional, only needed when running with actual robot
# These will be None if imports fail (e.g., missing dynamixel_sdk)
DynamixelController = None
VX300S = None
ArmController = None
GripperController = None
CameraController = None

try:
    from dynamixel_controller import DynamixelController
    from models.vx300s_model import VX300S
    from controllers.arm_controller import ArmController
    from controllers.gripper_controller import GripperController
    from controllers.camera_controller import CameraController
    ROBOT_HARDWARE_AVAILABLE = True
except ImportError as e:
    print(f"[Bridge] Robot hardware modules not available: {e}")
    print("[Bridge] Running in no-robot mode only")
    ROBOT_HARDWARE_AVAILABLE = False

# Load .env file from project root
env_path = Path(__file__).parent.parent / '.env'
load_dotenv(dotenv_path=env_path)

# In-memory conversation storage
# Key: conversation_id, Value: {history, client, task, step, created_at}
conversations = {}

# Multi-arm controller management
# Structure: arm_id -> {'dxl': DynamixelController, 'arm': ArmController,
#                       'gripper': GripperController, 'port': str, 'model': VX300S}
arm_controllers = {}

# Shared resources (independent of specific arms)
camera_controller = None

# System state
robot_connected = False  # Track if robot has completed opening ceremony

# Camera order for Gemini (left-to-right visual layout)
CAMERA_ORDER = ['left_gripper', 'overhead_camera', 'right_gripper']

# WebSocket client management
ws_clients: set = set()  # Connected WebSocket clients
ws_sequence = 0  # Message sequence number for ordering


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
        print(f"[WebSocket] Send error: {e}")
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
            print(f"[WebSocket] Broadcast error: {e}")
            disconnected.add(ws)

    ws_clients.difference_update(disconnected)


def build_tool_definitions(connected_arms: list) -> str:
    """
    Generate tool definitions with available arm(s) listed dynamically.

    Args:
        connected_arms: List of connected arm IDs (e.g., ['follower_right', 'follower_left'])

    Returns:
        String containing tool definitions for the system prompt
    """
    if not connected_arms:
        return "NO ARMS CONNECTED - Robot functions unavailable"

    arm_list = ', '.join(f'"{a}"' for a in connected_arms)
    arm_count = len(connected_arms)
    arm_plural = "arm" if arm_count == 1 else "arms"
    first_arm = connected_arms[0]

    return f'''CONNECTED ARM(S): [{arm_list}]

AVAILABLE ROBOT FUNCTIONS:

def move_arm(arm: str, position: list[float] = None, pose: str = None, moving_time: float = 1.5):
    \'\'\'Move robot end effector to target position or named pose.

    Args:
        arm: Target {arm_plural} - one of: {arm_list} (REQUIRED)
        position: Target position [x, y, z] in meters (use this OR pose)
        pose: Named pose - "home" [0, 0, 0.5], "ready" [0.3, 0, 0.3], or "sleep" [0, 0, 0.1] (use this OR position)
        moving_time: Time to complete movement in seconds (default: 1.5)

    Example: {{"function": "move_arm", "args": {{"arm": "{first_arm}", "position": [0.3, -0.1, 0.2]}}}}
    Example: {{"function": "move_arm", "args": {{"arm": "{first_arm}", "pose": "home"}}}}
    \'\'\'

def control_gripper(arm: str, action: str):
    \'\'\'Open or close the robot gripper.

    Args:
        arm: Target {arm_plural} - one of: {arm_list} (REQUIRED)
        action: "open" or "close"

    Example: {{"function": "control_gripper", "args": {{"arm": "{first_arm}", "action": "open"}}}}
    \'\'\'

def get_arm_status(arm: str):
    \'\'\'Get current arm state (joints, position, pose).

    Args:
        arm: Target {arm_plural} - one of: {arm_list} (REQUIRED)

    Returns: Dictionary with joint angles, end-effector position, current pose

    Example: {{"function": "get_arm_status", "args": {{"arm": "{first_arm}"}}}}
    \'\'\'

def get_gripper_status(arm: str):
    \'\'\'Get current gripper state.

    Args:
        arm: Target {arm_plural} - one of: {arm_list} (REQUIRED)

    Returns: Dictionary with gripper position and state

    Example: {{"function": "get_gripper_status", "args": {{"arm": "{first_arm}"}}}}
    \'\'\'
'''


async def initialize_robot_handler(request):
    """
    Initialize robot system - detect and initialize all available follower arms.

    Returns JSON with initialization results and connected arms.
    """
    global arm_controllers, camera_controller

    try:
        print("\n" + "=" * 60)
        print("[ER Bridge] Initializing Robot Hardware (Multi-Arm)")
        print("=" * 60)

        # Detect follower arms
        from dynamixel_controller import DynamixelController
        detected_arms = DynamixelController.detect_follower_ports()

        print(f"\n[Detection] Found {len(detected_arms)} follower arm(s)")
        for arm_info in detected_arms:
            print(f"  - {arm_info['arm_id']}: {arm_info['port']}")

        if not detected_arms:
            error_msg = "No follower arms detected. Check USB connections and udev rules."
            print(f"\n[ER Bridge] ✗ {error_msg}")
            return web.json_response({
                "success": False,
                "error": error_msg,
                "connected_arms": [],
                "arm_count": 0
            }, status=400)

        # Initialize each detected follower arm
        config_path = Path(__file__).parent.parent / 'config' / 'vx300s.yaml'
        arm_controllers = {}
        init_results = {}

        for arm_info in detected_arms:
            port = arm_info['port']
            arm_id = arm_info['arm_id']

            print(f"\n[{arm_id}] Initializing controller stack...")

            try:
                # Create robot model
                print(f"  [1/4] Loading robot model...")
                robot_model = VX300S()
                robot_info = robot_model.get_info()
                print(f"    ✓ VX300S model loaded (6-DOF, {robot_info['total_reach']:.2f}m reach)")

                # Initialize DynamixelController for this arm
                print(f"  [2/4] Connecting to Dynamixel bus at {port}...")
                dxl_controller = DynamixelController(
                    port=port,
                    baudrate=1000000,
                    config_file=str(config_path)
                )

                if not dxl_controller.initialize_motors():
                    init_results[arm_id] = False
                    logging.error(f"Failed to initialize motors for {arm_id}")
                    print(f"    ✗ Failed to initialize motors")
                    continue

                dxl_controller.enable_torque()
                dxl_controller.start_monitoring(frequency=10)
                print(f"    ✓ Dynamixel controller connected, torque enabled, monitoring at 10Hz")

                # Initialize ArmController
                print(f"  [3/4] Initializing arm controller...")
                arm_ctrl = ArmController(
                    dynamixel_controller=dxl_controller,
                    robot_model=robot_model,
                    enable_safety=True,
                    dry_run=False
                )

                if not arm_ctrl.initialize_without_movement():
                    init_results[arm_id] = False
                    logging.error(f"Failed to initialize arm controller for {arm_id}")
                    print(f"    ✗ Failed to initialize arm controller")
                    continue

                print(f"    ✓ Arm controller ready")

                # Initialize GripperController
                print(f"  [4/4] Initializing gripper controller...")
                gripper_ctrl = GripperController(
                    dynamixel_controller=dxl_controller,
                    dry_run=False
                )

                if not gripper_ctrl.initialize():
                    init_results[arm_id] = False
                    logging.error(f"Failed to initialize gripper controller for {arm_id}")
                    print(f"    ✗ Failed to initialize gripper controller")
                    continue

                print(f"    ✓ Gripper controller ready")

                # Store controller stack
                arm_controllers[arm_id] = {
                    'dxl': dxl_controller,
                    'arm': arm_ctrl,
                    'gripper': gripper_ctrl,
                    'port': port,
                    'model': robot_model
                }

                init_results[arm_id] = True
                logging.info(f"✓ Successfully initialized {arm_id} on {port}")
                print(f"  ✓ {arm_id} fully initialized")

            except Exception as e:
                init_results[arm_id] = False
                logging.error(f"✗ Failed to initialize {arm_id}: {e}")
                print(f"  ✗ {arm_id} initialization failed: {e}")
                import traceback
                traceback.print_exc()

        # Initialize cameras (independent of arms)
        print(f"\n[Cameras] Initializing camera controller...")
        camera_controller = CameraController()
        camera_init = camera_controller.initialize()
        if camera_init:
            print(f"  Camera controller ready: {camera_controller.get_camera_names()}")
        else:
            print(f"  Camera controller failed to initialize")

        # Return status
        connected_arms = [arm_id for arm_id, success in init_results.items() if success]

        print("\n" + "=" * 60)
        print(f"[ER Bridge] ✓ Initialized {len(connected_arms)} of {len(detected_arms)} detected arm(s)")
        print(f"[ER Bridge] Connected arms: {', '.join(connected_arms)}")
        print("[ER Bridge] ℹ️  Use /robot/connect to perform opening ceremony")
        print("=" * 60 + "\n")

        return web.json_response({
            "success": len(connected_arms) > 0,
            "initialization_results": init_results,
            "connected_arms": connected_arms,
            "arm_count": len(connected_arms),
            "camera_initialized": camera_init,
            "message": f"Initialized {len(connected_arms)} of {len(detected_arms)} detected arm(s)"
        })

    except Exception as e:
        logging.error(f"Robot initialization failed: {e}")
        print(f"\n[ER Bridge] ✗ Robot initialization failed: {e}")
        import traceback
        traceback.print_exc()
        return web.json_response({
            "success": False,
            "error": str(e),
            "connected_arms": [],
            "arm_count": 0
        }, status=500)


def initialize_robot_sync():
    """
    Synchronous wrapper for initialize_robot_handler() for use at startup.

    Returns:
        True if initialization successful, False otherwise
    """
    import asyncio

    # Create a fake request object
    class FakeRequest:
        pass

    try:
        # Run the async handler
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        result = loop.run_until_complete(initialize_robot_handler(FakeRequest()))
        loop.close()

        # Extract success status from JSON response
        if hasattr(result, 'body'):
            import json
            body = json.loads(result.body)
            return body.get('success', False)
        return False
    except Exception as e:
        print(f"Error during synchronous initialization: {e}")
        return False


def shutdown_robot():
    """Safely shutdown robot controllers."""
    global arm_controllers, camera_controller

    print("\n[ER Bridge] Shutting down robot...")

    # All motor IDs: 1-8 arm joints + 9 gripper
    ALL_MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9]

    # Shutdown all arms - explicitly disable torque on ALL motors
    for arm_id, arm_stack in arm_controllers.items():
        try:
            dxl_controller = arm_stack.get('dxl')
            if dxl_controller:
                # Explicitly disable torque on all motors including gripper
                dxl_controller.disable_torque(ALL_MOTOR_IDS)
                dxl_controller.close()
                print(f"  ✓ {arm_id} controller shutdown")
        except Exception as e:
            print(f"  ✗ {arm_id} shutdown error: {e}")

    # Shutdown camera controller
    try:
        if camera_controller:
            camera_controller.shutdown()
            print("  ✓ Camera controller shutdown")
    except Exception as e:
        print(f"  ✗ Camera shutdown error: {e}")

    print("[ER Bridge] Shutdown complete\n")


async def execute_robot_function(next_action: dict) -> dict:
    """
    Execute a robot function on specified arm.

    Args:
        next_action: Dict with 'function' and 'args' keys
                    args must include 'arm' parameter for robot functions

    Returns:
        Execution result dict with success, function, args, arm, and result data
    """
    if not next_action:
        return {'success': False, 'error': 'No action provided'}

    # Check if robot is initialized
    if not arm_controllers:
        return {
            'success': False,
            'error': 'Robot not initialized',
            'available_arms': []
        }

    function_name = next_action.get('function')
    args = next_action.get('args', {})

    result = {
        'success': False,
        'function': function_name,
        'args': args
    }

    try:
        # Camera functions don't require arm parameter
        if function_name == 'capture_camera_frame':
            result['success'] = True
            result['message'] = f"Camera frames captured ({args.get('reason', 'unknown')})"
            return result

        # All other functions require arm parameter
        arm_id = args.get('arm')
        if not arm_id:
            return {
                'success': False,
                'error': 'Missing required "arm" parameter',
                'available_arms': list(arm_controllers.keys())
            }

        # Get controller stack for specified arm
        arm_stack = arm_controllers.get(arm_id)
        if not arm_stack:
            return {
                'success': False,
                'error': f'Arm "{arm_id}" not found',
                'available_arms': list(arm_controllers.keys())
            }

        # Extract controllers from stack
        arm_ctrl = arm_stack['arm']
        gripper_ctrl = arm_stack['gripper']

        # Route to appropriate function
        if function_name == 'move_arm':
            position = args.get('position')
            pose = args.get('pose')
            moving_time = args.get('moving_time', 1.5)

            if pose:
                # Move to named pose
                move_result = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: arm_ctrl.move_to_pose(pose, moving_time=moving_time, blocking=True)
                )
                result['success'] = move_result.get('success', False)
                result['arm'] = arm_id
                result['message'] = f"Moved {arm_id} to pose '{pose}'"
            elif position:
                # Move to Cartesian position
                x, y, z = position
                move_result = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: arm_ctrl.move_to_position(position=[x, y, z], moving_time=moving_time, blocking=True)
                )
                result['success'] = move_result.get('success', False)
                result['arm'] = arm_id
                result['new_position'] = position
                if move_result.get('success'):
                    result['message'] = f"Moved {arm_id} to position [{x:.3f}, {y:.3f}, {z:.3f}]"
                else:
                    result['message'] = move_result.get('error', 'Move failed')
                    result['error'] = move_result.get('error')
            else:
                result['error'] = 'move_arm requires either position or pose argument'
                result['arm'] = arm_id

        elif function_name == 'control_gripper':
            action = args.get('action')

            if action == 'open':
                gripper_result = await asyncio.get_event_loop().run_in_executor(
                    None, gripper_ctrl.open_gripper
                )
                result['success'] = gripper_result.get('success', False)
                result['arm'] = arm_id
                result['gripper_state'] = 'open'
                result['message'] = f'{arm_id} gripper opened'
            elif action == 'close':
                gripper_result = await asyncio.get_event_loop().run_in_executor(
                    None, gripper_ctrl.close_gripper
                )
                result['success'] = gripper_result.get('success', False)
                result['arm'] = arm_id
                result['gripper_state'] = 'closed'
                result['message'] = f'{arm_id} gripper closed'
            else:
                result['error'] = f'Invalid gripper action: {action}'
                result['arm'] = arm_id

        elif function_name == 'get_arm_status':
            state = await asyncio.get_event_loop().run_in_executor(
                None, arm_ctrl.get_arm_state
            )
            result['success'] = True
            result['arm'] = arm_id
            result['arm_state'] = state
            result['message'] = f'{arm_id} status retrieved'

        elif function_name == 'get_gripper_status':
            state = await asyncio.get_event_loop().run_in_executor(
                None, gripper_ctrl.get_gripper_state
            )
            result['success'] = True
            result['arm'] = arm_id
            result['gripper_state'] = state
            result['message'] = f'{arm_id} gripper status retrieved'

        else:
            result['error'] = f'Unknown function: {function_name}'

    except Exception as e:
        result['error'] = str(e)
        if 'arm' in locals():
            result['arm'] = arm_id
        print(f"[ER Bridge] Function execution error: {e}")
        import traceback
        traceback.print_exc()

    return result


def capture_camera_images() -> dict:
    """
    Capture images from all available cameras.

    Returns:
        Dict mapping camera names to base64-encoded JPEG images
    """
    images = {}

    try:
        if camera_controller and camera_controller.initialized:
            # Get all available camera frames dynamically
            images = camera_controller.get_all_frames_base64()

            # Log capture results in explicit order (left-to-right)
            if images:
                sizes = [f"{name}={len(images[name])}B" for name in CAMERA_ORDER if name in images]
                print(f"[ER Bridge] Captured camera images: {', '.join(sizes)}")
            else:
                print("[ER Bridge] No camera frames available")
        else:
            print("[ER Bridge] Camera controller not initialized")

    except Exception as e:
        print(f"[ER Bridge] Camera capture error: {e}")
        images = {}

    return images


def get_current_robot_state() -> dict:
    """
    Get current robot state for all connected arms.

    Returns:
        Dict with per-arm state and backward-compatible top-level keys:
        - arms: {arm_id: {joints, end_effector_position, gripper_position}}
        - joints, end_effector_position, gripper_position: first arm's state (backward compat)
    """
    state = {
        'arms': {},
        'joints': [],
        'end_effector_position': {'x': 0, 'y': 0, 'z': 0},
        'gripper_position': 0
    }

    if not arm_controllers:
        return state

    try:
        first_arm_processed = False

        for arm_id, stack in arm_controllers.items():
            try:
                arm_ctrl = stack['arm']
                gripper_ctrl = stack['gripper']

                arm_state = arm_ctrl.get_arm_state()
                gripper_state = gripper_ctrl.get_gripper_state()

                joints = list(arm_state.get('joint_angles', [])) if arm_state else []
                ee_pos = arm_state.get('end_effector_position', {}) if arm_state else {}
                gripper_pos = gripper_state.get('position', 0) if gripper_state else 0

                state['arms'][arm_id] = {
                    'joints': joints,
                    'end_effector_position': {
                        'x': ee_pos.get('x', 0),
                        'y': ee_pos.get('y', 0),
                        'z': ee_pos.get('z', 0)
                    },
                    'gripper_position': gripper_pos
                }

                # Set top-level keys from first arm for backward compatibility
                if not first_arm_processed:
                    state['joints'] = joints
                    state['end_effector_position'] = {
                        'x': ee_pos.get('x', 0),
                        'y': ee_pos.get('y', 0),
                        'z': ee_pos.get('z', 0)
                    }
                    state['gripper_position'] = gripper_pos
                    first_arm_processed = True

            except Exception as e:
                print(f"[ER Bridge] Error getting state for {arm_id}: {e}")
                state['arms'][arm_id] = {'error': str(e)}

    except Exception as e:
        print(f"[ER Bridge] Error getting robot state: {e}")

    return state


# =============================================================================
# WebSocket Handler and Message Routing
# =============================================================================

async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    """
    Main WebSocket endpoint for unified client communication.

    Handles:
    - Task requests and execution
    - Camera frame streaming
    - Robot status updates
    - Connection lifecycle
    """
    ws = web.WebSocketResponse(heartbeat=30.0)
    await ws.prepare(request)

    # Register client
    ws_clients.add(ws)
    client_id = id(ws)
    print(f"[WebSocket] Client {client_id} connected. Total: {len(ws_clients)}")

    # Send connection info
    await send_ws(ws, 'connection_info', {
        'cameras': camera_controller.get_camera_names() if camera_controller and camera_controller.initialized else [],
        'connected_arms': list(arm_controllers.keys()),
        'robot_connected': robot_connected
    })

    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                    await handle_ws_message(ws, data)
                except json.JSONDecodeError as e:
                    await send_ws(ws, 'error', {'code': 'PARSE_ERROR', 'message': str(e)})
            elif msg.type == WSMsgType.ERROR:
                print(f"[WebSocket] Client {client_id} error: {ws.exception()}")
    finally:
        ws_clients.discard(ws)
        print(f"[WebSocket] Client {client_id} disconnected. Total: {len(ws_clients)}")

    return ws


async def handle_ws_message(ws: web.WebSocketResponse, data: dict):
    """Route incoming WebSocket messages to appropriate handlers."""
    msg_type = data.get('type')
    payload = data.get('payload', {})

    print(f"[WebSocket] Received: {msg_type}")

    if msg_type == 'task_request':
        await handle_ws_task_request(ws, payload)
    elif msg_type == 'robot_connect':
        await handle_ws_robot_connect(ws)
    elif msg_type == 'robot_disconnect':
        await handle_ws_robot_disconnect(ws)
    elif msg_type == 'status_request':
        await handle_ws_status_request(ws)
    else:
        await send_ws(ws, 'error', {
            'code': 'UNKNOWN_MESSAGE',
            'message': f'Unknown message type: {msg_type}'
        })


async def handle_ws_task_request(ws: web.WebSocketResponse, payload: dict):
    """Handle task request via WebSocket - stream results back."""
    prompt = payload.get('prompt', '')
    if not prompt:
        await send_ws(ws, 'error', {'code': 'INVALID_PROMPT', 'message': 'Missing prompt'})
        return

    print(f"\n{'=' * 60}")
    print(f"[WebSocket] NEW TASK: {prompt}")
    print(f"{'=' * 60}")

    # Capture and broadcast camera images (dynamic dict of camera_name -> base64)
    images = capture_camera_images()
    await send_ws(ws, 'camera_frame', images)

    # Get current robot state
    current_state = get_current_robot_state()

    # Check API key
    api_key = os.getenv('GEMINI_API_KEY')
    if not api_key:
        await send_ws(ws, 'error', {'code': 'NO_API_KEY', 'message': 'GEMINI_API_KEY not set'})
        return

    # Initialize Gemini client
    client = genai.Client(api_key=api_key)
    conversation_id = str(uuid.uuid4())

    # Build context prompt (reuse existing logic)
    camera_names = list(images.keys()) if images else []
    context_prompt = build_context_prompt(prompt, current_state, camera_names)

    # Build content parts
    content_parts = [types.Part(text=context_prompt)]

    # Add images in explicit order (left-to-right visual layout)
    for camera_name in CAMERA_ORDER:
        if camera_name in images:
            img_b64 = images[camera_name]
            if img_b64 and img_b64.strip():
                image_bytes = base64.b64decode(img_b64)
                image_part = types.Part.from_bytes(data=image_bytes, mime_type='image/jpeg')
                content_parts.append(image_part)

    # Initialize conversation history
    conversation_history = [
        types.Content(role='user', parts=content_parts)
    ]

    # Store conversation
    conversations[conversation_id] = {
        'history': conversation_history,
        'client': client,
        'task': prompt,
        'step': 0,
        'created_at': time.time(),
        'ws': ws  # Track which WebSocket initiated this
    }

    # Execute task loop
    await execute_task_loop(ws, conversation_id)


async def execute_task_loop(ws: web.WebSocketResponse, conversation_id: str):
    """Execute task steps in a loop, streaming results via WebSocket."""
    if conversation_id not in conversations:
        await send_ws(ws, 'error', {'code': 'NO_CONVERSATION', 'message': 'Conversation not found'})
        return

    conv = conversations[conversation_id]
    max_steps = 5  # Safety limit

    while conv['step'] < max_steps:
        conv['step'] += 1
        step = conv['step']

        print(f"\n[WebSocket] Executing step {step}...")

        # Call Gemini
        try:
            response = conv['client'].models.generate_content(
                model='gemini-robotics-er-1.5-preview',
                contents=conv['history'],
                config=types.GenerateContentConfig(
                    temperature=0.1,
                    thinking_config=types.ThinkingConfig(thinking_budget=0)
                )
            )
        except Exception as e:
            await send_ws(ws, 'error', {'code': 'GEMINI_ERROR', 'message': str(e)})
            break

        # Parse response
        result = parse_iterative_response(response)

        # Store assistant response in history
        if response.candidates and len(response.candidates) > 0:
            conv['history'].append(response.candidates[0].content)

        # Send reasoning
        if result.get('reasoning'):
            await send_ws(ws, 'reasoning', {
                'text': result['reasoning'],
                'step': step,
                'conversation_id': conversation_id
            })

        # Check if task complete
        if result.get('task_complete', False):
            await send_ws(ws, 'task_complete', {
                'conversation_id': conversation_id,
                'total_steps': step
            })
            # Cleanup
            conv['client'].close()
            del conversations[conversation_id]
            print(f"[WebSocket] Task complete after {step} steps")
            break

        # Execute next action
        next_action = result.get('next_action')
        if not next_action:
            await send_ws(ws, 'task_complete', {
                'conversation_id': conversation_id,
                'total_steps': step,
                'reason': 'no_action'
            })
            break

        # Send action notification
        await send_ws(ws, 'next_action', {
            'function': next_action.get('function'),
            'args': next_action.get('args', {}),
            'step': step
        })

        # Execute the function
        execution_result = await execute_robot_function(next_action)

        # Send execution result
        await send_ws(ws, 'execution_result', {
            'success': execution_result.get('success', False),
            'message': execution_result.get('message', ''),
            'step': step,
            'function': next_action.get('function'),
            'details': execution_result
        })

        # Capture new images after execution (dynamic dict)
        images = capture_camera_images()
        await send_ws(ws, 'camera_frame', images)

        # Build feedback for next iteration
        feedback_text = f"""EXECUTION RESULT (Step {step}):
Function: {next_action.get('function')}
Arguments: {json.dumps(next_action.get('args', {}), indent=2)}
Success: {execution_result.get('success', False)}
"""
        if execution_result.get('message'):
            feedback_text += f"Message: {execution_result['message']}\n"
        if not execution_result.get('success') and execution_result.get('error'):
            feedback_text += f"ERROR: {execution_result['error']}\n"

        feedback_text += "\nNEW CAMERA IMAGES (above) show current state. Verify and decide next action."

        # Add feedback to history (images in explicit order, left-to-right)
        feedback_parts = [types.Part(text=feedback_text)]
        for camera_name in CAMERA_ORDER:
            if camera_name in images:
                img_b64 = images[camera_name]
                if img_b64 and img_b64.strip():
                    image_bytes = base64.b64decode(img_b64)
                    feedback_parts.append(types.Part.from_bytes(data=image_bytes, mime_type='image/jpeg'))

        conv['history'].append(types.Content(role='user', parts=feedback_parts))

    else:
        # Max steps reached
        await send_ws(ws, 'error', {
            'code': 'MAX_STEPS',
            'message': f'Task exceeded maximum {max_steps} steps'
        })


async def handle_ws_robot_connect(ws: web.WebSocketResponse):
    """Handle robot connect via WebSocket."""
    global robot_connected

    if not arm_controllers:
        await send_ws(ws, 'error', {'code': 'NO_ROBOT', 'message': 'Robot not initialized'})
        return

    if robot_connected:
        await send_ws(ws, 'robot_status', {'connected': True, 'message': 'Already connected'})
        return

    try:
        print("\n[WebSocket] 🎬 OPENING CEREMONY")

        # Perform opening ceremony on ALL arms
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
        await send_ws(ws, 'error', {'code': 'CONNECT_ERROR', 'message': str(e)})


async def handle_ws_robot_disconnect(ws: web.WebSocketResponse):
    """Handle robot disconnect via WebSocket."""
    global robot_connected

    if not arm_controllers:
        await send_ws(ws, 'error', {'code': 'NO_ROBOT', 'message': 'Robot not initialized'})
        return

    if not robot_connected:
        await send_ws(ws, 'robot_status', {'connected': False, 'message': 'Already disconnected'})
        return

    try:
        print("\n[WebSocket] 🎬 CLOSING CEREMONY")

        # Perform closing ceremony on ALL arms
        for arm_id, arm_stack in arm_controllers.items():
            arm_ctrl = arm_stack['arm']
            gripper_ctrl = arm_stack['gripper']

            # Close gripper
            await asyncio.get_event_loop().run_in_executor(
                None,
                lambda gc=gripper_ctrl: gc.close_gripper()
            )

            print(f"  [{arm_id}] Moving to sleep position...")
            ceremony_result = await asyncio.get_event_loop().run_in_executor(
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
        await send_ws(ws, 'error', {'code': 'DISCONNECT_ERROR', 'message': str(e)})


async def handle_ws_status_request(ws: web.WebSocketResponse):
    """Handle status request via WebSocket."""
    await send_ws(ws, 'robot_status', {
        'connected': robot_connected,
        'connected_arms': list(arm_controllers.keys()),
        'cameras': camera_controller.get_camera_names() if camera_controller and camera_controller.initialized else []
    })


def _build_camera_descriptions(camera_names: list) -> str:
    """Build dynamic camera descriptions based on available cameras."""
    if not camera_names:
        return "No cameras available."

    names_str = ", ".join(camera_names)
    return f"You will receive {len(camera_names)} camera image(s). Each image has a label identifying the camera: {names_str}"


def build_context_prompt(prompt: str, current_state: dict, camera_names: list = None) -> str:
    """Build the context prompt for Gemini (extracted for reuse)."""
    joint_limits_deg = {
        'waist': '[-180°, 180°]',
        'shoulder': '[-108°, 114°]',
        'elbow': '[-123°, 92°]',
        'forearm_roll': '[-180°, 180°]',
        'wrist_angle': '[-100°, 123°]',
        'wrist_rotate': '[-180°, 180°]'
    }

    link_lengths = {
        'base_height': 0.08915,
        'shoulder_offset': 0.050,
        'upper_arm': 0.200,
        'elbow_offset': 0.050,
        'forearm': 0.200,
        'wrist_to_gripper': 0.065,
        'gripper_fingers': 0.025
    }

    robot_base = [0, 0, 0]

    current_joints = current_state.get('joints', [])
    current_joints_deg = [f"{j * 57.2958:.1f}°" for j in current_joints] if current_joints else []

    ee_pos = current_state.get('end_effector_position', {})
    ee_pos_str = f"[{ee_pos.get('x', 0):.3f}, {ee_pos.get('y', 0):.3f}, {ee_pos.get('z', 0):.3f}]" if ee_pos else "unknown"

    gripper_pos = current_state.get('gripper_position', 0)
    gripper_state_str = "open" if gripper_pos > 0.025 else "closed"

    return f"""NEW TASK: {prompt}

═══════════════════════════════════════════════════════════
ROBOT SPECIFICATIONS - ViperX 300s
═══════════════════════════════════════════════════════════

MODEL INFO:
- Manufacturer: Trossen Robotics
- Type: 6-DOF robotic arm
- Degrees of Freedom: 6
- Maximum Reach: 0.75m from base
- Payload Capacity: 0.75kg

JOINT LIMITS (in degrees):
- Waist (base rotation): {joint_limits_deg['waist']}
- Shoulder: {joint_limits_deg['shoulder']}
- Elbow: {joint_limits_deg['elbow']}
- Forearm Roll: {joint_limits_deg['forearm_roll']}
- Wrist Angle: {joint_limits_deg['wrist_angle']}
- Wrist Rotate: {joint_limits_deg['wrist_rotate']}

LINK LENGTHS (in meters):
- Base height: {link_lengths['base_height']:.5f}m
- Shoulder offset: {link_lengths['shoulder_offset']:.3f}m
- Upper arm: {link_lengths['upper_arm']:.3f}m
- Elbow offset: {link_lengths['elbow_offset']:.3f}m
- Forearm: {link_lengths['forearm']:.3f}m
- Wrist to gripper: {link_lengths['wrist_to_gripper']:.3f}m
- Gripper fingers: {link_lengths['gripper_fingers']:.3f}m

GRIPPER SPECIFICATIONS:
- Type: parallel_jaw
- Opening range: [0.000m, 0.074m]
- Max force: 30.0N (approximate)

COORDINATE SYSTEM:
- Convention: Right-handed coordinate system
- X-axis: Forward (X+ away from robot base, X- toward base)
- Y-axis: Left/Right (Y+ to robot's left, Y- to robot's right)
- Z-axis: Vertical (Z+ upward from table, Z=0 at table level)
- Units: meters
- Robot base position in world: {robot_base}

═══════════════════════════════════════════════════════════
CURRENT ROBOT STATE
═══════════════════════════════════════════════════════════

CURRENT JOINTS (degrees): {current_joints_deg}
END-EFFECTOR POSITION: {ee_pos_str} meters (relative to robot base)
GRIPPER STATE: {gripper_state_str} (position: {gripper_pos:.3f}m)

═══════════════════════════════════════════════════════════
CAMERA SETUP
═══════════════════════════════════════════════════════════

{_build_camera_descriptions(camera_names or [])}

{build_tool_definitions(list(arm_controllers.keys()))}

CRITICAL - ITERATIVE EXECUTION WITH VISUAL VERIFICATION:
You execute tasks ONE STEP AT A TIME with visual feedback:

1. Analyze current camera images
2. Return ONE function call for the NEXT step only
3. I will execute it and send you NEW camera images + execution result
4. You verify the result visually in the NEW images
5. If successful, continue to next step
6. If failed, adjust and retry
7. Repeat until task complete

RESPONSE FORMAT:
Return JSON with ONE function call:
{{
    "reasoning": "What I see in current images and why this step is needed",
    "next_action": {{
        "function": "move_arm",
        "args": {{"arm": "follower_right", "position": [0.3, 0.1, 0.24]}}
    }},
    "verification_check": "After execution, I expect gripper 10cm above red cube",
    "task_complete": false
}}

When task is fully complete:
{{
    "reasoning": "Verification complete - object is grasped and lifted as shown in images",
    "next_action": null,
    "verification_check": null,
    "task_complete": true
}}
"""


try:
    from google import genai
    from google.genai import types
except ImportError as e:
    print("=" * 60)
    print("ERROR: google-genai SDK not installed")
    print("=" * 60)
    print()
    print("This bridge requires the google-genai package.")
    print("Install it with:")
    print()
    print("    pip install google-genai")
    print()
    print("Documentation: https://googleapis.github.io/python-genai/")
    print("=" * 60)
    raise ImportError("google-genai package is required") from e


async def handle_er_request(request: web.Request) -> web.Response:
    """
    Handle Robotics ER request with conversation support.

    Two modes:
    1. Initial request (no conversation_id): Start new task
       Body: {"prompt": str, "images": [...], "context": {...}}

    2. Feedback request (has conversation_id): Continue with execution result
       Body: {"conversation_id": str, "execution_result": {...}, "images": [...]}
    """
    request_start_time = time.perf_counter()

    try:
        data = await request.json()

        # Check if this is initial request or feedback
        conversation_id = data.get('conversation_id')

        if conversation_id:
            # Feedback mode: continue existing conversation
            return await handle_feedback(conversation_id, data, request_start_time)
        else:
            # Initial mode: start new task
            return await handle_initial(data, request_start_time)

    except Exception as e:
        print(f"\n[Simple ER Bridge] ERROR: {e}")
        import traceback
        traceback.print_exc()

        return web.json_response({
            'success': False,
            'error': str(e)
        }, status=500)


async def handle_initial(data: dict, request_start_time: float) -> web.Response:
    """Handle initial task request - start new conversation."""

    # Validate required fields
    prompt = data.get('prompt', '')
    if not prompt or not isinstance(prompt, str):
        return web.json_response({
            'success': False,
            'error': 'Missing or invalid "prompt" field (must be non-empty string)'
        }, status=400)

    print(f"\n{'=' * 60}")
    print(f"[Simple ER Bridge] NEW TASK")
    print(f"{'=' * 60}")
    print(f"Prompt: {prompt}")

    # Capture camera images automatically
    images = capture_camera_images()
    print(f"Images: {len(images)} camera(s) captured: {list(images.keys())}")

    # Get current robot state automatically
    current_state = get_current_robot_state()

    api_key = os.getenv('GEMINI_API_KEY')
    if not api_key:
        return web.json_response({
            'success': False,
            'error': 'GEMINI_API_KEY environment variable not set'
        }, status=500)

    # Initialize client
    client = genai.Client(api_key=api_key)

    # Create conversation ID
    conversation_id = str(uuid.uuid4())

    # Build robot specs from actual robot model
    joint_limits_deg = {
        'waist': '[-180°, 180°]',
        'shoulder': '[-108°, 114°]',
        'elbow': '[-123°, 92°]',
        'forearm_roll': '[-180°, 180°]',
        'wrist_angle': '[-100°, 123°]',
        'wrist_rotate': '[-180°, 180°]'
    }

    link_lengths = {
        'base_height': 0.08915,
        'shoulder_offset': 0.050,
        'upper_arm': 0.200,
        'elbow_offset': 0.050,
        'forearm': 0.200,
        'wrist_to_gripper': 0.065,
        'gripper_fingers': 0.025
    }

    robot_base = [0, 0, 0]

    # Get current joint angles (in radians from actual robot)
    current_joints = current_state.get('joints', [])
    current_joints_deg = [f"{j * 57.2958:.1f}°" for j in current_joints] if current_joints else []

    # Get end effector position
    ee_pos = current_state.get('end_effector_position', {})
    ee_pos_str = f"[{ee_pos.get('x', 0):.3f}, {ee_pos.get('y', 0):.3f}, {ee_pos.get('z', 0):.3f}]" if ee_pos else "unknown"

    # Get gripper state
    gripper_pos = current_state.get('gripper_position', 0)
    gripper_state_str = "open" if gripper_pos > 0.025 else "closed"

    # Build initial context prompt
    context_prompt = f"""NEW TASK: {prompt}

═══════════════════════════════════════════════════════════
ROBOT SPECIFICATIONS - ViperX 300s
═══════════════════════════════════════════════════════════

MODEL INFO:
- Manufacturer: Trossen Robotics
- Type: 6-DOF robotic arm
- Degrees of Freedom: 6
- Maximum Reach: 0.75m from base
- Payload Capacity: 0.75kg

JOINT LIMITS (in degrees):
- Waist (base rotation): {joint_limits_deg['waist']}
- Shoulder: {joint_limits_deg['shoulder']}
- Elbow: {joint_limits_deg['elbow']}
- Forearm Roll: {joint_limits_deg['forearm_roll']}
- Wrist Angle: {joint_limits_deg['wrist_angle']}
- Wrist Rotate: {joint_limits_deg['wrist_rotate']}

LINK LENGTHS (in meters):
- Base height: {link_lengths['base_height']:.5f}m
- Shoulder offset: {link_lengths['shoulder_offset']:.3f}m
- Upper arm: {link_lengths['upper_arm']:.3f}m
- Elbow offset: {link_lengths['elbow_offset']:.3f}m
- Forearm: {link_lengths['forearm']:.3f}m
- Wrist to gripper: {link_lengths['wrist_to_gripper']:.3f}m
- Gripper fingers: {link_lengths['gripper_fingers']:.3f}m

GRIPPER SPECIFICATIONS:
- Type: parallel_jaw
- Opening range: [0.000m, 0.074m]
- Max force: 30.0N (approximate)

COORDINATE SYSTEM:
- Convention: Right-handed coordinate system
- X-axis: Forward (X+ away from robot base, X- toward base)
- Y-axis: Left/Right (Y+ to robot's left, Y- to robot's right)
- Z-axis: Vertical (Z+ upward from table, Z=0 at table level)
- Units: meters
- Robot base position in world: {robot_base}

═══════════════════════════════════════════════════════════
CURRENT ROBOT STATE
═══════════════════════════════════════════════════════════

CURRENT JOINTS (degrees): {current_joints_deg}
END-EFFECTOR POSITION: {ee_pos_str} meters (relative to robot base)
GRIPPER STATE: {gripper_state_str} (position: {gripper_pos:.3f}m)

═══════════════════════════════════════════════════════════
CAMERA SETUP
═══════════════════════════════════════════════════════════

You will receive 2 camera images with each request IN THIS EXACT ORDER:

IMAGE 1 (FIRST IMAGE) - GRIPPER CAMERA:
- Location: Mounted on robot end-effector (gripper base)
- Field of View: 70° FOV
- Orientation: Looking down from gripper at ~155° angle
- Purpose: VERIFY GRASPS - Check if objects are IN the gripper
- Use for: Confirming successful grasps, detecting objects in gripper

IMAGE 2 (SECOND IMAGE) - OVERHEAD CAMERA:
- Location: Bird's-eye view above workspace at [0, -0.3, 1.0]
- Field of View: 60° FOV
- Orientation: Looking down at entire workspace
- Purpose: SPATIAL REASONING - Localize objects, plan trajectories
- Use for: Object detection, position estimation, collision avoidance

CRITICAL: Images will always appear in the order above. First image = gripper view, Second image = overhead view.
          Use GRIPPER CAMERA to verify if an object is grasped.
          Use OVERHEAD CAMERA for spatial relationships and planning.

{build_tool_definitions(list(arm_controllers.keys()))}

def capture_camera_frame(reason: str):
    '''Capture fresh camera frames from gripper and overhead cameras.

    CRITICAL: Call this AFTER robot movements to see updated scene state.
    Always capture new frames before planning your next action to ensure
    you are analyzing the CURRENT state, not a stale view.

    Args:
        reason: Why you need fresh frames. Examples:
            - "verify_movement" - Check if robot reached target position
            - "inspect_object" - Get clear view of object for planning
            - "check_grasp" - Verify object is secured in gripper
            - "detect_objects" - Find and localize objects in scene
            - "final_verification" - Confirm task completion

    Returns: New camera images will be included in next response

    Example: {{"function": "capture_camera_frame", "args": {{"reason": "check_grasp"}}}}
    '''

CRITICAL - ITERATIVE EXECUTION WITH VISUAL VERIFICATION:
You execute tasks ONE STEP AT A TIME with visual feedback:

1. Analyze current camera images
2. Return ONE function call for the NEXT step only
3. I will execute it and send you NEW camera images + execution result
4. You verify the result visually in the NEW images
5. If successful, continue to next step
6. If failed (e.g., object not grasped), adjust and retry
7. Repeat until task complete

VISUAL VERIFICATION IS CRITICAL:
After EACH execution, you will receive:
- NEW camera images showing current robot state
- Execution result (position reached, gripper state, etc.)

YOU MUST:
- Check NEW images to verify robot is where expected
- Verify objects are positioned correctly
- For grasp attempts: VERIFY object is IN gripper before lifting
- If verification fails in images, adjust approach and retry

═══════════════════════════════════════════════════════════
GRASP VERIFICATION PROTOCOL - MANDATORY
═══════════════════════════════════════════════════════════

CRITICAL RULE: After ANY control_gripper("close") command, you MUST verify the grasp
in the GRIPPER CAMERA (first image) before proceeding to lift or move the object.

GRASP VERIFICATION CHECKLIST (check gripper camera - first image):
✓ Is the target object VISIBLE between the gripper jaws?
✓ Are the gripper fingers aligned around the object?
✓ Can you see the object's edges/surfaces inside the gripper?
✓ Is the object centered between the jaws?

VERIFICATION OUTCOMES:

1. GRASP SUCCESSFUL (object visible in gripper camera):
   - Reasoning: "Gripper camera shows [object] secured between jaws"
   - Next action: move_arm to lift position (e.g., Z + 0.15m)
   - Proceed with task

2. GRASP FAILED (object NOT visible in gripper camera):
   - Reasoning: "Gripper camera does NOT show [object] - grasp failed"
   - Next action: control_gripper("open") to release
   - Then: Reposition and retry grasp sequence

3. UNCLEAR VIEW (cannot determine from gripper camera):
   - Next action: capture_camera_frame(reason="verify_grasp")
   - Wait for fresh images, then re-evaluate

NEVER lift or move after closing gripper without explicit visual confirmation
that the object appears in the gripper camera view.

═══════════════════════════════════════════════════════════
EXAMPLE WORKFLOWS - FOLLOW THESE PATTERNS
═══════════════════════════════════════════════════════════

WORKFLOW 1: PICK AND PLACE (e.g., "Pick up red cube and move it to blue cube")

Step 1: DETECT & APPROACH
  - Analyze overhead camera (second image) to detect object positions
  - Reasoning: "Overhead camera shows red cube at [x, y]. Moving to approach position."
  - Action: move_arm to position above target (e.g., [0.3, 0.1, 0.30])
  [WAIT FOR EXECUTION + NEW IMAGES]

Step 2: DESCEND TO GRASP HEIGHT
  - Check NEW overhead camera - verify arm is positioned correctly
  - Reasoning: "Robot positioned above red cube. Descending to grasp height."
  - Action: move_arm to grasp height (e.g., [0.3, 0.1, 0.05] - just above object)
  [WAIT FOR EXECUTION + NEW IMAGES]

Step 3: OPEN GRIPPER (pre-grasp)
  - Reasoning: "At grasp height. Opening gripper to prepare for grasp."
  - Action: control_gripper("open")
  [WAIT FOR EXECUTION + NEW IMAGES]

Step 4: CLOSE GRIPPER (grasp attempt)
  - Check alignment in NEW images
  - Reasoning: "Gripper aligned with red cube. Closing gripper to grasp."
  - Action: control_gripper("close")
  [WAIT FOR EXECUTION + NEW IMAGES]

Step 5: VERIFY GRASP (MANDATORY - CHECK GRIPPER CAMERA)
  - CRITICAL: Examine GRIPPER CAMERA (first image) for object presence
  - Reasoning: "Checking gripper camera for grasp verification..."

  → If SUCCESS (object visible in gripper camera):
    - Reasoning: "✓ GRASP VERIFIED - Red cube is visible between gripper jaws in gripper camera."
    - Action: move_arm to lift position (e.g., [0.3, 0.1, 0.25])
    [PROCEED TO STEP 6]

  → If FAILED (object NOT in gripper camera):
    - Reasoning: "✗ GRASP FAILED - Red cube NOT visible in gripper camera. Retrying."
    - Action: control_gripper("open")
    [GO BACK TO STEP 2 after opening]

Step 6: TRANSPORT
  - Reasoning: "Object secured. Moving to target position near blue cube."
  - Action: move_arm to position above target (e.g., [0.3, -0.1, 0.25])
  [WAIT FOR EXECUTION + NEW IMAGES]

Step 7: DESCEND TO PLACE HEIGHT
  - Reasoning: "Positioned above blue cube. Descending to place."
  - Action: move_arm to place height (e.g., [0.3, -0.1, 0.08])
  [WAIT FOR EXECUTION + NEW IMAGES]

Step 8: RELEASE OBJECT
  - Reasoning: "At place position. Opening gripper to release red cube."
  - Action: control_gripper("open")
  [WAIT FOR EXECUTION + NEW IMAGES]

Step 9: RETREAT & HOME
  - Reasoning: "Object placed. Returning to home position."
  - Action: move_arm("home")
  [WAIT FOR EXECUTION + NEW IMAGES]

Step 10: VERIFY TASK COMPLETION
  - Check overhead camera - verify object was placed successfully
  - Reasoning: "Task complete - red cube successfully moved to blue cube position."
  - Action: null
  - task_complete: true

═══════════════════════════════════════════════════════════

WORKFLOW 2: GRASP RECOVERY (when grasp fails)

If Step 5 verification shows grasp failed:
  Step 5a: Open gripper
  Step 5b: Adjust position slightly (move_arm with small offset)
  Step 5c: Close gripper again (control_gripper("close"))
  Step 5d: Verify in gripper camera again
  Step 5e: If still failed after 2-3 attempts, report failure

REMEMBER:
- ALWAYS open gripper before closing (ensures clean grasp)
- ALWAYS verify in GRIPPER CAMERA (first image) after closing
- NEVER lift until object confirmed in gripper camera
- One function call at a time - wait for feedback between each step

Determine specific coordinates based on:
- Object positions detected in images
- Robot specifications (link lengths, joint limits)
- Workspace bounds provided
- Task requirements (clearance for safety, precision for grasping)

RESPONSE FORMAT:
Return JSON with ONE function call:
{{
    "reasoning": "What I see in current images and why this step is needed",
    "next_action": {{
        "function": "move_arm",
        "args": {{"position": [0.3, 0.1, 0.24]}}
    }},
    "verification_check": "After execution, I expect gripper 10cm above red cube",
    "task_complete": false
}}

When task is fully complete:
{{
    "reasoning": "Verification complete - object is grasped and lifted as shown in images",
    "next_action": null,
    "verification_check": null,
    "task_complete": true
}}

IMPORTANT RULES:
- Return ONE function call only (NOT an array!)
- Call capture_camera_frame AFTER movements to get fresh visual feedback
- Check NEW images BEFORE deciding next action
- For grasping: MUST verify object in gripper before lifting
- Be conservative - verify each critical step visually
- If camera view is unclear or outdated, call capture_camera_frame first
"""

    print(f"\nInitial context:\n{context_prompt}\n")

    # Build content parts - text must be wrapped in types.Part
    content_parts = [types.Part(text=context_prompt)]

    # Add images
    image_count = 0
    for i, img_b64 in enumerate(images):
        if img_b64 and img_b64.strip():
            image_bytes = base64.b64decode(img_b64)
            image_part = types.Part.from_bytes(
                data=image_bytes,
                mime_type='image/jpeg'
            )
            content_parts.append(image_part)
            image_count += 1
            print(f"Added image {image_count}")

    if image_count == 0:
        print("WARNING: No images - visual verification will not work properly!")

    # Initialize conversation history
    conversation_history = [
        types.Content(
            role='user',
            parts=content_parts
        )
    ]

    # Call Gemini ER (initial)
    print("\nCalling Gemini Robotics ER (initial)...")
    api_call_start_time = time.perf_counter()

    response = client.models.generate_content(
        model='gemini-robotics-er-1.5-preview',
        contents=conversation_history,
        config=types.GenerateContentConfig(
            temperature=0.1,
            thinking_config=types.ThinkingConfig(thinking_budget=0)
        )
    )

    api_call_end_time = time.perf_counter()
    api_call_duration = api_call_end_time - api_call_start_time

    print(f"\n{'=' * 60}")
    print(f"[Simple ER Bridge] Response received")
    print(f"{'=' * 60}")
    print(f"API call duration: {api_call_duration:.3f}s")

    # Parse response
    result = parse_iterative_response(response)

    # Store assistant response in history
    if response.candidates and len(response.candidates) > 0:
        conversation_history.append(response.candidates[0].content)

    # Store conversation state
    conversations[conversation_id] = {
        'history': conversation_history,
        'client': client,
        'task': prompt,
        'step': 1,
        'created_at': time.time()
    }

    print(f"\nConversation {conversation_id[:8]}... created")
    next_action = result.get('next_action')
    next_func = next_action.get('function', 'none') if next_action else 'none'
    print(f"Step 1 - Next action: {next_func}")
    print(f"Task complete: {result.get('task_complete', False)}")

    # Execute the function if there is one
    execution_result = None
    new_images = []
    if next_action and not result.get('task_complete', False):
        print(f"\n[ER Bridge] Executing: {next_func}")
        execution_result = await execute_robot_function(next_action)
        print(f"[ER Bridge] Execution result: {execution_result.get('success', False)}")

        # Capture fresh images after execution
        new_images = capture_camera_images()

    # Timing
    request_end_time = time.perf_counter()
    total_duration = request_end_time - request_start_time

    print(f"\n{'=' * 60}")
    print(f"[Timing] Total: {total_duration:.3f}s, API: {api_call_duration:.3f}s")
    print(f"{'=' * 60}\n")

    # Return response with conversation ID
    return web.json_response({
        'success': True,
        'conversation_id': conversation_id,
        'step': 1,
        'reasoning': result.get('reasoning', ''),
        'next_action': result.get('next_action'),
        'execution_result': execution_result,
        'images': new_images,
        'verification_check': result.get('verification_check', ''),
        'task_complete': result.get('task_complete', False)
    })


async def handle_feedback(conversation_id: str, data: dict, request_start_time: float) -> web.Response:
    """Handle feedback after executing a function - continue conversation."""

    # Get conversation
    if conversation_id not in conversations:
        return web.json_response({
            'success': False,
            'error': f'Conversation {conversation_id} not found or expired'
        }, status=404)

    conv = conversations[conversation_id]

    # Get the previous execution result from the data (sent by frontend from last response)
    prev_execution_result = data.get('execution_result', {})

    # Capture fresh images for this feedback
    images = capture_camera_images()

    print(f"\n{'=' * 60}")
    print(f"[Simple ER Bridge] FEEDBACK (conversation {conversation_id[:8]}...)")
    print(f"{'=' * 60}")
    print(f"Step: {conv['step'] + 1}")
    print(f"Previous function: {prev_execution_result.get('function', 'unknown')}")
    print(f"Previous success: {prev_execution_result.get('success', False)}")
    print(f"New images: {len(images)} captured")

    # Build feedback context
    feedback_text = f"""EXECUTION RESULT (Step {conv['step']}):

Function: {prev_execution_result.get('function', 'unknown')}
Arguments: {json.dumps(prev_execution_result.get('args', {}), indent=2)}
Success: {prev_execution_result.get('success', False)}
"""

    if prev_execution_result.get('success'):
        if 'new_position' in prev_execution_result:
            feedback_text += f"New end-effector position: {prev_execution_result['new_position']}\n"
        if 'gripper_state' in prev_execution_result:
            feedback_text += f"Gripper state: {prev_execution_result['gripper_state']}\n"
        if 'message' in prev_execution_result:
            feedback_text += f"Message: {prev_execution_result['message']}\n"
    else:
        feedback_text += f"ERROR: {prev_execution_result.get('error', 'Unknown error')}\n"

    feedback_text += """
NEW CAMERA IMAGES (above) show current robot state after execution.

VERIFICATION TASKS:
1. Analyze the NEW images - did the execution succeed as expected?
2. Check if robot/gripper is positioned correctly
3. If this was a grasp, verify object is IN the gripper
4. Decide next action based on visual verification

Provide next function call or mark task complete.
"""

    # Build content with NEW images - text must be wrapped in types.Part
    content_parts = [types.Part(text=feedback_text)]

    # Add fresh camera images
    image_count = 0
    for i, img_b64 in enumerate(images):
        if img_b64 and img_b64.strip():
            image_bytes = base64.b64decode(img_b64)
            image_part = types.Part.from_bytes(
                data=image_bytes,
                mime_type='image/jpeg'
            )
            content_parts.append(image_part)
            image_count += 1

    print(f"Added {image_count} new images for verification")

    # Add to conversation history
    conv['history'].append(
        types.Content(
            role='user',
            parts=content_parts
        )
    )

    # Call Gemini ER with conversation context
    print("\nCalling Gemini Robotics ER (feedback)...")
    api_call_start_time = time.perf_counter()

    response = conv['client'].models.generate_content(
        model='gemini-robotics-er-1.5-preview',
        contents=conv['history'],
        config=types.GenerateContentConfig(
            temperature=0.1,
            thinking_config=types.ThinkingConfig(thinking_budget=0)
        )
    )

    api_call_end_time = time.perf_counter()
    api_call_duration = api_call_end_time - api_call_start_time

    print(f"Response received in {api_call_duration:.2f}s")

    # Parse response
    result = parse_iterative_response(response)

    # Store assistant response
    if response.candidates and len(response.candidates) > 0:
        conv['history'].append(response.candidates[0].content)

    # Increment step
    conv['step'] += 1

    next_action = result.get('next_action')
    next_func = next_action.get('function', 'none') if next_action else 'none'
    print(f"\nStep {conv['step']} - Next action: {next_func}")
    print(f"Task complete: {result.get('task_complete', False)}")

    # Execute the function if there is one
    execution_result = None
    new_images = []
    if next_action and not result.get('task_complete', False):
        print(f"\n[ER Bridge] Executing: {next_func}")
        execution_result = await execute_robot_function(next_action)
        print(f"[ER Bridge] Execution result: {execution_result.get('success', False)}")

        # Capture fresh images after execution
        new_images = capture_camera_images()

    # Clean up if task complete
    if result.get('task_complete', False):
        print(f"Task complete - cleaning up conversation {conversation_id[:8]}...")
        conv['client'].close()
        del conversations[conversation_id]

    # Timing
    request_end_time = time.perf_counter()
    total_duration = request_end_time - request_start_time

    print(f"\n{'=' * 60}")
    print(f"[Timing] Total: {total_duration:.3f}s, API: {api_call_duration:.3f}s")
    print(f"{'=' * 60}\n")

    # Get current step before potential deletion
    current_step = conv['step'] if conversation_id in conversations else conv['step']

    return web.json_response({
        'success': True,
        'conversation_id': conversation_id,
        'step': current_step,
        'reasoning': result.get('reasoning', ''),
        'next_action': result.get('next_action'),
        'execution_result': execution_result,
        'images': new_images,
        'verification_check': result.get('verification_check', ''),
        'task_complete': result.get('task_complete', False)
    })


def parse_iterative_response(response) -> dict:
    """Parse iterative response expecting ONE function call."""
    result = {
        'reasoning': '',
        'next_action': None,
        'verification_check': '',
        'task_complete': False
    }

    try:
        if response.text:
            response_text = response.text

            # Handle markdown fencing
            if "```json" in response_text:
                json_start = response_text.find("```json") + 7
                json_end = response_text.find("```", json_start)
                response_text = response_text[json_start:json_end].strip()
            elif "```" in response_text:
                json_start = response_text.find("```") + 3
                json_end = response_text.find("```", json_start)
                response_text = response_text[json_start:json_end].strip()

            # Parse JSON
            parsed = json.loads(response_text)

            result['reasoning'] = parsed.get('reasoning', '')
            result['next_action'] = parsed.get('next_action')
            result['verification_check'] = parsed.get('verification_check', '')
            result['task_complete'] = parsed.get('task_complete', False)

            print(f"\nReasoning: {result['reasoning']}")
            if result['next_action']:
                print(f"Next action: {result['next_action'].get('function')} {result['next_action'].get('args', {})}")

    except (ValueError, json.JSONDecodeError) as e:
        print(f"Warning: Could not parse JSON response: {e}")
        print(f"Raw response (first 800 chars): {response.text[:800]}...")
        print(f"Response length: {len(response.text)} characters")
        # Try to salvage partial JSON
        try:
            error_pos = int(str(e).split('char ')[-1].rstrip(')'))
            partial_text = response.text[:error_pos]
            parsed = json.loads(partial_text + '}}')  # Attempt to close
            result['reasoning'] = parsed.get('reasoning', f"Parse error (recovered): {e}")
            result['next_action'] = parsed.get('next_action')
            print(f"Recovered partial response")
        except:
            result['reasoning'] = f"Parse error: {e}"
        result['task_complete'] = True  # Fail safe

    return result


async def handle_robot_status(request: web.Request) -> web.Response:
    """
    Get current robot status for all connected arms.

    Returns multi-arm status with consistent format for all scenarios:
    - No arms: success=false, error message, empty lists
    - One+ arms: success=true, connected_arms list, per-arm detailed state
    - Per-arm errors: arm marked as initialized=false with error message
    """
    # Case 1: Robot not initialized (empty arm_controllers)
    if not arm_controllers:
        return web.json_response({
            'success': False,
            'error': 'Robot not initialized',
            'connected_arms': [],
            'arm_count': 0
        })

    # Case 2+: At least one arm initialized
    try:
        arms_status = {}

        for arm_id, stack in arm_controllers.items():
            try:
                # Get arm state from this arm's controller
                arm_ctrl = stack['arm']
                state = arm_ctrl.get_arm_state()

                # Successful state read
                arms_status[arm_id] = {
                    'port': stack['port'],
                    'initialized': True,
                    'state': state
                }
            except Exception as e:
                # Per-arm error handling - mark this arm as failed but continue
                arms_status[arm_id] = {
                    'port': stack['port'],
                    'initialized': False,
                    'error': str(e)
                }

        return web.json_response({
            'success': True,
            'connected_arms': list(arm_controllers.keys()),
            'arm_count': len(arm_controllers),
            'arms': arms_status
        })

    except Exception as e:
        # Unexpected error at top level
        return web.json_response({
            'success': False,
            'error': str(e),
            'connected_arms': [],
            'arm_count': 0
        }, status=500)


async def handle_camera_frame(request: web.Request) -> web.Response:
    """Get a frame from a specific camera."""
    global camera_controller

    # Get camera name from request, default to first available camera
    camera_names = camera_controller.get_camera_names() if camera_controller and camera_controller.initialized else []
    default_camera = camera_names[0] if camera_names else 'camera_0'
    camera_name = request.match_info.get('camera_name', default_camera)

    if not camera_controller or not camera_controller.initialized:
        return web.json_response({
            'success': False,
            'error': 'Camera controller not initialized'
        }, status=503)

    # Get frame as base64 JPEG
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


async def handle_camera_info(request: web.Request) -> web.Response:
    """Get camera initialization status."""
    global camera_controller

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


async def on_cleanup(app):
    """Cleanup handler for graceful shutdown."""
    print("\n[ER Bridge] Application shutting down...")
    shutdown_robot()


def make_app() -> web.Application:
    """Create the application"""
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

    # Add routes
    app.router.add_post('/initialize', initialize_robot_handler)
    app.router.add_post('/robotics-er-request', handle_er_request)
    app.router.add_get('/robot/status', handle_robot_status)

    # Camera endpoints (for frontend compatibility)
    app.router.add_get('/camera/info', handle_camera_info)
    app.router.add_get('/camera/{camera_name}/frame', handle_camera_frame)

    # WebSocket endpoint (for unified real-time communication)
    app.router.add_get('/ws', websocket_handler)

    # Add CORS to HTTP routes only (WebSocket doesn't need CORS)
    for route in list(app.router.routes()):
        # Skip WebSocket route
        if route.resource and '/ws' not in str(route.resource):
            cors.add(route)

    # Register cleanup handler
    app.on_cleanup.append(on_cleanup)

    return app


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Gemini Robotics ER Bridge with Dynamixel Control')
    parser.add_argument('--port', type=int, default=8082, help='Server port (default: 8082)')
    parser.add_argument('--dxl-port', type=str, default='/dev/ttyDXL_follower_right', help='Dynamixel serial port (follower right arm)')
    parser.add_argument('--baudrate', type=int, default=1000000, help='Dynamixel baudrate')
    parser.add_argument('--no-robot', action='store_true', help='Run without robot hardware (for testing)')
    args = parser.parse_args()

    print("=" * 60)
    print("Gemini Robotics ER Bridge - Direct Dynamixel Control")
    print("=" * 60)
    print()
    print("Features:")
    print("  - Direct hardware control via Dynamixel SDK")
    print("  - Dynamic camera detection and capture")
    print("  - Iterative visual verification")
    print("  - Conversation state maintained")
    print()

    if not os.getenv('GEMINI_API_KEY'):
        print("WARNING: GEMINI_API_KEY environment variable not set!")
        print("Set it in your shell or .env file")
        print()

    # Initialize robot hardware
    if not args.no_robot:
        if not ROBOT_HARDWARE_AVAILABLE:
            print("\nERROR: Robot hardware modules not available!")
            print("Missing dependencies (dynamixel_sdk, etc.)")
            print("Run with --no-robot flag to start without hardware.")
            sys.exit(1)
        if not initialize_robot_sync():
            print("\nERROR: Failed to initialize robot hardware!")
            print("Run with --no-robot flag to start without hardware.")
            sys.exit(1)
    else:
        print("INFO: Running without robot hardware (--no-robot flag)")
        print("      WebSocket and API endpoints will work, but robot")
        print("      commands will return mock responses.")
        print()

    print(f"Starting server on http://localhost:{args.port}")
    print()
    print("HTTP Endpoints:")
    print("  - POST /initialize        (multi-arm detection and initialization)")
    print("  - POST /robotics-er-request")
    print("  - GET  /status")
    print("  - POST /robot/connect     (opening ceremony)")
    print("  - POST /robot/disconnect  (closing ceremony)")
    print("  - GET  /robot/status")
    print()
    print("WebSocket Endpoint:")
    print(f"  - ws://localhost:{args.port}/ws")
    print("    Messages: task_request, robot_connect, robot_disconnect, status_request")
    print()

    try:
        web.run_app(make_app(), host='0.0.0.0', port=args.port)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        shutdown_robot()
