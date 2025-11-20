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
import time
import base64
import uuid
import asyncio
from pathlib import Path
from aiohttp import web
from aiohttp_cors import setup, ResourceOptions
from dotenv import load_dotenv

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from dynamixel_controller import DynamixelController
from models.vx300s_model import VX300S
from controllers.arm_controller import ArmController
from controllers.gripper_controller import GripperController
from controllers.camera_controller import CameraController

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
        print(f"  ✓ Camera controller ready (gripper_cam + top_cam)")

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

    # Shutdown all arms
    for arm_id, arm_stack in arm_controllers.items():
        try:
            dxl_controller = arm_stack.get('dxl')
            if dxl_controller:
                dxl_controller.disable_torque()
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
    Execute a robot function from Gemini's response.

    Args:
        next_action: Dict with 'function' and 'args' keys

    Returns:
        Execution result dict with success, function, args, and result data
    """
    if not next_action:
        return {'success': False, 'error': 'No action provided'}

    function_name = next_action.get('function')
    args = next_action.get('args', {})

    result = {
        'success': False,
        'function': function_name,
        'args': args
    }

    try:
        if function_name == 'move_arm':
            position = args.get('position')
            pose = args.get('pose')
            moving_time = args.get('moving_time', 1.5)

            if pose:
                # Move to named pose
                move_result = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: arm_controller.move_to_pose(pose, moving_time=moving_time, blocking=True)
                )
                result['success'] = move_result.get('success', False)
                result['message'] = f"Moved to pose '{pose}'"
            elif position:
                # Move to Cartesian position
                x, y, z = position
                move_result = await asyncio.get_event_loop().run_in_executor(
                    None,
                    lambda: arm_controller.move_to_position(x, y, z, moving_time=moving_time, blocking=True)
                )
                result['success'] = move_result.get('success', False)
                result['new_position'] = position
                result['message'] = f"Moved to position [{x:.3f}, {y:.3f}, {z:.3f}]"
            else:
                result['error'] = 'move_arm requires either position or pose argument'

        elif function_name == 'control_gripper':
            action = args.get('action')

            if action == 'open':
                gripper_result = await asyncio.get_event_loop().run_in_executor(
                    None, gripper_controller.open_gripper
                )
                result['success'] = gripper_result.get('success', False)
                result['gripper_state'] = 'open'
                result['message'] = 'Gripper opened'
            elif action == 'close':
                gripper_result = await asyncio.get_event_loop().run_in_executor(
                    None, gripper_controller.close_gripper
                )
                result['success'] = gripper_result.get('success', False)
                result['gripper_state'] = 'closed'
                result['message'] = 'Gripper closed'
            else:
                result['error'] = f'Invalid gripper action: {action}'

        elif function_name == 'get_arm_status':
            state = await asyncio.get_event_loop().run_in_executor(
                None, arm_controller.get_arm_state
            )
            result['success'] = True
            result['arm_state'] = state
            result['message'] = 'Arm status retrieved'

        elif function_name == 'get_gripper_status':
            state = await asyncio.get_event_loop().run_in_executor(
                None, gripper_controller.get_gripper_state
            )
            result['success'] = True
            result['gripper_state'] = state
            result['message'] = 'Gripper status retrieved'

        elif function_name == 'capture_camera_frame':
            # Camera frames are captured automatically after each action
            result['success'] = True
            result['message'] = f"Camera frames captured ({args.get('reason', 'unknown')})"

        else:
            result['error'] = f'Unknown function: {function_name}'

    except Exception as e:
        result['error'] = str(e)
        print(f"[ER Bridge] Function execution error: {e}")
        import traceback
        traceback.print_exc()

    return result


def capture_camera_images() -> list:
    """
    Capture images from both cameras.

    Returns:
        List of two base64-encoded JPEG images [gripper_cam, top_cam]
    """
    images = []

    try:
        # Capture gripper camera (first) - get base64 encoded JPEG
        gripper_img = camera_controller.get_frame_base64('gripper_cam')
        images.append(gripper_img if gripper_img else '')

        # Capture top/overhead camera (second) - get base64 encoded JPEG
        top_img = camera_controller.get_frame_base64('top_cam')
        images.append(top_img if top_img else '')

        print(f"[ER Bridge] Captured camera images: gripper={len(gripper_img) if gripper_img else 0}B, top={len(top_img) if top_img else 0}B")

    except Exception as e:
        print(f"[ER Bridge] Camera capture error: {e}")
        images = ['', '']

    return images


def get_current_robot_state() -> dict:
    """
    Get current robot state for context.

    Returns:
        Dict with joints, end_effector_position, gripper_position
    """
    state = {
        'joints': [],
        'end_effector_position': {'x': 0, 'y': 0, 'z': 0},
        'gripper_position': 0
    }

    try:
        # Get arm state
        arm_state = arm_controller.get_arm_state()
        if arm_state:
            state['joints'] = list(arm_state.get('joint_angles', []))
            ee_pos = arm_state.get('end_effector_position', {})
            state['end_effector_position'] = {
                'x': ee_pos.get('x', 0),
                'y': ee_pos.get('y', 0),
                'z': ee_pos.get('z', 0)
            }

        # Get gripper state
        gripper_state = gripper_controller.get_gripper_state()
        if gripper_state:
            state['gripper_position'] = gripper_state.get('position', 0)

    except Exception as e:
        print(f"[ER Bridge] Error getting robot state: {e}")

    return state


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
    print(f"Images: {len(images)} frames captured (gripper_cam, top_cam)")

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

    workspace_bounds = robot_model.workspace_limits if robot_model else {
        'x': (0.15, 0.50),
        'y': (-0.30, 0.30),
        'z': (0.05, 0.40)
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

WORKSPACE BOUNDS (relative to robot base, in meters):
- X: {workspace_bounds.get('x', [0.1, 0.6])} (X+ forward, X- backward)
- Y: {workspace_bounds.get('y', [-0.3, 0.3])} (Y+ left, Y- right)
- Z: {workspace_bounds.get('z', [0.05, 0.55])} (Z+ up from table)

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

AVAILABLE ROBOT FUNCTIONS:

def move_arm(position: list[float] = None, pose: str = None, moving_time: float = 1.5):
    '''Move robot end effector to target position or named pose.

    Args:
        position: Target position [x, y, z] in meters (use this OR pose)
        pose: Named pose - "home" [0, 0, 0.5], "ready" [0.3, 0, 0.3], or "sleep" [0, 0, 0.1] (use this OR position)
        moving_time: Time to complete movement in seconds (default: 1.5)

    Example: {{"function": "move_arm", "args": {{"position": [0.3, -0.1, 0.2]}}}}
    Example: {{"function": "move_arm", "args": {{"pose": "home"}}}}
    '''

def control_gripper(action: str):
    '''Open or close the robot gripper.

    Args:
        action: "open" or "close"

    Example: {{"function": "control_gripper", "args": {{"action": "open"}}}}
    '''

def get_arm_status():
    '''Get current arm state (joints, position, pose).

    Returns: Dictionary with joint angles, end-effector position, current pose

    Example: {{"function": "get_arm_status", "args": {{}}}}
    '''

def get_gripper_status():
    '''Get current gripper state.

    Returns: Dictionary with gripper position and state

    Example: {{"function": "get_gripper_status", "args": {{}}}}
    '''

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


async def handle_status(request: web.Request) -> web.Response:
    """Status endpoint"""
    robot_initialized = dynamixel_controller is not None
    return web.json_response({
        'bridge': 'simple_er_iterative',
        'sdk': 'google-genai',
        'status': 'running',
        'mode': 'iterative_with_visual_verification',
        'active_conversations': len(conversations),
        'api_key_set': bool(os.getenv('GEMINI_API_KEY')),
        'robot_initialized': robot_initialized,
        'robot_connected': robot_connected
    })


async def handle_robot_connect(request: web.Request) -> web.Response:
    """
    Connect to robot and perform opening ceremony.

    This moves the arm slowly to ready position and prepares the gripper.
    Should be called when user clicks "Connect Robot" button.
    """
    global robot_connected

    if not dynamixel_controller:
        return web.json_response({
            'success': False,
            'error': 'Robot hardware not initialized'
        }, status=500)

    if robot_connected:
        return web.json_response({
            'success': False,
            'error': 'Robot already connected',
            'state': 'connected'
        }, status=400)

    try:
        print("\n" + "=" * 60)
        print("[ER Bridge] 🎬 OPENING CEREMONY - Connecting Robot")
        print("=" * 60)

        # Perform opening ceremony (move arm to ready position slowly)
        print("\n[Opening Ceremony] Moving arm to ready position...")
        ceremony_result = await asyncio.get_event_loop().run_in_executor(
            None,
            lambda: arm_controller.opening_ceremony(moving_time=4.0, blocking=True)
        )

        if not ceremony_result.get('success'):
            error_msg = ceremony_result.get('error', 'Opening ceremony failed')
            print(f"[Opening Ceremony] ✗ Failed: {error_msg}")
            return web.json_response({
                'success': False,
                'error': error_msg,
                'state': 'disconnected'
            }, status=500)

        # Open gripper to neutral position
        print("[Opening Ceremony] Opening gripper...")
        gripper_result = await asyncio.get_event_loop().run_in_executor(
            None,
            gripper_controller.open_gripper
        )

        robot_connected = True

        print("\n" + "=" * 60)
        print("[ER Bridge] ✓ Opening ceremony complete - Robot ready!")
        print("=" * 60 + "\n")

        # Get current state
        arm_state = arm_controller.get_arm_state()
        gripper_state = gripper_controller.get_gripper_state()

        return web.json_response({
            'success': True,
            'state': 'connected',
            'message': 'Robot connected and ready',
            'arm_state': arm_state,
            'gripper_state': gripper_state
        })

    except Exception as e:
        print(f"[ER Bridge] ✗ Opening ceremony error: {e}")
        import traceback
        traceback.print_exc()
        return web.json_response({
            'success': False,
            'error': str(e),
            'state': 'disconnected'
        }, status=500)


async def handle_robot_disconnect(request: web.Request) -> web.Response:
    """
    Disconnect from robot and perform closing ceremony.

    This moves the arm slowly to sleep position and keeps torque ON
    so the arm holds its position safely.
    Should be called when user clicks "Disconnect Robot" button.
    """
    global robot_connected

    if not dynamixel_controller:
        return web.json_response({
            'success': False,
            'error': 'Robot hardware not initialized'
        }, status=500)

    if not robot_connected:
        return web.json_response({
            'success': False,
            'error': 'Robot not connected',
            'state': 'disconnected'
        }, status=400)

    try:
        print("\n" + "=" * 60)
        print("[ER Bridge] 🎬 CLOSING CEREMONY - Disconnecting Robot")
        print("=" * 60)

        # Close gripper first
        print("\n[Closing Ceremony] Closing gripper...")
        await asyncio.get_event_loop().run_in_executor(
            None,
            gripper_controller.close_gripper
        )

        # Perform closing ceremony (move arm to sleep position slowly)
        print("[Closing Ceremony] Moving arm to sleep position...")
        ceremony_result = await asyncio.get_event_loop().run_in_executor(
            None,
            lambda: arm_controller.closing_ceremony(moving_time=4.0, blocking=True)
        )

        if not ceremony_result.get('success'):
            error_msg = ceremony_result.get('error', 'Closing ceremony failed')
            print(f"[Closing Ceremony] ✗ Failed: {error_msg}")
            return web.json_response({
                'success': False,
                'error': error_msg,
                'state': 'connected'
            }, status=500)

        robot_connected = False

        print("\n" + "=" * 60)
        print("[ER Bridge] ✓ Closing ceremony complete - Robot in sleep position")
        print("[ER Bridge] ℹ️  Torque remains ON to hold position safely")
        print("=" * 60 + "\n")

        # Get final state
        arm_state = arm_controller.get_arm_state()
        gripper_state = gripper_controller.get_gripper_state()

        return web.json_response({
            'success': True,
            'state': 'disconnected',
            'message': 'Robot disconnected - arm in sleep position with torque on',
            'arm_state': arm_state,
            'gripper_state': gripper_state
        })

    except Exception as e:
        print(f"[ER Bridge] ✗ Closing ceremony error: {e}")
        import traceback
        traceback.print_exc()
        return web.json_response({
            'success': False,
            'error': str(e),
            'state': 'connected'
        }, status=500)


async def handle_robot_status(request: web.Request) -> web.Response:
    """
    Get current robot connection status and state.
    """
    if not dynamixel_controller:
        return web.json_response({
            'connected': False,
            'initialized': False,
            'error': 'Robot hardware not initialized'
        })

    try:
        # Get current states
        arm_state = arm_controller.get_arm_state() if arm_controller else None
        gripper_state = gripper_controller.get_gripper_state() if gripper_controller else None

        return web.json_response({
            'connected': robot_connected,
            'initialized': True,
            'arm_state': arm_state,
            'gripper_state': gripper_state
        })

    except Exception as e:
        return web.json_response({
            'connected': robot_connected,
            'initialized': True,
            'error': str(e)
        })


async def handle_camera_frame(request: web.Request) -> web.Response:
    """Get a frame from a specific camera."""
    global camera_controller

    camera_name = request.match_info.get('camera_name', 'gripper_cam')

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
        'cameras': ['gripper_cam', 'top_cam'] if camera_controller.initialized else []
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
    app.router.add_get('/status', handle_status)
    app.router.add_post('/robot/connect', handle_robot_connect)
    app.router.add_post('/robot/disconnect', handle_robot_disconnect)
    app.router.add_get('/robot/status', handle_robot_status)

    # Camera endpoints (for frontend compatibility)
    app.router.add_get('/camera/info', handle_camera_info)
    app.router.add_get('/camera/{camera_name}/frame', handle_camera_frame)

    # Add CORS to routes
    for route in list(app.router.routes()):
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
    print("  - Automatic camera capture (gripper_cam + top_cam)")
    print("  - Iterative visual verification")
    print("  - Conversation state maintained")
    print()

    if not os.getenv('GEMINI_API_KEY'):
        print("WARNING: GEMINI_API_KEY environment variable not set!")
        print("Set it in your shell or .env file")
        print()

    # Initialize robot hardware
    if not args.no_robot:
        if not initialize_robot_sync():
            print("\nERROR: Failed to initialize robot hardware!")
            print("Run with --no-robot flag to start without hardware.")
            sys.exit(1)
    else:
        print("WARNING: Running without robot hardware (--no-robot flag)")
        print()

    print(f"Starting server on http://localhost:{args.port}")
    print("Endpoints:")
    print("  - POST /initialize        (multi-arm detection and initialization)")
    print("  - POST /robotics-er-request")
    print("      Initial: {prompt}")
    print("      Feedback: {conversation_id, execution_result}")
    print("  - GET /status")
    print("  - POST /robot/connect     (opening ceremony)")
    print("  - POST /robot/disconnect  (closing ceremony)")
    print("  - GET  /robot/status")
    print()

    try:
        web.run_app(make_app(), host='0.0.0.0', port=args.port)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        shutdown_robot()
