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
import json
import time
import base64
import uuid
from pathlib import Path
from aiohttp import web
from aiohttp_cors import setup, ResourceOptions
from dotenv import load_dotenv

# Load .env file from project root
env_path = Path(__file__).parent.parent / '.env'
load_dotenv(dotenv_path=env_path)

# In-memory conversation storage
# Key: conversation_id, Value: {history, client, task, step, created_at}
conversations = {}

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

    images = data.get('images', [])
    if not isinstance(images, list):
        return web.json_response({
            'success': False,
            'error': 'Invalid "images" field (must be array)'
        }, status=400)

    # Validate expected camera count
    if len(images) != 2:
        print(f"⚠️  WARNING: Expected 2 images (gripper_cam, top_cam), got {len(images)}")
        print(f"    Camera order MUST be: [gripper_cam, top_cam]")

    context = data.get('context', {})
    if not isinstance(context, dict):
        return web.json_response({
            'success': False,
            'error': 'Invalid "context" field (must be object)'
        }, status=400)

    print(f"\n{'=' * 60}")
    print(f"[Simple ER Bridge] NEW TASK")
    print(f"{'=' * 60}")
    print(f"Prompt: {prompt}")
    print(f"Images: {len(images)} frames (expected: [gripper_cam, top_cam])")

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

    # Extract robot specs from context
    robot_specs = context.get('robot_specs', {})
    current_state = context.get('current_state', {})
    workspace_bounds = context.get('workspace_bounds', {})
    robot_base = context.get('robot_base_position', [])

    # Extract joint limits (convert to degrees for readability)
    joint_limits = robot_specs.get('joint_limits', {})
    joint_limits_deg = {}
    for joint_name, limits in joint_limits.items():
        if isinstance(limits, dict):
            min_rad = limits.get('min', 0)
            max_rad = limits.get('max', 0)
            joint_limits_deg[joint_name] = f"[{min_rad * 57.2958:.0f}°, {max_rad * 57.2958:.0f}°]"

    # Extract link lengths
    link_lengths = robot_specs.get('link_lengths', {})

    # Extract gripper specs
    gripper = robot_specs.get('gripper', {})

    # Get current joint angles (in radians from frontend)
    current_joints = current_state.get('joints', [])
    current_joints_deg = [f"{j * 57.2958:.1f}°" for j in current_joints] if current_joints else []

    # Get end effector position
    ee_pos = current_state.get('end_effector_position', {})
    ee_pos_str = f"[{ee_pos.get('x', 0):.3f}, {ee_pos.get('y', 0):.3f}, {ee_pos.get('z', 0):.3f}]" if ee_pos else "unknown"

    # Get gripper state
    gripper_pos = current_state.get('gripper_position', 0)
    gripper_state = "open" if gripper_pos > 0.025 else "closed"

    # Build initial context prompt
    context_prompt = f"""NEW TASK: {prompt}

═══════════════════════════════════════════════════════════
ROBOT SPECIFICATIONS - {robot_specs.get('model', 'ViperX 300s')}
═══════════════════════════════════════════════════════════

MODEL INFO:
- Manufacturer: {robot_specs.get('manufacturer', 'Trossen Robotics')}
- Type: {robot_specs.get('description', '6-DOF robotic arm')}
- Degrees of Freedom: {robot_specs.get('dof', 6)}
- Maximum Reach: {robot_specs.get('max_reach', 0.8)}m from base
- Payload Capacity: {robot_specs.get('payload_capacity', 0.75)}kg

JOINT LIMITS (in degrees):
- Waist (base rotation): {joint_limits_deg.get('waist', '[-180°, 180°]')}
- Shoulder: {joint_limits_deg.get('shoulder', '[-108°, 114°]')}
- Elbow: {joint_limits_deg.get('elbow', '[-123°, 92°]')}
- Forearm Roll: {joint_limits_deg.get('forearm_roll', '[-180°, 180°]')}
- Wrist Angle: {joint_limits_deg.get('wrist_angle', '[-100°, 123°]')}
- Wrist Rotate: {joint_limits_deg.get('wrist_rotate', '[-180°, 180°]')}

LINK LENGTHS (in meters):
- Base height: {link_lengths.get('base_height', 0.08915):.5f}m
- Shoulder offset: {link_lengths.get('shoulder_offset', 0.050):.3f}m
- Upper arm: {link_lengths.get('upper_arm', 0.200):.3f}m
- Elbow offset: {link_lengths.get('elbow_offset', 0.050):.3f}m
- Forearm: {link_lengths.get('forearm', 0.200):.3f}m
- Wrist to gripper: {link_lengths.get('wrist_to_gripper', 0.065):.3f}m
- Gripper fingers: {link_lengths.get('gripper_fingers', 0.025):.3f}m

GRIPPER SPECIFICATIONS:
- Type: {gripper.get('type', 'parallel_jaw')}
- Opening range: [{gripper.get('min_opening', 0.0):.3f}m, {gripper.get('max_opening', 0.074):.3f}m]
- Max force: {gripper.get('max_force', 30.0)}N (approximate)
- Left finger range: [{gripper.get('left_finger_range', {}).get('min', 0.015):.3f}m, {gripper.get('left_finger_range', {}).get('max', 0.037):.3f}m]
- Right finger range: [{gripper.get('right_finger_range', {}).get('min', -0.037):.3f}m, {gripper.get('right_finger_range', {}).get('max', -0.015):.3f}m]

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
GRIPPER STATE: {gripper_state} (position: {gripper_pos:.3f}m)

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

═══════════════════════════════════════════════════════════
OBJECT DETECTION WITH POINT MARKERS
═══════════════════════════════════════════════════════════

Point to all visible objects in both camera images.
Identify what you see and return 2D point coordinates with descriptive labels.

Format: {{"detections": [{{"point": [y, x], "label": "descriptive_name"}}]}}

Requirements:
- MANDATORY: Include detections array in every response
- Coordinates normalized to 0-1000 (integers only)
- Format: [y, x] - point at CENTER of each object
- Use descriptive labels for what you see
- Limit to 10 most relevant objects
- If no objects detected, return empty array: {{"detections": []}}

DETECTION INSTRUCTIONS:

1. OVERHEAD CAMERA (second image):
   - Point to all objects visible on the table surface
   - Use descriptive labels for what you see (e.g., "green cube", "blue cube", "red object")
   - Point to the center of each object
   - Identify objects by their visual appearance (color, shape, size)

2. GRIPPER CAMERA (first image):
   - Point to the robot's gripper fingers
   - Label them descriptively (e.g., "left gripper finger", "right gripper finger")
   - Point to any objects visible in or near the gripper
   - Point to center of each visible element

IMPORTANT:
- Describe what you actually SEE in the images
- Use natural descriptive labels
- Don't assume object names - identify by appearance
- Point coordinates should be at object centers
- Include gripper fingers in EVERY response

Example response:
{{
  "detections": [
    {{"point": [400, 300], "label": "green cube"}},
    {{"point": [500, 600], "label": "blue cube"}},
    {{"point": [800, 200], "label": "left gripper finger"}},
    {{"point": [800, 800], "label": "right gripper finger"}}
  ]
}}

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
Return JSON with ONE function call and MANDATORY detections:
{{
    "reasoning": "What I see in current images and why this step is needed",
    "detections": [
        {{"point": [400, 300], "label": "green cube"}},
        {{"point": [500, 600], "label": "blue cube"}},
        {{"point": [800, 200], "label": "left gripper finger"}},
        {{"point": [800, 800], "label": "right gripper finger"}}
    ],
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
    "detections": [],  // ALWAYS include, even if empty
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
        'detections': result.get('detections', []),
        'next_action': result.get('next_action'),
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

    # Validate execution result
    execution_result = data.get('execution_result', {})
    images = data.get('images', [])

    # Validate expected camera count
    if len(images) != 2:
        print(f"⚠️  WARNING: Expected 2 images (gripper_cam, top_cam), got {len(images)}")
        print(f"    Camera order MUST be: [gripper_cam, top_cam]")

    print(f"\n{'=' * 60}")
    print(f"[Simple ER Bridge] FEEDBACK (conversation {conversation_id[:8]}...)")
    print(f"{'=' * 60}")
    print(f"Step: {conv['step'] + 1}")
    print(f"Function executed: {execution_result.get('function', 'unknown')}")
    print(f"Success: {execution_result.get('success', False)}")
    print(f"New images: {len(images)} (expected: [gripper_cam, top_cam])")

    # Build feedback context
    feedback_text = f"""EXECUTION RESULT (Step {conv['step']}):

Function: {execution_result.get('function', 'unknown')}
Arguments: {json.dumps(execution_result.get('args', {}), indent=2)}
Success: {execution_result.get('success', False)}
"""

    if execution_result.get('success'):
        if 'new_position' in execution_result:
            feedback_text += f"New end-effector position: {execution_result['new_position']}\n"
        if 'gripper_state' in execution_result:
            feedback_text += f"Gripper state: {execution_result['gripper_state']}\n"
        if 'message' in execution_result:
            feedback_text += f"Message: {execution_result['message']}\n"
    else:
        feedback_text += f"ERROR: {execution_result.get('error', 'Unknown error')}\n"

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

    return web.json_response({
        'success': True,
        'conversation_id': conversation_id,
        'step': conv['step'],
        'reasoning': result.get('reasoning', ''),
        'detections': result.get('detections', []),
        'next_action': result.get('next_action'),
        'verification_check': result.get('verification_check', ''),
        'task_complete': result.get('task_complete', False)
    })


def parse_iterative_response(response) -> dict:
    """Parse iterative response expecting ONE function call and optional detections."""
    result = {
        'reasoning': '',
        'detections': [],
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
            result['detections'] = parsed.get('detections', [])
            result['next_action'] = parsed.get('next_action')
            result['verification_check'] = parsed.get('verification_check', '')
            result['task_complete'] = parsed.get('task_complete', False)

            print(f"\nReasoning: {result['reasoning']}")
            if result['detections']:
                print(f"Detections: {len(result['detections'])} objects")
                for det in result['detections']:
                    if 'point' in det:
                        print(f"  - {det.get('label', 'unknown')}: point {det.get('point', [])}")
                    elif 'box_2d' in det:
                        print(f"  - {det.get('label', 'unknown')}: box {det.get('box_2d', [])}")
            if result['next_action']:
                print(f"Next action: {result['next_action'].get('function')} {result['next_action'].get('args', {})}")

    except (ValueError, json.JSONDecodeError) as e:
        print(f"Warning: Could not parse JSON response: {e}")
        print(f"Raw response (first 800 chars): {response.text[:800]}...")
        print(f"Response length: {len(response.text)} characters")
        # Try to salvage partial JSON by finding the last complete object
        try:
            # Find the last complete JSON object by parsing up to the error position
            error_pos = int(str(e).split('char ')[-1].rstrip(')'))
            partial_text = response.text[:error_pos]
            # Try to close any open objects/arrays
            parsed = json.loads(partial_text + ']}}')  # Attempt to close
            result['reasoning'] = parsed.get('reasoning', f"Parse error (recovered): {e}")
            result['detections'] = parsed.get('detections', [])
            result['next_action'] = parsed.get('next_action')
            print(f"Recovered partial response with {len(result.get('detections', []))} detections")
        except:
            result['reasoning'] = f"Parse error: {e}"
        result['task_complete'] = True  # Fail safe

    return result


async def handle_status(request: web.Request) -> web.Response:
    """Status endpoint"""
    return web.json_response({
        'bridge': 'simple_er_iterative',
        'sdk': 'google-genai',
        'status': 'running',
        'mode': 'iterative_with_visual_verification',
        'active_conversations': len(conversations),
        'api_key_set': bool(os.getenv('GEMINI_API_KEY'))
    })


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
    app.router.add_post('/robotics-er-request', handle_er_request)
    app.router.add_get('/status', handle_status)

    # Add CORS to routes
    for route in list(app.router.routes()):
        cors.add(route)

    return app


if __name__ == '__main__':
    print("=" * 60)
    print("Simple Robotics ER Bridge - Iterative with Visual Verification")
    print("=" * 60)
    print()
    print("Implements Google's 4-step function calling pattern:")
    print("  1. Define functions (in prompt)")
    print("  2. Call LLM with current camera images")
    print("  3. Execute ONE function at a time")
    print("  4. Return execution result + NEW images")
    print()
    print("Key features:")
    print("  - ONE function call per response (not arrays)")
    print("  - Visual verification after EACH step")
    print("  - Conversation state maintained")
    print("  - Model can verify, adjust, retry based on images")
    print()
    print("Starting server on http://localhost:8082")
    print("Endpoints:")
    print("  - POST /robotics-er-request")
    print("      Initial: {prompt, images, context}")
    print("      Feedback: {conversation_id, execution_result, images}")
    print("  - GET /status")
    print()

    if not os.getenv('GEMINI_API_KEY'):
        print("WARNING: GEMINI_API_KEY environment variable not set!")
        print("Set it in your shell or .env file")
        print()

    web.run_app(make_app(), host='0.0.0.0', port=8082)
