#!/usr/bin/env python3
"""
UPDATED Real ALOHA robot bridge for Gemini Live API.
Controls the actual Mobile ALOHA gripper and arm through voice commands.

KEY IMPROVEMENTS:
- True fire-and-forget pattern: All operations return immediately (~50ms)
- Unified operation tracking with unique IDs
- Background execution prevents Gemini timeouts
- Status polling for all operations (not just trajectories)
- Automatic cleanup of completed operations
- VISION INTEGRATION: Gemini 2.0 Flash for object detection
- PICK-AND-PLACE: Complete autonomous workflow with vision
"""

import asyncio
import json
import subprocess
import sys
import time
import os
import uuid
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional
from concurrent.futures import ThreadPoolExecutor
from aiohttp import web
from aiohttp_cors import setup, ResourceOptions

# Add parent directory to path to import controllers
sys.path.append(str(Path(__file__).parent.parent))
from dynamixel_controller import DynamixelController
from models.vx300s_model import VX300S
from controllers.gripper_controller import GripperController
from controllers.arm_controller import ArmController
from controllers.camera_controller import CameraController
from controllers.vision_controller import VisionController

# Global controller instances
dynamixel_controller = None  # Shared Dynamixel SDK controller
gripper_controller = None
arm_controller = None
camera_controller = None
vision_controller = None

# Global operation tracking
# Format: {operation_id: {type, status, started_at, completed_at, result, error}}
active_operations: Dict[str, dict] = {}

# Thread pool for blocking operations
executor = ThreadPoolExecutor(max_workers=4)

# Debug logging
DEBUG_LOG_PATH = "/home/aloha/gemini-live/debug/tool_calls.log"

# Operation cleanup interval (seconds)
OPERATION_CLEANUP_INTERVAL = 300  # 5 minutes
OPERATION_MAX_AGE = 300  # Keep operations for 5 minutes after completion


def log_debug(message, data=None):
    """Log debug information to file"""
    try:
        os.makedirs(os.path.dirname(DEBUG_LOG_PATH), exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with open(DEBUG_LOG_PATH, 'a') as f:
            f.write(f"[{timestamp}] {message}\n")
            if data:
                f.write(f"  Data: {json.dumps(data, indent=2)}\n")
            f.write("-" * 80 + "\n")
    except Exception as e:
        print(f"[Debug Log Error] {e}")


async def cleanup_old_operations():
    """Periodically clean up old completed operations."""
    while True:
        try:
            await asyncio.sleep(OPERATION_CLEANUP_INTERVAL)
            current_time = time.time()

            # Find operations to clean up
            to_remove = []
            for op_id, op_data in active_operations.items():
                if op_data['status'] in ['completed', 'failed']:
                    age = current_time - op_data.get('completed_at', op_data.get('failed_at', current_time))
                    if age > OPERATION_MAX_AGE:
                        to_remove.append(op_id)

            # Remove old operations
            for op_id in to_remove:
                del active_operations[op_id]

            if to_remove:
                print(f"[Bridge] Cleaned up {len(to_remove)} old operations")

        except Exception as e:
            print(f"[Bridge] Error in cleanup task: {e}")


# ============================================================================
# BACKGROUND EXECUTION FUNCTIONS
# ============================================================================

async def execute_gripper_command(operation_id: str, args: dict):
    """Execute gripper command in background."""
    global gripper_controller

    try:
        action = args.get('action', 'open').lower()

        active_operations[operation_id] = {
            'type': 'gripper',
            'action': action,
            'status': 'running',
            'started_at': time.time(),
            'args': args
        }

        log_debug(f"GRIPPER EXECUTION START: {operation_id}", {
            "operation_id": operation_id,
            "action": action
        })

        # Run blocking gripper operation in thread pool
        loop = asyncio.get_event_loop()

        if gripper_controller and gripper_controller.initialized:
            if action == 'open':
                result = await loop.run_in_executor(executor, gripper_controller.open_gripper)
            elif action == 'close':
                result = await loop.run_in_executor(executor, gripper_controller.close_gripper)
            else:
                result = {
                    "success": False,
                    "error": f"Unknown action: {action}",
                    "state": "unknown"
                }
        else:
            # Mock mode
            await asyncio.sleep(0.5)  # Simulate delay
            result = {
                "success": True,
                "state": action,
                "position_normalized": 1.0 if action == 'open' else 0.0,
                "note": "mock mode - no real robot"
            }

        # Update operation status
        active_operations[operation_id].update({
            'status': 'completed' if result.get('success') else 'failed',
            'result': result,
            'completed_at': time.time()
        })

        log_debug(f"GRIPPER EXECUTION COMPLETE: {operation_id}", {
            "operation_id": operation_id,
            "success": result.get('success'),
            "state": result.get('state')
        })

        print(f"[Bridge] ✓ Gripper {action} completed ({operation_id[:8]}...)")

    except Exception as e:
        active_operations[operation_id].update({
            'status': 'failed',
            'error': str(e),
            'failed_at': time.time()
        })
        log_debug(f"GRIPPER EXECUTION ERROR: {operation_id}", {"error": str(e)})
        print(f"[Bridge] ✗ Gripper {action} failed: {e}")


async def execute_arm_movement(operation_id: str, args: dict):
    """Execute arm movement in background with position tracking."""
    global arm_controller

    try:
        # Capture initial position
        initial_position = None
        if arm_controller and arm_controller.initialized:
            initial_state = arm_controller.get_arm_state()
            initial_position = {
                'cartesian': initial_state.get('ee_position'),
                'joints': initial_state.get('joints'),
                'joints_degrees': initial_state.get('joints_degrees')
            }

        active_operations[operation_id] = {
            'type': 'arm_move',
            'status': 'running',
            'started_at': time.time(),
            'args': args,
            'initial_position': initial_position
        }

        # Determine movement type
        move_type = 'unknown'
        target_value = None
        if 'pose' in args:
            move_type = 'pose'
            target_value = args['pose']
        elif 'joints' in args:
            move_type = 'joints'
            target_value = args['joints']
        elif 'position' in args:
            move_type = 'position'
            target_value = args['position']

        active_operations[operation_id]['move_type'] = move_type
        active_operations[operation_id]['target'] = target_value

        log_debug(f"ARM EXECUTION START: {operation_id}", {
            "operation_id": operation_id,
            "move_type": move_type,
            "target": target_value,
            "initial_position": initial_position
        })

        loop = asyncio.get_event_loop()

        if arm_controller and arm_controller.initialized:
            # Execute appropriate movement type
            if move_type == 'pose':
                result = await loop.run_in_executor(
                    executor,
                    lambda: arm_controller.move_to_pose(
                        args['pose'],
                        moving_time=args.get('moving_time'),
                        blocking=True
                    )
                )
            elif move_type == 'joints':
                result = await loop.run_in_executor(
                    executor,
                    lambda: arm_controller.move_joints(
                        args['joints'],
                        unit=args.get('unit', 'auto'),
                        moving_time=args.get('moving_time'),
                        blocking=True
                    )
                )
            elif move_type == 'position':
                result = await loop.run_in_executor(
                    executor,
                    lambda: arm_controller.move_to_position(
                        args['position'],
                        orientation=args.get('orientation'),
                        format=args.get('format', 'auto'),
                        moving_time=args.get('moving_time'),
                        blocking=True
                    )
                )
            else:
                result = {
                    "success": False,
                    "error": "No target specified (need 'pose', 'joints', or 'position')"
                }

            # Capture final position after movement
            if result.get('success'):
                final_state = arm_controller.get_arm_state()
                final_position = {
                    'cartesian': final_state.get('ee_position'),
                    'joints': final_state.get('joints'),
                    'joints_degrees': final_state.get('joints_degrees')
                }
                result['final_position'] = final_position

                # Log position change
                print(f"[Bridge] 📍 Position: {initial_position['cartesian']} → {final_position['cartesian']}")
        else:
            # Mock mode
            await asyncio.sleep(1.0)  # Simulate movement delay
            result = {
                "success": True,
                "state": "completed",
                "final_position": {
                    "cartesian": {"x": 0.3, "y": 0.0, "z": 0.2},
                    "joints": [0.0] * 6,
                    "joints_degrees": [0.0] * 6
                },
                "note": "mock mode - no real robot"
            }

        # Update operation status
        active_operations[operation_id].update({
            'status': 'completed' if result.get('success') else 'failed',
            'result': result,
            'completed_at': time.time()
        })

        log_debug(f"ARM EXECUTION COMPLETE: {operation_id}", {
            "operation_id": operation_id,
            "success": result.get('success'),
            "initial_position": initial_position,
            "final_position": result.get('final_position')
        })

        print(f"[Bridge] ✓ Arm {move_type} completed ({operation_id[:8]}...)")

    except Exception as e:
        active_operations[operation_id].update({
            'status': 'failed',
            'error': str(e),
            'failed_at': time.time()
        })
        log_debug(f"ARM EXECUTION ERROR: {operation_id}", {"error": str(e)})
        print(f"[Bridge] ✗ Arm movement failed: {e}")


async def execute_trajectory(operation_id: str, args: dict):
    """Execute trajectory in background."""
    global arm_controller, gripper_controller

    try:
        trajectory = args.get('trajectory', [])
        speed = args.get('speed', 'slow')

        active_operations[operation_id] = {
            'type': 'trajectory',
            'status': 'running',
            'started_at': time.time(),
            'total_waypoints': len(trajectory),
            'args': args
        }

        log_debug(f"TRAJECTORY EXECUTION START: {operation_id}", {
            "operation_id": operation_id,
            "waypoints": len(trajectory),
            "speed": speed
        })

        loop = asyncio.get_event_loop()

        if arm_controller and arm_controller.initialized:
            # Execute trajectory (this already uses its own background tracking)
            result = await loop.run_in_executor(
                executor,
                lambda: arm_controller.execute_trajectory(
                    waypoints=trajectory,
                    speed=speed,
                    coordinate_with_gripper=gripper_controller if gripper_controller and gripper_controller.initialized else None,
                    blocking=True  # Block in executor, not in main loop
                )
            )

            # Note: arm_controller returns its own trajectory_id
            # We'll store both our operation_id and the trajectory_id
            if result.get('success'):
                trajectory_id = result.get('trajectory_id')
                active_operations[operation_id]['trajectory_id'] = trajectory_id
        else:
            # Mock mode
            await asyncio.sleep(2.0)  # Simulate trajectory execution
            result = {
                "success": True,
                "status": "completed",
                "note": "mock mode - no real robot"
            }

        # Update operation status
        active_operations[operation_id].update({
            'status': 'completed' if result.get('success') else 'failed',
            'result': result,
            'completed_at': time.time()
        })

        log_debug(f"TRAJECTORY EXECUTION COMPLETE: {operation_id}", {
            "operation_id": operation_id,
            "success": result.get('success')
        })

        print(f"[Bridge] ✓ Trajectory completed ({operation_id[:8]}...)")

    except Exception as e:
        active_operations[operation_id].update({
            'status': 'failed',
            'error': str(e),
            'failed_at': time.time()
        })
        log_debug(f"TRAJECTORY EXECUTION ERROR: {operation_id}", {"error": str(e)})
        print(f"[Bridge] ✗ Trajectory failed: {e}")


async def execute_pick_and_place(operation_id: str, args: dict):
    """
    Execute complete pick-and-place workflow in background.

    Workflow:
    1. Detect object to pick
    2. Approach and grasp object
    3. Lift object
    4. Detect target location (bowl/destination)
    5. Move to target
    6. Release object
    7. Return to home
    """
    global arm_controller, gripper_controller, camera_controller, vision_controller

    try:
        object_to_pick = args.get('object_to_pick', '')
        target_location = args.get('target_location', '')
        approach_height = args.get('approach_height', 0.05)
        lift_height = args.get('lift_height', 0.15)
        speed = args.get('speed', 'medium')

        active_operations[operation_id] = {
            'type': 'pick_and_place',
            'status': 'running',
            'started_at': time.time(),
            'object_to_pick': object_to_pick,
            'target_location': target_location,
            'steps': [],
            'args': args
        }

        def update_step(step_name, status, data=None):
            """Helper to track workflow steps."""
            step_info = {
                'step': step_name,
                'status': status,
                'timestamp': time.time()
            }
            if data:
                step_info['data'] = data
            active_operations[operation_id]['steps'].append(step_info)
            active_operations[operation_id]['current_step'] = step_name
            print(f"[Bridge] 🔄 Pick&Place Step: {step_name} - {status}")

        log_debug(f"PICK AND PLACE START: {operation_id}", {
            "operation_id": operation_id,
            "object_to_pick": object_to_pick,
            "target_location": target_location
        })

        loop = asyncio.get_event_loop()

        # ============ STEP 1: Detect object to pick ============
        update_step('detect_object', 'running')

        if not camera_controller or not camera_controller.initialized:
            raise Exception("Camera controller not initialized")

        if not vision_controller:
            raise Exception("Vision controller not initialized - set GEMINI_API_KEY")

        # Get camera frames
        rgb_frame, depth_frame = await loop.run_in_executor(
            executor,
            lambda: camera_controller.get_rgbd_frames('gripper_cam')
        )

        if rgb_frame is None:
            raise Exception("No camera frame available")

        # Detect object
        detection_result = await loop.run_in_executor(
            executor,
            lambda: vision_controller.detect_object(
                rgb_frame,
                depth_frame,
                object_to_pick,
                camera_name='gripper_cam'
            )
        )

        if not detection_result.get('object_found'):
            raise Exception(f"Object '{object_to_pick}' not found in workspace")

        if 'position_3d' not in detection_result:
            raise Exception("No 3D position available for object")

        pick_position = detection_result['position_3d']
        update_step('detect_object', 'completed', {'position': pick_position})

        # ============ STEP 2: Open gripper ============
        update_step('open_gripper', 'running')

        if gripper_controller and gripper_controller.initialized:
            gripper_result = await loop.run_in_executor(
                executor,
                gripper_controller.open_gripper
            )
            if not gripper_result.get('success'):
                raise Exception("Failed to open gripper")
            update_step('open_gripper', 'completed')
        else:
            update_step('open_gripper', 'skipped', {'reason': 'mock mode'})

        # ============ STEP 3: Approach object ============
        update_step('approach_object', 'running')

        x, y, z = pick_position
        approach_trajectory = [
            {
                'point': [x, y, z + approach_height],
                'label': 'approach',
                'gripper_action': 'maintain'
            }
        ]

        if arm_controller and arm_controller.initialized:
            approach_result = await loop.run_in_executor(
                executor,
                lambda: arm_controller.execute_trajectory(
                    waypoints=approach_trajectory,
                    speed=speed,
                    coordinate_with_gripper=None,
                    blocking=True
                )
            )
            if not approach_result.get('success'):
                raise Exception("Failed to approach object")
            update_step('approach_object', 'completed')
        else:
            await asyncio.sleep(1.0)
            update_step('approach_object', 'completed', {'note': 'mock mode'})

        # ============ STEP 4: Lower to grasp position ============
        update_step('lower_to_grasp', 'running')

        grasp_trajectory = [
            {
                'point': [x, y, z + 0.01],  # Just above object
                'label': 'grasp_position',
                'gripper_action': 'maintain'
            }
        ]

        if arm_controller and arm_controller.initialized:
            lower_result = await loop.run_in_executor(
                executor,
                lambda: arm_controller.execute_trajectory(
                    waypoints=grasp_trajectory,
                    speed='slow',  # Slow for precision
                    coordinate_with_gripper=None,
                    blocking=True
                )
            )
            if not lower_result.get('success'):
                raise Exception("Failed to lower to grasp position")
            update_step('lower_to_grasp', 'completed')
        else:
            await asyncio.sleep(0.5)
            update_step('lower_to_grasp', 'completed', {'note': 'mock mode'})

        # ============ STEP 5: Close gripper (grasp) WITH VERIFICATION ============
        update_step('grasp_object', 'running')

        if gripper_controller and gripper_controller.initialized:
            # Close gripper with verification enabled
            grasp_result = await loop.run_in_executor(
                executor,
                lambda: gripper_controller.close_gripper(blocking=True, verify_grasp=True)
            )
            if not grasp_result.get('success'):
                raise Exception("Failed to close gripper")

            # Check if object was actually grasped
            if not grasp_result.get('object_grasped', False):
                # Grasp failed - object not detected in gripper
                confidence = grasp_result.get('confidence', 0.0)
                details = grasp_result.get('details', {})
                raise Exception(
                    f"Grasp verification failed (confidence: {confidence:.2f}). "
                    f"Object not detected in gripper. Details: {details}"
                )

            # Wait a bit for grasp to stabilize
            await asyncio.sleep(0.5)
            update_step('grasp_object', 'completed', {
                'object_grasped': True,
                'confidence': grasp_result.get('confidence', 0.0)
            })
            print(f"[Bridge] ✓ Grasp verified (confidence: {grasp_result.get('confidence', 0.0):.2f})")
        else:
            update_step('grasp_object', 'skipped', {'reason': 'mock mode'})

        # ============ STEP 6: Lift object ============
        update_step('lift_object', 'running')

        lift_trajectory = [
            {
                'point': [x, y, z + lift_height],
                'label': 'lift',
                'gripper_action': 'maintain'
            }
        ]

        if arm_controller and arm_controller.initialized:
            lift_result = await loop.run_in_executor(
                executor,
                lambda: arm_controller.execute_trajectory(
                    waypoints=lift_trajectory,
                    speed=speed,
                    coordinate_with_gripper=None,
                    blocking=True
                )
            )
            if not lift_result.get('success'):
                raise Exception("Failed to lift object")
            update_step('lift_object', 'completed')
        else:
            await asyncio.sleep(1.0)
            update_step('lift_object', 'completed', {'note': 'mock mode'})

        # ============ STEP 6.5: Verify object still in gripper (NEW!) ============
        update_step('verify_grasp_after_lift', 'running')

        if gripper_controller and gripper_controller.initialized:
            # Re-verify grasp after movement
            verify_result = await loop.run_in_executor(
                executor,
                gripper_controller.verify_grasp
            )

            if not verify_result.get('object_grasped', False):
                raise Exception(
                    f"Object lost during lift! Grasp confidence: {verify_result.get('confidence', 0.0):.2f}"
                )

            update_step('verify_grasp_after_lift', 'completed', {
                'object_still_grasped': True,
                'confidence': verify_result.get('confidence', 0.0)
            })
            print(f"[Bridge] ✓ Object still in gripper after lift (confidence: {verify_result.get('confidence', 0.0):.2f})")
        else:
            update_step('verify_grasp_after_lift', 'skipped', {'reason': 'mock mode'})

        # ============ STEP 7: Detect target location ============
        update_step('detect_target', 'running')

        # Get fresh camera frames for target detection
        rgb_frame_target, depth_frame_target = await loop.run_in_executor(
            executor,
            lambda: camera_controller.get_rgbd_frames('top_cam')  # Use top camera for better view
        )

        if rgb_frame_target is None:
            # Fallback to gripper cam
            rgb_frame_target, depth_frame_target = await loop.run_in_executor(
                executor,
                lambda: camera_controller.get_rgbd_frames('gripper_cam')
            )

        if rgb_frame_target is None:
            raise Exception("No camera frame available for target detection")

        # Detect target location
        target_detection = await loop.run_in_executor(
            executor,
            lambda: vision_controller.detect_object(
                rgb_frame_target,
                depth_frame_target,
                target_location,
                camera_name='top_cam'
            )
        )

        if not target_detection.get('object_found'):
            raise Exception(f"Target location '{target_location}' not found")

        if 'position_3d' not in target_detection:
            raise Exception("No 3D position available for target")

        place_position = target_detection['position_3d']
        update_step('detect_target', 'completed', {'position': place_position})

        # ============ STEP 8: Move to target ============
        update_step('move_to_target', 'running')

        tx, ty, tz = place_position

        # Approach target from above
        place_trajectory = [
            {
                'point': [tx, ty, tz + lift_height],
                'label': 'approach_target',
                'gripper_action': 'maintain'
            }
        ]

        if arm_controller and arm_controller.initialized:
            move_result = await loop.run_in_executor(
                executor,
                lambda: arm_controller.execute_trajectory(
                    waypoints=place_trajectory,
                    speed=speed,
                    coordinate_with_gripper=None,
                    blocking=True
                )
            )
            if not move_result.get('success'):
                raise Exception("Failed to move to target location")
            update_step('move_to_target', 'completed')
        else:
            await asyncio.sleep(1.0)
            update_step('move_to_target', 'completed', {'note': 'mock mode'})

        # ============ STEP 9: Lower to place position ============
        update_step('lower_to_place', 'running')

        lower_place_trajectory = [
            {
                'point': [tx, ty, tz + 0.05],  # Slightly above target
                'label': 'place_position',
                'gripper_action': 'maintain'
            }
        ]

        if arm_controller and arm_controller.initialized:
            lower_place_result = await loop.run_in_executor(
                executor,
                lambda: arm_controller.execute_trajectory(
                    waypoints=lower_place_trajectory,
                    speed='slow',
                    coordinate_with_gripper=None,
                    blocking=True
                )
            )
            if not lower_place_result.get('success'):
                raise Exception("Failed to lower to place position")
            update_step('lower_to_place', 'completed')
        else:
            await asyncio.sleep(0.5)
            update_step('lower_to_place', 'completed', {'note': 'mock mode'})

        # ============ STEP 10: Release object ============
        update_step('release_object', 'running')

        if gripper_controller and gripper_controller.initialized:
            release_result = await loop.run_in_executor(
                executor,
                gripper_controller.open_gripper
            )
            if not release_result.get('success'):
                raise Exception("Failed to release object")

            # Wait for release
            await asyncio.sleep(0.5)
            update_step('release_object', 'completed')
        else:
            update_step('release_object', 'skipped', {'reason': 'mock mode'})

        # ============ STEP 11: Retract ============
        update_step('retract', 'running')

        retract_trajectory = [
            {
                'point': [tx, ty, tz + lift_height],
                'label': 'retract',
                'gripper_action': 'maintain'
            }
        ]

        if arm_controller and arm_controller.initialized:
            retract_result = await loop.run_in_executor(
                executor,
                lambda: arm_controller.execute_trajectory(
                    waypoints=retract_trajectory,
                    speed=speed,
                    coordinate_with_gripper=None,
                    blocking=True
                )
            )
            if not retract_result.get('success'):
                raise Exception("Failed to retract from target")
            update_step('retract', 'completed')
        else:
            await asyncio.sleep(0.5)
            update_step('retract', 'completed', {'note': 'mock mode'})

        # ============ STEP 12: Return to home ============
        update_step('return_home', 'running')

        if arm_controller and arm_controller.initialized:
            home_result = await loop.run_in_executor(
                executor,
                lambda: arm_controller.move_to_pose('home', blocking=True)
            )
            if not home_result.get('success'):
                raise Exception("Failed to return to home position")
            update_step('return_home', 'completed')
        else:
            await asyncio.sleep(1.0)
            update_step('return_home', 'completed', {'note': 'mock mode'})

        # ============ STEP 13: Verify task completion (NEW!) ============
        update_step('verify_task_completion', 'running')

        task_verified = False
        verification_note = "Visual verification skipped"

        if vision_controller and camera_controller and camera_controller.initialized:
            try:
                # Get overhead view of target location
                rgb_verify, depth_verify = await loop.run_in_executor(
                    executor,
                    lambda: camera_controller.get_rgbd_frames('top_cam')
                )

                if rgb_verify is not None:
                    # Check if object is now at target location
                    verification_result = await loop.run_in_executor(
                        executor,
                        lambda: vision_controller.detect_object(
                            rgb_verify,
                            depth_verify,
                            object_to_pick,
                            camera_name='top_cam'
                        )
                    )

                    if verification_result.get('object_found'):
                        # Object found - check if it's near target location
                        if 'position_3d' in verification_result:
                            obj_pos = verification_result['position_3d']
                            # Calculate distance from target
                            distance = np.sqrt(
                                (obj_pos[0] - place_position[0])**2 +
                                (obj_pos[1] - place_position[1])**2 +
                                (obj_pos[2] - place_position[2])**2
                            )

                            # Consider successful if within 10cm of target
                            if distance < 0.10:
                                task_verified = True
                                verification_note = f"Object confirmed at target (distance: {distance*100:.1f}cm)"
                                print(f"[Bridge] ✓ Task verified: {verification_note}")
                            else:
                                verification_note = f"Object found but far from target (distance: {distance*100:.1f}cm)"
                                print(f"[Bridge] ⚠️  {verification_note}")
                        else:
                            verification_note = "Object found but no 3D position available"
                    else:
                        # Object not found at target - might be hidden in bowl (acceptable)
                        verification_note = "Object not visible at target (may be inside container)"
                        task_verified = True  # Assume success if not visible (in bowl)
                        print(f"[Bridge] ℹ️  {verification_note}")
                else:
                    verification_note = "No camera frame available for verification"

            except Exception as e:
                verification_note = f"Verification error: {str(e)}"
                print(f"[Bridge] ⚠️  {verification_note}")

        update_step('verify_task_completion', 'completed', {
            'verified': task_verified,
            'note': verification_note
        })

        # ============ COMPLETE ============
        result = {
            'success': True,
            'verified': task_verified,
            'verification_note': verification_note,
            'object_picked': object_to_pick,
            'target_reached': target_location,
            'pick_position': pick_position,
            'place_position': place_position,
            'total_steps': len(active_operations[operation_id]['steps']),
            'workflow': 'pick_and_place_complete'
        }

        active_operations[operation_id].update({
            'status': 'completed',
            'result': result,
            'completed_at': time.time()
        })

        log_debug(f"PICK AND PLACE COMPLETE: {operation_id}", {
            "operation_id": operation_id,
            "success": True,
            "steps_completed": len(active_operations[operation_id]['steps'])
        })

        print(f"[Bridge] ✅ Pick-and-place workflow completed ({operation_id[:8]}...)")
        print(f"[Bridge]    Picked: {object_to_pick} from {pick_position}")
        print(f"[Bridge]    Placed: into {target_location} at {place_position}")

    except Exception as e:
        # Log failure
        active_operations[operation_id].update({
            'status': 'failed',
            'error': str(e),
            'failed_at': time.time()
        })
        log_debug(f"PICK AND PLACE ERROR: {operation_id}", {
            "error": str(e),
            "steps_completed": len(active_operations[operation_id].get('steps', []))
        })
        print(f"[Bridge] ✗ Pick-and-place failed: {e}")

        # Try to recover - open gripper and return home
        try:
            if gripper_controller and gripper_controller.initialized:
                await loop.run_in_executor(executor, gripper_controller.open_gripper)
            if arm_controller and arm_controller.initialized:
                await loop.run_in_executor(
                    executor,
                    lambda: arm_controller.move_to_pose('home', blocking=True)
                )
            print(f"[Bridge] ⚠️  Recovery: Opened gripper and returned to home")
        except:
            print(f"[Bridge] ✗ Recovery failed")


# ============================================================================
# ROBOT INITIALIZATION
# ============================================================================

async def initialize_robot():
    """Initialize the robot on startup."""
    global dynamixel_controller, gripper_controller, arm_controller, camera_controller, vision_controller

    print("[Bridge] Initializing Dynamixel SDK...")

    # Initialize DynamixelController (shared by arm and gripper)
    try:
        port = "/dev/ttyDXL"  # or /dev/ttyUSB0
        baudrate = 1000000

        print(f"[Bridge] Connecting to Dynamixel port: {port} at {baudrate} baud")
        dynamixel_controller = DynamixelController(port=port, baudrate=baudrate)

        # Connect to motors
        if not dynamixel_controller.connect():
            print(f"[Bridge] ✗ Failed to connect to Dynamixel port: {port}")
            print("[Bridge] Please check:")
            print("  1. Port exists: ls -l /dev/ttyDXL /dev/ttyUSB*")
            print("  2. User has permissions: sudo usermod -aG dialout $USER")
            print("  3. Power supply is connected")
            raise Exception("Failed to connect to Dynamixel motors")

        print("[Bridge] ✓ Connected to Dynamixel motors")

        # Initialize all motors
        print("[Bridge] Initializing motors...")
        if not dynamixel_controller.initialize_motors():
            print("[Bridge] ✗ Failed to initialize motors")
            raise Exception("Failed to initialize Dynamixel motors")

        print("[Bridge] ✓ All motors initialized successfully")

    except Exception as e:
        print(f"[Bridge] ✗ Dynamixel initialization failed: {e}")
        print("[Bridge] Running in mock mode - no real robot control")
        dynamixel_controller = None

    # Initialize gripper controller (shares DynamixelController)
    print("[Bridge] Initializing gripper controller...")
    try:
        gripper_controller = GripperController(
            dynamixel_controller=dynamixel_controller
        )

        gripper_success = gripper_controller.initialize()
        if gripper_success:
            print("[Bridge] ✓ Gripper controller initialized successfully")
            # Get initial state
            state = gripper_controller.get_gripper_state()
            print(f"[Bridge] Initial gripper state: {state['state']} ({state['position_normalized']*100:.1f}% open)")
        else:
            print("[Bridge] ✗ Failed to initialize gripper controller")
            gripper_success = False

    except Exception as e:
        print(f"[Bridge] ✗ Error initializing gripper controller: {e}")
        gripper_success = False

    # Initialize arm controller (shares DynamixelController)
    print("[Bridge] Initializing arm controller...")
    try:
        robot_model = VX300S()
        arm_controller = ArmController(
            dynamixel_controller=dynamixel_controller,
            robot_model=robot_model
        )

        arm_success = arm_controller.initialize()
        if arm_success:
            print("[Bridge] ✓ Arm controller initialized successfully")
            # Get initial state
            arm_state = arm_controller.get_arm_state()
            if arm_state.get('pose'):
                print(f"[Bridge] Arm at {arm_state['pose']} pose")
            else:
                print(f"[Bridge] Arm joints: {[f'{j:.2f}' for j in arm_state['joints_degrees']]}°")
        else:
            print("[Bridge] ✗ Failed to initialize arm controller")
            arm_success = False

    except Exception as e:
        print(f"[Bridge] ✗ Error initializing arm controller: {e}")
        arm_success = False

    if not (gripper_success and arm_success):
        print("[Bridge] Make sure the robot is powered on and connected")
        print("[Bridge] Running in mock mode for failed controllers")

    # Initialize camera controller (separate from robot control)
    print("[Bridge] Initializing camera controller...")
    camera_controller = CameraController()
    try:
        camera_success = camera_controller.initialize()
        if camera_success:
            print("[Bridge] ✓ Camera controller initialized successfully")
            info = camera_controller.get_camera_info()
            for cam_name in info['cameras'].keys():
                print(f"[Bridge]   - {cam_name} ready")
        else:
            print("[Bridge] ✗ Camera controller initialization failed")
            print("[Bridge] Camera feeds will not be available")
    except Exception as e:
        print(f"[Bridge] ✗ Error initializing cameras: {e}")

    # Initialize vision controller (requires camera_controller and GOOGLE_API_KEY)
    print("[Bridge] Initializing vision controller...")
    try:
        if camera_controller and camera_controller.initialized:
            gemini_key = os.environ.get('GOOGLE_API_KEY') or os.environ.get('GEMINI_API_KEY')

            # Check for calibration file (YAML format)
            calibration_file = Path(__file__).parent.parent / "camera_calibration.yaml"

            if calibration_file.exists():
                vision_controller = VisionController(
                    camera_controller=camera_controller,
                    gemini_api_key=gemini_key,
                    calibration_file=str(calibration_file)
                )
                print("[Bridge] ✓ Vision controller initialized with calibrated transforms")
                print(f"[Bridge]   Loaded calibration from: {calibration_file}")
            else:
                vision_controller = VisionController(
                    camera_controller=camera_controller,
                    gemini_api_key=gemini_key
                )
                print("[Bridge] ✓ Vision controller initialized with default transforms")
                print("[Bridge] ⚠️  For accurate object positioning, calibrate cameras:")
                print("[Bridge]    python3 -m vision.calibrate_cameras --camera top_cam --tag-id 0")

            # Get vision status
            status = vision_controller.get_status()
            print(f"[Bridge]   Vision initialized: {status['initialized']}")
        else:
            print("[Bridge] ⚠️  Camera controller not available - vision disabled")
            print("[Bridge]    Vision features will not work until cameras are initialized")
    except Exception as e:
        print(f"[Bridge] ✗ Error initializing vision controller: {e}")
        print(f"[Bridge]   {type(e).__name__}: {str(e)}")
        print("[Bridge]   Vision features disabled")
        import traceback
        traceback.print_exc()


# ============================================================================
# HTTP REQUEST HANDLERS - FIRE AND FORGET PATTERN
# ============================================================================

async def handle_tool_call(request: web.Request) -> web.Response:
    """
    Handle tool calls from Gemini Live API via React bridge.

    FIRE-AND-FORGET PATTERN:
    - Returns immediately with operation_id (~50ms response time)
    - Actual execution happens in background
    - Status can be polled via /operation/{id}/status
    """
    try:
        data = await request.json()
        name = data.get('name')
        args = data.get('args', {})
        call_id = data.get('id')

        # Log incoming tool call
        log_debug(f"TOOL CALL RECEIVED: {name}", {
            "name": name,
            "args": args,
            "call_id": call_id,
            "timestamp": datetime.now().isoformat()
        })

        print(f"[Bridge] 📞 Tool call: {name}")

        # Generate unique operation ID
        operation_id = str(uuid.uuid4())

        # ============ FIRE-AND-FORGET OPERATIONS ============

        if name == 'control_gripper':
            # Start gripper operation in background
            asyncio.create_task(execute_gripper_command(operation_id, args))

            return web.json_response({
                'success': True,
                'operation_id': operation_id,
                'status': 'started',
                'message': f"Gripper {args.get('action', 'open')} command accepted",
                'call_id': call_id
            })

        elif name == 'move_arm':
            # Start arm movement in background
            asyncio.create_task(execute_arm_movement(operation_id, args))

            move_type = 'pose' if 'pose' in args else 'position' if 'position' in args else 'joints'
            return web.json_response({
                'success': True,
                'operation_id': operation_id,
                'status': 'started',
                'message': f"Arm {move_type} movement accepted",
                'call_id': call_id
            })

        elif name == 'move_arm_trajectory':
            # Start trajectory in background
            asyncio.create_task(execute_trajectory(operation_id, args))

            trajectory = args.get('trajectory', [])
            return web.json_response({
                'success': True,
                'operation_id': operation_id,
                'status': 'started',
                'message': f"Trajectory with {len(trajectory)} waypoints accepted",
                'total_waypoints': len(trajectory),
                'call_id': call_id
            })

        elif name == 'pick_and_place':
            # Execute complete pick-and-place workflow in background
            asyncio.create_task(execute_pick_and_place(operation_id, args))

            object_to_pick = args.get('object_to_pick', 'object')
            target_location = args.get('target_location', 'target')

            return web.json_response({
                'success': True,
                'operation_id': operation_id,
                'status': 'started',
                'message': f"Pick-and-place workflow started: {object_to_pick} → {target_location}",
                'workflow': 'pick_and_place',
                'object_to_pick': object_to_pick,
                'target_location': target_location,
                'call_id': call_id
            })

        # ============ INSTANT STATUS QUERIES ============

        elif name == 'get_gripper_status':
            # Status queries can be synchronous (they're fast)
            if gripper_controller and gripper_controller.initialized:
                result = gripper_controller.get_gripper_state()
            else:
                result = {
                    "success": True,
                    "state": "open",
                    "position": 1.62,
                    "position_normalized": 1.0,
                    "note": "mock mode - no real robot"
                }

            percentage = result.get('position_normalized', 0) * 100
            print(f"[Bridge] 📊 Gripper status: {result['state']} ({percentage:.1f}% open)")

            return web.json_response({
                'success': True,
                'result': result,
                'call_id': call_id
            })

        elif name == 'get_arm_status':
            # Status queries can be synchronous (they're fast)
            if arm_controller and arm_controller.initialized:
                result = arm_controller.get_arm_state()
            else:
                result = {
                    "success": True,
                    "state": "idle",
                    "joints": [0.0] * 6,
                    "joints_degrees": [0.0] * 6,
                    "pose": "home",
                    "note": "mock mode - no real robot"
                }

            print(f"[Bridge] 📊 Arm status: {result['state']}")

            return web.json_response({
                'success': True,
                'result': result,
                'call_id': call_id
            })

        elif name == 'get_robot_status':
            # Combined status for compatibility
            status_result = {"ts": time.time()}

            if gripper_controller and gripper_controller.initialized:
                gripper_state = gripper_controller.get_gripper_state()
                status_result["gripper"] = {
                    "state": gripper_state['state'],
                    "position_percent": gripper_state['position_normalized'] * 100
                }
            else:
                status_result["gripper"] = {"state": "unknown", "position_percent": 0}

            if arm_controller and arm_controller.initialized:
                arm_state = arm_controller.get_arm_state()
                status_result["arm"] = {
                    "state": arm_state['state'],
                    "pose": arm_state.get('pose'),
                    "joints_degrees": arm_state.get('joints_degrees', [0]*6)
                }
            else:
                status_result["arm"] = {"state": "unknown", "pose": None}

            return web.json_response({
                'success': True,
                'result': status_result,
                'call_id': call_id
            })

        # ============ VISION OPERATIONS ============

        elif name == 'detect_and_target_object':
            # Visual object detection using Gemini API
            object_desc = args.get('object_description', '')
            action = args.get('action', 'approach')
            approach_height = args.get('approach_height', 0.05)

            print(f"[Bridge] 👁️ Object detection for: {object_desc}")

            # Check if we have all required components
            if not camera_controller or not camera_controller.initialized:
                result = {
                    'success': False,
                    'error': 'Camera controller not initialized',
                    'object_found': False
                }
                return web.json_response({'success': False, 'result': result, 'call_id': call_id})

            if not vision_controller:
                # Fallback to placeholder if vision not available
                print("[Bridge] ⚠️  Vision controller not available, using placeholder")
                result = {
                    'success': True,
                    'object_found': True,
                    'object_description': object_desc,
                    'suggested_trajectory': [
                        {'point': [0.3, 0.0, 0.25], 'label': 'approach', 'gripper_action': 'open'},
                        {'point': [0.3, 0.0, 0.15], 'label': 'target', 'gripper_action': 'close'}
                    ],
                    'note': 'Visual detection placeholder - GEMINI_API_KEY not set'
                }
                return web.json_response({'success': True, 'result': result, 'call_id': call_id})

            try:
                # Get RGB and depth frames from gripper camera
                rgb_frame, depth_frame = camera_controller.get_rgbd_frames('gripper_cam')

                if rgb_frame is None:
                    result = {
                        'success': False,
                        'error': 'No camera frame available',
                        'object_found': False
                    }
                    return web.json_response({'success': False, 'result': result, 'call_id': call_id})

                # Run object detection with Gemini API
                detection_result = vision_controller.detect_object(
                    rgb_frame,
                    depth_frame,
                    object_desc,
                    camera_name='gripper_cam'
                )

                if not detection_result.get('object_found'):
                    # Object not found
                    print(f"[Bridge] ✗ Object '{object_desc}' not found in frame")
                    return web.json_response({
                        'success': True,
                        'result': detection_result,
                        'call_id': call_id
                    })

                # Object found! Generate trajectory based on 3D position
                if 'position_3d' in detection_result:
                    target_pos = detection_result['position_3d']
                    x, y, z = target_pos

                    # Generate approach trajectory
                    trajectory = [
                        {
                            'point': [x, y, z + approach_height],
                            'label': 'approach',
                            'gripper_action': 'open'
                        },
                        {
                            'point': [x, y, z],
                            'label': 'target',
                            'gripper_action': 'close' if action == 'grasp' else 'maintain'
                        }
                    ]

                    # Add lift if grasping
                    if action == 'grasp':
                        trajectory.append({
                            'point': [x, y, z + 0.1],
                            'label': 'lift',
                            'gripper_action': 'maintain'
                        })

                    detection_result['suggested_trajectory'] = trajectory
                    print(f"[Bridge] ✓ Found '{object_desc}' at {target_pos}")
                else:
                    # Detection succeeded but no 3D position (no depth)
                    print(f"[Bridge] ⚠️  Found '{object_desc}' but no depth data")
                    # Use placeholder trajectory
                    detection_result['suggested_trajectory'] = [
                        {'point': [0.3, 0.0, 0.25], 'label': 'approach', 'gripper_action': 'open'},
                        {'point': [0.3, 0.0, 0.15], 'label': 'target', 'gripper_action': 'close'}
                    ]
                    detection_result['note'] = 'Object detected but no depth data - using estimated position'

                return web.json_response({
                    'success': True,
                    'result': detection_result,
                    'call_id': call_id
                })

            except Exception as e:
                print(f"[Bridge] ✗ Object detection error: {e}")
                result = {
                    'success': False,
                    'error': str(e),
                    'object_found': False,
                    'object_description': object_desc
                }
                return web.json_response({'success': False, 'result': result, 'call_id': call_id})

        elif name == 'analyze_workspace':
            # Workspace analysis using Gemini API
            analysis_type = args.get('analysis_type', 'objects')

            print(f"[Bridge] 🔍 Workspace analysis: {analysis_type}")

            # Check if we have required components
            if not camera_controller or not camera_controller.initialized:
                result = {
                    'success': False,
                    'error': 'Camera controller not initialized'
                }
                return web.json_response({'success': False, 'result': result, 'call_id': call_id})

            camera_info = camera_controller.get_camera_info()

            if not vision_controller:
                # Fallback to placeholder
                print("[Bridge] ⚠️  Vision controller not available, using placeholder")
                result = {
                    'success': True,
                    'analysis_type': analysis_type,
                    'camera_status': camera_info,
                    'workspace_clear': True,
                    'objects_detected': [],
                    'note': 'Workspace analysis placeholder - GEMINI_API_KEY not set'
                }
                return web.json_response({'success': True, 'result': result, 'call_id': call_id})

            try:
                # Get frames from top camera (better view of workspace)
                rgb_frame, depth_frame = camera_controller.get_rgbd_frames('top_cam')

                if rgb_frame is None:
                    # Try gripper camera as fallback
                    rgb_frame, depth_frame = camera_controller.get_rgbd_frames('gripper_cam')

                if rgb_frame is None:
                    result = {
                        'success': False,
                        'error': 'No camera frame available'
                    }
                    return web.json_response({'success': False, 'result': result, 'call_id': call_id})

                # Run workspace analysis with Gemini API
                analysis_result = vision_controller.analyze_workspace(
                    rgb_frame,
                    depth_frame,
                    analysis_type
                )

                # Add camera status
                analysis_result['camera_status'] = camera_info

                print(f"[Bridge] ✓ Workspace analysis complete")

                return web.json_response({
                    'success': True,
                    'result': analysis_result,
                    'call_id': call_id
                })

            except Exception as e:
                print(f"[Bridge] ✗ Workspace analysis error: {e}")
                result = {
                    'success': False,
                    'error': str(e),
                    'analysis_type': analysis_type
                }
                return web.json_response({'success': False, 'result': result, 'call_id': call_id})

        elif name == 'emergency_stop':
            # Emergency stop - execute immediately
            results = []
            if arm_controller and arm_controller.initialized:
                arm_result = arm_controller.emergency_stop()
                results.append(f"Arm: {arm_result.get('state', 'error')}")

            result = {
                "success": True,
                "results": results,
                "state": "emergency_stopped"
            }
            print(f"[Bridge] 🛑 EMERGENCY STOP executed")

            return web.json_response({
                'success': True,
                'result': result,
                'call_id': call_id
            })

        else:
            # Unknown tool
            log_debug(f"UNKNOWN TOOL: {name}", {
                "name": name,
                "args": args,
                "error": "Tool not recognized"
            })
            return web.json_response({
                'success': False,
                'error': f'Unknown tool: {name}'
            }, status=400)

    except Exception as e:
        print(f"[Bridge] ✗ Error handling tool call: {e}")
        log_debug(f"TOOL ERROR", {
            "error": str(e),
            "name": name if 'name' in locals() else 'unknown'
        })
        return web.json_response({
            'success': False,
            'error': str(e)
        }, status=500)


async def handle_operation_status(request: web.Request) -> web.Response:
    """Get status of any operation (gripper, arm, trajectory)."""
    operation_id = request.match_info.get('operation_id')

    if not operation_id:
        return web.json_response({
            'success': False,
            'error': 'Missing operation_id parameter'
        }, status=400)

    if operation_id not in active_operations:
        return web.json_response({
            'success': False,
            'error': f'Operation {operation_id} not found'
        }, status=404)

    op_status = active_operations[operation_id].copy()

    # Add duration if completed
    if 'completed_at' in op_status:
        op_status['duration'] = op_status['completed_at'] - op_status['started_at']

    return web.json_response({
        'success': True,
        'operation_id': operation_id,
        **op_status
    })


async def handle_list_operations(request: web.Request) -> web.Response:
    """List all tracked operations."""
    # Get query parameters
    status_filter = request.query.get('status')  # running, completed, failed
    limit = int(request.query.get('limit', 50))

    # Filter operations
    ops = []
    for op_id, op_data in list(active_operations.items())[:limit]:
        if status_filter and op_data['status'] != status_filter:
            continue

        op_summary = {
            'operation_id': op_id,
            'type': op_data['type'],
            'status': op_data['status'],
            'started_at': op_data['started_at']
        }

        if 'completed_at' in op_data:
            op_summary['completed_at'] = op_data['completed_at']
            op_summary['duration'] = op_data['completed_at'] - op_data['started_at']

        if 'error' in op_data:
            op_summary['error'] = op_data['error']

        ops.append(op_summary)

    return web.json_response({
        'success': True,
        'total': len(ops),
        'operations': ops
    })


async def handle_cancel_operation(request: web.Request) -> web.Response:
    """Cancel a running operation."""
    operation_id = request.match_info.get('operation_id')

    if not operation_id:
        return web.json_response({
            'success': False,
            'error': 'Missing operation_id parameter'
        }, status=400)

    if operation_id not in active_operations:
        return web.json_response({
            'success': False,
            'error': f'Operation {operation_id} not found'
        }, status=404)

    op = active_operations[operation_id]

    if op['status'] != 'running':
        return web.json_response({
            'success': False,
            'error': f'Operation is not running (status: {op["status"]})'
        }, status=400)

    # For trajectory operations, use arm_controller's cancel
    if op['type'] == 'trajectory' and 'trajectory_id' in op:
        if arm_controller and arm_controller.initialized:
            result = arm_controller.cancel_trajectory(op['trajectory_id'])
            op['status'] = 'cancelled'
            op['cancelled_at'] = time.time()
            return web.json_response(result)

    # For other operations, mark as cancelled
    # (Note: actual cancellation of in-progress gripper/arm moves is harder)
    op['status'] = 'cancelled'
    op['cancelled_at'] = time.time()

    return web.json_response({
        'success': True,
        'operation_id': operation_id,
        'message': f'Operation {op["type"]} cancelled'
    })


async def handle_get_current_position(request: web.Request) -> web.Response:
    """Get current arm position in real-time."""
    global arm_controller

    if not arm_controller or not arm_controller.initialized:
        return web.json_response({
            'success': False,
            'error': 'Arm controller not initialized'
        }, status=503)

    try:
        # Get current arm state
        arm_state = arm_controller.get_arm_state()

        position_data = {
            'success': True,
            'timestamp': time.time(),
            'cartesian': arm_state.get('ee_position'),  # End-effector position {x, y, z}
            'joints': arm_state.get('joints'),  # Joint angles in radians
            'joints_degrees': arm_state.get('joints_degrees'),  # Joint angles in degrees
            'state': arm_state.get('state'),  # idle, moving, etc.
            'pose': arm_state.get('pose')  # Named pose if any (home, sleep, etc.)
        }

        return web.json_response(position_data)

    except Exception as e:
        return web.json_response({
            'success': False,
            'error': f'Failed to get position: {str(e)}'
        }, status=500)


# ============================================================================
# CAMERA ENDPOINTS
# ============================================================================

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
    """Get information about available cameras."""
    global camera_controller

    if not camera_controller:
        return web.json_response({
            'success': False,
            'error': 'Camera controller not initialized'
        }, status=503)

    info = camera_controller.get_camera_info()
    return web.json_response({
        'success': True,
        **info
    })


# ============================================================================
# LEGACY TRAJECTORY ENDPOINTS (for backward compatibility)
# ============================================================================

async def handle_trajectory_status(request: web.Request) -> web.Response:
    """Get status of trajectory execution (legacy endpoint)."""
    global arm_controller

    trajectory_id = request.match_info.get('trajectory_id')

    if not trajectory_id:
        return web.json_response({
            'success': False,
            'error': 'Missing trajectory_id parameter'
        }, status=400)

    if arm_controller and arm_controller.initialized:
        status = arm_controller.get_trajectory_status(trajectory_id)
        return web.json_response(status)
    else:
        return web.json_response({
            'success': False,
            'error': 'Arm controller not initialized'
        }, status=503)


async def handle_cancel_trajectory(request: web.Request) -> web.Response:
    """Cancel trajectory execution (legacy endpoint)."""
    global arm_controller

    trajectory_id = request.match_info.get('trajectory_id')

    if not trajectory_id:
        return web.json_response({
            'success': False,
            'error': 'Missing trajectory_id parameter'
        }, status=400)

    if arm_controller and arm_controller.initialized:
        result = arm_controller.cancel_trajectory(trajectory_id)
        return web.json_response(result)
    else:
        return web.json_response({
            'success': False,
            'error': 'Arm controller not initialized'
        }, status=503)


async def handle_list_trajectories(request: web.Request) -> web.Response:
    """List all tracked trajectories (legacy endpoint)."""
    global arm_controller

    if arm_controller and arm_controller.initialized:
        result = arm_controller.list_trajectories()
        return web.json_response(result)
    else:
        return web.json_response({
            'success': False,
            'error': 'Arm controller not initialized'
        }, status=503)


# ============================================================================
# STATUS ENDPOINT
# ============================================================================

async def handle_status(request: web.Request) -> web.Response:
    """Simple status endpoint to check if bridge is running."""
    global gripper_controller, arm_controller, camera_controller

    status = {
        "bridge": "running",
        "version": "2.0-fire-and-forget",
        "gripper_initialized": gripper_controller.initialized if gripper_controller else False,
        "arm_initialized": arm_controller.initialized if arm_controller else False,
        "camera_initialized": camera_controller.initialized if camera_controller else False,
        "active_operations": len(active_operations),
        "timestamp": time.time()
    }

    if gripper_controller and gripper_controller.initialized:
        gripper_state = gripper_controller.get_gripper_state()
        status["gripper"] = {
            "state": gripper_state['state'],
            "position_percent": gripper_state['position_normalized'] * 100
        }

    if arm_controller and arm_controller.initialized:
        arm_state = arm_controller.get_arm_state()
        status["arm"] = {
            "state": arm_state['state'],
            "pose": arm_state.get('pose'),
            "joints_degrees": arm_state.get('joints_degrees', [0]*6)[:3]  # Show first 3 joints
        }

    if camera_controller and camera_controller.initialized:
        camera_info = camera_controller.get_camera_info()
        status["cameras"] = list(camera_info['cameras'].keys())

    return web.json_response(status)


# ============================================================================
# LIFECYCLE MANAGEMENT
# ============================================================================

async def cleanup(app):
    """Cleanup on shutdown."""
    global dynamixel_controller, gripper_controller, arm_controller, camera_controller, vision_controller, executor

    print("\n[Bridge] Shutting down...")

    # Shutdown arm controller
    if arm_controller:
        try:
            arm_controller.move_to_pose('sleep', blocking=True)
            arm_controller.shutdown()
            print("[Bridge] Arm controller shutdown complete")
        except:
            pass

    # Shutdown camera controller
    if camera_controller:
        try:
            camera_controller.shutdown()
            print("[Bridge] Camera controller shutdown complete")
        except:
            pass

    # Shutdown gripper controller
    if gripper_controller:
        try:
            gripper_controller.shutdown()
            print("[Bridge] Gripper controller shutdown complete")
        except:
            pass

    # Close DynamixelController
    if dynamixel_controller:
        try:
            dynamixel_controller.disconnect()
            print("[Bridge] Dynamixel controller disconnected")
        except:
            pass

    # Shutdown thread pool
    executor.shutdown(wait=True)
    print("[Bridge] Thread pool shutdown complete")


async def startup(app):
    """Initialize robot on startup."""
    # Initialize robot
    await initialize_robot()

    # Start cleanup task
    asyncio.create_task(cleanup_old_operations())
    print("[Bridge] ✓ Operation cleanup task started")


def make_app() -> web.Application:
    """Create the aiohttp application."""
    app = web.Application()

    # Setup CORS for browser access
    cors = setup(app, defaults={
        '*': ResourceOptions(
            allow_credentials=True,
            expose_headers='*',
            allow_headers='*',
            allow_methods='*'
        )
    })

    # Add routes
    # Main tool call endpoint (FIRE-AND-FORGET)
    app.router.add_post('/aloha-tool-call', handle_tool_call)

    # Unified operation management
    app.router.add_get('/operation/{operation_id}/status', handle_operation_status)
    app.router.add_post('/operation/{operation_id}/cancel', handle_cancel_operation)
    app.router.add_get('/operations', handle_list_operations)

    # Status endpoints
    app.router.add_get('/status', handle_status)
    app.router.add_get('/arm/position', handle_get_current_position)

    # Camera endpoints
    app.router.add_get('/camera/{camera_name}/frame', handle_camera_frame)
    app.router.add_get('/camera/info', handle_camera_info)

    # Legacy trajectory endpoints (backward compatibility)
    app.router.add_get('/trajectory/{trajectory_id}/status', handle_trajectory_status)
    app.router.add_post('/trajectory/{trajectory_id}/cancel', handle_cancel_trajectory)
    app.router.add_get('/trajectories', handle_list_trajectories)

    # Add CORS to routes
    for route in list(app.router.routes()):
        cors.add(route)

    # Add startup and cleanup handlers
    app.on_startup.append(startup)
    app.on_cleanup.append(cleanup)

    return app


if __name__ == '__main__':
    print("=" * 80)
    print("ALOHA Robot Bridge for Gemini Live API v2.0")
    print("FIRE-AND-FORGET + VISION INTEGRATION EDITION")
    print("=" * 80)
    print()
    print("✨ NEW FEATURES:")
    print("  - True fire-and-forget: All operations return in ~50ms")
    print("  - Unified operation tracking with unique IDs")
    print("  - Background execution prevents Gemini timeouts")
    print("  - Status polling for all operations")
    print("  - Automatic cleanup of old operations")
    print("  - Position tracking on every arm movement")
    print("  - 🔥 VISION INTEGRATION: Gemini 2.0 Flash for object detection")
    print("  - 🔥 PICK-AND-PLACE: Complete autonomous workflow with vision")
    print()
    print("This bridge controls the REAL Mobile ALOHA robot.")
    print("Make sure the robot is powered on and connected.")
    print()
    print("⚙️  REQUIRED ENVIRONMENT VARIABLES:")
    print("  - GEMINI_API_KEY: Required for vision features (object detection)")
    print()
    print("Starting bridge server on http://localhost:8081")
    print()
    print("ENDPOINTS:")
    print("  POST   /aloha-tool-call              - Execute tool (returns operation_id)")
    print("  GET    /operation/{id}/status        - Check operation status (includes positions)")
    print("  POST   /operation/{id}/cancel        - Cancel operation")
    print("  GET    /operations                   - List all operations")
    print("  GET    /status                       - Bridge status")
    print("  GET    /arm/position                 - Get current arm position (real-time)")
    print("  GET    /camera/{name}/frame          - Get camera frame")
    print("  GET    /camera/info                  - Camera info")
    print()
    print("TOOL FUNCTIONS:")
    print("  - control_gripper                    - Open/close gripper")
    print("  - move_arm                           - Move arm to position/pose/joints")
    print("  - move_arm_trajectory                - Execute multi-waypoint trajectory")
    print("  - 🔥 pick_and_place                  - Complete pick-and-place with vision")
    print("  - detect_and_target_object           - Visual object detection")
    print("  - analyze_workspace                  - Workspace analysis")
    print("  - get_gripper_status                 - Gripper state query")
    print("  - get_arm_status                     - Arm state query")
    print("  - emergency_stop                     - Immediate stop")
    print()

    # Check for dependencies
    try:
        import aiohttp
        import aiohttp_cors
    except ImportError:
        print("ERROR: Missing dependencies!")
        print("Please install: pip install aiohttp aiohttp-cors")
        sys.exit(1)

    # Run the server
    web.run_app(make_app(), host='0.0.0.0', port=8081)
