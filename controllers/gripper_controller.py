#!/usr/bin/env python3

"""
Gripper Controller API for Mobile ALOHA
Designed to be called from external scripts (e.g., Gemini Live API integration)
"""

import time
import threading
import warnings
from enum import Enum
from typing import Dict, Optional, Tuple
import logging

from aloha.robot_utils import move_arms, move_grippers, torque_on
from aloha.constants import (
    FOLLOWER_GRIPPER_JOINT_OPEN, 
    FOLLOWER_GRIPPER_JOINT_CLOSE,
    START_ARM_POSE
)
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class GripperState(Enum):
    """Gripper states for easy status checking"""
    OPEN = "open"
    CLOSED = "closed"
    OPENING = "opening"
    CLOSING = "closing"
    UNKNOWN = "unknown"
    ERROR = "error"


class GripperController:
    """
    Main API class for controlling the Mobile ALOHA gripper.
    
    Usage:
        controller = GripperController()
        controller.initialize()
        controller.open_gripper()
        state = controller.get_gripper_state()
        controller.close_gripper()
        controller.shutdown()
    """
    
    def __init__(self, robot_model='vx300s', robot_name='follower_left', dry_run=False):
        """Initialize controller (does not connect to robot yet)"""
        self.robot_model = robot_model
        self.robot_name = robot_name
        self.dry_run = dry_run
        self.bot = None
        self.node = None
        self.initialized = False
        self.current_state = GripperState.UNKNOWN
        self.gripper_position = 0.0
        self.state_lock = threading.Lock()
        self.monitor_failure_count = 0
        
        # Gripper position thresholds
        self.OPEN_THRESHOLD = FOLLOWER_GRIPPER_JOINT_OPEN - 0.1
        self.CLOSE_THRESHOLD = FOLLOWER_GRIPPER_JOINT_CLOSE + 0.1
        
    def initialize(self) -> bool:
        """
        Initialize robot connection and move to starting position.
        Returns True if successful, False otherwise.
        """
        # Dry-run mode: Skip hardware initialization
        if self.dry_run:
            print("[GripperController] 🔧 DRY-RUN MODE: Skipping hardware initialization")
            self.initialized = True
            self.current_state = GripperState.CLOSED
            self.gripper_position = FOLLOWER_GRIPPER_JOINT_CLOSE
            print("[GripperController] ✓ Dry-run initialization complete")
            return True

        try:
            print("[GripperController] Initializing robot connection...")
            
            # Create ROS node
            self.node = create_interbotix_global_node('gripper_controller')
            
            # Create robot interface
            self.bot = InterbotixManipulatorXS(
                robot_model=self.robot_model,
                robot_name=self.robot_name,
                node=self.node,
                iterative_update_fk=False,
            )
            
            # Start ROS
            robot_startup(self.node)
            
            # Configure motors
            print("[GripperController] Configuring motors...")
            self.bot.core.robot_reboot_motors('single', 'gripper', True)
            self.bot.core.robot_set_operating_modes('group', 'arm', 'position')
            self.bot.core.robot_set_operating_modes('single', 'gripper', 'current_based_position')
            self.bot.core.robot_set_motor_registers('single', 'gripper', 'current_limit', 300)
            
            # Enable torque
            torque_on(self.bot)
            
            # Move to starting position
            print("[GripperController] Moving to starting position...")
            start_arm_qpos = START_ARM_POSE[:6]
            move_arms([self.bot], [start_arm_qpos], moving_time=4.0)
            move_grippers([self.bot], [FOLLOWER_GRIPPER_JOINT_CLOSE], moving_time=0.5)
            
            self.initialized = True
            self.current_state = GripperState.CLOSED
            
            # Start position monitoring thread
            self._start_position_monitor()
            
            print("[GripperController] ✓ Initialization complete")
            return True
            
        except Exception as e:
            print(f"[GripperController] ✗ Initialization failed: {e}")
            return False
    
    def _start_position_monitor(self):
        """Start background thread to monitor gripper position"""
        # Dry-run mode: Skip monitoring thread (position is set manually)
        if self.dry_run:
            print("[GripperController] 🔧 DRY-RUN: Skipping position monitor thread")
            return

        def monitor():
            while self.initialized:
                try:
                    # Get current gripper position from hardware
                    with self.bot.core.js_mutex:
                        gripper_index = self.bot.gripper.left_finger_index
                        position = self.bot.core.joint_states.position[gripper_index]

                    # Update shared state with lock protection
                    with self.state_lock:
                        self.gripper_position = position

                        # Update state based on position
                        if self.current_state in [GripperState.OPENING, GripperState.CLOSING]:
                            # Check if movement completed
                            if position >= self.OPEN_THRESHOLD:
                                if self.current_state == GripperState.OPENING:
                                    self.current_state = GripperState.OPEN
                            elif position <= self.CLOSE_THRESHOLD:
                                if self.current_state == GripperState.CLOSING:
                                    self.current_state = GripperState.CLOSED
                    # Reset failure counter on success
                    self.monitor_failure_count = 0
                except Exception as e:
                    self.monitor_failure_count += 1
                    logging.error(
                        f"Gripper position monitor failed (failure #{self.monitor_failure_count}): "
                        f"{type(e).__name__}: {e}",
                        exc_info=True
                    )
                    # Alert if failures are excessive
                    if self.monitor_failure_count >= 5:
                        logging.critical(
                            f"Gripper monitor has failed {self.monitor_failure_count} consecutive times! "
                            "This may indicate a serious hardware or connection issue."
                        )

                time.sleep(0.1)  # Check 10 times per second

        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
    
    def open_gripper(self, blocking: bool = True) -> Dict:
        """
        Open the gripper.

        Args:
            blocking: If True, wait for movement to complete

        Returns:
            Dictionary with status and gripper position
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized", "state": "unknown"}

        # Check if system is in ERROR state (e.g., after emergency stop)
        with self.state_lock:
            if self.current_state == GripperState.ERROR:
                return {"success": False, "error": "System in ERROR state. Call resume_after_stop() to recover.", "state": "error"}

        with self.state_lock:
            self.current_state = GripperState.OPENING

        print("[GripperController] Opening gripper...")

        # Dry-run mode: Simulate movement
        if self.dry_run:
            if blocking:
                time.sleep(0.1)  # Simulate brief movement
                with self.state_lock:
                    self.gripper_position = FOLLOWER_GRIPPER_JOINT_OPEN
                    self.current_state = GripperState.OPEN
            print("[GripperController] 🔧 DRY-RUN: Simulated gripper open")
        else:
            # Hardware mode: Execute real movement
            move_grippers([self.bot], [FOLLOWER_GRIPPER_JOINT_OPEN], moving_time=1.0)

            if blocking:
                time.sleep(1.0)
                with self.state_lock:
                    self.current_state = GripperState.OPEN

        return self.get_gripper_state()
    
    def close_gripper(self, blocking: bool = True, verify_grasp: bool = False) -> Dict:
        """
        Close the gripper with optional grasp verification.

        Args:
            blocking: If True, wait for movement to complete
            verify_grasp: If True, check if object was actually grasped

        Returns:
            Dictionary with status, gripper position, and grasp verification
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized", "state": "unknown"}
      
        # Check if system is in ERROR state (e.g., after emergency stop)
        with self.state_lock:
            if self.current_state == GripperState.ERROR:
                return {"success": False, "error": "System in ERROR state. Call resume_after_stop() to recover.", "state": "error"}

        with self.state_lock:
            self.current_state = GripperState.CLOSING

        print("[GripperController] Closing gripper...")

        result = self.get_gripper_state()  
        # Verify grasp if requested
        if verify_grasp and blocking:
            grasp_check = self.verify_grasp()
            result.update(grasp_check)
        
        return result
    
    def get_gripper_state(self) -> Dict:
        """
        Get current gripper state and position.

        Returns:
            Dictionary containing:
            - success: bool
            - state: current state (open/closed/opening/closing/unknown)
            - position: current position in radians
            - position_normalized: 0.0 (closed) to 1.0 (open)
        """
        if not self.initialized:
            return {
                "success": False,
                "error": "Not initialized",
                "state": GripperState.UNKNOWN.value,
                "position": 0.0,
                "position_normalized": 0.0
            }

        with self.state_lock:
            # Capture position while holding lock to prevent race condition
            position = self.gripper_position
            state = self.current_state.value

            # Normalize position from 0 (closed) to 1 (open)
            pos_range = FOLLOWER_GRIPPER_JOINT_OPEN - FOLLOWER_GRIPPER_JOINT_CLOSE
            pos_normalized = (position - FOLLOWER_GRIPPER_JOINT_CLOSE) / pos_range
            pos_normalized = max(0.0, min(1.0, pos_normalized))  # Clamp to [0, 1]

            return {
                "success": True,
                "state": state,
                "position": position,
                "position_normalized": pos_normalized,
                "position_open": FOLLOWER_GRIPPER_JOINT_OPEN,
                "position_closed": FOLLOWER_GRIPPER_JOINT_CLOSE
            }
    
    def set_gripper_position(self, position: float, blocking: bool = True) -> Dict:
        """
        Set gripper to a specific position.

        Args:
            position: Position in radians or normalized (0.0 to 1.0)
            blocking: If True, wait for movement to complete

        Returns:
            Dictionary with status and gripper position
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized", "state": "unknown"}

        # Check if system is in ERROR state (e.g., after emergency stop)
        with self.state_lock:
            if self.current_state == GripperState.ERROR:
                return {"success": False, "error": "System in ERROR state. Call resume_after_stop() to recover.", "state": "error"}

        # If position is between 0 and 1, treat as normalized
        if 0.0 <= position <= 1.0:
            # Convert normalized to actual position
            pos_range = FOLLOWER_GRIPPER_JOINT_OPEN - FOLLOWER_GRIPPER_JOINT_CLOSE
            actual_position = FOLLOWER_GRIPPER_JOINT_CLOSE + (position * pos_range)
        else:
            actual_position = position
        
        # Clamp to valid range
        actual_position = max(FOLLOWER_GRIPPER_JOINT_CLOSE,
                             min(FOLLOWER_GRIPPER_JOINT_OPEN, actual_position))

        print(f"[GripperController] Setting gripper to position: {actual_position:.3f}")

        # Dry-run mode: Simulate movement
        if self.dry_run:
            if blocking:
                time.sleep(0.1)  # Simulate brief movement
                with self.state_lock:
                    self.gripper_position = actual_position
                    # Update state based on position
                    if actual_position >= self.OPEN_THRESHOLD:
                        self.current_state = GripperState.OPEN
                    elif actual_position <= self.CLOSE_THRESHOLD:
                        self.current_state = GripperState.CLOSED
                    else:
                        self.current_state = GripperState.UNKNOWN
            print(f"[GripperController] 🔧 DRY-RUN: Simulated gripper position {actual_position:.3f}")
        else:
            # Hardware mode: Execute real movement
            move_grippers([self.bot], [actual_position], moving_time=1.0)

            if blocking:
                time.sleep(1.0)

        return self.get_gripper_state()
    def verify_grasp(self) -> Dict:
        """
        Verify if an object is actually grasped.

        Uses multiple methods:
        1. Position check: Gripper should not close fully if object is held
        2. Current check: Motor current should be elevated if holding object
        3. Stability check: Position should be stable (not drifting)

        Returns:
        Dictionary with:
        - object_grasped: bool - True if object detected in gripper
        - grasp_verified: bool - True if verification succeeded
        - confidence: float - Confidence level (0.0-1.0)
        - method: str - Verification method used
        - details: dict - Detailed measurements
        """
        if not self.initialized:
            return {
                "object_grasped": False,
                "grasp_verified": False,
                "error": "Not initialized"
            }

        # Dry-run mode: Always return successful grasp
        if self.dry_run:
            return {
                "object_grasped": True,
                "grasp_verified": True,
                "confidence": 1.0,
                "method": "dry_run_simulation",
                "details": {"note": "Dry-run mode - simulated grasp"}
            }

        try:
            # Get current gripper position and motor current
            with self.bot.core.js_mutex:
                gripper_index = self.bot.gripper.left_finger_index
                current_position = self.bot.core.joint_states.position[gripper_index]
                motor_current = abs(self.bot.core.joint_states.current[gripper_index])  # mA

            # Method 1: Position-based detection
            # If gripper commanded to close but stopped before fully closed,
            # something is likely in the gripper
            FULLY_CLOSED_THRESHOLD = FOLLOWER_GRIPPER_JOINT_CLOSE + 0.05
            OPEN_ENOUGH_FOR_OBJECT = FOLLOWER_GRIPPER_JOINT_CLOSE + 0.15

            position_indicates_grasp = (
                current_position > FULLY_CLOSED_THRESHOLD and
                current_position < OPEN_ENOUGH_FOR_OBJECT
            )

            # Method 2: Current-based detection
            # Motor current should be elevated when holding an object
            GRASP_CURRENT_THRESHOLD = 100  # mA - adjust based on testing
            EMPTY_CURRENT_THRESHOLD = 50   # mA - typical empty gripper current

            current_indicates_grasp = motor_current > GRASP_CURRENT_THRESHOLD
            # Method 3: Stability check
            # Wait a moment and check if position is stable
            time.sleep(0.2)
            with self.bot.core.js_mutex:
                position_after = self.bot.core.joint_states.position[gripper_index]

            position_stable = abs(position_after - current_position) < 0.02

            # Combine methods for confidence score
            confidence_score = 0.0
            methods_positive = []

            if position_indicates_grasp:
                confidence_score += 0.5
                methods_positive.append("position")

            if current_indicates_grasp:
                confidence_score += 0.4
                methods_positive.append("current")

            if position_stable:
                confidence_score += 0.1
                methods_positive.append("stability")

            # Decision: Consider grasped if confidence > 0.5
            object_grasped = confidence_score >= 0.5

            result = {
                "object_grasped": object_grasped,
                "grasp_verified": True,
                "confidence": confidence_score,
                "method": "multi_sensor",
                "details": {
                    "position": current_position,
                    "position_indicates_grasp": position_indicates_grasp,
                    "motor_current_mA": motor_current,
                    "current_indicates_grasp": current_indicates_grasp,
                    "position_stable": position_stable,
                    "methods_positive": methods_positive
                }
            }

            if object_grasped:
                print(f"[GripperController] ✓ Object grasped (confidence: {confidence_score:.2f})")
                print(f"[GripperController]   Position: {current_position:.3f} rad, Current: {motor_current:.1f} mA")
            else:
                print(f"[GripperController] ✗ No object detected (confidence: {confidence_score:.2f})")
                print(f"[GripperController]   Position: {current_position:.3f} rad, Current: {motor_current:.1f} mA")

            return result

        except Exception as e:
            print(f"[GripperController] ✗ Grasp verification error: {e}")
            return {
                "object_grasped": False,
                "grasp_verified": False,
                "error": str(e),
                "confidence": 0.0
            }
        
    def emergency_stop(self) -> Dict:
        """
        Emergency stop - immediately disable torque on gripper and enter ERROR state.

        After calling this method:
        - Gripper torque is disabled
        - System enters ERROR state
        - All movement commands will be rejected
        - Call resume_after_stop() to recover and resume operations

        Returns:
            Dict with success status and state
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized"}

        try:
            print("[GripperController] ⚠️ EMERGENCY STOP ACTIVATED!")

            # Dry-run mode: Simulate emergency stop
            if self.dry_run:
                print("[GripperController] 🔧 DRY-RUN: Simulated emergency stop")
            else:
                # Hardware mode: Actually disable torque
                self.bot.core.robot_torque_enable('single', 'gripper', False)

            with self.state_lock:
                self.current_state = GripperState.ERROR
            print("[GripperController] System in ERROR state. Call resume_after_stop() to recover.")
            return {"success": True, "state": "emergency_stopped"}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def resume_after_stop(self) -> Dict:
        """
        Re-enable torque and resume operations after emergency stop.

        This method performs recovery validation before resuming:
        1. Verifies system is in ERROR state
        2. Re-enables gripper torque
        3. Captures current gripper position
        4. Validates current position is safe
        5. Clears ERROR state to allow movements

        Returns:
            Dict with success status, state, and validation details
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized"}

        # Check if we're actually in ERROR state
        with self.state_lock:
            if self.current_state != GripperState.ERROR:
                return {
                    "success": False,
                    "error": f"Not in ERROR state. Current state: {self.current_state.value}",
                    "state": self.current_state.value
                }

        try:
            print("[GripperController] Resuming after emergency stop...")

            # Dry-run mode: Simulate resume
            if self.dry_run:
                print("[GripperController] 🔧 DRY-RUN: Simulated resume after emergency stop")
                with self.state_lock:
                    self.current_state = GripperState.CLOSED
                    current_position = self.gripper_position
                return {
                    "success": True,
                    "state": "resumed",
                    "current_state": "closed",
                    "current_position": current_position,
                    "message": "System recovered from ERROR state (dry-run)"
                }

            # Hardware mode: Actual recovery
            # Re-enable torque
            print("[GripperController] Re-enabling gripper torque")
            self.bot.core.robot_torque_enable('single', 'gripper', True)

            # Capture current position
            print("[GripperController] Capturing current position")
            time.sleep(0.1)  # Brief wait for torque to stabilize

            with self.bot.core.js_mutex:
                gripper_index = self.bot.gripper.left_finger_index
                current_position = self.bot.core.joint_states.position[gripper_index]

            # Validate current position is within safe range
            if current_position < FOLLOWER_GRIPPER_JOINT_CLOSE or current_position > FOLLOWER_GRIPPER_JOINT_OPEN:
                print(f"[GripperController] ⚠️ WARNING: Current position {current_position:.3f} outside safe range")
                print(f"[GripperController] Safe range: [{FOLLOWER_GRIPPER_JOINT_CLOSE:.3f}, {FOLLOWER_GRIPPER_JOINT_OPEN:.3f}]")
                # Still allow resume but warn user
                with self.state_lock:
                    self.current_state = GripperState.UNKNOWN
                    self.gripper_position = current_position
                return {
                    "success": True,
                    "state": "resumed_with_warnings",
                    "warning": f"Position {current_position:.3f} outside normal range",
                    "current_position": current_position,
                    "recommendation": "Check gripper position and reset to safe state"
                }

            # Determine state based on position
            if current_position >= self.OPEN_THRESHOLD:
                new_state = GripperState.OPEN
            elif current_position <= self.CLOSE_THRESHOLD:
                new_state = GripperState.CLOSED
            else:
                new_state = GripperState.UNKNOWN

            # All validations passed
            with self.state_lock:
                self.current_state = new_state
                self.gripper_position = current_position

            print("[GripperController] ✓ System resumed successfully")
            return {
                "success": True,
                "state": "resumed",
                "current_state": new_state.value,
                "current_position": current_position,
                "message": "System recovered from ERROR state"
            }

        except Exception as e:
            print(f"[GripperController] ✗ Failed to resume: {e}")
            # If we enabled torque but failed during position capture, disable it again for safety
            try:
                if not self.dry_run:
                    self.bot.core.robot_torque_enable('single', 'gripper', False)
                    print("[GripperController] Torque disabled after resume failure")
            except:
                pass  # If disabling torque also fails, we can't do much more
            # Keep ERROR state if resume fails
            return {"success": False, "error": str(e), "state": "error"}

    def sleep_arm(self) -> bool:
        """
        Move arm to sleep position.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.initialized:
            print("[GripperController] Cannot sleep - not initialized")
            return False
        
        try:
            print("[GripperController] Moving arm to sleep position...")
            
            # Home position first
            home_position = [0.0, -0.96, 1.16, 0.0, -0.3, 0.0]
            move_arms([self.bot], [home_position], moving_time=3.0)
            
            # Sleep position with wrist pointing up
            sleep_positions = [0.0, -1.85, 1.55, 0.0, -1.57, 0.0]
            move_arms([self.bot], [sleep_positions], moving_time=3.0)
            
            print("[GripperController] ✓ Arm in sleep position")
            return True
            
        except Exception as e:
            print(f"[GripperController] ✗ Sleep failed: {e}")
            return False
    
    def shutdown(self):
        """
        Shutdown robot connection and cleanup.
        """
        print("[GripperController] Shutting down...")
        self.initialized = False

        # Dry-run mode: Skip hardware shutdown
        if self.dry_run:
            print("[GripperController] 🔧 DRY-RUN: Skipping hardware shutdown")
        elif self.node:
            robot_shutdown(self.node)

        print("[GripperController] ✓ Shutdown complete")
    
    def __del__(self):
        """Cleanup on deletion"""
        if self.initialized:
            self.shutdown()


# Convenience functions for simple usage
_global_controller = None

def get_controller() -> GripperController:
    """
    Get or create global controller instance.

    .. deprecated:: 2.3
        The global controller singleton pattern is deprecated and will be removed in version 2.0.
        Instead, create and manage controller instances explicitly:

        Example:
            # Old (deprecated):
            controller = get_controller()

            # New (recommended):
            controller = GripperController(robot_model='vx300s', robot_name='follower_left')
            controller.initialize()
    """
    warnings.warn(
        "get_controller() is deprecated and will be removed in version 2.0. "
        "Create controller instances explicitly: controller = GripperController(robot_model='vx300s', robot_name='follower_left'); controller.initialize()",
        DeprecationWarning,
        stacklevel=2
    )
    global _global_controller
    if _global_controller is None:
        _global_controller = GripperController()
        _global_controller.initialize()
    return _global_controller

def open_gripper() -> Dict:
    """
    Simple function to open gripper.

    .. deprecated:: 2.3
        This convenience function is deprecated and will be removed in version 2.0.
        Use an explicit controller instance instead:

        Example:
            # Old (deprecated):
            open_gripper()

            # New (recommended):
            controller = GripperController(robot_model='vx300s', robot_name='follower_left')
            controller.initialize()
            controller.open_gripper()
    """
    warnings.warn(
        "open_gripper() is deprecated and will be removed in version 2.0. "
        "Use controller.open_gripper() with an explicit GripperController instance.",
        DeprecationWarning,
        stacklevel=2
    )
    return get_controller().open_gripper()

def close_gripper() -> Dict:
    """
    Simple function to close gripper.

    .. deprecated:: 2.3
        This convenience function is deprecated and will be removed in version 2.0.
        Use an explicit controller instance instead:

        Example:
            # Old (deprecated):
            close_gripper()

            # New (recommended):
            controller = GripperController(robot_model='vx300s', robot_name='follower_left')
            controller.initialize()
            controller.close_gripper()
    """
    warnings.warn(
        "close_gripper() is deprecated and will be removed in version 2.0. "
        "Use controller.close_gripper() with an explicit GripperController instance.",
        DeprecationWarning,
        stacklevel=2
    )
    return get_controller().close_gripper()

def get_gripper_state() -> Dict:
    """
    Simple function to get gripper state.

    .. deprecated:: 2.3
        This convenience function is deprecated and will be removed in version 2.0.
        Use an explicit controller instance instead:

        Example:
            # Old (deprecated):
            state = get_gripper_state()

            # New (recommended):
            controller = GripperController(robot_model='vx300s', robot_name='follower_left')
            controller.initialize()
            state = controller.get_gripper_state()
    """
    warnings.warn(
        "get_gripper_state() is deprecated and will be removed in version 2.0. "
        "Use controller.get_gripper_state() with an explicit GripperController instance.",
        DeprecationWarning,
        stacklevel=2
    )
    return get_controller().get_gripper_state()

def cleanup():
    """
    Cleanup global controller.

    .. deprecated:: 2.3
        This cleanup function is deprecated and will be removed in version 2.0.
        Manage controller lifecycle explicitly instead:

        Example:
            # Old (deprecated):
            cleanup()

            # New (recommended):
            controller = GripperController(robot_model='vx300s', robot_name='follower_left')
            controller.initialize()
            # ... use controller ...
            controller.shutdown()
    """
    warnings.warn(
        "cleanup() is deprecated and will be removed in version 2.0. "
        "Use controller.shutdown() with an explicit GripperController instance.",
        DeprecationWarning,
        stacklevel=2
    )
    global _global_controller
    if _global_controller:
        _global_controller.shutdown()
        _global_controller = None