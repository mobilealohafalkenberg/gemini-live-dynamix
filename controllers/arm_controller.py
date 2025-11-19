#!/usr/bin/env python3

"""
Arm Controller API for Mobile ALOHA
Provides flexible arm control with automatic format detection and conversion
Designed for integration with Gemini Live API

REFACTORED: Now uses DynamixelController + Modern Robotics IK (No ROS2)
"""

import time
import threading
import math
import uuid
from enum import Enum
from typing import Dict, List, Optional, Tuple, Union
import numpy as np
import logging
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

# Direct Dynamixel control (replaces ROS2)
from dynamixel_controller import DynamixelController
from models.vx300s_model import VX300S

# Modern Robotics for IK/FK
import modern_robotics as mr

# Import safety validator
try:
    from controllers.safety_validator import SafetyValidator, RiskLevel
    SAFETY_ENABLED = True
except ImportError:
    print("[ArmController] Warning: Safety validator not available")
    SAFETY_ENABLED = False


class ArmState(Enum):
    """Arm states for status tracking"""
    IDLE = "idle"
    MOVING = "moving"
    AT_HOME = "at_home"
    AT_SLEEP = "at_sleep"
    AT_TARGET = "at_target"
    ERROR = "error"
    UNKNOWN = "unknown"


class TrajectoryStatus(Enum):
    """Trajectory execution status"""
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELED = "canceled"


class ArmController:
    """
    Main API class for controlling the Mobile ALOHA arm.
    Supports multiple input formats with automatic detection and conversion.
    
    Features:
    - Joint space control (radians or degrees)
    - Cartesian space control (x, y, z, roll, pitch, yaw)
    - Named poses (home, sleep, ready)
    - Automatic unit detection
    - Safe workspace limits
    """
    
    # Standard poses (in radians)
    POSES = {
        'home': [0.0, -0.3, 0.6, 0.0, -0.3, 0.0],  # More centered, moderate position
        'sleep': [0.0, -1.85, 1.55, 0.0, -1.57, 0.0],
        'ready': [0.0, -0.96, 1.16, 0.0, -0.3, 0.0],  # START_ARM_POSE[:6]
    }
    
    # Workspace limits (meters) - matches VX300S model
    WORKSPACE = {
        'x': (0.15, 0.50),   # Forward reach only - cannot reach behind base
        'y': (-0.30, 0.30),  # Left-right reach
        'z': (0.05, 0.40),   # Vertical reach above table
    }
    
    # Safety constraints to prevent self-collision
    SAFETY_CONSTRAINTS = {
        # Absolute maximum wrist rotation limit (prevents cable/camera damage)
        'max_safe_wrist_rotation': math.radians(120),  # Absolute maximum safe rotation ±120°

        # If end effector is close to base (x < threshold), limit wrist rotation
        'close_to_base_x_threshold': 0.15,  # meters
        'close_to_base_y_threshold': 0.10,  # meters
        'wrist_rotate_limit_when_close': math.radians(30),  # Max ±30° when close to base

        # Dangerous joint combinations to avoid
        'min_shoulder_angle': math.radians(-110),  # Don't fold too far back
        'max_elbow_angle': math.radians(100),      # Don't over-extend elbow

        # When gripper is pointing down and close to base, restrict rotation
        'wrist_angle_down_threshold': math.radians(-45),  # Wrist pointing down
        'safe_distance_from_base': 0.25,  # meters - minimum safe distance for full rotation
        'wrist_rotate_limit_when_down_at_base': math.radians(20),  # Very restricted when down near base

        # Additional constraints for low z positions
        'low_z_threshold': 0.15,  # meters - when gripper is low
        'wrist_rotate_limit_when_low': math.radians(45),  # Max rotation when low
        'wrist_rotate_limit_at_table_level': math.radians(60),  # Rotation limit at low position to prevent camera collision

        # Dangerous combination thresholds
        'dangerous_shoulder_back_threshold': math.radians(-90),  # Shoulder back threshold
        'dangerous_elbow_extended_threshold': math.radians(80),  # Elbow extended threshold
        'dangerous_wrist_rotation_threshold': math.radians(90),  # Wrist rotation in dangerous combo
    }
    
    def __init__(self, dynamixel_controller: Optional[DynamixelController] = None,
                 robot_model: Optional[VX300S] = None,
                 enable_safety=True, dry_run=False):
        """Initialize controller (does not connect to robot yet)

        Args:
            dynamixel_controller: Shared DynamixelController instance
            robot_model: VX300S kinematics model (created if not provided)
            enable_safety: Enable safety validation
            dry_run: If True, validate but don't execute movements
        """
        self.dxl = dynamixel_controller  # Shared DynamixelController
        self.model = robot_model or VX300S()  # Kinematics model
        self.initialized = False
        self.current_state = ArmState.UNKNOWN
        self.current_joints = [0.0] * 6
        self.current_ee_pose = None
        self.state_lock = threading.Lock()
        self.monitor_failure_count = 0  # Tracks how many times the monitor has failed consecutively
        self.dry_run = dry_run

        # Movement parameters
        self.default_moving_time = 2.0
        self.default_accel_time = 0.3

        # Trajectory tracking for async execution
        self.active_trajectories = {}  # trajectory_id -> trajectory info
        self.trajectory_lock = threading.Lock()
        self.cancel_flags = {}  # trajectory_id -> threading.Event for cancellation

        # Initialize safety validator
        self.safety_enabled = enable_safety and SAFETY_ENABLED
        if self.safety_enabled:
            self.safety_validator = SafetyValidator(safety_profile="strict")
            print(f"[ArmController] Safety validator enabled (dry_run={dry_run})")
        else:
            self.safety_validator = None
            if enable_safety and not SAFETY_ENABLED:
                print("[ArmController] Safety requested but validator not available")
        
    def initialize(self) -> bool:
        """
        Initialize robot connection and move to ready position.
        Returns True if successful, False otherwise.
        """
        try:
            print("[ArmController] Initializing arm controller...")

            # Verify DynamixelController is provided and initialized
            if self.dxl is None:
                print("[ArmController] ✗ No DynamixelController provided")
                return False

            # Verify DynamixelController is connected
            if not self.dxl.port_handler or not self.dxl.port_handler.is_open:
                print("[ArmController] ✗ DynamixelController not connected")
                return False

            # Enable torque on arm motors
            print("[ArmController] Enabling torque on arm motors...")
            arm_motor_ids = [1, 2, 4, 6, 7, 8]  # Skip shadow motors 3, 5 and gripper 9
            self.dxl.enable_torque(arm_motor_ids)

            # Read current position
            print("[ArmController] Reading current position...")
            current_joints = self.dxl.get_joint_positions_radians()
            if current_joints is None:
                print("[ArmController] ✗ Failed to read joint positions")
                return False

            with self.state_lock:
                self.current_joints = list(current_joints)

            # Move to ready position
            print("[ArmController] Moving to ready position...")
            self.move_to_pose('ready', blocking=True)

            self.initialized = True

            # Start position monitoring thread
            self._start_position_monitor()

            print("[ArmController] ✓ Initialization complete")
            return True

        except Exception as e:
            print(f"[ArmController] ✗ Initialization failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _start_position_monitor(self):
        """Start background thread to monitor arm position.

        Uses cached state from DynamixelController to avoid port contention.
        The DynamixelController's monitoring thread handles the actual serial reads.
        """
        def monitor():
            while self.initialized:
                try:
                    # Get cached joint positions from DynamixelController
                    # This does NOT access the serial port - it reads from cached state
                    joints = self.dxl.get_cached_joint_radians()

                    if joints is not None and len(joints) == 6:
                        # Compute forward kinematics to get end effector pose
                        ee_pose = mr.FKinSpace(self.model.M, self.model.Slist, joints)

                        # Update shared state with lock protection
                        with self.state_lock:
                            self.current_joints = list(joints)
                            self.current_ee_pose = ee_pose

                        # Reset failure counter on success
                        self.monitor_failure_count = 0
                    else:
                        self.monitor_failure_count += 1

                except Exception as e:
                    self.monitor_failure_count += 1
                    logging.error(
                        f"Position monitor failed (failure #{self.monitor_failure_count}): {type(e).__name__}: {e}",
                        exc_info=True
                    )
                    # Alert if failures are excessive
                    if self.monitor_failure_count >= 5:
                        logging.critical(
                            f"Position monitor has failed {self.monitor_failure_count} consecutive times! "
                            "This may indicate a serious hardware or connection issue."
                        )

                time.sleep(0.1)  # Check 10 times per second

        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
    
    def _build_transformation_matrix(self, position: Dict[str, float], orientation: List[float]) -> np.ndarray:
        """
        Build SE(3) transformation matrix from position and orientation.

        Args:
            position: Dict with x, y, z in meters
            orientation: List of [roll, pitch, yaw] in radians

        Returns:
            4x4 homogeneous transformation matrix
        """
        roll, pitch, yaw = orientation

        # Build rotation matrix from RPY (Z-Y-X convention)
        R = mr.MatrixExp3(mr.VecToso3([0, 0, yaw])) @ \
            mr.MatrixExp3(mr.VecToso3([0, pitch, 0])) @ \
            mr.MatrixExp3(mr.VecToso3([roll, 0, 0]))

        # Build transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [position['x'], position['y'], position['z']]

        return T

    def check_safety_constraints(self, joint_positions: List[float], ee_position: Optional[Dict] = None) -> Tuple[bool, str]:
        """
        Check if joint positions are safe (no self-collision risk).

        Args:
            joint_positions: List of 6 joint angles in radians
            ee_position: Optional end effector position dict with x, y, z

        Returns:
            (is_safe, warning_message)
        """
        if len(joint_positions) != 6:
            return True, ""  # Skip check if invalid input
        
        waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate = joint_positions
        
        # Log for debugging
        print(f"[Safety] Checking joints: wrist_rotate={math.degrees(wrist_rotate):.1f}°, "
              f"wrist_angle={math.degrees(wrist_angle):.1f}°, shoulder={math.degrees(shoulder):.1f}°")
        if ee_position:
            print(f"[Safety] Position: x={ee_position.get('x', 0):.3f}, "
                  f"y={ee_position.get('y', 0):.3f}, z={ee_position.get('z', 0):.3f}")
        
        # Check 1: Shoulder angle limit
        if shoulder < self.SAFETY_CONSTRAINTS['min_shoulder_angle']:
            return False, f"Shoulder angle too far back ({math.degrees(shoulder):.1f}°), risk of self-collision"
        
        # Check 2: Elbow over-extension
        if elbow > self.SAFETY_CONSTRAINTS['max_elbow_angle']:
            return False, f"Elbow over-extended ({math.degrees(elbow):.1f}°), risk of mechanical damage"
        
        # Check 3: Large wrist rotation is ALWAYS dangerous with this robot configuration
        # The camera and cables can get damaged with large rotations
        max_safe_rotation = self.SAFETY_CONSTRAINTS['max_safe_wrist_rotation']
        if abs(wrist_rotate) > max_safe_rotation:
            return False, (f"Wrist rotation ({math.degrees(wrist_rotate):.1f}°) exceeds safe limit. "
                         f"Max allowed: ±{math.degrees(max_safe_rotation):.1f}°")
        
        # Check 4: Position-based constraints
        if ee_position:
            x = ee_position.get('x', 0.5)
            y = ee_position.get('y', 0.5)
            z = ee_position.get('z', 0.5)
            
            # Calculate distance from base in XY plane
            distance_from_base = math.sqrt(x**2 + y**2)
            
            # Check 4a: If close to base in XY, restrict wrist rotation
            if distance_from_base < self.SAFETY_CONSTRAINTS['safe_distance_from_base']:
                max_rotation = self.SAFETY_CONSTRAINTS['wrist_rotate_limit_when_close']
                if abs(wrist_rotate) > max_rotation:
                    return False, (f"Wrist rotation ({math.degrees(wrist_rotate):.1f}°) too large "
                                 f"when close to base (distance: {distance_from_base:.2f}m). "
                                 f"Max allowed: ±{math.degrees(max_rotation):.1f}°")
            
            # Check 4b: If low to table (small z), restrict wrist rotation
            if z <= self.SAFETY_CONSTRAINTS['low_z_threshold']:
                max_rotation = self.SAFETY_CONSTRAINTS['wrist_rotate_limit_when_low']
                if abs(wrist_rotate) > max_rotation:
                    return False, (f"Wrist rotation ({math.degrees(wrist_rotate):.1f}°) too large "
                                 f"when low to table (z={z:.2f}m). "
                                 f"Max allowed: ±{math.degrees(max_rotation):.1f}°")
            
            # Check 4c: Wrist pointing down and close to base
            if (wrist_angle < self.SAFETY_CONSTRAINTS['wrist_angle_down_threshold'] and
                distance_from_base < self.SAFETY_CONSTRAINTS['safe_distance_from_base']):
                # Extra strict on rotation when pointing down near base
                max_rotation = self.SAFETY_CONSTRAINTS['wrist_rotate_limit_when_down_at_base']
                if abs(wrist_rotate) > max_rotation:
                    return False, (f"Wrist rotation restricted when pointing down "
                                 f"({math.degrees(wrist_angle):.1f}°) near base. "
                                 f"Max allowed: ±{math.degrees(max_rotation):.1f}°. "
                                 f"Risk of cable/camera damage")
        
        # Check 5: Dangerous combination - shoulder back + elbow extended + wrist rotated
        if (shoulder < self.SAFETY_CONSTRAINTS['dangerous_shoulder_back_threshold'] and
            elbow > self.SAFETY_CONSTRAINTS['dangerous_elbow_extended_threshold'] and
            abs(wrist_rotate) > self.SAFETY_CONSTRAINTS['dangerous_wrist_rotation_threshold']):
            return False, "Dangerous joint combination: shoulder back + elbow extended + wrist rotated"
        
        # Check 6: Specific dangerous configuration when reaching forward at table level
        # This is the configuration that was hitting the camera
        max_rotation_at_table = self.SAFETY_CONSTRAINTS['wrist_rotate_limit_at_table_level']
        if (ee_position and ee_position.get('z', 1.0) <= self.SAFETY_CONSTRAINTS['low_z_threshold'] and
            abs(wrist_rotate) > max_rotation_at_table):
            return False, (f"Wrist rotation ({math.degrees(wrist_rotate):.1f}°) restricted "
                         f"at low position (z={ee_position.get('z', 0):.2f}m). "
                         f"Max allowed: ±{math.degrees(max_rotation_at_table):.1f}° to prevent camera collision")
        
        return True, ""
    
    def parse_joint_angles(self, angles: List[float], unit: str = 'auto') -> List[float]:
        """
        Parse joint angles with automatic unit detection.
        
        Args:
            angles: List of 6 joint angles
            unit: 'auto', 'radians', or 'degrees'
            
        Returns:
            List of angles in radians
        """
        if len(angles) != 6:
            raise ValueError(f"Expected 6 joint angles, got {len(angles)}")
        
        # Auto-detect unit based on value ranges
        if unit == 'auto':
            # If any absolute value > 2π, likely degrees
            if any(abs(a) > 2 * math.pi for a in angles):
                print(f"[ArmController] Auto-detected degrees (max value: {max(abs(a) for a in angles):.2f})")
                return [math.radians(a) for a in angles]
            else:
                print(f"[ArmController] Auto-detected radians (max value: {max(abs(a) for a in angles):.2f})")
                return angles
        elif unit == 'degrees':
            return [math.radians(a) for a in angles]
        else:  # radians
            return angles
    
    def parse_position(self, 
                      position: Union[List[float], Dict[str, float]], 
                      format: str = 'auto') -> Dict[str, float]:
        """
        Parse position coordinates with format detection.
        
        Args:
            position: List [x,y,z] or [y,x] or dict with x,y,z keys
            format: 'auto', 'xyz', 'yx', or 'dict'
            
        Returns:
            Dictionary with x, y, z in meters
        """
        if isinstance(position, dict):
            # Already in dict format
            result = {
                'x': position.get('x', 0.0),
                'y': position.get('y', 0.0),
                'z': position.get('z', 0.0)
            }
        elif isinstance(position, (list, tuple)):
            if len(position) == 2:
                # Assume [y, x] format (Gemini trajectory style)
                # Check if values are normalized (0-1000 range)
                if any(abs(v) > 10 for v in position):
                    # Likely normalized coordinates
                    result = {
                        'x': position[1] / 1000.0,
                        'y': position[0] / 1000.0,
                        'z': 0.0
                    }
                else:
                    # Already in meters
                    result = {
                        'x': position[1],
                        'y': position[0],
                        'z': 0.0
                    }
            elif len(position) == 3:
                # Standard [x, y, z] format
                result = {'x': position[0], 'y': position[1], 'z': position[2]}
            else:
                raise ValueError(f"Position must have 2 or 3 elements, got {len(position)}")
        else:
            raise ValueError(f"Position must be list or dict, got {type(position)}")
        
        # Validate workspace limits
        for axis, (min_val, max_val) in self.WORKSPACE.items():
            if result[axis] < min_val or result[axis] > max_val:
                print(f"[ArmController] Warning: {axis}={result[axis]:.3f} outside workspace [{min_val}, {max_val}]")
                result[axis] = max(min_val, min(max_val, result[axis]))
        
        return result

    def _convert_waypoint_to_position(self, point: List[float]) -> List[float]:
        """
        Convert waypoint format to position list [x, y, z].

        Handles two formats:
        - 2D: [y, x] (normalized or meters) -> [x, y, z] with default z=0.2
        - 3D: [x, y, z] -> returned as-is

        For 2D points, coordinates > 1 are treated as normalized (0-1000 range)
        and divided by 1000 to convert to meters.

        Args:
            point: Waypoint coordinates as list

        Returns:
            Position as [x, y, z] in meters

        Examples:
            >>> _convert_waypoint_to_position([500, 300])  # normalized [y, x]
            [0.3, 0.5, 0.2]

            >>> _convert_waypoint_to_position([0.5, 0.3])  # meters [y, x]
            [0.3, 0.5, 0.2]

            >>> _convert_waypoint_to_position([0.25, 0.1, 0.15])  # [x, y, z]
            [0.25, 0.1, 0.15]
        """
        if len(point) == 2:
            # [y, x] format - convert to [x, y, z]
            # Normalize coordinates > 1 (assumed to be in 0-1000 range)
            x = point[1] / 1000.0 if point[1] > 1 else point[1]
            y = point[0] / 1000.0 if point[0] > 1 else point[0]
            z = 0.2  # Default working height
            return [x, y, z]
        else:
            # 3D format or other - return as-is
            return point

    def move_joints(self, 
                   joint_positions: List[float],
                   unit: str = 'auto',
                   moving_time: Optional[float] = None,
                   blocking: bool = True) -> Dict:
        """
        Move arm to specified joint positions.
        
        Args:
            joint_positions: List of 6 joint angles
            unit: 'auto', 'radians', or 'degrees'
            moving_time: Time to complete movement (seconds)
            blocking: Wait for movement to complete
            
        Returns:
            Status dictionary
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized", "state": "unknown"}
        

        # Check if system is in ERROR state (e.g., after emergency stop)
        with self.state_lock:
            if self.current_state == ArmState.ERROR:
                return {"success": False, "error": "System in ERROR state. Call resume_after_stop() to recover.", "state": "error"}
        try:
            # Parse and convert angles
            angles_rad = self.parse_joint_angles(joint_positions, unit)
            
            # Safety check before movement
            is_safe, warning = self.check_safety_constraints(angles_rad)
            if not is_safe:
                print(f"[ArmController] ⚠️ SAFETY WARNING: {warning}")
                with self.state_lock:
                    self.current_state = ArmState.ERROR
                return {"success": False, "error": f"Safety constraint violated: {warning}", "state": "error"}

            # If dry run mode, don't execute actual movement
            if self.dry_run:
                print(f"[ArmController] DRY RUN: Would move to joint positions: {[f'{a:.3f}' for a in angles_rad]}")
                angles_deg = [math.degrees(a) for a in angles_rad]
                return {
                    "success": True,
                    "state": "dry_run",
                    "target_joints": angles_rad,
                    "target_joints_degrees": angles_deg,
                    "message": "Dry run - movement validated but not executed"
                }

            with self.state_lock:
                self.current_state = ArmState.MOVING

            print(f"[ArmController] Moving to joints: {[f'{a:.3f}' for a in angles_rad]}")

            # Use DynamixelController to set joint positions
            self.dxl.set_joint_positions_radians(np.array(angles_rad))

            # If blocking, wait for movement to complete
            if blocking:
                time.sleep(moving_time or self.default_moving_time)

            with self.state_lock:
                self.current_state = ArmState.AT_TARGET
                self.current_joints = angles_rad

            return self.get_arm_state()
            
        except Exception as e:
            with self.state_lock:
                self.current_state = ArmState.ERROR
            return {"success": False, "error": str(e), "state": "error"}
    
    def move_to_position(self,
                        position: Union[List[float], Dict[str, float]],
                        orientation: Optional[List[float]] = None,
                        format: str = 'auto',
                        moving_time: Optional[float] = None,
                        blocking: bool = True) -> Dict:
        """
        Move end effector to Cartesian position.
        
        Args:
            position: Target position (x,y,z) or (y,x)
            orientation: Optional (roll, pitch, yaw) in radians
            format: Position format ('auto', 'xyz', 'yx')
            moving_time: Time to complete movement
            blocking: Wait for movement to complete
            
        Returns:
            Status dictionary
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized", "state": "unknown"}
        

        # Check if system is in ERROR state (e.g., after emergency stop)
        with self.state_lock:
            if self.current_state == ArmState.ERROR:
                return {"success": False, "error": "System in ERROR state. Call resume_after_stop() to recover.", "state": "error"}
        try:
            # Parse position
            pos = self.parse_position(position, format)
            
            # Safety validation
            if self.safety_enabled and self.safety_validator:
                validation = self.safety_validator.validate_position(pos['x'], pos['y'], pos['z'])
                if not validation.valid:
                    print(f"[ArmController] ⚠️ SAFETY: {validation.message}")
                    return {
                        "success": False,
                        "error": validation.message,
                        "state": "blocked",
                        "safety": validation.to_dict()
                    }
                elif validation.risk_level == RiskLevel.NEEDS_CONFIRMATION:
                    print(f"[ArmController] ⚠️ SAFETY WARNING: {validation.message}")
                    # In production, you might want to require confirmation here
                    # For now, we'll log but continue
                
                # If dry run mode, don't execute actual movement
                if self.dry_run:
                    print(f"[ArmController] DRY RUN: Would move to x={pos['x']:.3f}, y={pos['y']:.3f}, z={pos['z']:.3f}")
                    return {
                        "success": True,
                        "state": "dry_run",
                        "target_position": pos,
                        "safety": validation.to_dict(),
                        "message": "Dry run - movement validated but not executed"
                    }
            
            # Default orientation if not provided
            if orientation is None:
                # Calculate yaw to face the target
                yaw = math.atan2(pos['y'], pos['x'])
                orientation = [0.0, 0.0, yaw]
            
            with self.state_lock:
                self.current_state = ArmState.MOVING

            print(f"[ArmController] Moving to position: x={pos['x']:.3f}, y={pos['y']:.3f}, z={pos['z']:.3f}")

            # Build SE(3) transformation matrix for target pose
            T_target = self._build_transformation_matrix(pos, orientation)

            # Get current joint positions for IK initial guess
            current_joints = self.dxl.get_joint_positions_radians()
            if current_joints is None:
                with self.state_lock:
                    self.current_state = ArmState.ERROR
                return {"success": False, "error": "Failed to read current position", "state": "error"}

            # Run IK using Modern Robotics
            print(f"[ArmController] Computing IK solution...")
            joint_solution, success = mr.IKinSpace(
                self.model.Slist,
                self.model.M,
                T_target,
                current_joints,
                eomg=0.01,  # Angular error tolerance
                ev=0.001    # Linear error tolerance
            )

            if not success:
                with self.state_lock:
                    self.current_state = ArmState.ERROR
                return {"success": False, "error": "IK solution not found", "state": "error"}

            # Convert to list
            joint_list = list(joint_solution)

            print(f"[ArmController] IK solution: joints={[f'{math.degrees(j):.1f}°' for j in joint_list]}")

            # Safety check the IK solution before executing
            is_safe, warning = self.check_safety_constraints(joint_list, ee_position=pos)
            if not is_safe:
                print(f"[ArmController] ⚠️ SAFETY BLOCKED: {warning}")
                with self.state_lock:
                    self.current_state = ArmState.ERROR
                return {"success": False, "error": f"Safety: {warning}", "state": "error"}

            print(f"[ArmController] Safety check passed, executing movement")

            # Execute the safe movement
            self.dxl.set_joint_positions_radians(np.array(joint_list))

            # If blocking, wait for movement to complete
            if blocking:
                time.sleep(moving_time or self.default_moving_time)

            with self.state_lock:
                self.current_state = ArmState.AT_TARGET
                self.current_joints = joint_list

            return self.get_arm_state()
            
        except Exception as e:
            with self.state_lock:
                self.current_state = ArmState.ERROR
            return {"success": False, "error": str(e), "state": "error"}
    
    def move_to_pose(self, 
                     pose_name: str,
                     moving_time: Optional[float] = None,
                     blocking: bool = True) -> Dict:
        """
        Move to a named pose.
        
        Args:
            pose_name: 'home', 'sleep', or 'ready'
            moving_time: Time to complete movement
            blocking: Wait for movement to complete
            
        Returns:
            Status dictionary
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized", "state": "unknown"}

        # Check if system is in ERROR state (e.g., after emergency stop)
        with self.state_lock:
            if self.current_state == ArmState.ERROR:
                return {"success": False, "error": "System in ERROR state. Call resume_after_stop() to recover.", "state": "error"}

        if pose_name not in self.POSES:
            return {"success": False, "error": f"Unknown pose: {pose_name}", "state": "error"}
        
        # Safety check the named pose (these should always be safe, but check anyway)
        pose_joints = self.POSES[pose_name]
        is_safe, warning = self.check_safety_constraints(pose_joints)
        if not is_safe:
            print(f"[ArmController] ⚠️ WARNING: Named pose '{pose_name}' failed safety check: {warning}")
            # Still allow named poses but log the warning

        # If dry run mode, log and delegate to move_joints (which handles dry-run)
        if self.dry_run:
            print(f"[ArmController] DRY RUN: Would move to {pose_name} pose")
        else:
            print(f"[ArmController] Moving to {pose_name} pose")

        # Update state based on target pose
        target_state = {
            'home': ArmState.AT_HOME,
            'sleep': ArmState.AT_SLEEP,
            'ready': ArmState.IDLE
        }.get(pose_name, ArmState.AT_TARGET)

        # Use slower movement for sleep and home positions if not specified
        if moving_time is None and pose_name in ['sleep', 'home']:
            moving_time = 3.0  # 3 seconds for slow, safe transition
            if not self.dry_run:
                print(f"[ArmController] Using slow transition ({moving_time}s) for {pose_name} pose")

        result = self.move_joints(
            pose_joints,
            unit='radians',
            moving_time=moving_time,
            blocking=blocking
        )
        
        if result['success']:
            with self.state_lock:
                self.current_state = target_state

        return result

    def opening_ceremony(self, moving_time: float = 4.0, blocking: bool = True) -> Dict:
        """
        Perform opening ceremony - slow, deliberate movement to ready position.

        This is intended to be called when establishing connection with the robot,
        providing a safe and predictable startup sequence.

        Args:
            moving_time: Time for the movement (default 4.0 seconds for safety)
            blocking: Wait for movement to complete

        Returns:
            Status dictionary with ceremony result
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized", "state": "unknown"}

        print(f"[ArmController] Starting opening ceremony (moving to ready in {moving_time}s)")

        # Set slow profile velocity for smooth ceremony movement
        # Value 40 is very slow compared to default 131
        self.dxl.set_profile_velocity(40)
        self.dxl.set_profile_acceleration(50)

        # Move slowly to ready position
        result = self.move_to_pose('ready', moving_time=moving_time, blocking=blocking)

        # Restore normal profile velocity for regular operations
        self.dxl.set_profile_velocity(131)
        self.dxl.set_profile_acceleration(0)  # 0 = use default/immediate

        if result.get('success'):
            print("[ArmController] Opening ceremony complete - arm is ready")
        else:
            print(f"[ArmController] Opening ceremony failed: {result.get('error')}")

        return result

    def closing_ceremony(self, moving_time: float = 4.0, blocking: bool = True) -> Dict:
        """
        Perform closing ceremony - slow, deliberate movement to sleep position.

        IMPORTANT: This keeps torque ON so the arm holds its sleep position safely.
        This prevents the arm from falling and potentially breaking.

        Args:
            moving_time: Time for the movement (default 4.0 seconds for safety)
            blocking: Wait for movement to complete

        Returns:
            Status dictionary with ceremony result
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized", "state": "unknown"}

        print(f"[ArmController] Starting closing ceremony (moving to sleep in {moving_time}s)")

        # Set slow profile velocity for smooth ceremony movement
        # Value 40 is very slow compared to default 131
        self.dxl.set_profile_velocity(40)
        self.dxl.set_profile_acceleration(50)

        # Move slowly to sleep position
        result = self.move_to_pose('sleep', moving_time=moving_time, blocking=blocking)

        # Restore normal profile velocity for regular operations
        self.dxl.set_profile_velocity(131)
        self.dxl.set_profile_acceleration(0)  # 0 = use default/immediate

        if result.get('success'):
            print("[ArmController] Closing ceremony complete - arm is in sleep position")
            print("[ArmController] Torque remains ON to hold position safely")
        else:
            print(f"[ArmController] Closing ceremony failed: {result.get('error')}")

        return result

    def initialize_without_movement(self) -> bool:
        """
        Initialize robot connection WITHOUT moving to ready position.

        This is useful for delayed initialization where you want to control
        when the opening ceremony happens (e.g., from frontend connect button).

        Returns:
            True if successful, False otherwise.
        """
        try:
            print("[ArmController] Initializing arm controller (no movement)...")

            # Verify DynamixelController is provided and initialized
            if self.dxl is None:
                print("[ArmController] ✗ No DynamixelController provided")
                return False

            # Verify DynamixelController is connected
            if not self.dxl.port_handler or not self.dxl.port_handler.is_open:
                print("[ArmController] ✗ DynamixelController not connected")
                return False

            # Enable torque on arm motors
            print("[ArmController] Enabling torque on arm motors...")
            arm_motor_ids = [1, 2, 4, 6, 7, 8]  # Skip shadow motors 3, 5 and gripper 9
            self.dxl.enable_torque(arm_motor_ids)

            # Read current position
            print("[ArmController] Reading current position...")
            current_joints = self.dxl.get_joint_positions_radians()
            if current_joints is None:
                print("[ArmController] ✗ Failed to read joint positions")
                return False

            with self.state_lock:
                self.current_joints = list(current_joints)
                self.current_state = ArmState.IDLE

            self.initialized = True

            # Start position monitoring thread
            self._start_position_monitor()

            print("[ArmController] ✓ Initialization complete (awaiting opening ceremony)")
            return True

        except Exception as e:
            print(f"[ArmController] ✗ Initialization failed: {e}")
            import traceback
            traceback.print_exc()
            return False

    def execute_trajectory(self, waypoints: List[Dict], speed: str = 'slow', coordinate_with_gripper=None, blocking: bool = True) -> Dict:
        """
        Execute a multi-waypoint trajectory with optional gripper coordination.

        Args:
            waypoints: List of waypoint dictionaries with:
                - 'point': [x,y,z] position or [y,x] normalized
                - 'label': descriptive name for waypoint
                - 'gripper_action': optional 'open', 'close', or 'maintain'
            speed: 'slow', 'medium', or 'fast'
            coordinate_with_gripper: Optional gripper controller for coordinated actions
            blocking: If True, wait for trajectory completion. If False, return immediately with trajectory_id

        Returns:
            Dictionary with trajectory execution results (if blocking=True) or trajectory_id (if blocking=False)
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized", "state": "unknown"}

        # Check if system is in ERROR state (e.g., after emergency stop)
        with self.state_lock:
            if self.current_state == ArmState.ERROR:
                return {"success": False, "error": "System in ERROR state. Call resume_after_stop() to recover.", "state": "error"}

        # For non-blocking mode, start trajectory in background thread
        if not blocking:
            trajectory_id = str(uuid.uuid4())

            # Pre-validate all waypoints if safety is enabled
            if self.safety_enabled and self.safety_validator:
                print(f"[ArmController] Pre-validating {len(waypoints)} waypoints for safety...")
                for i, waypoint in enumerate(waypoints):
                    point = waypoint.get('point', [])
                    label = waypoint.get('label', f'waypoint_{i}')

                    # Convert point format if needed
                    position = self._convert_waypoint_to_position(point)

                    if len(position) >= 3:
                        validation = self.safety_validator.validate_position(position[0], position[1], position[2])
                        if not validation.valid:
                            print(f"[ArmController] ⚠️ SAFETY: Waypoint '{label}' blocked: {validation.message}")
                            return {
                                "success": False,
                                "error": f"Waypoint '{label}' failed safety check: {validation.message}",
                                "waypoint_index": i,
                                "safety": validation.to_dict()
                            }

            # Initialize trajectory tracking
            with self.trajectory_lock:
                self.active_trajectories[trajectory_id] = {
                    'status': TrajectoryStatus.RUNNING,
                    'waypoints': waypoints,
                    'speed': speed,
                    'progress': 0,
                    'current_waypoint': 0,
                    'total_waypoints': len(waypoints),
                    'started_at': time.time(),
                    'completed_at': None,
                    'result': None,
                    'error': None
                }
                self.cancel_flags[trajectory_id] = threading.Event()

            # Start trajectory in background thread
            thread = threading.Thread(
                target=self._execute_trajectory_async,
                args=(trajectory_id, waypoints, speed, coordinate_with_gripper),
                daemon=True
            )
            thread.start()

            print(f"[ArmController] Started trajectory {trajectory_id} in background ({len(waypoints)} waypoints)")

            return {
                'success': True,
                'trajectory_id': trajectory_id,
                'blocking': False,
                'total_waypoints': len(waypoints)
            }

        # Blocking mode - execute synchronously
        return self._execute_trajectory_sync(waypoints, speed, coordinate_with_gripper)

    def _execute_trajectory_sync(self, waypoints: List[Dict], speed: str, coordinate_with_gripper=None) -> Dict:
        """Execute trajectory synchronously (blocking mode)"""
        # Map speed to moving_time
        speed_map = {'slow': 2.5, 'medium': 1.5, 'fast': 0.8}
        moving_time = speed_map.get(speed, 1.5)

        # Pre-validate all waypoints if safety is enabled
        if self.safety_enabled and self.safety_validator:
            print(f"[ArmController] Pre-validating {len(waypoints)} waypoints for safety...")
            for i, waypoint in enumerate(waypoints):
                point = waypoint.get('point', [])
                label = waypoint.get('label', f'waypoint_{i}')

                # Convert point format if needed
                position = self._convert_waypoint_to_position(point)

                if len(position) >= 3:
                    validation = self.safety_validator.validate_position(position[0], position[1], position[2])
                    if not validation.valid:
                        print(f"[ArmController] ⚠️ SAFETY: Waypoint '{label}' blocked: {validation.message}")
                        return {
                            "success": False,
                            "error": f"Waypoint '{label}' failed safety check: {validation.message}",
                            "waypoint_index": i,
                            "safety": validation.to_dict()
                        }

        waypoint_results = []
        overall_success = True

        print(f"[ArmController] Starting trajectory with {len(waypoints)} waypoints at {speed} speed")

        for i, waypoint in enumerate(waypoints):
            point = waypoint.get('point', [])
            label = waypoint.get('label', f'waypoint_{i}')
            gripper_action = waypoint.get('gripper_action')

            # Convert point format if needed
            position = self._convert_waypoint_to_position(point)

            print(f"[ArmController] Waypoint {i+1}/{len(waypoints)} '{label}': {[f'{p:.3f}' for p in position]}")

            # Move arm to waypoint
            result = self.move_to_position(
                position=position,
                moving_time=moving_time,
                blocking=True  # Wait for completion
            )

            # Coordinate gripper action if controller provided
            if gripper_action and coordinate_with_gripper:
                try:
                    if gripper_action == 'open':
                        coordinate_with_gripper.open_gripper()
                        time.sleep(0.5)
                        print(f"[ArmController] ✓ Gripper opened")
                    elif gripper_action == 'close':
                        coordinate_with_gripper.close_gripper()
                        time.sleep(0.5)
                        print(f"[ArmController] ✓ Gripper closed")
                except Exception as e:
                    print(f"[ArmController] ✗ Gripper action failed: {e}")

            waypoint_results.append({
                'label': label,
                'position': position,
                'success': result.get('success', False)
            })

            if not result.get('success', False):
                overall_success = False
                print(f"[ArmController] ✗ Trajectory aborted at waypoint '{label}'")
                break

        completion_msg = f"✓ Completed {len(waypoint_results)}/{len(waypoints)} waypoints" if overall_success else f"✗ Failed at waypoint {len(waypoint_results)}"
        print(f"[ArmController] {completion_msg}")

        return {
            'success': overall_success,
            'waypoints_completed': waypoint_results,
            'total_waypoints': len(waypoints),
            'final_state': self.get_arm_state()
        }

    def _execute_trajectory_async(self, trajectory_id: str, waypoints: List[Dict], speed: str, coordinate_with_gripper=None):
        """Execute trajectory asynchronously in background thread"""
        try:
            # Map speed to moving_time
            speed_map = {'slow': 2.5, 'medium': 1.5, 'fast': 0.8}
            moving_time = speed_map.get(speed, 1.5)

            waypoint_results = []
            overall_success = True
            cancel_event = self.cancel_flags.get(trajectory_id)

            print(f"[ArmController] Executing trajectory {trajectory_id} with {len(waypoints)} waypoints at {speed} speed")

            for i, waypoint in enumerate(waypoints):
                # Check for cancellation
                if cancel_event and cancel_event.is_set():
                    print(f"[ArmController] Trajectory {trajectory_id} canceled at waypoint {i}")
                    with self.trajectory_lock:
                        self.active_trajectories[trajectory_id]['status'] = TrajectoryStatus.CANCELED
                        self.active_trajectories[trajectory_id]['error'] = 'Canceled by user'
                        self.active_trajectories[trajectory_id]['completed_at'] = time.time()
                    return

                point = waypoint.get('point', [])
                label = waypoint.get('label', f'waypoint_{i}')
                gripper_action = waypoint.get('gripper_action')

                # Convert point format if needed
                position = self._convert_waypoint_to_position(point)

                print(f"[ArmController] Trajectory {trajectory_id} - Waypoint {i+1}/{len(waypoints)} '{label}': {[f'{p:.3f}' for p in position]}")

                # Update progress
                with self.trajectory_lock:
                    self.active_trajectories[trajectory_id]['current_waypoint'] = i
                    self.active_trajectories[trajectory_id]['progress'] = i / len(waypoints)

                # Move arm to waypoint
                result = self.move_to_position(
                    position=position,
                    moving_time=moving_time,
                    blocking=True  # Still block within the async thread
                )

                # Coordinate gripper action if controller provided
                if gripper_action and coordinate_with_gripper:
                    try:
                        if gripper_action == 'open':
                            coordinate_with_gripper.open_gripper()
                            time.sleep(0.5)
                            print(f"[ArmController] Trajectory {trajectory_id} - ✓ Gripper opened")
                        elif gripper_action == 'close':
                            coordinate_with_gripper.close_gripper()
                            time.sleep(0.5)
                            print(f"[ArmController] Trajectory {trajectory_id} - ✓ Gripper closed")
                    except Exception as e:
                        print(f"[ArmController] Trajectory {trajectory_id} - ✗ Gripper action failed: {e}")

                waypoint_results.append({
                    'label': label,
                    'position': position,
                    'success': result.get('success', False)
                })

                if not result.get('success', False):
                    overall_success = False
                    print(f"[ArmController] Trajectory {trajectory_id} - ✗ Aborted at waypoint '{label}'")
                    break

            # Update final status
            completion_msg = f"✓ Completed {len(waypoint_results)}/{len(waypoints)} waypoints" if overall_success else f"✗ Failed at waypoint {len(waypoint_results)}"
            print(f"[ArmController] Trajectory {trajectory_id} - {completion_msg}")

            with self.trajectory_lock:
                self.active_trajectories[trajectory_id]['status'] = TrajectoryStatus.COMPLETED if overall_success else TrajectoryStatus.FAILED
                self.active_trajectories[trajectory_id]['progress'] = 1.0
                self.active_trajectories[trajectory_id]['completed_at'] = time.time()
                self.active_trajectories[trajectory_id]['result'] = {
                    'success': overall_success,
                    'waypoints_completed': waypoint_results,
                    'total_waypoints': len(waypoints),
                    'final_state': self.get_arm_state()
                }

        except Exception as e:
            print(f"[ArmController] Trajectory {trajectory_id} - Exception: {e}")
            with self.trajectory_lock:
                self.active_trajectories[trajectory_id]['status'] = TrajectoryStatus.FAILED
                self.active_trajectories[trajectory_id]['error'] = str(e)
                self.active_trajectories[trajectory_id]['completed_at'] = time.time()
        finally:
            # Clean up cancel flag
            if trajectory_id in self.cancel_flags:
                del self.cancel_flags[trajectory_id]

    def get_trajectory_status(self, trajectory_id: str) -> Dict:
        """
        Get status of an async trajectory execution.

        Args:
            trajectory_id: The trajectory ID returned from execute_trajectory(blocking=False)

        Returns:
            Dictionary containing:
            - found: bool - whether trajectory exists
            - status: TrajectoryStatus - current status (running/completed/failed/canceled)
            - progress: float - completion progress (0.0 to 1.0)
            - current_waypoint: int - index of current waypoint being executed
            - total_waypoints: int - total number of waypoints
            - started_at: float - timestamp when trajectory started
            - completed_at: float - timestamp when trajectory completed (if finished)
            - result: dict - final result (if completed)
            - error: str - error message (if failed or canceled)
        """
        with self.trajectory_lock:
            if trajectory_id not in self.active_trajectories:
                return {
                    'found': False,
                    'error': 'Trajectory not found'
                }

            traj = self.active_trajectories[trajectory_id]

            return {
                'found': True,
                'trajectory_id': trajectory_id,
                'status': traj['status'].value,
                'progress': traj['progress'],
                'current_waypoint': traj['current_waypoint'],
                'total_waypoints': traj['total_waypoints'],
                'started_at': traj['started_at'],
                'completed_at': traj['completed_at'],
                'result': traj['result'],
                'error': traj['error']
            }

    def cancel_trajectory(self, trajectory_id: str) -> Dict:
        """
        Cancel an async trajectory execution.

        Args:
            trajectory_id: The trajectory ID to cancel

        Returns:
            Dictionary with success status and message
        """
        with self.trajectory_lock:
            if trajectory_id not in self.active_trajectories:
                return {
                    'success': False,
                    'error': 'Trajectory not found'
                }

            traj = self.active_trajectories[trajectory_id]

            # Check if already completed
            if traj['status'] in [TrajectoryStatus.COMPLETED, TrajectoryStatus.FAILED, TrajectoryStatus.CANCELED]:
                return {
                    'success': False,
                    'error': f'Trajectory already {traj["status"].value}',
                    'status': traj['status'].value
                }

        # Set cancel flag
        if trajectory_id in self.cancel_flags:
            self.cancel_flags[trajectory_id].set()
            print(f"[ArmController] Cancellation requested for trajectory {trajectory_id}")
            return {
                'success': True,
                'message': 'Trajectory cancellation requested',
                'trajectory_id': trajectory_id
            }
        else:
            return {
                'success': False,
                'error': 'Cancel flag not found - trajectory may have completed'
            }

    def list_trajectories(self) -> Dict:
        """
        List all tracked trajectories (active and completed).

        Returns:
            Dictionary with list of trajectory summaries
        """
        with self.trajectory_lock:
            trajectories = []
            for traj_id, traj in self.active_trajectories.items():
                trajectories.append({
                    'trajectory_id': traj_id,
                    'status': traj['status'].value,
                    'progress': traj['progress'],
                    'current_waypoint': traj['current_waypoint'],
                    'total_waypoints': traj['total_waypoints'],
                    'started_at': traj['started_at'],
                    'completed_at': traj['completed_at']
                })

            return {
                'success': True,
                'trajectories': trajectories,
                'count': len(trajectories)
            }

    def get_arm_state(self) -> Dict:
        """
        Get current arm state and position.
        
        Returns:
            Dictionary containing:
            - success: bool
            - state: current state
            - joints: current joint positions (radians)
            - joints_degrees: current joint positions (degrees)
            - ee_position: end effector position (x,y,z)
            - ee_orientation: end effector orientation (roll,pitch,yaw)
        """
        if not self.initialized:
            return {
                "success": False,
                "error": "Not initialized",
                "state": ArmState.UNKNOWN.value,
                "joints": [0.0] * 6,
                "joints_degrees": [0.0] * 6,
            }
        
        with self.state_lock:
            # Get end effector pose from transformation matrix
            ee_pos = None
            ee_orient = None
            if self.current_ee_pose is not None:
                # Extract position from transformation matrix
                ee_pos = {
                    'x': float(self.current_ee_pose[0, 3]),
                    'y': float(self.current_ee_pose[1, 3]),
                    'z': float(self.current_ee_pose[2, 3])
                }
                
                # Extract orientation (simplified - would need proper rotation matrix to euler conversion)
                # For now, just report the yaw based on position
                yaw = math.atan2(ee_pos['y'], ee_pos['x'])
                ee_orient = {'roll': 0.0, 'pitch': 0.0, 'yaw': yaw}
            
            return {
                "success": True,
                "state": self.current_state.value,
                "joints": list(self.current_joints),
                "joints_degrees": [math.degrees(j) for j in self.current_joints],
                "ee_position": ee_pos,
                "ee_orientation": ee_orient,
                "pose": self._detect_current_pose()
            }
    
    def _detect_current_pose(self) -> Optional[str]:
        """Detect if current position matches a named pose"""
        tolerance = 0.1  # radians
        
        for pose_name, pose_joints in self.POSES.items():
            if all(abs(a - b) < tolerance for a, b in zip(self.current_joints, pose_joints)):
                return pose_name
        
        return None
    
    def set_speed(self, moving_time: float, accel_time: Optional[float] = None):
        """
        Set default movement speed.

        Args:
            moving_time: Default time for movements (seconds), must be positive
            accel_time: Acceleration time (seconds), must be positive and less than moving_time

        Raises:
            ValueError: If parameters are invalid (negative, zero, or accel_time >= moving_time)
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized", "state": "unknown"}

        # Check if system is in ERROR state (e.g., after emergency stop)
        with self.state_lock:
            if self.current_state == ArmState.ERROR:
                return {"success": False, "error": "System in ERROR state. Call resume_after_stop() to recover.", "state": "error"}

        # Define reasonable bounds for safety
        MAX_MOVING_TIME = 10.0  # Maximum 10 seconds per movement
        MAX_ACCEL_TIME = 5.0    # Maximum 5 seconds acceleration
        MIN_TIME = 0.01         # Minimum 10ms (practical lower bound)

        # Validate moving_time
        if moving_time <= 0:
            raise ValueError(f"moving_time must be positive, got {moving_time}")

        if moving_time < MIN_TIME:
            raise ValueError(f"moving_time must be at least {MIN_TIME}s for safe operation, got {moving_time}s")

        if moving_time > MAX_MOVING_TIME:
            raise ValueError(f"moving_time exceeds maximum safe limit of {MAX_MOVING_TIME}s, got {moving_time}s")

        # Validate accel_time if provided
        if accel_time is not None:
            if accel_time <= 0:
                raise ValueError(f"accel_time must be positive, got {accel_time}")

            if accel_time < MIN_TIME:
                raise ValueError(f"accel_time must be at least {MIN_TIME}s for safe operation, got {accel_time}s")

            if accel_time > MAX_ACCEL_TIME:
                raise ValueError(f"accel_time exceeds maximum safe limit of {MAX_ACCEL_TIME}s, got {accel_time}s")

            # Validate relationship: accel_time must be less than moving_time
            if accel_time >= moving_time:
                raise ValueError(
                    f"accel_time ({accel_time}s) must be less than moving_time ({moving_time}s). "
                    f"Robot cannot accelerate for longer than the total movement time."
                )

        # All validations passed, set the values
        self.default_moving_time = moving_time
        if accel_time is not None:
            self.default_accel_time = accel_time

        print(f"[ArmController] Speed set: moving_time={moving_time}s, accel_time={self.default_accel_time}s")
    
    def emergency_stop(self) -> Dict:
        """
        Emergency stop - immediately disable torque on all joints and enter ERROR state.

        After calling this method:
        - All joint torques are disabled
        - System enters ERROR state
        - All movement commands will be rejected
        - Call resume_after_stop() to recover and resume operations

        Returns:
            Dict with success status and state
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized"}

        try:
            print("[ArmController] ⚠️ EMERGENCY STOP ACTIVATED!")
            # Disable torque on arm motors
            arm_motor_ids = [1, 2, 4, 6, 7, 8]
            self.dxl.disable_torque(arm_motor_ids)
            with self.state_lock:
                self.current_state = ArmState.ERROR
            print("[ArmController] System in ERROR state. Call resume_after_stop() to recover.")
            return {"success": True, "state": "emergency_stopped"}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def resume_after_stop(self) -> Dict:
        """
        Re-enable torque and resume operations after emergency stop.

        This method performs recovery validation before resuming:
        1. Verifies system is in ERROR state
        2. Re-enables motor torque
        3. Captures current arm position
        4. Validates current position is safe
        5. Clears ERROR state to allow movements

        Returns:
            Dict with success status, state, and validation details
        """
        if not self.initialized:
            return {"success": False, "error": "Not initialized"}

        # Check if we're actually in ERROR state
        with self.state_lock:
            if self.current_state != ArmState.ERROR:
                return {
                    "success": False,
                    "error": f"Not in ERROR state. Current state: {self.current_state.value}",
                    "state": self.current_state.value
                }

        try:
            print("[ArmController] Resuming after emergency stop...")

            # Re-enable torque
            print("[ArmController] Re-enabling motor torque")
            arm_motor_ids = [1, 2, 4, 6, 7, 8]
            self.dxl.enable_torque(arm_motor_ids)

            # Read and validate current position
            print("[ArmController] Reading current position")
            current_joints = self.dxl.get_joint_positions_radians()
            if current_joints is None:
                print("[ArmController] ✗ Failed to read current position")
                return {"success": False, "error": "Failed to read current position", "state": "error"}

            current_joints_list = list(current_joints)

            # Validate current position is safe
            is_safe, warning = self.check_safety_constraints(current_joints_list)
            if not is_safe:
                print(f"[ArmController] ⚠️ WARNING: Current position unsafe after resume: {warning}")
                print("[ArmController] Recommend moving to 'home' or 'ready' pose")
                # Still allow resume but warn user
                with self.state_lock:
                    self.current_state = ArmState.IDLE
                    self.current_joints = current_joints_list
                return {
                    "success": True,
                    "state": "resumed_with_warnings",
                    "warning": warning,
                    "current_joints": current_joints_list,
                    "recommendation": "Move to a safe pose ('home' or 'ready') before other operations"
                }

            # All validations passed
            with self.state_lock:
                self.current_state = ArmState.IDLE
                self.current_joints = current_joints_list

            print("[ArmController] ✓ System resumed successfully")
            return {
                "success": True,
                "state": "resumed",
                "current_joints": current_joints_list,
                "message": "System recovered from ERROR state"
            }

        except Exception as e:
            print(f"[ArmController] ✗ Failed to resume: {e}")
            # Keep ERROR state if resume fails
            return {"success": False, "error": str(e), "state": "error"}

    def shutdown(self):
        """Shutdown robot connection and cleanup."""
        print("[ArmController] Shutting down...")
        self.initialized = False

        # Note: DynamixelController is shared, so we don't close it here
        # The bridge that created it will handle cleanup

        print("[ArmController] ✓ Shutdown complete")
    
    def __del__(self):
        """Cleanup on deletion"""
        if self.initialized:
            self.shutdown()


# Convenience functions for simple usage
_global_controller = None

def get_controller() -> ArmController:
    """
    Get or create global controller instance.

    .. deprecated:: 1.9
        The global controller singleton pattern is deprecated and will be removed in version 2.0.
        Instead, create and manage controller instances explicitly:

        Example:
            # Old (deprecated):
            controller = get_controller()

            # New (recommended):
            controller = ArmController(robot_name='vx300s', group_name='arm')
            controller.initialize()

    Returns:
        ArmController: The global controller instance
    """
    import warnings
    warnings.warn(
        "get_controller() is deprecated and will be removed in version 2.0. "
        "Create controller instances explicitly: controller = ArmController(robot_name='vx300s', group_name='arm'); controller.initialize()",
        DeprecationWarning,
        stacklevel=2
    )
    global _global_controller
    if _global_controller is None:
        _global_controller = ArmController()
        _global_controller.initialize()
    return _global_controller

def move_arm(position=None, joints=None, pose=None, **kwargs) -> Dict:
    """
    Simple function to move arm using global controller.

    .. deprecated:: 1.9
        The global controller singleton pattern is deprecated and will be removed in version 2.0.
        Instead, create and manage controller instances explicitly:

        Example:
            # Old (deprecated):
            move_arm(position=[0.3, 0.0, 0.2])

            # New (recommended):
            controller = ArmController(robot_name='vx300s', group_name='arm')
            controller.initialize()
            controller.move_to_position([0.3, 0.0, 0.2])

    Args:
        position: Target position [x, y, z]
        joints: Target joint angles
        pose: Named pose ('home', 'ready', 'sleep')
        **kwargs: Additional arguments passed to movement methods

    Returns:
        Dict: Movement result
    """
    import warnings
    warnings.warn(
        "move_arm() is deprecated and will be removed in version 2.0. "
        "Create controller instances explicitly and call movement methods directly.",
        DeprecationWarning,
        stacklevel=2
    )
    controller = get_controller()

    if pose is not None:
        return controller.move_to_pose(pose, **kwargs)
    elif joints is not None:
        return controller.move_joints(joints, **kwargs)
    elif position is not None:
        return controller.move_to_position(position, **kwargs)
    else:
        return {"success": False, "error": "No target specified"}

def get_arm_state() -> Dict:
    """
    Simple function to get arm state from global controller.

    .. deprecated:: 1.9
        The global controller singleton pattern is deprecated and will be removed in version 2.0.
        Instead, create and manage controller instances explicitly:

        Example:
            # Old (deprecated):
            state = get_arm_state()

            # New (recommended):
            controller = ArmController(robot_name='vx300s', group_name='arm')
            controller.initialize()
            state = controller.get_arm_state()

    Returns:
        Dict: Current arm state
    """
    import warnings
    warnings.warn(
        "get_arm_state() is deprecated and will be removed in version 2.0. "
        "Create controller instances explicitly and call get_arm_state() method directly.",
        DeprecationWarning,
        stacklevel=2
    )
    return get_controller().get_arm_state()

def cleanup():
    """
    Cleanup global controller.

    .. deprecated:: 1.9
        The global controller singleton pattern is deprecated and will be removed in version 2.0.
        Instead, create and manage controller instances explicitly:

        Example:
            # Old (deprecated):
            cleanup()

            # New (recommended):
            controller = ArmController(robot_name='vx300s', group_name='arm')
            controller.initialize()
            # ... use controller ...
            controller.shutdown()
    """
    import warnings
    warnings.warn(
        "cleanup() is deprecated and will be removed in version 2.0. "
        "Create controller instances explicitly and call shutdown() method directly.",
        DeprecationWarning,
        stacklevel=2
    )
    global _global_controller
    if _global_controller:
        _global_controller.shutdown()
        _global_controller = None
