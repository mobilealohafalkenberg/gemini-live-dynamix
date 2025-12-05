#!/usr/bin/env python3
"""
Dynamixel Controller for VX300S Robot Arm
Direct motor control via Dynamixel SDK (no ROS2 required)

This controller manages all 9 Dynamixel motors:
- 6 arm joints (waist, shoulder, elbow, forearm, wrist angle, wrist rotate)
- 2 shadow motors (shoulder + elbow, for higher torque)
- 1 gripper motor

CRITICAL: Shadow motors 2�3 (shoulder) and 4�5 (elbow) must ALWAYS move together!
"""

import time
import threading
import numpy as np
import yaml
import glob
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dynamixel_sdk import *


class DynamixelController:
    """
    Direct Dynamixel motor controller using Dynamixel SDK.

    Replaces ROS2 + Interbotix SDK for simpler, faster control.
    """

    @staticmethod
    def detect_follower_ports() -> List[Dict[str, str]]:
        """
        Detect available follower arms by scanning /dev/ttyDXL_* ports.

        Returns:
            List of dicts with 'port' and 'arm_id' keys
            Example: [
                {'port': '/dev/ttyDXL_follower_left', 'arm_id': 'follower_left'},
                {'port': '/dev/ttyDXL_follower_right', 'arm_id': 'follower_right'}
            ]
            Returns empty list if no follower arms found.
        """
        # Find all ttyDXL devices
        all_ports = glob.glob('/dev/ttyDXL_*')

        # Filter for follower arms only
        follower_pattern = re.compile(r'/dev/ttyDXL_(follower_(?:left|right))')

        detected = []
        for port in all_ports:
            match = follower_pattern.match(port)
            if match:
                arm_id = match.group(1)  # e.g., 'follower_left'
                detected.append({
                    'port': port,
                    'arm_id': arm_id
                })

        return detected

    # Dynamixel Protocol 2.0 Control Table Addresses
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132
    ADDR_PRESENT_VELOCITY = 128
    ADDR_PRESENT_CURRENT = 126
    ADDR_OPERATING_MODE = 11
    ADDR_VELOCITY_LIMIT = 44
    ADDR_CURRENT_LIMIT = 38
    ADDR_DRIVE_MODE = 10
    ADDR_POSITION_P_GAIN = 84
    ADDR_POSITION_I_GAIN = 82
    ADDR_POSITION_D_GAIN = 80
    ADDR_PROFILE_VELOCITY = 112
    ADDR_PROFILE_ACCELERATION = 108

    # Dynamixel Protocol Version
    PROTOCOL_VERSION = 2.0

    def __init__(self, port: str = '/dev/ttyDXL_follower_right', baudrate: int = 1000000,
                 config_file: Optional[str] = None):
        """
        Initialize Dynamixel controller.

        Args:
            port: Serial port device (e.g., '/dev/ttyDXL_follower_right', '/dev/ttyUSB0')
            baudrate: Communication baudrate (default 1Mbps)
            config_file: Path to YAML configuration file
        """
        self.port = port
        self.baudrate = baudrate
        self.config_file = config_file

        # Extract arm_id from port name (e.g., '/dev/ttyDXL_follower_right' -> 'follower_right')
        self.arm_id = None
        if 'follower_left' in port:
            self.arm_id = 'follower_left'
        elif 'follower_right' in port:
            self.arm_id = 'follower_right'

        # Port and packet handlers
        self.port_handler = None
        self.packet_handler = None

        # Motor configuration
        self.motor_config = {}
        self.shadow_motors = {}  # Maps primary -> shadow and vice versa
        self.gripper_calibration = {}  # Per-arm gripper calibration values

        # State tracking
        self.current_positions = {}  # motor_id -> position (Dynamixel units)
        self.current_positions_rad = np.zeros(6)  # Joint angles in radians
        self.state_lock = threading.Lock()

        # Port access lock - CRITICAL: All serial I/O must acquire this lock
        # The Dynamixel SDK PortHandler is NOT thread-safe
        self.port_lock = threading.Lock()

        # Monitoring thread
        self.monitoring_thread = None
        self.monitoring_active = False

        # Load configuration if provided
        if config_file:
            self.load_config(config_file)

    def load_config(self, config_file: str):
        """
        Load motor configuration from YAML file.

        Args:
            config_file: Path to configuration file
        """
        config_path = Path(config_file)
        if not config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_file}")

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.motor_config = config.get('motors', {})
        self.gripper_calibration = config.get('gripper_calibration', {})
        self.joint_calibration = config.get('joint_calibration', {})

        # Build shadow motor mapping
        for motor_id, motor_data in self.motor_config.items():
            if 'Secondary_ID' in motor_data:
                secondary = motor_data['Secondary_ID']
                self.shadow_motors[motor_id] = secondary
                print(f"[DynamixelController] Shadow pair: {motor_id} ↔ {secondary}")

        print(f"[DynamixelController] Loaded configuration for {len(self.motor_config)} motors")
        if self.arm_id and self.arm_id in self.joint_calibration:
            print(f"[DynamixelController] Using calibrated joint limits for {self.arm_id}")

    def get_calibrated_limits(self, motor_id: int) -> Tuple[int, int]:
        """
        Get calibrated position limits for a motor.

        Uses per-arm calibration if available, falls back to global motor config.

        Args:
            motor_id: Motor ID

        Returns:
            Tuple of (min_position, max_position) in Dynamixel units
        """
        # Try per-arm calibration first
        if self.arm_id and self.arm_id in self.joint_calibration:
            arm_calib = self.joint_calibration[self.arm_id]
            if motor_id in arm_calib:
                return (arm_calib[motor_id]['min'], arm_calib[motor_id]['max'])

        # Fall back to global motor config
        if motor_id in self.motor_config:
            motor = self.motor_config[motor_id]
            return (motor.get('Min_Position', 0), motor.get('Max_Position', 4095))

        # Default to full range
        return (0, 4095)

    def initialize_motors(self) -> bool:
        """
        Initialize Dynamixel communication and configure motors.

        Returns:
            True if successful
        """
        try:
            print(f"[DynamixelController] Opening port {self.port} at {self.baudrate} baud...")

            # Initialize port
            self.port_handler = PortHandler(self.port)
            self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

            # Open port
            if not self.port_handler.openPort():
                raise Exception(f"Failed to open port {self.port}")

            # Set baudrate
            if not self.port_handler.setBaudRate(self.baudrate):
                raise Exception(f"Failed to set baudrate to {self.baudrate}")

            print(f"[DynamixelController] ✓ Connected to {self.port} at {self.baudrate} baud")

            # Configure each motor
            print("[DynamixelController] Initializing motors...")
            for motor_id, config in self.motor_config.items():
                self._configure_motor(motor_id, config)

            print("[DynamixelController] ✓ Motor initialization complete")

            self.set_profile_velocity(40)
            self.set_profile_acceleration(50)

            # Read initial positions
            self.sync_read_positions()

            return True

        except Exception as e:
            print(f"[DynamixelController] ✗ Initialization failed: {e}")
            return False

    def _configure_motor(self, motor_id: int, config: dict):
        """
        Configure a single motor with settings from config.

        Args:
            motor_id: Motor ID (1-9)
            config: Motor configuration dictionary
        """
        # Set operating mode (Position Control = 3)
        self.write_register(motor_id, self.ADDR_OPERATING_MODE, 1, 3)

        # Set drive mode (normal or reversed)
        drive_mode = config.get('Drive_Mode', 0)
        self.write_register(motor_id, self.ADDR_DRIVE_MODE, 1, drive_mode)

        # Set velocity limit
        vel_limit = config.get('Velocity_Limit', 131)
        self.write_register(motor_id, self.ADDR_VELOCITY_LIMIT, 4, vel_limit)

        # Show calibrated limits for this arm
        min_pos, max_pos = self.get_calibrated_limits(motor_id)
        print(f"[DynamixelController]   Motor {motor_id}: Mode=3, Drive={drive_mode}, "
              f"Limits=[{min_pos}, {max_pos}]")

    def enable_torque(self, motor_ids: Optional[List[int]] = None):
        """
        Enable torque on specified motors (or all if None).

        Args:
            motor_ids: List of motor IDs, or None for all
        """
        if motor_ids is None:
            motor_ids = list(self.motor_config.keys())

        for motor_id in motor_ids:
            self.write_register(motor_id, self.ADDR_TORQUE_ENABLE, 1, 1)

        print(f"[DynamixelController] Torque enabled on motors: {motor_ids}")

    def disable_torque(self, motor_ids: Optional[List[int]] = None):
        """
        Disable torque on specified motors (or all if None).

        Args:
            motor_ids: List of motor IDs, or None for all
        """
        if motor_ids is None:
            motor_ids = list(self.motor_config.keys())

        for motor_id in motor_ids:
            self.write_register(motor_id, self.ADDR_TORQUE_ENABLE, 1, 0)

        print(f"[DynamixelController] Torque disabled on motors: {motor_ids}")

    def set_profile_velocity(self, velocity: int, motor_ids: Optional[List[int]] = None):
        """
        Set Profile Velocity for motors to control movement speed.

        Profile Velocity determines how fast the motor moves to its goal position.
        Lower values = slower movement, higher values = faster movement.
        Value of 0 means use the maximum velocity (Velocity_Limit).

        Args:
            velocity: Profile velocity value (0-1023 typical range)
                     - 0: Maximum speed (no profile, uses Velocity_Limit)
                     - 30-50: Very slow (good for ceremonies)
                     - 100: Moderate speed
                     - 131: Default/normal speed
            motor_ids: List of motor IDs, or None for all arm motors
        """
        if motor_ids is None:
            # Default to arm motors (skip gripper 9)
            motor_ids = [1, 2, 3, 4, 5, 6, 7, 8]

        for motor_id in motor_ids:
            self.write_register(motor_id, self.ADDR_PROFILE_VELOCITY, 4, velocity)

        print(f"[DynamixelController] Profile velocity set to {velocity} on motors: {motor_ids}")

    def set_profile_acceleration(self, acceleration: int, motor_ids: Optional[List[int]] = None):
        """
        Set Profile Acceleration for motors to control movement smoothness.

        Profile Acceleration determines how quickly the motor accelerates/decelerates.
        Lower values = smoother but slower start/stop, higher values = quicker response.

        Args:
            acceleration: Profile acceleration value (0-32767 typical range)
                         - 0: Infinite acceleration (immediate)
                         - 50-100: Smooth acceleration (good for ceremonies)
                         - 200+: Quick response
            motor_ids: List of motor IDs, or None for all arm motors
        """
        if motor_ids is None:
            # Default to arm motors (skip gripper 9)
            motor_ids = [1, 2, 3, 4, 5, 6, 7, 8]

        for motor_id in motor_ids:
            self.write_register(motor_id, self.ADDR_PROFILE_ACCELERATION, 4, acceleration)

        print(f"[DynamixelController] Profile acceleration set to {acceleration} on motors: {motor_ids}")

    def write_register(self, motor_id: int, address: int, size: int, value: int) -> bool:
        """
        Write value to motor control table.

        Args:
            motor_id: Motor ID
            address: Control table address
            size: Data size (1, 2, or 4 bytes)
            value: Value to write

        Returns:
            True if successful
        """
        with self.port_lock:
            if size == 1:
                result, error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, motor_id, address, value)
            elif size == 2:
                result, error = self.packet_handler.write2ByteTxRx(
                    self.port_handler, motor_id, address, value)
            elif size == 4:
                result, error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, motor_id, address, value)
            else:
                raise ValueError(f"Invalid size: {size}")

            if result != COMM_SUCCESS:
                print(f"[DynamixelController] Write error: {self.packet_handler.getTxRxResult(result)}")
                return False

            if error != 0:
                print(f"[DynamixelController] Motor {motor_id} error: "
                      f"{self.packet_handler.getRxPacketError(error)}")
                return False

            return True

    def read_register(self, motor_id: int, address: int, size: int) -> Optional[int]:
        """
        Read value from motor control table.

        Args:
            motor_id: Motor ID
            address: Control table address
            size: Data size (1, 2, or 4 bytes)

        Returns:
            Value or None if error
        """
        with self.port_lock:
            if size == 1:
                value, result, error = self.packet_handler.read1ByteTxRx(
                    self.port_handler, motor_id, address)
            elif size == 2:
                value, result, error = self.packet_handler.read2ByteTxRx(
                    self.port_handler, motor_id, address)
            elif size == 4:
                value, result, error = self.packet_handler.read4ByteTxRx(
                    self.port_handler, motor_id, address)
            else:
                raise ValueError(f"Invalid size: {size}")

            if result != COMM_SUCCESS or error != 0:
                return None

            return value

    def sync_read_positions(self) -> Dict[int, int]:
        """
        Read positions from all motors efficiently using GroupSyncRead.

        Returns:
            Dictionary mapping motor_id -> position
        """
        with self.port_lock:
            # Create GroupSyncRead
            group_sync_read = GroupSyncRead(
                self.port_handler, self.packet_handler,
                self.ADDR_PRESENT_POSITION, 4
            )

            # Add all motors to read
            for motor_id in self.motor_config.keys():
                group_sync_read.addParam(motor_id)

            # Transmit packet
            result = group_sync_read.txRxPacket()
            if result != COMM_SUCCESS:
                print(f"[DynamixelController] Sync read error: "
                      f"{self.packet_handler.getTxRxResult(result)}")
                return {}

            # Get data
            positions = {}
            for motor_id in self.motor_config.keys():
                if group_sync_read.isAvailable(motor_id, self.ADDR_PRESENT_POSITION, 4):
                    position = group_sync_read.getData(motor_id, self.ADDR_PRESENT_POSITION, 4)
                    positions[motor_id] = position
                else:
                    print(f"[DynamixelController] Failed to read motor {motor_id}")

            # Clear parameters
            group_sync_read.clearParam()

        # Update state (outside port_lock to avoid nested locks)
        with self.state_lock:
            self.current_positions = positions

        return positions

    def sync_write_positions(self, positions: Dict[int, int]):
        """
        Write positions to multiple motors efficiently using GroupSyncWrite.

        CRITICAL: Automatically handles shadow motor coordination!
        If you write to motor 2, motor 3 will also be commanded.
        If you write to motor 4, motor 5 will also be commanded.

        Args:
            positions: Dictionary mapping motor_id -> position (Dynamixel units)
        """
        # Expand positions to include shadow motors
        expanded_positions = positions.copy()

        # Handle shadow motor coordination
        for primary_id, shadow_id in self.shadow_motors.items():
            if primary_id in positions and shadow_id not in positions:
                # Primary commanded, also command shadow
                expanded_positions[shadow_id] = positions[primary_id]
            elif shadow_id in positions and primary_id not in positions:
                # Shadow commanded, also command primary
                expanded_positions[primary_id] = positions[shadow_id]

        with self.port_lock:
            # Create GroupSyncWrite
            group_sync_write = GroupSyncWrite(
                self.port_handler, self.packet_handler,
                self.ADDR_GOAL_POSITION, 4
            )

            # Add parameters for each motor
            for motor_id, position in expanded_positions.items():
                # Clamp position to calibrated limits for this arm/motor
                min_pos, max_pos = self.get_calibrated_limits(motor_id)
                position = max(min_pos, min(max_pos, int(position)))

                # Convert to byte array
                position_bytes = [
                    DXL_LOBYTE(DXL_LOWORD(position)),
                    DXL_HIBYTE(DXL_LOWORD(position)),
                    DXL_LOBYTE(DXL_HIWORD(position)),
                    DXL_HIBYTE(DXL_HIWORD(position))
                ]

                # Add to group
                if not group_sync_write.addParam(motor_id, position_bytes):
                    print(f"[DynamixelController] Failed to add motor {motor_id} to sync write")

            # Transmit packet
            result = group_sync_write.txPacket()
            if result != COMM_SUCCESS:
                print(f"[DynamixelController] Sync write error: "
                      f"{self.packet_handler.getTxRxResult(result)}")

            # Clear parameters
            group_sync_write.clearParam()

    def get_joint_positions_radians(self) -> Optional[np.ndarray]:
        """
        Get current arm joint positions in radians.

        Returns:
            Array of 6 joint angles [waist, shoulder, elbow, forearm, wrist_angle, wrist_rotate]
            or None if error
        """
        # Read all positions
        positions = self.sync_read_positions()
        if not positions:
            return None

        # Extract arm joints (motors 1, 2, 4, 6, 7, 8)
        # Skip shadow motors 3 and 5, skip gripper 9
        arm_motor_ids = [1, 2, 4, 6, 7, 8]

        joint_angles = []
        for motor_id in arm_motor_ids:
            if motor_id not in positions:
                return None

            # Convert Dynamixel units to radians
            # Center position = 2048, range = 4096 units = 2π radians
            position = positions[motor_id]
            radians = (position - 2048) * (2 * np.pi / 4096)
            joint_angles.append(radians)

        return np.array(joint_angles)

    def set_joint_positions_radians(self, joint_angles: np.ndarray):
        """
        Set arm joint positions in radians.

        Args:
            joint_angles: Array of 6 joint angles in radians
        """
        if len(joint_angles) != 6:
            raise ValueError(f"Expected 6 joint angles, got {len(joint_angles)}")

        # Convert radians to Dynamixel units
        # Center position = 2048, range = 4096 units = 2π radians
        arm_motor_ids = [1, 2, 4, 6, 7, 8]
        positions = {}

        for motor_id, angle in zip(arm_motor_ids, joint_angles):
            # Convert to Dynamixel units
            position = 2048 + int(angle * (4096 / (2 * np.pi)))
            positions[motor_id] = position

        # Write positions (shadow motors handled automatically)
        self.sync_write_positions(positions)

    def start_monitoring(self, frequency: float = 10.0):
        """
        Start background thread to monitor motor positions.

        Args:
            frequency: Monitoring frequency in Hz
        """
        if self.monitoring_active:
            print("[DynamixelController] Monitoring already active")
            return

        self.monitoring_active = True

        def monitor_loop():
            period = 1.0 / frequency
            while self.monitoring_active:
                try:
                    # Read positions
                    self.sync_read_positions()

                    # Update joint angles
                    joint_angles = self.get_joint_positions_radians()
                    if joint_angles is not None:
                        with self.state_lock:
                            self.current_positions_rad = joint_angles

                except Exception as e:
                    print(f"[DynamixelController] Monitoring error: {e}")

                time.sleep(period)

        self.monitoring_thread = threading.Thread(target=monitor_loop, daemon=True)
        self.monitoring_thread.start()

        print(f"[DynamixelController] Started position monitoring at {frequency}Hz")

    def stop_monitoring(self):
        """Stop background monitoring thread."""
        self.monitoring_active = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=2.0)
            print("[DynamixelController] Monitoring stopped")

    def get_status(self) -> Dict:
        """
        Get controller status.

        Returns:
            Dictionary with status information
        """
        with self.state_lock:
            return {
                'connected': self.port_handler is not None,
                'port': self.port,
                'baudrate': self.baudrate,
                'num_motors': len(self.motor_config),
                'monitoring_active': self.monitoring_active,
                'current_positions': self.current_positions.copy(),
                'current_joints_rad': self.current_positions_rad.copy()
            }

    def get_cached_positions(self) -> Dict[int, int]:
        """
        Get cached motor positions without accessing the port.

        Use this method when you need motor positions but don't need
        the absolute latest values. This is thread-safe and doesn't
        block on port access.

        Returns:
            Dictionary mapping motor_id -> position (Dynamixel units)
        """
        with self.state_lock:
            return self.current_positions.copy()

    def get_cached_joint_radians(self) -> np.ndarray:
        """
        Get cached joint positions in radians without accessing the port.

        Use this method when you need joint angles but don't need
        the absolute latest values. This is thread-safe and doesn't
        block on port access.

        Returns:
            Array of 6 joint angles in radians
        """
        with self.state_lock:
            return self.current_positions_rad.copy()

    def get_cached_gripper_position(self) -> Optional[int]:
        """
        Get cached gripper position without accessing the port.

        Returns:
            Gripper position in Dynamixel units or None if not available
        """
        with self.state_lock:
            return self.current_positions.get(9, None)

    def close(self):
        """Close port and cleanup."""
        print("[DynamixelController] Closing...")

        # Stop monitoring
        self.stop_monitoring()

        # Disable torque on ALL motors (1-8 arm + 9 gripper)
        ALL_MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9]
        self.disable_torque(ALL_MOTOR_IDS)

        # Close port
        if self.port_handler:
            self.port_handler.closePort()
            print("[DynamixelController] ✓ Port closed")


# =============================================================================
# Test/Demo Code
# =============================================================================

if __name__ == "__main__":
    print("=" * 80)
    print("Dynamixel Controller Test")
    print("=" * 80)

    # Initialize controller
    config_file = "config/vx300s.yaml"
    controller = DynamixelController(
        port='/dev/ttyDXL',
        baudrate=1000000,
        config_file=config_file
    )

    if controller.initialize_motors():
        print("\n✓ Controller initialized successfully\n")

        # Enable torque
        controller.enable_torque()

        # Start monitoring
        controller.start_monitoring(frequency=10)

        # Read current positions
        print("Current motor positions:")
        positions = controller.sync_read_positions()
        for motor_id, pos in positions.items():
            print(f"  Motor {motor_id}: {pos}")

        # Read joint angles
        print("\nCurrent joint angles (radians):")
        joints = controller.get_joint_positions_radians()
        if joints is not None:
            for i, angle in enumerate(joints):
                print(f"  Joint {i+1}: {angle:.4f} rad ({np.degrees(angle):.2f}°)")

        # Test movement - move to home position
        print("\nMoving to home position...")
        home_pose = np.array([0.0, -0.3, 0.6, 0.0, -0.3, 0.0])
        controller.set_joint_positions_radians(home_pose)

        time.sleep(3)

        # Read final positions
        print("\nFinal joint angles:")
        joints = controller.get_joint_positions_radians()
        if joints is not None:
            for i, angle in enumerate(joints):
                print(f"  Joint {i+1}: {angle:.4f} rad ({np.degrees(angle):.2f}°)")

        # Cleanup
        print("\nCleaning up...")
        controller.close()

        print("\n✓ Test complete!")
    else:
        print("\n✗ Failed to initialize controller")
