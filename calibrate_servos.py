#!/usr/bin/env python3
"""
Servo Calibration Script for VX300S Robot Arms

Interactive script to calibrate position limits for shoulder (motors 2/3)
and gripper (motor 9) on both follower arms.

Usage:
    python3 calibrate_servos.py                  # Run full calibration
    python3 calibrate_servos.py read [arm]       # Read motor positions
    python3 calibrate_servos.py torque-off [arm] # Disable torque on shoulder/gripper

    arm: 'left' or 'right' (default: 'right')
"""

import sys
import shutil
from pathlib import Path
from datetime import datetime

import yaml

from dynamixel_controller import DynamixelController

# Safety margin in Dynamixel units (added inside physical limits)
SAFETY_MARGIN = 50

# Motors to calibrate
CALIBRATION_MOTORS = {
    2: "Shoulder (Primary)",
    3: "Shoulder (Shadow)",
    9: "Gripper"
}

# Motor names for display
MOTOR_NAMES = {
    1: "Waist",
    2: "Shoulder (Primary)",
    3: "Shoulder (Shadow)",
    4: "Elbow (Primary)",
    5: "Elbow (Shadow)",
    6: "Forearm Roll",
    7: "Wrist Angle",
    8: "Wrist Rotate",
    9: "Gripper"
}

CONFIG_FILE = "config/vx300s.yaml"


def get_port(arm: str) -> str:
    """Get port path for arm name."""
    if arm == "left":
        return "/dev/ttyDXL_follower_left"
    else:
        return "/dev/ttyDXL_follower_right"


def wait_for_enter(prompt: str):
    """Wait for user to press Enter."""
    input(prompt)


def read_motor_position(controller: DynamixelController, motor_id: int) -> int:
    """Read current position of a single motor."""
    position = controller.read_register(motor_id, controller.ADDR_PRESENT_POSITION, 4)
    return position


# =============================================================================
# Read Positions Command
# =============================================================================

def cmd_read_positions(arm: str):
    """Read and display motor positions for an arm."""
    port = get_port(arm)
    print(f"Connecting to {port}...")

    controller = DynamixelController(
        port=port,
        baudrate=1000000,
        config_file=CONFIG_FILE
    )

    if not controller.initialize_motors():
        print("Failed to initialize!")
        return

    # Read positions
    positions = controller.sync_read_positions()

    print(f"\nMotor Positions ({arm}):")
    print("-" * 40)

    for motor_id in sorted(positions.keys()):
        name = MOTOR_NAMES.get(motor_id, f"Motor {motor_id}")
        pos = positions[motor_id]
        print(f"  Motor {motor_id} ({name:18}): {pos}")

    controller.close()


# =============================================================================
# Torque Off Command
# =============================================================================

def cmd_torque_off(arm: str):
    """Disable torque on shoulder and gripper motors for an arm."""
    port = get_port(arm)
    print(f"Connecting to {port}...")

    controller = DynamixelController(
        port=port,
        baudrate=1000000,
        config_file=CONFIG_FILE
    )

    if not controller.initialize_motors():
        print("Failed to initialize!")
        return

    # Disable torque on shoulder (2,3) and gripper (9)
    motors = [2, 3, 9]
    print(f"Disabling torque on motors {motors}...")
    controller.disable_torque(motors)

    print("\nTorque disabled! You can now move:")
    print("  - Shoulder (motors 2 & 3)")
    print("  - Gripper (motor 9)")
    print(f"\nRun 'python3 calibrate_servos.py read {arm}' to see current positions")

    controller.port_handler.closePort()


# =============================================================================
# Calibration Functions
# =============================================================================

def calibrate_motor(controller: DynamixelController, motor_id: int, motor_name: str) -> dict:
    """
    Calibrate a single motor by having user move it to min/max positions.

    Returns:
        dict with 'raw_min', 'raw_max', 'safe_min', 'safe_max' positions
    """
    print(f"\n{'='*60}")
    print(f"Calibrating Motor {motor_id}: {motor_name}")
    print(f"{'='*60}")

    # Read current position
    current = read_motor_position(controller, motor_id)
    print(f"Current position: {current}")

    # Get MIN position
    print(f"\n>>> Move motor {motor_id} to its MINIMUM position")
    print("    (the physical limit in the negative direction)")
    wait_for_enter("Press Enter when ready...")
    raw_min = read_motor_position(controller, motor_id)
    print(f"    Recorded MIN: {raw_min}")

    # Get MAX position
    print(f"\n>>> Move motor {motor_id} to its MAXIMUM position")
    print("    (the physical limit in the positive direction)")
    wait_for_enter("Press Enter when ready...")
    raw_max = read_motor_position(controller, motor_id)
    print(f"    Recorded MAX: {raw_max}")

    # Ensure min < max
    if raw_min > raw_max:
        print("    Note: Swapping min/max (min was greater than max)")
        raw_min, raw_max = raw_max, raw_min

    # Apply safety margin
    safe_min = raw_min + SAFETY_MARGIN
    safe_max = raw_max - SAFETY_MARGIN

    print(f"\n    Raw limits:  [{raw_min}, {raw_max}]")
    print(f"    Safe limits: [{safe_min}, {safe_max}] (with {SAFETY_MARGIN} unit margin)")

    return {
        'raw_min': raw_min,
        'raw_max': raw_max,
        'safe_min': safe_min,
        'safe_max': safe_max
    }


def calibrate_arm(port: str, arm_id: str) -> dict:
    """
    Calibrate all target motors on a single arm.

    Returns:
        dict mapping motor_id -> calibration results
    """
    print(f"\n{'#'*60}")
    print(f"# Calibrating: {arm_id}")
    print(f"# Port: {port}")
    print(f"{'#'*60}")

    # Initialize controller
    controller = DynamixelController(
        port=port,
        baudrate=1000000,
        config_file=CONFIG_FILE
    )

    if not controller.initialize_motors():
        print(f"ERROR: Failed to initialize {arm_id}")
        return {}

    # Disable torque on calibration motors so user can move them
    motor_ids = list(CALIBRATION_MOTORS.keys())
    print(f"\nDisabling torque on motors {motor_ids}...")
    controller.disable_torque(motor_ids)
    print("You can now freely move the shoulder and gripper.")

    # Calibrate each motor
    results = {}

    # For shoulder, we only calibrate motor 2 and apply same limits to motor 3
    print("\n" + "-"*60)
    print("Note: Shoulder motors 2 and 3 are mechanically coupled.")
    print("      We'll calibrate motor 2 and apply the same limits to motor 3.")
    print("-"*60)

    # Calibrate shoulder (motor 2)
    shoulder_result = calibrate_motor(controller, 2, "Shoulder (Primary)")
    results[2] = shoulder_result
    results[3] = shoulder_result.copy()  # Same limits for shadow

    # Calibrate gripper (motor 9)
    gripper_result = calibrate_motor(controller, 9, "Gripper")
    results[9] = gripper_result

    # Cleanup
    print(f"\nClosing connection to {arm_id}...")
    controller.close()

    return results


def backup_config():
    """Create a backup of the config file."""
    config_path = Path(CONFIG_FILE)
    if config_path.exists():
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = config_path.with_suffix(f".yaml.backup_{timestamp}")
        shutil.copy(config_path, backup_path)
        print(f"Config backed up to: {backup_path}")
        return backup_path
    return None


def update_config(all_results: dict):
    """
    Update the config file with new calibration values.

    Args:
        all_results: dict mapping arm_id -> {motor_id -> calibration_results}
    """
    config_path = Path(CONFIG_FILE)

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # We'll use the average of both arms if both were calibrated,
    # or just use the single arm's values
    motor_limits = {}

    for arm_id, arm_results in all_results.items():
        for motor_id, cal in arm_results.items():
            if motor_id not in motor_limits:
                motor_limits[motor_id] = {'mins': [], 'maxs': []}
            motor_limits[motor_id]['mins'].append(cal['safe_min'])
            motor_limits[motor_id]['maxs'].append(cal['safe_max'])

    # Average the limits and update config
    for motor_id, limits in motor_limits.items():
        avg_min = int(sum(limits['mins']) / len(limits['mins']))
        avg_max = int(sum(limits['maxs']) / len(limits['maxs']))

        if motor_id in config['motors']:
            old_min = config['motors'][motor_id].get('Min_Position', 'N/A')
            old_max = config['motors'][motor_id].get('Max_Position', 'N/A')

            config['motors'][motor_id]['Min_Position'] = avg_min
            config['motors'][motor_id]['Max_Position'] = avg_max

            print(f"Motor {motor_id}: [{old_min}, {old_max}] -> [{avg_min}, {avg_max}]")

    # Write updated config
    with open(config_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    print(f"\nConfig updated: {config_path}")


def cmd_calibrate():
    """Run full interactive calibration."""
    print("="*60)
    print("  VX300S Servo Calibration Tool")
    print("="*60)
    print(f"\nThis tool will calibrate position limits for:")
    for motor_id, name in CALIBRATION_MOTORS.items():
        print(f"  - Motor {motor_id}: {name}")
    print(f"\nSafety margin: {SAFETY_MARGIN} units inside physical limits")
    print(f"Config file: {CONFIG_FILE}")

    # Detect available follower arms
    print("\nDetecting follower arms...")
    detected = DynamixelController.detect_follower_ports()

    if not detected:
        print("ERROR: No follower arms detected!")
        print("       Looking for /dev/ttyDXL_follower_left or /dev/ttyDXL_follower_right")
        sys.exit(1)

    print(f"Found {len(detected)} arm(s):")
    for arm in detected:
        print(f"  - {arm['arm_id']} on {arm['port']}")

    wait_for_enter("\nPress Enter to begin calibration...")

    # Calibrate each arm
    all_results = {}

    for arm in detected:
        results = calibrate_arm(arm['port'], arm['arm_id'])
        if results:
            all_results[arm['arm_id']] = results

    if not all_results:
        print("\nERROR: No calibration data collected!")
        sys.exit(1)

    # Summary
    print("\n" + "="*60)
    print("  CALIBRATION SUMMARY")
    print("="*60)

    for arm_id, arm_results in all_results.items():
        print(f"\n{arm_id}:")
        for motor_id, cal in arm_results.items():
            if motor_id == 3:
                continue  # Skip shadow, same as motor 2
            name = CALIBRATION_MOTORS.get(motor_id, f"Motor {motor_id}")
            print(f"  {name} (Motor {motor_id}):")
            print(f"    Raw:  [{cal['raw_min']}, {cal['raw_max']}]")
            print(f"    Safe: [{cal['safe_min']}, {cal['safe_max']}]")

    # Ask to save
    print("\n" + "-"*60)
    response = input("Save these values to config? [y/N]: ").strip().lower()

    if response == 'y':
        backup_config()
        update_config(all_results)
        print("\nCalibration complete!")
    else:
        print("\nCalibration values NOT saved.")
        print("You can manually update config/vx300s.yaml with the values above.")

    print("\nDone.")


# =============================================================================
# Main Entry Point
# =============================================================================

def print_usage():
    """Print usage information."""
    print("Usage:")
    print("  python3 calibrate_servos.py                  # Run full calibration")
    print("  python3 calibrate_servos.py read [arm]       # Read motor positions")
    print("  python3 calibrate_servos.py torque-off [arm] # Disable torque on shoulder/gripper")
    print("")
    print("  arm: 'left' or 'right' (default: 'right')")


def main():
    if len(sys.argv) < 2:
        # No command specified, run calibration
        cmd_calibrate()
        return

    command = sys.argv[1].lower()
    arm = sys.argv[2] if len(sys.argv) > 2 else "right"

    if command == "read":
        cmd_read_positions(arm)
    elif command == "torque-off":
        cmd_torque_off(arm)
    elif command == "calibrate":
        cmd_calibrate()
    elif command in ["-h", "--help", "help"]:
        print_usage()
    else:
        print(f"Unknown command: {command}")
        print_usage()
        sys.exit(1)


if __name__ == "__main__":
    main()
