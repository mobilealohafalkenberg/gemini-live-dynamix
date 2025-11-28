#!/usr/bin/env python3
"""
Camera controller for Mobile ALOHA - captures from RealSense cameras
and provides video feed for Gemini API.

Simplified RGB-only version with dynamic camera detection.
"""

import cv2
import numpy as np
import threading
import time
import base64
import yaml
import os
from typing import Optional, Dict, List, Tuple
import pyrealsense2 as rs


class CameraController:
    """
    Controls camera feeds from Mobile ALOHA's RealSense cameras.
    Provides RGB streams with dynamic camera detection.
    """

    def __init__(self, config_path: str = None):
        """Initialize camera controller

        Args:
            config_path: Path to config file with camera serial-to-name mapping.
                        If None, uses default path config/vx300s.yaml
        """
        self.pipelines = {}
        self.configs = {}
        self.cameras = {}  # name -> serial mapping
        self.frames = {}   # RGB frames
        self.frame_locks = {}
        self.capture_threads = {}
        self.running = False
        self.initialized = False

        # Serial to name mapping (loaded from config)
        self.serial_to_name = {}
        self._load_camera_config(config_path)

        # Camera configuration (RGB only)
        self.camera_config = {
            'resolution': (640, 480),
            'fps': 30,
            'color_format': rs.format.rgb8,
        }

    def _load_camera_config(self, config_path: str = None):
        """Load camera serial-to-name mapping from config file."""
        if config_path is None:
            # Default path relative to this file
            base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            config_path = os.path.join(base_dir, 'config', 'vx300s.yaml')

        try:
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                    if config and 'cameras' in config:
                        self.serial_to_name = config['cameras']
                        print(f"[CameraController] Loaded camera mapping: {self.serial_to_name}")
                    else:
                        print("[CameraController] No camera mapping in config, using generic names")
            else:
                print(f"[CameraController] Config file not found: {config_path}")
        except Exception as e:
            print(f"[CameraController] Error loading config: {e}")

    def initialize(self) -> bool:
        """
        Initialize RealSense cameras with RGB streams.
        Uses dynamic detection - initializes any available cameras.

        Returns:
            True if at least one camera initialized successfully
        """
        try:
            # Create context to query devices
            ctx = rs.context()
            devices = ctx.query_devices()

            if len(devices) == 0:
                print("[CameraController] No RealSense devices found")
                return False

            print(f"[CameraController] Found {len(devices)} RealSense devices")

            # Initialize each available camera with name from config (or generic fallback)
            for i, dev in enumerate(devices):
                serial = dev.get_info(rs.camera_info.serial_number)
                # Use config name if available, otherwise fall back to generic name
                name = self.serial_to_name.get(serial, f'camera_{i}')

                print(f"[CameraController] Initializing {name} (serial: {serial})")

                # Create pipeline for this camera
                pipeline = rs.pipeline()
                config = rs.config()

                # Configure the specific device
                config.enable_device(serial)

                # Enable COLOR stream only (no depth)
                config.enable_stream(
                    rs.stream.color,
                    self.camera_config['resolution'][0],
                    self.camera_config['resolution'][1],
                    self.camera_config['color_format'],
                    self.camera_config['fps']
                )

                try:
                    # Start the pipeline
                    profile = pipeline.start(config)

                    # Prime the pipeline with reduced warmup (2 iterations instead of 5)
                    print(f"[CameraController] Warming up {name}...")
                    for attempt in range(2):
                        try:
                            frames = pipeline.wait_for_frames(timeout_ms=3000)
                            if frames:
                                print(f"[CameraController] {name} primed successfully")
                                break
                        except Exception as e:
                            if attempt == 1:
                                print(f"[CameraController] Warning: {name} warmup incomplete: {e}")

                    # Store references
                    self.pipelines[name] = pipeline
                    self.configs[name] = config
                    self.cameras[name] = serial
                    self.frames[name] = None
                    self.frame_locks[name] = threading.Lock()

                    print(f"[CameraController] {name} initialized (RGB only)")

                except Exception as e:
                    print(f"[CameraController] Failed to initialize {name}: {e}")
                    continue

            if len(self.pipelines) > 0:
                self.initialized = True
                self.running = True
                self.start_capture_threads()
                print(f"[CameraController] Initialized {len(self.pipelines)} cameras: {list(self.pipelines.keys())}")
                return True
            else:
                print("[CameraController] No cameras could be initialized")
                return False

        except Exception as e:
            print(f"[CameraController] Initialization error: {e}")
            # Try fallback to regular USB cameras
            return self.initialize_usb_cameras()

    def initialize_usb_cameras(self) -> bool:
        """
        Fallback initialization for regular USB cameras.
        """
        try:
            print("[CameraController] Trying USB camera fallback...")

            # Try to find available video devices
            camera_count = 0
            for idx in range(10):  # Check first 10 video devices
                try:
                    cap = cv2.VideoCapture(idx)
                    if cap.isOpened():
                        # Set resolution
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        cap.set(cv2.CAP_PROP_FPS, 30)

                        name = f'camera_{camera_count}'
                        self.pipelines[name] = cap
                        self.frames[name] = None
                        self.frame_locks[name] = threading.Lock()
                        self.cameras[name] = f'USB-{idx}'

                        print(f"[CameraController] {name} initialized (USB /dev/video{idx})")
                        camera_count += 1

                        if camera_count >= 2:  # Limit to 2 USB cameras
                            break
                    else:
                        cap.release()
                except Exception as e:
                    continue

            if len(self.pipelines) > 0:
                self.initialized = True
                self.running = True
                self.start_capture_threads()
                print(f"[CameraController] Initialized {len(self.pipelines)} USB cameras")
                return True

        except Exception as e:
            print(f"[CameraController] USB fallback failed: {e}")

        return False

    def start_capture_threads(self):
        """Start capture threads for each camera"""
        print("[CameraController] Starting capture threads...")

        for name in self.pipelines.keys():
            thread = threading.Thread(
                target=self.capture_loop,
                args=(name,),
                daemon=True
            )
            thread.start()
            self.capture_threads[name] = thread

        print(f"[CameraController] Capture threads started for {list(self.pipelines.keys())}")

    def capture_loop(self, camera_name: str):
        """
        Continuous capture loop for a camera.
        Captures RGB frames from RealSense or USB cameras.

        Args:
            camera_name: Name of the camera to capture from
        """
        pipeline = self.pipelines[camera_name]
        consecutive_errors = 0
        max_consecutive_errors = 5

        while self.running:
            try:
                if isinstance(pipeline, rs.pipeline):
                    # RealSense camera - RGB only
                    frames = pipeline.wait_for_frames(timeout_ms=3000)
                    consecutive_errors = 0  # Reset on success

                    color_frame = frames.get_color_frame()

                    if color_frame:
                        # Convert to numpy array (RGB)
                        rgb_array = np.asanyarray(color_frame.get_data())

                        with self.frame_locks[camera_name]:
                            self.frames[camera_name] = rgb_array

                else:
                    # USB camera (OpenCV) - RGB only
                    ret, frame = pipeline.read()
                    if ret:
                        # Convert BGR to RGB
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                        with self.frame_locks[camera_name]:
                            self.frames[camera_name] = frame
                    else:
                        consecutive_errors += 1

            except Exception as e:
                consecutive_errors += 1
                if self.running and consecutive_errors >= max_consecutive_errors:
                    print(f"[CameraController] Capture error on {camera_name}: {e}")
                    consecutive_errors = 0
                time.sleep(0.1)

    def _add_label_to_frame(self, frame: np.ndarray, label: str) -> np.ndarray:
        """
        Add camera label banner to top of frame.

        Args:
            frame: RGB numpy array
            label: Text label to add

        Returns:
            Frame with label overlay
        """
        h, w = frame.shape[:2]
        result = frame.copy()

        # Draw semi-transparent dark banner at top
        overlay = result.copy()
        cv2.rectangle(overlay, (0, 0), (w, 32), (40, 40, 40), -1)
        result = cv2.addWeighted(overlay, 0.7, result, 0.3, 0)

        # Add white text
        cv2.putText(result, label, (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        return result

    def get_frame(self, camera_name: str) -> Optional[np.ndarray]:
        """
        Get the latest RGB frame from a camera.

        Args:
            camera_name: Name of the camera

        Returns:
            RGB frame as numpy array or None
        """
        if camera_name not in self.frames:
            return None

        with self.frame_locks[camera_name]:
            return self.frames[camera_name].copy() if self.frames[camera_name] is not None else None

    def get_rgbd_frames(self, camera_name: str) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get RGB frame from a camera (depth always None in RGB-only mode).
        Kept for API compatibility.

        Args:
            camera_name: Name of the camera

        Returns:
            Tuple of (rgb_frame, None)
        """
        rgb = self.get_frame(camera_name)
        return rgb, None

    def get_frame_base64(self, camera_name: str, quality: int = 85, add_label: bool = True) -> Optional[str]:
        """
        Get RGB frame as base64-encoded JPEG with optional label overlay.

        Args:
            camera_name: Name of the camera
            quality: JPEG quality (1-100)
            add_label: Whether to add camera name label to image

        Returns:
            Base64-encoded JPEG string or None
        """
        frame = self.get_frame(camera_name)
        if frame is None:
            return None

        try:
            # Add label overlay if requested
            if add_label:
                frame = self._add_label_to_frame(frame, camera_name)

            # Convert RGB to BGR for OpenCV
            bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Encode as JPEG
            _, buffer = cv2.imencode('.jpg', bgr_frame, [cv2.IMWRITE_JPEG_QUALITY, quality])

            # Convert to base64
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            return jpg_as_text

        except Exception as e:
            print(f"[CameraController] Error encoding frame: {e}")
            return None

    def get_all_frames(self) -> Dict[str, np.ndarray]:
        """
        Get RGB frames from all active cameras.

        Returns:
            Dictionary mapping camera names to RGB frames
        """
        frames = {}
        for name in self.pipelines.keys():
            frame = self.get_frame(name)
            if frame is not None:
                frames[name] = frame
        return frames

    def get_all_frames_base64(self) -> Dict[str, str]:
        """
        Get base64-encoded JPEG frames from all active cameras.

        Returns:
            Dictionary mapping camera names to base64 JPEG strings
        """
        frames = {}
        for name in self.pipelines.keys():
            frame_b64 = self.get_frame_base64(name)
            if frame_b64:
                frames[name] = frame_b64
        return frames

    def get_camera_names(self) -> List[str]:
        """
        Get list of available camera names.

        Returns:
            List of camera names
        """
        return list(self.pipelines.keys())

    def get_camera_info(self) -> Dict:
        """
        Get information about available cameras.

        Returns:
            Dictionary with camera information
        """
        info = {
            'initialized': self.initialized,
            'running': self.running,
            'cameras': {}
        }

        for name in self.pipelines.keys():
            has_frame = self.frames.get(name) is not None

            info['cameras'][name] = {
                'active': has_frame,
                'serial': self.cameras.get(name, 'unknown'),
                'resolution': self.camera_config['resolution'],
                'fps': self.camera_config['fps']
            }

        return info

    def shutdown(self):
        """Shutdown all cameras and cleanup"""
        print("[CameraController] Shutting down cameras...")
        self.running = False

        # Wait for capture threads to stop
        time.sleep(0.5)

        # Stop all pipelines
        for name, pipeline in self.pipelines.items():
            try:
                if isinstance(pipeline, rs.pipeline):
                    pipeline.stop()
                else:
                    pipeline.release()
                print(f"[CameraController] Stopped {name}")
            except:
                pass

        self.pipelines.clear()
        self.frames.clear()
        self.initialized = False
        print("[CameraController] Shutdown complete")


# Test script
if __name__ == "__main__":
    print("=" * 80)
    print("Testing Camera Controller (RGB-only, Dynamic Detection)")
    print("=" * 80)

    controller = CameraController()

    if controller.initialize():
        print("\n" + "=" * 80)
        print("Camera Info:")
        print("=" * 80)
        info = controller.get_camera_info()
        print(f"Cameras found: {list(info['cameras'].keys())}")
        for cam_name, cam_info in info['cameras'].items():
            print(f"\n{cam_name}:")
            print(f"  Active: {cam_info['active']}")
            print(f"  Serial: {cam_info['serial']}")
            print(f"  Resolution: {cam_info['resolution']}")

        print("\n" + "=" * 80)
        print("Capturing frames for 3 seconds...")
        print("=" * 80)

        for i in range(3):
            time.sleep(1)
            print(f"\n--- Frame {i+1} ---")

            frames = controller.get_all_frames_base64()
            for cam_name, frame_b64 in frames.items():
                print(f"{cam_name}: {len(frame_b64)} bytes (base64)")

        print("\n" + "=" * 80)
        print("Shutting down...")
        print("=" * 80)
        controller.shutdown()
        print("\nTest complete!")
    else:
        print("Failed to initialize cameras")
