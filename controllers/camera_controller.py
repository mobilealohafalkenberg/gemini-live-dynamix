#!/usr/bin/env python3
"""
Camera controller for Mobile ALOHA - captures from RealSense cameras
and provides video feed for Gemini Live API.

FIXES APPLIED:
- Added get_rgbd_frames() method for synchronized RGB+Depth capture
- Enabled depth stream in RealSense configuration
- Added depth frame storage and capture
- Added alignment of depth to color frames for accurate 3D positioning
"""

import cv2
import numpy as np
import threading
import time
import base64
from typing import Optional, Dict, List, Tuple
import pyrealsense2 as rs

class CameraController:
    """
    Controls camera feeds from Mobile ALOHA's RealSense cameras.
    Provides RGB+Depth streams from gripper and top cameras.
    """

    def __init__(self):
        """Initialize camera controller"""
        self.pipelines = {}
        self.configs = {}
        self.cameras = {}
        self.frames = {}           # RGB frames
        self.depth_frames = {}     # ← NEW: Depth frame storage
        self.frame_locks = {}
        self.capture_threads = {}
        self.running = False
        self.initialized = False

        # ← NEW: Align object to align depth to color
        self.align_objects = {}

        # Camera configuration
        self.camera_config = {
            'resolution': (640, 480),
            'fps': 30,
            'color_format': rs.format.rgb8,
            'depth_format': rs.format.z16  # ← NEW: Depth format
        }

    def initialize(self) -> bool:
        """
        Initialize RealSense cameras with RGB+Depth streams.

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

            # Map cameras by serial number to correct positions
            camera_mapping = {
                '130322273629': 'top_cam',      # Top overhead camera
                '130322273632': 'gripper_cam',  # LEFT arm gripper camera
                '130322270224': 'unused_cam',   # Right arm camera (looking at ceiling)
            }

            for dev in devices:
                serial = dev.get_info(rs.camera_info.serial_number)
                name = camera_mapping.get(serial, None)

                # Skip if this camera is not one we want to use
                if not name or name == 'unused_cam':
                    print(f"[CameraController] Skipping camera {serial} ({name or 'unknown'})")
                    continue

                print(f"[CameraController] Initializing {name} (serial: {serial})")

                # Create pipeline for this camera
                pipeline = rs.pipeline()
                config = rs.config()

                # Configure the specific device
                config.enable_device(serial)

                # ← NEW: Enable COLOR stream
                config.enable_stream(
                    rs.stream.color,
                    self.camera_config['resolution'][0],
                    self.camera_config['resolution'][1],
                    self.camera_config['color_format'],
                    self.camera_config['fps']
                )

                # ← NEW: Enable DEPTH stream
                config.enable_stream(
                    rs.stream.depth,
                    self.camera_config['resolution'][0],
                    self.camera_config['resolution'][1],
                    self.camera_config['depth_format'],
                    self.camera_config['fps']
                )

                try:
                    # Start the pipeline
                    profile = pipeline.start(config)

                    # ← NEW: Create align object to align depth to color
                    align = rs.align(rs.stream.color)

                    # Prime the pipeline by waiting for a few frames (RealSense needs this)
                    print(f"[CameraController] Warming up {name}...")
                    for i in range(5):
                        try:
                            frames = pipeline.wait_for_frames(timeout_ms=5000)
                            if frames:
                                print(f"[CameraController] {name} primed successfully")
                                break
                        except Exception as e:
                            if i == 4:  # Last attempt
                                print(f"[CameraController] Warning: {name} warmup incomplete: {e}")

                    # Store references
                    self.pipelines[name] = pipeline
                    self.configs[name] = config
                    self.cameras[name] = serial
                    self.frames[name] = None
                    self.depth_frames[name] = None  # ← NEW
                    self.frame_locks[name] = threading.Lock()
                    self.align_objects[name] = align  # ← NEW

                    print(f"[CameraController] ✓ {name} initialized (RGB+Depth)")

                except Exception as e:
                    print(f"[CameraController] ✗ Failed to initialize {name}: {e}")
                    continue

            if len(self.pipelines) > 0:
                self.initialized = True
                self.running = True
                self.start_capture_threads()
                print(f"[CameraController] ✓ Initialized {len(self.pipelines)} cameras")
                return True
            else:
                print("[CameraController] ✗ No cameras could be initialized")
                return False

        except Exception as e:
            print(f"[CameraController] ✗ Initialization error: {e}")
            # Try fallback to regular USB cameras
            return self.initialize_usb_cameras()

    def initialize_usb_cameras(self) -> bool:
        """
        Fallback initialization for regular USB cameras.
        NOTE: USB cameras don't have depth - depth_frames will be None
        """
        try:
            print("[CameraController] Trying USB camera fallback...")

            # Try to open video devices directly
            camera_indices = [2, 8]  # Based on v4l2 output
            camera_names = ['gripper_cam', 'top_cam']

            for name, idx in zip(camera_names, camera_indices):
                try:
                    cap = cv2.VideoCapture(f'/dev/video{idx}')
                    if cap.isOpened():
                        # Set resolution
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        cap.set(cv2.CAP_PROP_FPS, 30)

                        self.pipelines[name] = cap
                        self.frames[name] = None
                        self.depth_frames[name] = None  # ← NEW: No depth for USB cams
                        self.frame_locks[name] = threading.Lock()

                        print(f"[CameraController] ✓ {name} initialized (USB fallback, no depth)")
                    else:
                        print(f"[CameraController] ✗ Could not open /dev/video{idx}")
                except Exception as e:
                    print(f"[CameraController] ✗ Error opening camera {idx}: {e}")

            if len(self.pipelines) > 0:
                self.initialized = True
                self.running = True
                self.start_capture_threads()
                print(f"[CameraController] ✓ Initialized {len(self.pipelines)} USB cameras")
                return True

        except Exception as e:
            print(f"[CameraController] ✗ USB fallback failed: {e}")

        return False

    def start_capture_threads(self):
        """Start capture threads for each camera"""
        # Wait for cameras to warm up before starting capture loops
        # RealSense cameras need a brief period after pipeline.start() before frames are available
        print("[CameraController] Waiting for cameras to warm up...")
        time.sleep(1.0)  # Give cameras time to start streaming

        for name in self.pipelines.keys():
            thread = threading.Thread(
                target=self.capture_loop,
                args=(name,),
                daemon=True
            )
            thread.start()
            self.capture_threads[name] = thread

        print("[CameraController] Capture threads started")

    def capture_loop(self, camera_name: str):
        """
        Continuous capture loop for a camera.
        Captures both RGB and depth frames from RealSense cameras.

        Args:
            camera_name: Name of the camera to capture from
        """
        pipeline = self.pipelines[camera_name]
        consecutive_errors = 0
        max_consecutive_errors = 5  # Only log after multiple failures

        while self.running:
            try:
                if isinstance(pipeline, rs.pipeline):
                    # ═══════════════════════════════════════════════════
                    # RealSense camera - RGB + Depth
                    # ═══════════════════════════════════════════════════

                    # Wait for frames with extended timeout
                    frames = pipeline.wait_for_frames(timeout_ms=3000)
                    consecutive_errors = 0  # Reset error counter on success

                    # ← NEW: Align depth to color for accurate correspondence
                    if camera_name in self.align_objects:
                        aligned_frames = self.align_objects[camera_name].process(frames)
                        color_frame = aligned_frames.get_color_frame()
                        depth_frame = aligned_frames.get_depth_frame()
                    else:
                        color_frame = frames.get_color_frame()
                        depth_frame = frames.get_depth_frame()

                    # Store RGB frame
                    if color_frame:
                        # Convert to numpy array (RGB)
                        rgb_array = np.asanyarray(color_frame.get_data())

                        with self.frame_locks[camera_name]:
                            self.frames[camera_name] = rgb_array

                    # ← NEW: Store depth frame
                    if depth_frame:
                        # Convert to numpy array (depth in millimeters)
                        depth_array = np.asanyarray(depth_frame.get_data())

                        # Convert from millimeters to meters
                        depth_meters = depth_array.astype(np.float32) / 1000.0

                        with self.frame_locks[camera_name]:
                            self.depth_frames[camera_name] = depth_meters

                else:
                    # ═══════════════════════════════════════════════════
                    # USB camera (OpenCV) - RGB only
                    # ═══════════════════════════════════════════════════
                    ret, frame = pipeline.read()
                    if ret:
                        # Convert BGR to RGB
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                        # Store RGB frame
                        with self.frame_locks[camera_name]:
                            self.frames[camera_name] = frame
                            # No depth available for USB cameras
                            self.depth_frames[camera_name] = None

            except Exception as e:
                consecutive_errors += 1
                if self.running:  # Only print if we're still supposed to be running
                    # Only log after multiple consecutive errors to avoid flooding the console
                    if consecutive_errors >= max_consecutive_errors:
                        print(f"[CameraController] Capture error on {camera_name}: {e} (after {consecutive_errors} attempts)")
                        consecutive_errors = 0  # Reset to allow next batch of errors
                time.sleep(0.1)

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

    # ═══════════════════════════════════════════════════════════════════════
    # ← NEW METHOD: Get synchronized RGB + Depth frames
    # ═══════════════════════════════════════════════════════════════════════
    def get_rgbd_frames(self, camera_name: str) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get synchronized RGB and depth frames from a camera.

        This is the method called by vision_controller and updated_bridge_aloha.py
        for object detection with 3D positioning.

        Args:
            camera_name: Name of the camera ('gripper_cam' or 'top_cam')

        Returns:
            Tuple of (rgb_frame, depth_frame):
            - rgb_frame: RGB image as numpy array (H, W, 3) uint8, or None
            - depth_frame: Depth image as numpy array (H, W) float32 in meters, or None

        Example:
            >>> rgb, depth = camera_controller.get_rgbd_frames('gripper_cam')
            >>> if rgb is not None:
            >>>     print(f"RGB shape: {rgb.shape}")  # (480, 640, 3)
            >>> if depth is not None:
            >>>     print(f"Depth at center: {depth[240, 320]:.3f} meters")
        """
        if camera_name not in self.frames:
            print(f"[CameraController] Warning: Camera '{camera_name}' not found")
            return None, None

        # Get both frames atomically
        with self.frame_locks[camera_name]:
            rgb = self.frames[camera_name].copy() if self.frames[camera_name] is not None else None
            depth = self.depth_frames[camera_name].copy() if self.depth_frames[camera_name] is not None else None

        return rgb, depth

    def get_depth_at_pixel(self, camera_name: str, u: int, v: int) -> Optional[float]:
        """
        Get depth value at a specific pixel coordinate.

        Args:
            camera_name: Name of the camera
            u: Pixel x-coordinate
            v: Pixel y-coordinate

        Returns:
            Depth in meters, or None if not available
        """
        if camera_name not in self.depth_frames:
            return None

        with self.frame_locks[camera_name]:
            depth_frame = self.depth_frames[camera_name]
            if depth_frame is None:
                return None

            # Check bounds
            h, w = depth_frame.shape
            if 0 <= v < h and 0 <= u < w:
                depth_value = depth_frame[v, u]
                # Filter out invalid depth (0 or NaN)
                if depth_value > 0 and not np.isnan(depth_value):
                    return float(depth_value)

        return None

    def get_frame_base64(self, camera_name: str, quality: int = 85) -> Optional[str]:
        """
        Get RGB frame as base64-encoded JPEG.

        Args:
            camera_name: Name of the camera
            quality: JPEG quality (1-100)

        Returns:
            Base64-encoded JPEG string or None
        """
        frame = self.get_frame(camera_name)
        if frame is None:
            return None

        try:
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

    def get_all_rgbd_frames(self) -> Dict[str, Tuple[np.ndarray, Optional[np.ndarray]]]:
        """
        Get RGB+Depth frames from all active cameras.

        Returns:
            Dictionary mapping camera names to (rgb_frame, depth_frame) tuples
        """
        frames = {}
        for name in self.pipelines.keys():
            rgb, depth = self.get_rgbd_frames(name)
            if rgb is not None:
                frames[name] = (rgb, depth)
        return frames

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
            has_rgb = self.frames.get(name) is not None
            has_depth = self.depth_frames.get(name) is not None

            info['cameras'][name] = {
                'active': has_rgb,
                'has_depth': has_depth,
                'serial': self.cameras.get(name, 'USB'),
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
        self.depth_frames.clear()  # ← NEW
        self.align_objects.clear()  # ← NEW
        self.initialized = False
        print("[CameraController] ✓ Shutdown complete")


# ═══════════════════════════════════════════════════════════════════════════
# Test script
# ═══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("=" * 80)
    print("Testing Camera Controller with RGB+Depth")
    print("=" * 80)

    controller = CameraController()

    if controller.initialize():
        print("\n" + "=" * 80)
        print("Camera Info:")
        print("=" * 80)
        info = controller.get_camera_info()
        for cam_name, cam_info in info['cameras'].items():
            print(f"\n{cam_name}:")
            print(f"  Active: {cam_info['active']}")
            print(f"  Has Depth: {cam_info['has_depth']}")
            print(f"  Serial: {cam_info['serial']}")
            print(f"  Resolution: {cam_info['resolution']}")
            print(f"  FPS: {cam_info['fps']}")

        print("\n" + "=" * 80)
        print("Capturing RGB+Depth frames for 5 seconds...")
        print("=" * 80)

        for i in range(5):
            time.sleep(1)

            print(f"\n--- Frame {i+1} ---")

            # Test get_rgbd_frames() for each camera
            for cam_name in info['cameras'].keys():
                rgb, depth = controller.get_rgbd_frames(cam_name)

                if rgb is not None:
                    print(f"{cam_name}:")
                    print(f"  RGB: {rgb.shape} dtype={rgb.dtype}")

                    if depth is not None:
                        print(f"  Depth: {depth.shape} dtype={depth.dtype}")
                        # Get depth at center pixel
                        h, w = depth.shape
                        center_depth = depth[h//2, w//2]
                        print(f"  Center depth: {center_depth:.3f} meters")

                        # Show depth statistics
                        valid_depth = depth[depth > 0]
                        if len(valid_depth) > 0:
                            print(f"  Depth range: {valid_depth.min():.3f} - {valid_depth.max():.3f} meters")
                    else:
                        print(f"  Depth: None (USB camera)")

                    # Test base64 encoding
                    b64 = controller.get_frame_base64(cam_name)
                    if b64:
                        print(f"  Base64: {len(b64)} bytes")
                else:
                    print(f"{cam_name}: No frame available")

        print("\n" + "=" * 80)
        print("Shutting down...")
        print("=" * 80)
        controller.shutdown()

        print("\n✓ Test complete!")
    else:
        print("✗ Failed to initialize cameras")
