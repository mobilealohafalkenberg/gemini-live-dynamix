#!/usr/bin/env python3
"""
Depth Projector - Convert 2D pixel coordinates + depth to 3D points

This module uses RealSense camera intrinsics to project 2D image coordinates
and depth values into 3D points in the camera coordinate frame.
"""

import numpy as np
import logging
from typing import Tuple, Optional, List
import pyrealsense2 as rs

logger = logging.getLogger(__name__)


class DepthProjector:
    """
    Projects 2D image coordinates to 3D points using depth information.

    Uses pinhole camera model with RealSense intrinsics:
    - fx, fy: Focal lengths in pixels
    - ppx, ppy: Principal point (image center)
    - Distortion coefficients (handled by RealSense)
    """

    def __init__(self, camera_intrinsics: Optional[rs.intrinsics] = None):
        """
        Initialize depth projector.

        Args:
            camera_intrinsics: RealSense intrinsics object
                              If None, uses default D405 intrinsics
        """
        if camera_intrinsics is None:
            # Default RealSense D405 intrinsics (640x480)
            # These are approximate - MUST be replaced with actual calibrated values
            self.intrinsics = self._get_default_intrinsics()
            logger.warning("[DepthProjector] Using default intrinsics - calibrate for accuracy!")
        else:
            self.intrinsics = camera_intrinsics

        logger.info(f"[DepthProjector] Initialized with intrinsics: "
                   f"fx={self.intrinsics.fx:.1f}, fy={self.intrinsics.fy:.1f}, "
                   f"ppx={self.intrinsics.ppx:.1f}, ppy={self.intrinsics.ppy:.1f}")

    def pixel_to_point(
        self,
        pixel_x: float,
        pixel_y: float,
        depth_value: float,
        depth_scale: float = 0.001
    ) -> Optional[np.ndarray]:
        """
        Convert a single pixel + depth to 3D point in camera frame.

        Args:
            pixel_x: X coordinate in image (0 = left edge)
            pixel_y: Y coordinate in image (0 = top edge)
            depth_value: Raw depth value from depth image
            depth_scale: Depth unit scale (default 0.001 = mm to meters)

        Returns:
            3D point [x, y, z] in meters (camera frame) or None if invalid
            Camera frame: +X right, +Y down, +Z forward (into scene)
        """
        # Check if depth is valid
        if depth_value == 0:
            logger.warning(f"[DepthProjector] Invalid depth at ({pixel_x}, {pixel_y})")
            return None

        # Convert depth to meters
        depth_meters = depth_value * depth_scale

        # Use RealSense deprojection function
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [pixel_x, pixel_y],
            depth_meters
        )

        return np.array(point_3d)

    def pixels_to_points(
        self,
        pixel_coords: np.ndarray,
        depth_image: np.ndarray,
        depth_scale: float = 0.001
    ) -> np.ndarray:
        """
        Convert multiple pixels to 3D points.

        Args:
            pixel_coords: Array of shape (N, 2) with [x, y] coordinates
            depth_image: Full depth image (H, W)
            depth_scale: Depth unit scale

        Returns:
            Array of shape (N, 3) with 3D points [x, y, z]
            Invalid points (depth=0) are set to [nan, nan, nan]
        """
        points_3d = []

        for px, py in pixel_coords:
            # Ensure pixel coords are within image bounds
            py_int = int(np.clip(py, 0, depth_image.shape[0] - 1))
            px_int = int(np.clip(px, 0, depth_image.shape[1] - 1))

            depth_value = depth_image[py_int, px_int]

            point = self.pixel_to_point(px, py, depth_value, depth_scale)

            if point is not None:
                points_3d.append(point)
            else:
                points_3d.append([np.nan, np.nan, np.nan])

        return np.array(points_3d)

    def bbox_to_3d_points(
        self,
        bbox: List[float],
        depth_image: np.ndarray,
        sample_points: int = 9,
        depth_scale: float = 0.001
    ) -> dict:
        """
        Convert bounding box to 3D points with statistics.

        Args:
            bbox: Bounding box [x1, y1, x2, y2] in pixels
            depth_image: Depth image
            sample_points: Number of points to sample (e.g., 9 = 3x3 grid)
            depth_scale: Depth unit scale

        Returns:
            Dictionary with:
            - center_3d: Center point in 3D
            - points_3d: All sampled 3D points
            - mean_depth: Mean depth value
            - dimensions_3d: Estimated 3D bounding box dimensions [width, height, depth]
        """
        x1, y1, x2, y2 = bbox

        # Sample points in a grid within bbox
        grid_size = int(np.sqrt(sample_points))
        x_samples = np.linspace(x1, x2, grid_size)
        y_samples = np.linspace(y1, y2, grid_size)

        points_2d = []
        for x in x_samples:
            for y in y_samples:
                points_2d.append([x, y])

        # Convert to 3D
        points_3d = self.pixels_to_points(
            np.array(points_2d),
            depth_image,
            depth_scale
        )

        # Filter out invalid points
        valid_points = points_3d[~np.isnan(points_3d).any(axis=1)]

        if len(valid_points) == 0:
            logger.warning("[DepthProjector] No valid depth in bounding box")
            return {
                'center_3d': None,
                'points_3d': [],
                'mean_depth': 0,
                'dimensions_3d': [0, 0, 0]
            }

        # Calculate statistics
        center_3d = np.mean(valid_points, axis=0)
        mean_depth = np.mean(valid_points[:, 2])

        # Estimate 3D dimensions
        min_coords = np.min(valid_points, axis=0)
        max_coords = np.max(valid_points, axis=0)
        dimensions_3d = max_coords - min_coords

        logger.debug(f"[DepthProjector] Bbox 3D center: {center_3d}, "
                    f"dimensions: {dimensions_3d}")

        return {
            'center_3d': center_3d,
            'points_3d': valid_points,
            'mean_depth': mean_depth,
            'dimensions_3d': dimensions_3d
        }

    def depth_image_to_point_cloud(
        self,
        depth_image: np.ndarray,
        depth_scale: float = 0.001,
        subsample: int = 1
    ) -> np.ndarray:
        """
        Convert entire depth image to point cloud.

        Args:
            depth_image: Depth image (H, W)
            depth_scale: Depth unit scale
            subsample: Subsample factor (e.g., 2 = every 2nd pixel)

        Returns:
            Point cloud array of shape (N, 3)
        """
        h, w = depth_image.shape
        points = []

        for y in range(0, h, subsample):
            for x in range(0, w, subsample):
                depth = depth_image[y, x]
                if depth > 0:
                    point = self.pixel_to_point(x, y, depth, depth_scale)
                    if point is not None:
                        points.append(point)

        return np.array(points)

    def get_roi_average_depth(
        self,
        depth_image: np.ndarray,
        center_x: float,
        center_y: float,
        roi_size: int = 5,
        depth_scale: float = 0.001
    ) -> Optional[Tuple[float, float]]:
        """
        Get average depth in a region of interest around a pixel.

        Args:
            depth_image: Depth image
            center_x: Center X coordinate
            center_y: Center Y coordinate
            roi_size: Size of ROI square (pixels)
            depth_scale: Depth unit scale

        Returns:
            Tuple of (mean_depth_meters, std_depth_meters) or None if invalid
        """
        # Define ROI bounds
        x1 = int(max(0, center_x - roi_size // 2))
        x2 = int(min(depth_image.shape[1], center_x + roi_size // 2))
        y1 = int(max(0, center_y - roi_size // 2))
        y2 = int(min(depth_image.shape[0], center_y + roi_size // 2))

        # Extract ROI
        roi = depth_image[y1:y2, x1:x2]

        # Filter out invalid depths
        valid_depths = roi[roi > 0] * depth_scale

        if len(valid_depths) == 0:
            return None

        mean_depth = np.mean(valid_depths)
        std_depth = np.std(valid_depths)

        return mean_depth, std_depth

    def _get_default_intrinsics(self) -> rs.intrinsics:
        """
        Get default D405 intrinsics for 640x480 resolution.

        WARNING: These are approximate values. For accurate 3D positioning,
        you MUST calibrate and use actual camera intrinsics.
        """
        intrinsics = rs.intrinsics()
        intrinsics.width = 640
        intrinsics.height = 480
        intrinsics.ppx = 320.0  # Principal point x
        intrinsics.ppy = 240.0  # Principal point y
        intrinsics.fx = 450.0   # Focal length x
        intrinsics.fy = 450.0   # Focal length y
        intrinsics.model = rs.distortion.none
        intrinsics.coeffs = [0, 0, 0, 0, 0]  # No distortion (simplified)

        return intrinsics

    @staticmethod
    def get_intrinsics_from_stream(pipeline: rs.pipeline) -> rs.intrinsics:
        """
        Extract intrinsics from active RealSense pipeline.

        Args:
            pipeline: Active RealSense pipeline

        Returns:
            Camera intrinsics object
        """
        profile = pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        intrinsics = depth_profile.get_intrinsics()

        logger.info(f"[DepthProjector] Extracted intrinsics from camera: "
                   f"fx={intrinsics.fx}, fy={intrinsics.fy}")

        return intrinsics


# Example usage
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    # Create projector with default intrinsics
    projector = DepthProjector()

    # Simulate depth image
    depth_image = np.random.randint(500, 1500, (480, 640), dtype=np.uint16)

    # Test single pixel projection
    point_3d = projector.pixel_to_point(320, 240, depth_image[240, 320])
    print(f"Center pixel 3D point: {point_3d}")

    # Test bounding box projection
    bbox = [200, 150, 400, 350]
    bbox_3d = projector.bbox_to_3d_points(bbox, depth_image)
    print(f"Bbox 3D center: {bbox_3d['center_3d']}")
    print(f"Bbox 3D dimensions: {bbox_3d['dimensions_3d']}")

    # Test ROI average depth
    avg_depth = projector.get_roi_average_depth(depth_image, 320, 240, roi_size=10)
    if avg_depth:
        print(f"ROI average depth: {avg_depth[0]:.3f}m ± {avg_depth[1]:.3f}m")
