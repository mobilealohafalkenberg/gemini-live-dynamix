#!/usr/bin/env python3
"""
Unit tests for vision system components

Tests:
- DepthProjector: 2D to 3D projection
- CameraCalibration: Frame transformations
- ObjectLocalizer: Integration test
"""

import unittest
import numpy as np
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision.depth_projector import DepthProjector
from vision.camera_calibration import CameraCalibration


class TestDepthProjector(unittest.TestCase):
    """Test DepthProjector functionality"""

    def setUp(self):
        """Initialize projector with default intrinsics"""
        self.projector = DepthProjector()

    def test_pixel_to_point_center(self):
        """Test projection of center pixel"""
        # Center pixel should project to approximately [0, 0, depth]
        point_3d = self.projector.pixel_to_point(320, 240, 1000, depth_scale=0.001)

        self.assertIsNotNone(point_3d)
        self.assertEqual(len(point_3d), 3)

        # Z should be approximately 1.0m (1000 * 0.001)
        self.assertAlmostEqual(point_3d[2], 1.0, places=2)

    def test_pixel_to_point_invalid_depth(self):
        """Test handling of invalid depth (0)"""
        point_3d = self.projector.pixel_to_point(320, 240, 0)

        self.assertIsNone(point_3d)

    def test_pixels_to_points_batch(self):
        """Test batch projection of multiple pixels"""
        pixel_coords = np.array([
            [320, 240],  # Center
            [100, 100],  # Top-left region
            [500, 400]   # Bottom-right region
        ])

        depth_image = np.full((480, 640), 1000, dtype=np.uint16)

        points_3d = self.projector.pixels_to_points(pixel_coords, depth_image)

        self.assertEqual(points_3d.shape, (3, 3))

        # All points should have valid Z values around 1.0m
        for point in points_3d:
            self.assertAlmostEqual(point[2], 1.0, places=2)

    def test_bbox_to_3d_points(self):
        """Test bounding box to 3D projection"""
        bbox = [200, 150, 400, 350]
        depth_image = np.full((480, 640), 1000, dtype=np.uint16)

        result = self.projector.bbox_to_3d_points(bbox, depth_image, sample_points=9)

        self.assertIn('center_3d', result)
        self.assertIn('points_3d', result)
        self.assertIn('mean_depth', result)

        # Center should be valid
        self.assertIsNotNone(result['center_3d'])
        self.assertEqual(len(result['center_3d']), 3)

        # Should have sampled points
        self.assertGreater(len(result['points_3d']), 0)

    def test_roi_average_depth(self):
        """Test ROI depth averaging"""
        depth_image = np.full((480, 640), 1000, dtype=np.uint16)

        result = self.projector.get_roi_average_depth(
            depth_image, 320, 240, roi_size=10
        )

        self.assertIsNotNone(result)
        mean_depth, std_depth = result

        self.assertAlmostEqual(mean_depth, 1.0, places=2)
        self.assertLess(std_depth, 0.01)  # Should be very low for uniform depth


class TestCameraCalibration(unittest.TestCase):
    """Test CameraCalibration functionality"""

    def setUp(self):
        """Initialize calibration with default transforms"""
        self.calibration = CameraCalibration()

    def test_initialization(self):
        """Test that calibration initializes with default transforms"""
        self.assertIn('top_cam', self.calibration.transforms)
        self.assertIn('gripper_cam', self.calibration.transforms)

    def test_transform_point_top_cam(self):
        """Test transforming a point from top camera to base"""
        # Point in camera frame
        point_camera = np.array([0.0, 0.0, 0.5])  # 50cm in front of camera

        point_base = self.calibration.transform_point(point_camera, 'top_cam')

        self.assertEqual(len(point_base), 3)

        # With default top_cam at [0.3, 0.0, 0.6] and rotated 180° around X,
        # a point at [0, 0, 0.5] in camera frame should transform to base frame
        # The exact value depends on the rotation, but Z should be less than 0.6
        self.assertLess(point_base[2], 0.6)

    def test_transform_multiple_points(self):
        """Test batch transformation of points"""
        points_camera = np.array([
            [0.0, 0.0, 0.5],
            [0.1, 0.0, 0.5],
            [0.0, 0.1, 0.5]
        ])

        points_base = self.calibration.transform_points(points_camera, 'top_cam')

        self.assertEqual(points_base.shape, (3, 3))

    def test_get_transform_matrix(self):
        """Test getting transformation matrix"""
        T = self.calibration.get_transform_matrix('top_cam')

        # Should be 4x4 homogeneous transform
        self.assertEqual(T.shape, (4, 4))

        # Bottom row should be [0, 0, 0, 1]
        np.testing.assert_array_almost_equal(T[3, :], [0, 0, 0, 1])

    def test_is_calibrated(self):
        """Test calibration status check"""
        # Default transforms should not be marked as calibrated
        self.assertFalse(self.calibration.is_calibrated('top_cam'))

    def test_set_transform(self):
        """Test manually setting a transform"""
        translation = np.array([0.5, 0.0, 0.8])
        rotation = np.eye(3)  # Identity rotation

        self.calibration.set_transform(
            'top_cam',
            translation,
            rotation,
            calibrated=True
        )

        # Should now be marked as calibrated
        self.assertTrue(self.calibration.is_calibrated('top_cam'))

        # Transform should match what we set
        T = self.calibration.get_transform_matrix('top_cam')
        np.testing.assert_array_almost_equal(T[:3, 3], translation)

    def test_save_load_calibration(self):
        """Test saving and loading calibration"""
        import tempfile

        # Create temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_file = f.name

        try:
            # Save calibration
            self.calibration.save_calibration(temp_file)

            # Load into new instance
            calibration2 = CameraCalibration(temp_file)

            # Should have same transforms
            T1 = self.calibration.get_transform_matrix('top_cam')
            T2 = calibration2.get_transform_matrix('top_cam')

            np.testing.assert_array_almost_equal(T1, T2)

        finally:
            # Clean up
            if os.path.exists(temp_file):
                os.remove(temp_file)


class TestVisionIntegration(unittest.TestCase):
    """Integration tests for complete vision pipeline"""

    def test_end_to_end_localization_simulation(self):
        """Simulate complete object localization pipeline"""
        # This test simulates the workflow without requiring actual images

        # 1. Create components
        projector = DepthProjector()
        calibration = CameraCalibration()

        # 2. Simulate object detection result
        # Object detected at pixel (320, 240) with depth 1000mm
        pixel_x, pixel_y = 320, 240
        depth_value = 1000

        # 3. Project to 3D in camera frame
        point_3d_camera = projector.pixel_to_point(
            pixel_x, pixel_y, depth_value, depth_scale=0.001
        )

        self.assertIsNotNone(point_3d_camera)

        # 4. Transform to base frame
        point_3d_base = calibration.transform_point(point_3d_camera, 'top_cam')

        self.assertEqual(len(point_3d_base), 3)

        # 5. Verify point is within reasonable workspace
        # X should be positive (forward)
        self.assertGreater(point_3d_base[0], 0)

        print(f"\n  Simulated object location in base frame: {point_3d_base}")


def run_tests():
    """Run all vision tests"""
    print("\n" + "="*60)
    print("Running Vision System Tests")
    print("="*60 + "\n")

    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    suite.addTests(loader.loadTestsFromTestCase(TestDepthProjector))
    suite.addTests(loader.loadTestsFromTestCase(TestCameraCalibration))
    suite.addTests(loader.loadTestsFromTestCase(TestVisionIntegration))

    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Print summary
    print("\n" + "="*60)
    if result.wasSuccessful():
        print("✓ All tests passed!")
    else:
        print(f"✗ {len(result.failures)} test(s) failed")
        print(f"✗ {len(result.errors)} test(s) had errors")
    print("="*60 + "\n")

    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
