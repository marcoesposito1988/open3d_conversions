#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import rospkg
import unittest

import numpy as np
import open3d
import open3d_conversions
from ros_numpy.point_cloud2 import pointcloud2_to_array, pointcloud2_to_xyz_array, get_xyz_points, split_rgb_field

try:
    # https://github.com/dimatura/pypcd (must follow README to install it with ROS support)
    import pypcd
except ImportError:
    print('pypcd is required to run the tests; follow the README of https://github.com/dimatura/pypcd')


rospack = rospkg.RosPack()
our_module_path = rospack.get_path('open3d_conversions')
pcd_path_xyz = os.path.join(our_module_path, 'test', 'test_cloud_XYZ_noRGB.pcd')
pcd_path_xyzrgb = os.path.join(our_module_path, 'test', 'test_cloud_XYZRGB.pcd')


def get_rgb_points(cloud_array):
    # pull out rgb values
    points = np.zeros(cloud_array.shape + (3,), dtype=np.float)
    points[..., 0] = cloud_array['r']
    points[..., 1] = cloud_array['g']
    points[..., 2] = cloud_array['b']

    return points


class PointCloudTestCase(unittest.TestCase):
    def are_clouds_equal(self, ros_cloud, open3d_cloud):
        open3d_xyz_asarray = np.asarray(open3d_cloud.points)
        ros_xyz_asarray = pointcloud2_to_xyz_array(ros_cloud)

        self.assertEqual(1, ros_cloud.height)

        self.assertEqual(ros_cloud.width, open3d_xyz_asarray.shape[0])

        self.assertTrue(np.all(np.isclose(open3d_xyz_asarray, ros_xyz_asarray)))

        if open3d_cloud.has_colors():
            open3d_colors_asarray = np.asarray(open3d_cloud.colors) * 255
            ros_colors_asarray = get_rgb_points(split_rgb_field(pointcloud2_to_array(ros_cloud)))

            self.assertTrue(np.all(np.isclose(open3d_colors_asarray, ros_colors_asarray)))


class TestOpen3dToRos(PointCloudTestCase):
    def dotest(self, filename):
        open3d_cloud = open3d.read_point_cloud(filename)

        ros_cloud = open3d_conversions.to_msg(open3d_cloud)

        self.are_clouds_equal(ros_cloud, open3d_cloud)

    def test_xyz(self):
        self.dotest(pcd_path_xyz)

    def test_xyzrgb(self):
        self.dotest(pcd_path_xyzrgb)


class TestRosToOpen3d(PointCloudTestCase):
    def dotest(self, filename):
        ros_cloud = pypcd.PointCloud.from_path(filename).to_msg()

        open3d_cloud = open3d_conversions.from_msg(ros_cloud)

        self.are_clouds_equal(ros_cloud, open3d_cloud)

    def test_xyz(self):
        self.dotest(pcd_path_xyz)

    def test_xyzrgb(self):
        self.dotest(pcd_path_xyzrgb)
