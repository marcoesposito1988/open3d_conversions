#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import rospkg
import unittest

import numpy as np
import open3d
import open3d_conversions
from sensor_msgs.msg import PointCloud2, PointField
from ros_numpy.point_cloud2 import pointcloud2_to_array, pointcloud2_to_xyz_array, get_xyz_points, split_rgb_field

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


# extracted from pypcd and brought to ROS2/python3

def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None, merge_rgb=False):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    if merge_rgb:
        cloud_arr = merge_rgb_fields(cloud_arr)

    # make it 2d (even if height will be 1)
    cloud_arr = np.atleast_2d(cloud_arr)

    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = arr_to_fields(cloud_arr)
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()
    return cloud_msg


def merge_rgb_fields(cloud_arr):
    '''Takes an array with named np.uint8 fields 'r', 'g', and 'b', and returns an array in
    which they have been merged into a single np.float32 'rgb' field. The first byte of this
    field is the 'r' uint8, the second is the 'g', uint8, and the third is the 'b' uint8.
    This is the way that pcl likes to handle RGB colors for some reason.
    '''
    r = np.asarray(cloud_arr['r'], dtype=np.uint32)
    g = np.asarray(cloud_arr['g'], dtype=np.uint32)
    b = np.asarray(cloud_arr['b'], dtype=np.uint32)
    rgb_arr = np.array((r << 16) | (g << 8) | (b << 0), dtype=np.uint32)

    # not sure if there is a better way to do this. i'm changing the type of the array
    # from uint32 to float32, but i don't want any conversion to take place -jdb
    rgb_arr.dtype = np.float32

    # create a new array, without r, g, and b, but with rgb float32 field
    new_dtype = []
    for field_name in cloud_arr.dtype.names:
        field_type, field_offset = cloud_arr.dtype.fields[field_name]
        if field_name not in ('r', 'g', 'b'):
            new_dtype.append((field_name, field_type))
    new_dtype.append(('rgb', np.float32))
    new_cloud_arr = np.zeros(cloud_arr.shape, new_dtype)

    # fill in the new array
    for field_name in new_cloud_arr.dtype.names:
        if field_name == 'rgb':
            new_cloud_arr[field_name] = rgb_arr
        else:
            new_cloud_arr[field_name] = cloud_arr[field_name]

    return new_cloud_arr


def arr_to_fields(cloud_arr):
    '''Convert a numpy record datatype into a list of PointFields.
    '''
    fields = []
    for field_name in cloud_arr.dtype.names:
        np_field_type, field_offset = cloud_arr.dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        pf.count = 1 # is this ever more than one?
        fields.append(pf)
    return fields