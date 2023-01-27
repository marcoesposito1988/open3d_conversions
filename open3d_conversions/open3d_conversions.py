#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import open3d
import ros2_numpy
from sensor_msgs.msg import PointCloud2 as pc2
from numpy.lib import recfunctions
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
                [PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]


def to_msg(open3d_cloud, frame_id=None, stamp=None):
    header = Header()
    if stamp is not None:
        header.stamp = stamp
    if frame_id is not None:
        header.frame_id = frame_id

    o3d_asarray = np.asarray(open3d_cloud.points)

    o3d_x = o3d_asarray[:, 0]
    o3d_y = o3d_asarray[:, 1]
    o3d_z = o3d_asarray[:, 2]

    cloud_data = np.core.records.fromarrays([o3d_x, o3d_y, o3d_z], names='x,y,z')

    if not open3d_cloud.colors:  # XYZ only
        fields = FIELDS_XYZ
    else:  # XYZ + RGB
        fields = FIELDS_XYZRGB
        color_array = np.array(np.floor(np.asarray(open3d_cloud.colors) * 255), dtype=np.uint8)

        o3d_r = color_array[:, 0]
        o3d_g = color_array[:, 1]
        o3d_b = color_array[:, 2]

        cloud_data = np.lib.recfunctions.append_fields(cloud_data, ['r', 'g', 'b'], [o3d_r, o3d_g, o3d_b])

        cloud_data = ros_numpy.point_cloud2.merge_rgb_fields(cloud_data)

    return pc2.create_cloud(header, fields, cloud_data)


def from_msg(ros_cloud):
    xyzrgb_array = ros_numpy.point_cloud2.pointcloud2_to_array(ros_cloud)

    mask = np.isfinite(xyzrgb_array['x']) & np.isfinite(xyzrgb_array['y']) & np.isfinite(xyzrgb_array['z'])
    cloud_array = xyzrgb_array[mask]

    open3d_cloud = open3d.PointCloud()

    points = np.zeros(cloud_array.shape + (3,), dtype=np.float)
    points[..., 0] = cloud_array['x']
    points[..., 1] = cloud_array['y']
    points[..., 2] = cloud_array['z']
    open3d_cloud.points = open3d.Vector3dVector(points)

    if 'rgb' in xyzrgb_array.dtype.names:
        rgb_array = ros_numpy.point_cloud2.split_rgb_field(xyzrgb_array)
        cloud_array = rgb_array[mask]

        colors = np.zeros(cloud_array.shape + (3,), dtype=np.float)
        colors[..., 0] = cloud_array['r']
        colors[..., 1] = cloud_array['g']
        colors[..., 2] = cloud_array['b']

        open3d_cloud.colors = open3d.Vector3dVector(colors / 255.0)

    return open3d_cloud
