# open3d_conversions

Python ROS package to convert objects between the `sensor_msgs/PointCloud2` messages and Open3D `PointCloud` formats.

## Getting started

```shell script
# cd into your workspace src folder
cd ~/ros/colcon_ws/src

# fetch ros_numpy (which was not released for Melodic)
git clone https://github.com/eric-wieser/ros_numpy.git

# fetch this repository
git clone https://github.com/marcoesposito1988/open3d_conversions.git

# build
cd ..
colcon build

# resource the environment 
source install/setup.bash
```

## Minimal example

```python
import open3d_conversions
from sensor_msgs.msg import PointCloud2

ros_cloud = PointCloud2(...)

o3d_cloud = open3d_conversions.from_msg(current_cloud)

# do open3d things
# ...

ros_cloud2 = open3d_conversions.to_msg(o3d_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)


```

## Running the tests

Reading a `.pcd` file into a `sensor_msgs/PointCloud2` requires [`pypcd`](https://github.com/dimatura/pypcd), which requires a manual step for installation.

## Acknowledgements

The [open3d_ros_pointcloud_conversion](https://github.com/felixchenfy/open3d_ros_pointcloud_conversion) repository from @felixchenfy was used as a starting point. 