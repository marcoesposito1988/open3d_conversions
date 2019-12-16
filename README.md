# open3d_conversions

Python ROS package to convert objects between the `sensor_msgs/PointCloud2` messages and Open3D `PointCloud` formats.

## Getting started

```shell script
# cd into your workspace src folder
cd ~/ros/catkin_ws/src

# fetch ros_numpy (which was not released for Melodic)
git clone https://github.com/eric-wieser/ros_numpy.git

# fetch this repository
git clone https://github.com/marcoesposito1988/open3d_conversions.git

# build
cd ..
catkin build

# resource the environment 
source devel/setup.bash
```

## Minimal example

```python
import rospy
import open3d_conversions
from sensor_msgs.msg import PointCloud2

rospy.init_node('open3d_conversions_example')

current_cloud = None

def handle_pointcloud(pointcloud2_msg):
    global current_cloud
    current_cloud = pointcloud2_msg

rate = rospy.Rate(1)

listener = rospy.Subscriber('/some_rgbd_camera/depth_registered/points', PointCloud2, handle_pointcloud, queue_size=1)
publisher = rospy.Publisher('~processed_point_cloud', PointCloud2, queue_size=1)

while not rospy.is_shutdown():
    if current_cloud is None:
        continue

    o3d_cloud = open3d_conversions.from_msg(current_cloud)

    # do open3d things
    # ...

    ros_cloud = open3d_conversions.to_msg(o3d_cloud, frame_id=current_cloud.header.frame_id, stamp=current_cloud.header.stamp)
        
    current_cloud = None
    rate.sleep()

```

## Running the tests

Reading a `.pcd` file into a `sensor_msgs/PointCloud2` requires [`pypcd`](https://github.com/dimatura/pypcd), which requires a manual step for installation.

## Acknowledgements

The [open3d_ros_pointcloud_conversion](https://github.com/felixchenfy/open3d_ros_pointcloud_conversion) repository from @felixchenfy was used as a starting point. 