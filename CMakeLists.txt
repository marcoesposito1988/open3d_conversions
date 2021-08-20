cmake_minimum_required(VERSION 3.0.2)

project(open3d_conversions)

find_package(catkin REQUIRED)
catkin_python_setup()

catkin_package()

catkin_add_nosetests(test/test.py)
