# Package for icra2020

Version: 1.0.0 (documentation 0.1.0)

### Development environment
* Ubuntu 16.04
* ROS Kinetic

### Prerequiste
* Will be updated
* 

### How to use
0. roscore
1. rosbag play $target_bag_file (or roslaunch icra20 all_sensors.launch)
2. roslaunch icra20 all_nodes.launch

### Scripts descriptions
- script/laser2pc.py
   - Transform HOKUYO LaserScan to PointCloud2 
- src/data_synchronizer.cpp
   - Synchronize following data: image, HOKUYO LaserScan, HOKUYO PointClouds, realsense depth 
- src/data_saver.cpp
   - Saving thread for aforementioned data

