# Package for icra2020

Version: 1.0.0 (documentation 0.1.0)

### Development environment
* Ubuntu 16.04
* ROS Kinetic

### Prerequiste
* Will be updated
* 

### How to save data
0. roscore
1. roslaunch icra20 all_sensors.launch)
2. rosrun rqt_reconfigure rqt_reconfigure , lowering 6 param
3. rosbag record /scan /camera/aligned_depth_to_color/image_raw/compressedDepth /camera/color/image_raw/compressed /camera/depth/color/points
4. roslaunch icra20 all_nodes.launch
      This makes dir + saves all data.
      make sure to change launch file parameter.

### How to make h5 + visualize it
1. python3 generate_h5.py //should change target_path
2. python3 analyze_h5.py
   +)functions in analyze_h5.py
      save_projected_img : save img file with rp pcl projected to 'img+rp' directory.
      visualize_depth_and_projected_img : concat 'img file with rp pcl projected' and 'colored depth' + visualize it.(Fast)
      save_concat_img : concat 'img file with rp pcl projected' and 'colored depth' + save it to 'concat' directory.


### Scripts descriptions
- script/laser2pc.py
   - Transform HOKUYO LaserScan to PointCloud2 
- src/data_synchronizer.cpp
   - Synchronize following data: image, HOKUYO LaserScan, HOKUYO PointClouds, realsense depth 
- src/data_saver.cpp
   - Saving thread for aforementioned data

