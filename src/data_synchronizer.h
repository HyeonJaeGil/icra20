#ifndef DATA_SYNCHRONIZER_H
#define DATA_SYNCHRONIZER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <unavlib/convt.h>
#include <iostream>
#include "chrono"
#include <mutex>
#include "icra20/synced_node.h"
#include <math.h>

class dataSynchronizer
{
private:

  int m_count;
  int m_width;
  int m_height;
  int m_focal_length;

  int m_x_pixel;
  int m_y_pixel;

  int m_sync_count;

  float m_x_center;
  float m_y_center;

  float m_max_range;

  //ros::Timer timer;
//  std::mutex m_mtx_lock;
  ros::Publisher m_pub_node;
//  std::float offset;
  std::vector<sensor_msgs::CompressedImage> m_image;
  std::vector<sensor_msgs::Image> m_depth;
  std::vector<sensor_msgs::PointCloud2> m_rp_pc;
  std::vector<sensor_msgs::LaserScan> m_rp;
  //std::vector<sensor_msgs::PointCloud2> m_depth_pc;

  std::vector<icra20::synced_node> m_synced_node;

  //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > m_pc_2d;

  // void synchronize_data();

  void callback_image (const sensor_msgs::CompressedImage::ConstPtr &msg);
  void callback_depth (const sensor_msgs::Image::ConstPtr &msg);
  void callback_Raw (const sensor_msgs::LaserScan::ConstPtr& msg);
  void callback_2dLidar (const sensor_msgs::PointCloud2::ConstPtr& msg);
  //void callback_pcDepth (const sensor_msgs::PointCloud2::ConstPtr& msg);

  bool update(icra20::synced_node& msg);
//  void print_hi();
public:
  ros::NodeHandle m_nh;
  dataSynchronizer();
  ~dataSynchronizer();

};

#endif // DATA_SYNCHRONIZER_H
