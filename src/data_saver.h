#ifndef DATA_SAVER_H
#define DATA_SAVER_H

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
#include <fstream>
#include <string>
#include "chrono"
#include <mutex>
#include "icra20/synced_node.h"
#include <math.h>
#include <unavlib/convt.h>
#include <unavlib/others.h>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include "opencv2/core.hpp"
#include <math.h>
#include <sys/stat.h>


class dataSaver
{
private:
  int m_count;
  int m_width;
  int m_height;
  int m_focal_length;

  int m_x_pixel;
  int m_y_pixel;

  int m_sync_count;
  int save_count;
  float m_x_center;
  float m_y_center;

  std::string abs_path;
  std::string data_name;
  std::string index;

  //ros::Publisher m_pub_rp_debug;

  float m_max_range;
  float m_max_range_search;
  cv::Scalar get_colour(float v, float vmin, float vmax);
  void writeCSV(std::string filename, cv::Mat m);
  void callback_saving_data(const icra20::synced_node::ConstPtr& msg);
  bool saving_flag;

public:
  ros::NodeHandle m_nh;
  dataSaver();
  dataSaver(std::string abs_folder);
  ~dataSaver();
};

#endif // DATASAVER_H
