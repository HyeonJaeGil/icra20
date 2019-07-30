#include <ros/ros.h>
#include <data_saver.h>
// PCL specific includes

int main (int argc, char** argv)
{
  // Initialize ROS
  ROS_INFO("Start operating Saving node...");

  ros::init (argc, argv, "Saving_synced_node");
  ros::NodeHandle nh;

  std::string abs_folder;
  nh.param("/pclxyz_to_save/path", abs_folder, std::string("/home/hj/ICRA2020/190403"));
  // if(nh.param("path", abs_folder, std::string("/home/hj/ICRA2020/190403"))) {ROS_INFO("Got param %s",abs_folder.c_str());}
  // else {ROS_ERROR("Failed to get param");}
  dataSaver saverClass = dataSaver(abs_folder);
  std::cout<<abs_folder<<std::endl;
//  ros::Timer timer = syncClass.m_nh.createTimer(ros::Duration(0.1), print_hi);

  ros::spin();
  return 0;
}
