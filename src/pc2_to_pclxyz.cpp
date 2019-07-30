#include <ros/ros.h>
#include <data_synchronizer.h>

// PCL specific includes

int main (int argc, char** argv)
{
  // Initialize ROS
  ROS_INFO("Start operating Synchronizing node...");
  ros::init (argc, argv, "Sync_node");
  dataSynchronizer syncClass = dataSynchronizer();

//  ros::Timer timer = syncClass.m_nh.createTimer(ros::Duration(0.1), print_hi);

  ros::spin();
  return 0;
}
