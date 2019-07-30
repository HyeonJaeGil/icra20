#include <ros/ros.h>
#include <iostream>
#include "sensor_msgs/CompressedImage.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

#include <image_transport/image_transport.h>

int count = 0;
std::string abs_path = "/home/hj/ICRA2020/190403";
std::string data_name = "sa0403_";

void writeCSV(std::string filename, cv::Mat m)
{
   std::ofstream myfile;
   myfile.open(filename.c_str());
   myfile<< cv::format(m, cv::Formatter::FMT_CSV) << std::endl;
   myfile.close();
}


void SaveCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  double min, max;
  cv::minMaxLoc(cv_ptr->image, &min, &max);
  cv::Mat colorMap;
  cv::convertScaleAbs(cv_ptr->image, colorMap,255.0/(max-min),-255.0/(max-min)*min);
  cv::applyColorMap(colorMap, colorMap, 2);
  cv::imshow("imge", colorMap);
  //cv::imwrite("/home/hj/ICRA2020/190403/depth2csv/depth.png", colorMap);
  cv::waitKey(1);

  std::string index = std::to_string(count);
  std::string depth_img2csv_path =abs_path + "/depth2csv/" + data_name + index + ".csv";
  writeCSV(depth_img2csv_path, cv_ptr->image);
  count ++;

  std::cout<<"max: "<<max<<", min: "<<min<<std::endl;
  //ROS_INFO("Saving complete.");
  //save Image
  // ros::shutdown();

}

// void SaveCallback(const sensor_msgs::Image::ConstPtr& msg)
// {
//   cv_bridge::CvImagePtr cv_ptr;
//   cv_ptr = cv_bridge::toCvCopy(msg->depth, sensor_msgs::image_encodings::TYPE_16UC1);
//   double min, max;
//   cv::minMaxLoc(cv_ptr->image, &min, &max);
//   cv::Mat colorMap;
//   cv::convertScaleAbs(cv_ptr->image, colorMap,255.0/(max-min),-255.0/(max-min)*min);
//   cv::applyColorMap(colorMap, colorMap, 2);
//   cv::imshow("imge", colorMap);
//   //cv::imwrite("/home/hj/ICRA2020/190403/depth2csv/depth.png", colorMap);
//   cv::waitKey(1);
//
//   std::string index = std::to_string(count);
//   std::string depth_img2csv_path =abs_path + "/depth2csv/" + data_name + index + ".csv";
//   writeCSV(depth_img2csv_path, cv_ptr->image);
//   count ++;
//
//   std::cout<<"max: "<<max<<", min: "<<min<<std::endl;
//   //ROS_INFO("Saving complete.");
//   //save Image
//   // ros::shutdown();
//
// }



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "img_save_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/depth", 100, SaveCallback);

  ros::spin();
  return 0;
}
