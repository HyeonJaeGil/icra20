#include "data_saver.h"

using namespace unavlib;

dataSaver::dataSaver()
{
  m_width = 640;
  m_height = 480;
  m_focal_length = 609.89844; //f = (fx + fy)/2

  m_x_center = 316.0020446777344;
  m_y_center = 250.0917205810547;
  m_max_range = 10;
  m_sync_count = 0;
  save_count = 0;
  static ros::Subscriber sub5 = m_nh.subscribe<icra20::synced_node>("/Synced_node", 1, &dataSaver::callback_saving_data, this);

  //m_pub_rp_debug = m_nh.advertise<sensor_msgs::PointCloud2>("/filtered_rp_pc",100);

  m_max_range_search = 0;
  saving_flag = true;
  //// For saving data
  abs_path = "/home/hj/ICRA2020/190403";
  data_name = "sa0403_";

}

dataSaver::dataSaver(std::string abs_folder)
{
  m_width = 640;
  m_height = 480;
  m_focal_length = 609.89844; //f = (fx + fy)/2

  m_x_center = 316.0020446777344;
  m_y_center = 250.0917205810547;
  m_max_range = 10;
  m_sync_count = 0;
  save_count = 0;
  static ros::Subscriber sub5 = m_nh.subscribe<icra20::synced_node>("/Synced_node", 1, &dataSaver::callback_saving_data, this);

  //m_pub_rp_debug = m_nh.advertise<sensor_msgs::PointCloud2>("/filtered_rp_pc",100);

  m_max_range_search = 0;
  saving_flag = true;
  //// For saving data
  abs_path = abs_folder;

  data_name = "0000_";

  std::string sub_directory[5] = {"/img", "/rp_img2csv", "/rp_raw", "/rp_intensity", "/depth2csv"};

  const int dir_err = mkdir(abs_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if (-1 == dir_err)
  {
      printf("Error creating new directory or Already existing directory.");
      exit(1);
  }

  int if_error;
  for(int i = 0 ; i < 5 ; i++)
  {
    if_error = mkdir((abs_path+sub_directory[i]).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if(if_error == -1)
    {
      printf("Error creating sub_directory or Already existing directory.");
      exit(1);
    }
  }


}

dataSaver::~dataSaver()
{
}

cv::Scalar dataSaver::get_colour(float v, float vmin, float vmax)
{
    float normalized_v = v/vmax;
    if(normalized_v<=0)
      return cv::Scalar(255, 255, 255);
    cv::Vec3b getColor = unavlib::datahandle3d::heightcolor(normalized_v);
    return cv::Scalar(getColor[2], getColor[1], getColor[0]);
}

/*________________________________________________________________________________________________________________________________________*/


void dataSaver::writeCSV(std::string filename, cv::Mat m)
{
   std::ofstream myfile;
   myfile.open(filename.c_str());
   // myfile<< cv::format(m, cv::Formatter::FMT_CSV) << std::endl;
   myfile<< cv::format(m, cv::Formatter::FMT_CSV);
   myfile.close();
}

/*________________________________________________________________________________________________________________________________________*/


void dataSaver::callback_saving_data(const icra20::synced_node::ConstPtr &msg)
{

   // std::cout<<"Receiving " <<msg->idx<<"th synced data!"<<std::endl;

/*________________________________________________________________________________________________________________________________________*/
///Raw data part
   int scanQuantity =((msg->Raw.angle_max)-(msg->Raw.angle_min))/(msg->Raw.angle_increment)+1;
   Eigen::MatrixX3f eigenRaw = Eigen::MatrixX3f::Zero(1,3);
   int save_index = 0;

   //Extract information and decide whether to save or not.
   for(int i=0 ; i<scanQuantity ; i++){

     float dist = msg->Raw.ranges[i];
     float theta = msg->Raw.angle_min + ( msg->Raw.angle_increment * i);
     float intensity = msg->Raw.intensities[i];

     // float x = -1 * dist * sin(theta);
     // float y = -1 * 0.064;
     // float z = dist * cos(theta) - 0.369;
     //
     // float relative_x, relative_y;
     // relative_x = x * m_focal_length / z;
     // relative_y = y * m_focal_length / z;
     //
     // m_x_pixel = static_cast<int>(round(m_x_center + relative_x));
     // m_y_pixel = static_cast<int>(round(m_y_center + relative_y));
     //
     // if (m_x_pixel >= 0 && m_x_pixel < m_width){
     //   if(m_y_pixel >= 0 && m_y_pixel < m_height){
     //     if(z >= 0.1 && z <= m_max_range){
     //       //save the raw data and intensity data.
     //       save_index++;
     //       eigenRaw.conservativeResize(save_index,3);
     //       eigenRaw(save_index-1 , 0) = theta;
     //       eigenRaw(save_index-1 , 1) = dist;
     //       eigenRaw(save_index-1 , 2) = intensity;
     //     }
     //   }
     // }

     if(theta >= -0.453786 && theta <= 0.431969){
               //save the raw data and intensity data.
         save_index++;
         eigenRaw.conservativeResize(save_index,3);
         eigenRaw(save_index-1 , 0) = theta;
         if(std::isnan(dist) || dist == 0 || dist >= 30) eigenRaw(save_index-1, 1) = 0;
         else eigenRaw(save_index-1 , 1) = dist;
         eigenRaw(save_index-1 , 2) = intensity;
     }


   }
  //ROS_INFO("save_index : %d",save_index);

/*________________________________________________________________________________________________________________________________________*/

//// RP LIDAR PART
   pcl::PointCloud<pcl::PointXYZ> rp_output;
   pcl::PointCloud<pcl::PointXYZ>::Ptr rp_filtered(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::fromROSMsg(msg->rp_lidar, rp_output);
   pcl::PassThrough<pcl::PointXYZ> filter;
   Eigen::Matrix4f rp_trans;
   rp_trans<< 0, -1, 0, 0,
              0,  0, -1, -0.064,
             1,  0, 0, -0.0369,
              0,  0, 0, 1;
   pcl::transformPointCloud(rp_output, *rp_filtered, rp_trans);
   filter.setInputCloud(rp_filtered);
   filter.setFilterFieldName("z");
   filter.setFilterLimits(0.1, m_max_range);
   filter.filter(*rp_filtered);

   // For debugging
   // m_pub_rp_debug.publish(cvt::cloud2msg(*rp_filtered));

   // Transform (x,y,z) set to eigen
   Eigen::MatrixXd mat_rp_pc(m_width, m_height);
   mat_rp_pc = Eigen::MatrixXd::Zero(m_width, m_height);

   int save_index2 = 0;
   //ROS_INFO("DEBUG 0");
   for (int i=0; i < rp_filtered->points.size(); i++){
     float relative_x, relative_y;
     relative_x = rp_filtered->points[i].x * m_focal_length / rp_filtered->points[i].z;
     relative_y = rp_filtered->points[i].y * m_focal_length / rp_filtered->points[i].z;

     m_x_pixel = static_cast<int>(round(m_x_center + relative_x));
     m_y_pixel = static_cast<int>(round(m_y_center + relative_y));

     if (m_x_pixel >= 0 && m_x_pixel< m_width){
       if (m_y_pixel >= 0 && m_y_pixel< m_height){
         mat_rp_pc(m_x_pixel, m_y_pixel) = rp_filtered->points[i].z;
         save_index2++;
       }
     }
   }
   // if (save_index + 1 != save_index2){
   //   ROS_INFO("==================================");
   //   ROS_INFO("save index for Raw : %d , PCL : %d at %d th index",save_index,save_index2,msg->idx);
   //   ROS_INFO("==================================");
   // }

/*________________________________________________________________________________________________________________________________________*/

////depth pcl
  // pcl::PointCloud<pcl::PointXYZ> depth_output;
  // // pcl::PointCloud<pcl::PointXYZ>::Ptr depth_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromROSMsg(msg->depth_pc, depth_output);
  // // pcl::PassThrough<pcl::PointXYZ> depth_filter;
  // // filter.setInputCloud(depth_filtered);
  // // filter.setFilterFieldName("z");
  // // filter.setFilterLimits(0.1, m_max_range);
  // // filter.filter(*depth_filtered);
  //
  // Eigen::MatrixXd mat_depth_pc(m_width, m_height);
  // mat_depth_pc = Eigen::MatrixXd::Zero(m_width, m_height);
  //
  // int save_index3 = 0;
  // std::cout<<depth_output.points.size()<<std::endl;
  // for (int i=0; i < depth_output.points.size(); i++){
  //   float relative_x, relative_y;
  //   relative_x = depth_output.points[i].x * m_focal_length / depth_output.points[i].z;
  //   relative_y = depth_output.points[i].y * m_focal_length / depth_output.points[i].z;
  //
  //   m_x_pixel = static_cast<int>(round(m_x_center + relative_x));
  //   m_y_pixel = static_cast<int>(round(m_y_center + relative_y));
  //
  //   if (m_x_pixel >= 0 && m_x_pixel< m_width){
  //     if (m_y_pixel >= 0 && m_y_pixel< m_height){
  //       if(depth_output.points[i].z >= 0 && depth_output.points[i].z <= m_max_range)
  //       {
  //       mat_depth_pc(m_x_pixel, m_y_pixel) = depth_output.points[i].z;
  //       save_index3++;
  //       }
  //     }
  //   }
  // }
  // ROS_INFO("save index 3 = %d",save_index3);

/*________________________________________________________________________________________________________________________________________*/

////Image
   cv::Mat decoded_image = cv::imdecode(cv::Mat(msg->image.data), 1);
   //cv::Mat decoded_depth = cv::imdecode(cv::Mat(msg->depth.data), 1);
   cv::Mat camMatrix = cv::Mat::zeros(3, 3, CV_32FC1);
   camMatrix.at<float>(0,0) = 609.9140625;
   camMatrix.at<float>(0,1) = 0.0;
   camMatrix.at<float>(0,2) = 316.0020446777344;
   camMatrix.at<float>(1,0) = 0.0;
   camMatrix.at<float>(1,1) = 609.8822631835938;
   camMatrix.at<float>(1,2) = 250.0917205810547;
   camMatrix.at<float>(2,0) = 0.0;
   camMatrix.at<float>(2,1) = 0.0;
   camMatrix.at<float>(2,2) = 1;

   cv::Mat distCoeffs = cv::Mat::zeros(1, 4, CV_32FC1);
   // distCoeffs.at<float>(0) = -0.00712492670165381;
   // distCoeffs.at<float>(1) = 0.72739103919728;
   // distCoeffs.at<float>(2) = 0.0127217011712637;
   // distCoeffs.at<float>(3) = -0.0036617178232812;

   cv::Mat imgUndistorted;
   //cv::Mat depthUndistorted;
   cv::undistort(decoded_image, imgUndistorted, camMatrix, distCoeffs);
   //cv::undistort(decoded_depth, depthUndistorted, camMatrix, distCoeffs);
/*________________________________________________________________________________________________________________________________________*/


////Visualization
   cv::Mat img_for_rp = imgUndistorted.clone();

   int width = imgUndistorted.cols;
   int height = imgUndistorted.rows;

   for (int y = 0; y < height; y++){
     for (int x = 0; x < width; x++){
       //// B, G, R in order

       /// / cvMat.at<uchar>(cv::Point(X,Y))
       cv::Scalar rp_rgb = get_colour(mat_rp_pc(x, y), 0.0, m_max_range);

       if(!(rp_rgb.val[0]==255 && rp_rgb.val[1]==255 && rp_rgb.val[2]==255))
       {
         if (mat_rp_pc(x, y)<=1.0){
           cv::circle(img_for_rp, cv::Point(x,y), 6, rp_rgb, -1);
         }else if (mat_rp_pc(x, y)<4.0 && mat_rp_pc(x, y)>1.0){
           cv::circle(img_for_rp, cv::Point(x,y), 3, rp_rgb, -1);
         }else if (mat_rp_pc(x, y)>=4.0) {
           cv::circle(img_for_rp, cv::Point(x,y), 1, rp_rgb, -1);
         }
       }
     }
   }

/*________________________________________________________________________________________________________________________________________*/

////Depth visualization

  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg->depth, sensor_msgs::image_encodings::TYPE_16UC1);
  double min, max;
  cv::minMaxLoc(cv_ptr->image, &min, &max);
  cv::Mat colorMap;
  cv::convertScaleAbs(cv_ptr->image, colorMap,255.0/(max-min),-255.0/(max-min)*min);
  cv::applyColorMap(colorMap, colorMap, 2);
  cv::imshow("imge", colorMap);
  //cv::imwrite("/home/hj/ICRA2020/190403/depth2csv/depth.png", colorMap);
  cv::waitKey(1);

  //std::cout<<"max: "<<max<<", min: "<<min<<std::endl;

/*________________________________________________________________________________________________________________________________________*/

//// Saving part
   if (saving_flag){

     if (m_sync_count % 3 == 0){
       cv::Mat saving_vel_mat(640, 480,CV_32FC3, cv::Scalar(0,0,0));
       cv::Mat saving_rp_mat(640, 480,CV_32FC3, cv::Scalar(0,0,0));

       index = std::to_string(save_count);

       std::string img_path =abs_path + "/img/" + data_name + index + ".png";
       //std::string depth_path =abs_path + "/depth/" + data_name + index + ".png";
       cv::imwrite(img_path, imgUndistorted);
       //cv::imwrite(depth_path, depthUndistorted);

       std::string rp_img2csv_path =abs_path + "/rp_img2csv/" + data_name + index + ".csv";
       std::string rp_raw_path =abs_path + "/rp_raw/" + data_name + index + ".csv";
       std::string rp_intensity_path =abs_path + "/rp_intensity/" + data_name + index + ".csv";
       //std::string depth_img2csv_path =abs_path + "/depth_img2csv/" + data_name + index + ".csv";
       std::string depth2csv_path =abs_path + "/depth2csv/" + data_name + index + ".csv";

       // std::cout<<"[Debug]: "<<img_path<<std::endl;
       // std::cout<<"[Debug]: "<<rp_img2csv_path<<std::endl;
       // std::cout<<"[Debug]: "<<rp_raw_path<<std::endl;
       // std::cout<<"[Debug]: "<<rp_intensity_path<<std::endl;

       std::ofstream rp_img2csv_output(rp_img2csv_path);
       std::ofstream rp_raw_output(rp_raw_path);
       std::ofstream rp_intensity_output(rp_intensity_path);
       //std::ofstream depth_img2csv_output(depth_img2csv_path);
       writeCSV(depth2csv_path, cv_ptr->image);

 /*________________________________________________________________________________________________________________________________________*/
       /// Saving point2img.
       /// Currently, we just need rp_img2csv!
       for (int y = 0; y < height; y++){
         for (int x = 0; x < width; x++){
           //// B, G, R in order
           /// / cvMat.at<uchar>(cv::Point(X,Y))
           rp_img2csv_output<<mat_rp_pc(x, y)<<",";
           cv::Scalar rp_rgb = get_colour(mat_rp_pc(x, y), 0.0, m_max_range);

           if(!(rp_rgb.val[0]==255 && rp_rgb.val[1]==255 && rp_rgb.val[2]==255))
           {
             if (mat_rp_pc(x, y)<=1.0){
               cv::circle(saving_rp_mat, cv::Point(x,y), 6, rp_rgb, -1);
             }else if (mat_rp_pc(x, y)<4.0 && mat_rp_pc(x, y)>1.0){
               cv::circle(saving_rp_mat, cv::Point(x,y), 3, rp_rgb, -1);
             }else if (mat_rp_pc(x, y)>=4.0) {
               cv::circle(saving_rp_mat, cv::Point(x,y), 1, rp_rgb, -1);
             }
           }
         }
         rp_img2csv_output<<std::endl;
       }
       rp_img2csv_output.close();

 /*________________________________________________________________________________________________________________________________________*/
       /// Saving depth_img2csv
       // for (int y = 0; y < height; y++){
       //   for (int x = 0; x < width; x++){
       //     //// B, G, R in order
       //     /// / cvMat.at<uchar>(cv::Point(X,Y))
       //     depth_img2csv_output<<mat_depth_pc(x, y)<<",";
       //   }
       //   depth_img2csv_output<<std::endl;
       // }
       // depth_img2csv_output.close();


/*________________________________________________________________________________________________________________________________________*/
       //saving raw data and intensity
       //Total size : RawLength
       int RawLength = eigenRaw.rows();

       //check starting point.
       //since original theta data starts from (-pi,-135 degree) and discontinuosly jump to (135degree, pi)
       //we should check where the data of leftmost pixel starts.
       //135degree is the leftmost pixel, so startpoint is when the theta first become positive.
       // int startpoint = 0;
       // for(int i = 0 ; i < RawLength; i++){
       //   if(eigenRaw(i,0) > 0){
       //     startpoint = i;
       //     break;
       //   }
       // }

       //save the data when theta is in (135 degree, pi)
       for(int i = 0 ; i < RawLength; i++){
         rp_raw_output<<eigenRaw(i,0)<<","<<eigenRaw(i,1)<<","<<std::endl;
         rp_intensity_output<<eigenRaw(i,2)<<","<<std::endl;
       }

       //save the data when theta is in (-pi,-135 degree)
       // for(int i = 0 ; i < startpoint; i++){
       //   rp_raw_output<<eigenRaw(i,0)<<","<<eigenRaw(i,1)<<","<<std::endl;
       //   rp_intensity_output<<eigenRaw(i,2)<<","<<std::endl;
       // }

       rp_raw_output.close();
       rp_intensity_output.close();

       // std::cout<<"Completed saving " <<index<<"th data"<<std::endl;
       save_count = save_count + 1;
     }
   }
   //cv::imshow("hi", img_for_depth);
   cv::imshow("rgb", img_for_rp);
   cv::waitKey(1);

   m_sync_count = m_sync_count + 1;
}
