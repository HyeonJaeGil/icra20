#include "data_synchronizer.h"

//void dataSynchronizer::synchronize_data(){ }

dataSynchronizer::dataSynchronizer()
{
  m_width = 640;
  m_height = 480;
  m_focal_length = 609.89844; //f = (fx + fy)/2

  m_x_center = 316.0020446777344;
  m_y_center = 250.0917205810547;
  m_max_range = 10;
  m_sync_count = 0;
  static ros::Subscriber sub0 = m_nh.subscribe<sensor_msgs::PointCloud2>("/converted_pc", 1, &dataSynchronizer::callback_2dLidar, this);
  static ros::Subscriber sub1 = m_nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &dataSynchronizer::callback_Raw, this);
  static ros::Subscriber sub2 = m_nh.subscribe<sensor_msgs::CompressedImage>("/camera/color/image_raw/compressed", 1, &dataSynchronizer::callback_image, this);
  static ros::Subscriber sub3 = m_nh.subscribe<sensor_msgs::Image>("/depth", 1, &dataSynchronizer::callback_depth, this);
  //static ros::Subscriber sub4 = m_nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, &dataSynchronizer::callback_pcDepth, this);
  // Subscriber for synced node!
  //timer = m_nh.createTimer(ros::Duration(0.1), boost::bind(&dataSynchronizer::synchronize_data, this));
  m_pub_node = m_nh.advertise<icra20::synced_node>("/Synced_node", 5);

}

dataSynchronizer::~dataSynchronizer()
{
}

void dataSynchronizer::callback_2dLidar(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
//  std::cout<<std::setprecision(13)<<"2D"<<msg->header.stamp.toSec()<<std::endl;
  m_rp_pc.push_back(*msg);

  icra20::synced_node nodeOut;
  if (update(nodeOut)){
    m_pub_node.publish(nodeOut);
    std::cout<<"Publish complete!"<<m_sync_count<<std::endl;
    //std::cout<<"Len of img: "<<m_image.size()<<", Len of Raw: "<<m_rp.size()<<", Len of RP_LiDAR: "<<m_rp_pc.size()<<std::endl;

    m_sync_count = m_sync_count + 1;
  }
}

void dataSynchronizer::callback_Raw(const sensor_msgs::LaserScan::ConstPtr &msg)
{
//  std::cout<<std::setprecision(13)<<"3D"<<msg->header.stamp.toSec()<<std::endl;

  m_rp.push_back(*msg);

  icra20::synced_node nodeOut;
  if (update(nodeOut)){
    m_pub_node.publish(nodeOut);
    std::cout<<"Publish complete!"<<m_sync_count<<std::endl;
    //std::cout<<"Len of img: "<<m_image.size()<<", Len of Raw: "<<m_rp.size()<<", Len of RP_LiDAR: "<<m_rp_pc.size()<<std::endl;

    m_sync_count = m_sync_count + 1;
  }

}

void dataSynchronizer::callback_image(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
  m_image.push_back(*msg);

  icra20::synced_node nodeOut;
  if (update(nodeOut)){
    m_pub_node.publish(nodeOut);
    std::cout<<"Publish complete!"<<m_sync_count<<std::endl;
    //std::cout<<"Len of img: "<<m_image.size()<<", Len of Raw: "<<m_rp.size()<<", Len of RP_LiDAR: "<<m_rp_pc.size()<<std::endl;

    m_sync_count = m_sync_count + 1;
  }

}

void dataSynchronizer::callback_depth(const sensor_msgs::Image::ConstPtr &msg)
{
  m_depth.push_back(*msg);

  icra20::synced_node nodeOut;
  if (update(nodeOut)){
    m_pub_node.publish(nodeOut);
    std::cout<<"Publish complete!"<<m_sync_count<<std::endl;
    //std::cout<<"Len of img: "<<m_image.size()<<", Len of Raw: "<<m_rp.size()<<", Len of RP_LiDAR: "<<m_rp_pc.size()<<std::endl;

    m_sync_count = m_sync_count + 1;
  }

}

// void dataSynchronizer::callback_pcDepth(const sensor_msgs::PointCloud2::ConstPtr &msg)
// {
// //  std::cout<<std::setprecision(13)<<"2D"<<msg->header.stamp.toSec()<<std::endl;
//   m_depth_pc.push_back(*msg);
//
//   icra20::synced_node nodeOut;
//   if (update(nodeOut)){
//     m_pub_node.publish(nodeOut);
//     std::cout<<"Publish complete!"<<m_sync_count<<std::endl;
//     //std::cout<<"Len of img: "<<m_image.size()<<", Len of Raw: "<<m_rp.size()<<", Len of RP_LiDAR: "<<m_rp_pc.size()<<std::endl;
//
//     m_sync_count = m_sync_count + 1;
//   }
// }


bool dataSynchronizer::update(icra20::synced_node& node)
{
//  double time_offset = 2.0;
//  double time_offset = 1331888.78;
  double time_offset = 0;
  double threshold = 0.02;

  while (!(m_image.empty() || m_rp.empty() || m_rp_pc.empty() || m_depth.empty() ) )
  {
    std::cout<<"Len of img: "<<m_image.size()<<", Len of Raw: "<<m_rp.size()<<", Len of RP_LiDAR: "<<m_rp_pc.size()
    <<" Len of depth: "<<m_depth.size()<<std::endl;
    //std::cout<<"Timestamp of img: "<<m_image.front().header.stamp.toNsecs()<<", Raw: "<<m_rp.front().header.stamp.toNsecs()<<", RP_LiDAR: "<<m_rp_pc.front().header.stamp.toNsecs()<<std::endl;
    std::vector<double> times = {m_image.front().header.stamp.toSec()+time_offset,
                                 m_depth.front().header.stamp.toSec()+time_offset,
                                 m_rp.front().header.stamp.toSec(),
                                 m_rp_pc.front().header.stamp.toSec(),
                                };
    auto idxMM = std::minmax_element(times.begin(),times.end());
//    std::cout<< std::setprecision(13) << "TIMES : "<<times[0]<<","<<times[1]<<","<<times[2]<<std::endl;
    if(*idxMM.second - *idxMM.first > threshold)
    {
      if(idxMM.first-times.begin()==0){
        if (m_image.front().header.stamp.toSec() == m_depth.front().header.stamp.toSec()) m_depth.erase(m_depth.begin());
        // if (m_image.front().header.stamp.toSec() == m_depth_pc.front().header.stamp.toSec()) m_depth_pc.erase(m_depth_pc.begin());
        m_image.erase(m_image.begin());
      }
      else if(idxMM.first-times.begin()==1){
        if (m_depth.front().header.stamp.toSec() == m_image.front().header.stamp.toSec()) m_image.erase(m_image.begin());
        // if (m_depth.front().header.stamp.toSec() == m_depth_pc.front().header.stamp.toSec()) m_depth_pc.erase(m_depth_pc.begin());
        m_depth.erase(m_depth.begin());
      }
      else if(idxMM.first-times.begin()==2) m_rp.erase(m_rp.begin());
      else if(idxMM.first-times.begin()==3) m_rp_pc.erase(m_rp_pc.begin());
    }
    else
    {
      std::cout<<"Expecting publishing..."<<std::endl;
      node.Raw = m_rp.front();
      node.rp_lidar = m_rp_pc.front();
      node.image = m_image.front();
      node.depth = m_depth.front();
      // node.depth_pc = m_depth_pc.front();
      node.idx = m_sync_count;
      m_rp.erase(m_rp.begin());
      m_rp_pc.erase(m_rp_pc.begin());
      m_image.erase(m_image.begin());
      m_depth.erase(m_depth.begin());
      // m_depth_pc.erase(m_depth_pc.begin());
      return true;
    }
  return false;
  }

}
