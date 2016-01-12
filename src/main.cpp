#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/io/pcd_io.h>

using namespace sensor_msgs;
using namespace message_filters;

typedef sensor_msgs::PointCloud2 PointCloudT;

namespace enc = sensor_msgs::image_encodings;

void callback(const PointCloudT::ConstPtr& cloud_dx,
              const PointCloudT::ConstPtr& cloud_sx,
              const PointCloudT::ConstPtr& clouds,
              const ImageConstPtr& image_dx,
              const ImageConstPtr& image_sx)
{
  //std::cout << "Approximate Callback!" << std::endl;
    cv_bridge::CvImageConstPtr cv_ptr1 = cv_bridge::toCvShare(image_dx, enc::BGR8);
    cv_bridge::CvImageConstPtr cv_ptr2 = cv_bridge::toCvShare(image_sx, enc::BGR8);


    std::cout << "Caught synchronized callback, saving to file ...";

    std::stringstream ss;
    ss << "RGB_DX_" << image_dx->header.stamp << ".png";
    cv::imwrite(ss.str(), cv_ptr1->image);
    ss.str(std::string());
    ss << "RGB_SX_" << image_dx->header.stamp << ".png";
    cv::imwrite(ss.str(), cv_ptr2->image);
    ss.str(std::string());
    ss << "CLOUD_DX_" << image_dx->header.stamp << ".pcd";
    pcl::io::savePCDFile(ss.str(),*cloud_dx);
    ss.str(std::string());
    ss << "CLOUD_SX_" << image_dx->header.stamp << ".pcd";
    pcl::io::savePCDFile(ss.str(),*cloud_sx);
    ss.str(std::string());
    ss << "CLOUD_ZZ_" << image_dx->header.stamp << ".pcd";
    pcl::io::savePCDFile(ss.str(),*clouds);

    std::cout << "Done" << std::endl;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<PointCloudT> cloud1_sub(nh, "/cloud_dx", 1);
  message_filters::Subscriber<PointCloudT> cloud2_sub(nh, "/cloud_sx", 1);
  message_filters::Subscriber<PointCloudT> cloud3_sub(nh, "/clouds", 1);
  message_filters::Subscriber<Image> image1_sub(nh, "/Kinect_right/rgb/image_color", 1);
  message_filters::Subscriber<Image> image2_sub(nh, "/Kinect_left/rgb/image_color", 1);

  typedef sync_policies::ApproximateTime<PointCloudT, PointCloudT, PointCloudT, Image, Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), cloud1_sub, cloud2_sub, cloud3_sub, image1_sub, image2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

  ros::spin();

  return 0;
}
