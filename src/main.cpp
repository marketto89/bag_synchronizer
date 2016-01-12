#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>

using namespace sensor_msgs;
using namespace message_filters;

typedef sensor_msgs::PointCloud2 PointCloudT;

void callback(const PointCloudT::ConstPtr& cloud1,
              const PointCloudT::ConstPtr& cloud2,
              const PointCloudT::ConstPtr& clouds,
              const ImageConstPtr& image )
{
  std::cout << "Approximate Callback!" << std::endl;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<PointCloudT> cloud1_sub(nh, "/cloud1", 1);
  message_filters::Subscriber<PointCloudT> cloud2_sub(nh, "/cloud2", 1);
  message_filters::Subscriber<PointCloudT> cloud3_sub(nh, "/clouds", 1);
  message_filters::Subscriber<Image> image_sub(nh, "/Kinect_left/rgb/image_color", 1);

  typedef sync_policies::ApproximateTime<PointCloudT, PointCloudT, PointCloudT, Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), cloud1_sub, cloud2_sub, cloud3_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::spin();

  return 0;
}
