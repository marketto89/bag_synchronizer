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
typedef sensor_msgs::Image Image;

std::string _prefix;

namespace enc = sensor_msgs::image_encodings;

void
saveDepth(const Image::ConstPtr& msg,
          const ros::Time& time,
          const std::string radix)
{
    cv::Mat _depth_image;
    cv_bridge::CvImage::Ptr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg); //, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //    // Save depth images as PGM files with distances in millimiters:
    _depth_image = cv_ptr->image;
    _depth_image.convertTo(_depth_image, CV_32FC1);
    //		cv::imshow(DEPTH_WINDOW, 0.1*_depth_image);

    _depth_image = 1000 * _depth_image;
    std::stringstream filename;
    //filename << rec_folder << depthTimestamp.sec << "." << depthTimestamp.nsec << ".pgm";
    filename << _prefix << "/DEPTH_" << radix << "_" << time << ".png";
    cv::imwrite(filename.str(),_depth_image);
//    std::ofstream image(filename.str().c_str());
//    image << "P2" << std::endl;
//    image << "# " << time << ".pgm" << std::endl;
//    image << _depth_image.cols << " " << _depth_image.rows << std::endl;
//    image << 65535 << std::endl;		// max value allowed for depth values (in mm)
//    for (int i=0;i<_depth_image.rows;i=i+1)
//    {
//        for (int j=0;j<_depth_image.cols;j=j+1)
//        {
//            float depthValue = _depth_image.at<float>(i,j);
//            if (depthValue != depthValue)
//            {
//                image << "0" << " ";
//            }
//            else
//            {
//                image << depthValue << " ";
//            }
//        }
//        image << std::endl;
//    }
//    image.close();
}

void
callback(const PointCloudT::ConstPtr& cloud_right,
         const PointCloudT::ConstPtr& cloud_left,
         const PointCloudT::ConstPtr& clouds,
         const Image::ConstPtr& depth_image_right,
         const Image::ConstPtr& depth_image_left)
{
    //    //std::cout << "Approximate Callback!" << std::endl;
    //    cv_bridge::CvImageConstPtr cv_ptr1 = cv_bridge::toCvShare(image_dx,
    //                                                              enc::BGR8);
    //    cv_bridge::CvImageConstPtr cv_ptr2 = cv_bridge::toCvShare(image_sx,
    //                                                              enc::BGR8);


    std::cout << ros::Time::now() <<
                 "Caught synchronized callback, saving to file ..." <<
                 std::flush;

    std::stringstream ss;
    //    ss << _prefix << "/RGB_DX_" << image_dx->header.stamp << ".png";
    //    cv::imwrite(ss.str(), cv_ptr1->image);
    //    ss.str(std::string());
    //    ss << _prefix << "/RGB_SX_" << image_dx->header.stamp << ".png";
    //    cv::imwrite(ss.str(), cv_ptr2->image);
    //    ss.str(std::string());
    ss << _prefix << "/CLOUD_DX_" << cloud_right->header.stamp << ".pcd";
    pcl::io::savePCDFile(ss.str(),*cloud_right);
    ss.str(std::string());
    ss << _prefix << "/CLOUD_SX_" << cloud_right->header.stamp << ".pcd";
    pcl::io::savePCDFile(ss.str(),*cloud_left);
    ss.str(std::string());
    ss << _prefix << "/CLOUD_ZZ_" << cloud_right->header.stamp << ".pcd";
    pcl::io::savePCDFile(ss.str(),*clouds);
    saveDepth(depth_image_left, cloud_right->header.stamp, "LEFT");
    saveDepth(depth_image_right, cloud_right->header.stamp, "RIGHT");

    std::cout << "Done" << std::endl;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "bag_synchronizer");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    if(!pnh.getParam("prefix",_prefix))
        ROS_WARN_STREAM("prefix not set!");

    message_filters::Subscriber<PointCloudT> cloud_right_sub(nh, "/cloud_right", 1);
    message_filters::Subscriber<PointCloudT> cloud_left_sub(nh, "/cloud_left", 1);
    message_filters::Subscriber<PointCloudT> clouds_sub(nh, "/clouds", 1);
    message_filters::Subscriber<Image> depth_right_sub(nh, "/Kinect_right/depth_registered/image", 1);
    message_filters::Subscriber<Image> depth_left_sub(nh, "/Kinect_left/depth_registered/image", 1);
            //    message_filters::Subscriber<Image> image1_sub(nh,
            //                                                  "/Kinect_right/rgb/image_rect_color",
            //                                                  1);
            //    message_filters::Subscriber<Image> image2_sub(nh,
            //                                                  "/Kinect_left/rgb/image_rect_color",
            //                                                  1);

            typedef sync_policies::ApproximateTime<PointCloudT, PointCloudT, PointCloudT,
            Image, Image>
            MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument,
    // hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_right_sub,
                                    cloud_left_sub, clouds_sub,
                                    depth_right_sub, depth_left_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

    ros::spin();

    return 0;
}
