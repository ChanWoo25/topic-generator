#include <ros/ros.h>
#include <string>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>

// int main(int argc, char** argv)
// {


//   image_transport::ImageTransport it(nh);
//   image_transport::Publisher pub = it.advertise("camera/image", 1);
//   cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
//   cv::waitKey(30);
//   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

//   ros::Rate loop_rate(5);
//   while (nh.ok()) {
//     pub.publish(msg);
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
// }

void img_callback(const sensor_msgs::ImageConstPtr & msg)
{
  // LOG(INFO) << msg;

  std_msgs::Header stamp = msg->header;

  cv_bridge::CvImageConstPtr ptr;
  if (msg->encoding == "8UC1")
  {
    std::cout << "Encoding == 8UC1\n";
    sensor_msgs::Image img;
    img.header = msg->header;
    img.height = msg->height;
    img.width = msg->width;
    img.is_bigendian = msg->is_bigendian;
    img.step = msg->step;
    img.data = msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  }
  else
  {
    LOG(INFO) << "image's encode: " << msg->encoding;
    ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

  }

  cv::Mat show_img = ptr->image;
  std::string filename = "./image/img_" + std::to_string(msg->header.stamp.toSec()) + ".png" ;
  std::cout << "Write to " << filename << std::endl;


  imwrite(filename, show_img);
}


int main(int argc, char** argv)
{
  // Setting log level
  google::InitGoogleLogging(argv[0]);                // 구글 로그 초기화
  google::SetLogDestination( google::GLOG_INFO, "./test_img" );
  FLAGS_stderrthreshold = 0;

  LOG(INFO) << "Info logging";
  // LOG (WARNING) << "Warn logging";
  // LOG (ERROR) << "Warn logging";

  // ROS Init
  ros::init(argc, argv, "image_subscriber");
  ros::NodeHandle nh("~");
  ros::Subscriber sub_img = nh.subscribe<sensor_msgs::Image>("/cam0/image_raw", 100, img_callback); //  [sensor_msgs/Image]
  ros::spin();

  return 0;
}
