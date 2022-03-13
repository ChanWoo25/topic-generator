#include <ros/ros.h>
#include <string>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <sensor_msgs/Imu.h>

int img_cnt = 0;
int imu_cnt = 0;

void img_callback(const sensor_msgs::ImageConstPtr & msg)
{
  // LOG(INFO) << msg;

  std_msgs::Header stamp = msg->header;

  cv_bridge::CvImageConstPtr ptr;
  if (msg->encoding == "8UC1")
  {
    // std::cout << "Encoding == 8UC1\n";
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

  std::cout << "img: " << ++img_cnt << std::endl;
  // cv::Mat show_img = ptr->image;
  // std::string filename = "./image/img_" + std::to_string(msg->header.stamp.toSec()) + ".png" ;
  // imwrite(filename, show_img);

}

void imu_callback(const sensor_msgs::ImuConstPtr & msg)
{
  // std::cout << msg->header.stamp.toNSec() << ","
  //   << msg->angular_velocity.x << ","
  //   << msg->angular_velocity.y << ","
  //   << msg->angular_velocity.z << ","
  //   << msg->linear_acceleration.x << ","
  //   << msg->linear_acceleration.y << ","
  //   << msg->linear_acceleration.z << std::endl;
  std::cout << "imu: " << ++imu_cnt << std::endl;
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
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // ros::Subscriber sub_img = nh.subscribe<sensor_msgs::Image>("/cam0/image_raw", 1000, img_callback);
  // ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu0", 1000, imu_callback);
  ros::Subscriber sub_img = nh.subscribe<sensor_msgs::Image>("/topic/cam0", 1000, img_callback);
  ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/topic/imu0", 1000, imu_callback);
  ros::spin();

  return 0;
}
