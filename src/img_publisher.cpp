#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>


image_transport::Publisher pub_img;
ros::Publisher pub_imu;

void print_2d_vector(std::vector< std::vector <std::string> > & v)
{
  for (uint i = 0; i < v.size(); ++i)
  {
    for (uint j = 0; j < v[i].size(); ++j)
    {
      std::cout << v[i][j] << ' ';
    }
    std::cout << std::endl;
  }
}

void csv_load_to(std::vector< std::vector <std::string> > & content, const std::string & filename)
{
  std::fstream file (filename.data(), std::ios::in);
  std::vector<std::string> row;
  std::string line, word;

  if (file.is_open())
  {
    while(std::getline(file, line))
    {
      row.clear();
      std::stringstream sstream(line);
      if (line[0] == '#')
        continue;

      while(std::getline(sstream, word, ','))
      {

        char last_char = word[word.length() - 1];
        if (last_char == '\r' || last_char == '\n')
        {
          word.pop_back();
        }
        row.push_back(word);
      }
      content.push_back(row);
    }
  }
  else
  {
    LOG(FATAL) << "Could not open the image file.\n";
  }
}

void convert_topic(std::vector<ros::V_string>::iterator & iter)
{
  ros::V_string::iterator it;
  for(it = iter->begin(); it != iter->end(); ++it)
  {
    std::cout << (*it) << ' ';
  } std::cout << std::endl;
  ++iter;
}

enum msg_type {IMAGE, IMU, END};

struct Topic
{
  msg_type type;
  uint64_t timestamp;
  std::string img_filename;
  std::vector<double> * imu_data;
};

#define BASE 10
char *stopstring;

Topic next_topic(
  std::vector<ros::V_string>::iterator & row_img,
  std::vector<ros::V_string> & content_img,
  std::vector<ros::V_string>::iterator & row_imu,
  std::vector<ros::V_string> & content_imu)
{
  Topic next_msg;
  bool imu_end = row_imu == content_imu.end();
  bool img_end = row_img == content_img.end();

  if (img_end && imu_end)
    next_msg.type = END;
  else if (imu_end)
    next_msg.type = IMAGE;
  else if (img_end)
    next_msg.type = IMU;
  else
    next_msg.type = (row_img->at(0) <= row_imu->at(0)) ? IMAGE : IMU;

  if (next_msg.type == IMAGE)
  {
    next_msg.timestamp = strtoul(row_img->at(0).c_str(), &stopstring, BASE);
    next_msg.img_filename  = row_img->at(1);
    ++row_img;
  }
  else if (next_msg.type == IMU)
  {
    next_msg.timestamp = strtoul(row_imu->at(0).c_str(), &stopstring, BASE);
    next_msg.imu_data = new std::vector<double>();
    for (uint i = 1; i <= 6; ++i)
    {
      next_msg.imu_data->push_back(atof(row_imu->at(i).c_str()));
    }
    ++row_imu;
  }

  return next_msg;
}


void publish_imu(Topic & topic)
{
  sensor_msgs::Imu msg;
  ros::Time timestamp;
  timestamp.fromNSec(topic.timestamp);
  msg.header.stamp = timestamp;
  msg.angular_velocity.x = topic.imu_data->at(0);
  msg.angular_velocity.y = topic.imu_data->at(1);
  msg.angular_velocity.z = topic.imu_data->at(2);
  msg.linear_acceleration.x = topic.imu_data->at(3);
  msg.linear_acceleration.y = topic.imu_data->at(4);
  msg.linear_acceleration.z = topic.imu_data->at(5);
  pub_imu.publish(msg);
  // std::cout << msg.header.stamp.toNSec() << ","
  //   << msg.angular_velocity.x << ","
  //   << msg.angular_velocity.y << ","
  //   << msg.angular_velocity.z << ","
  //   << msg.linear_acceleration.x << ","
  //   << msg.linear_acceleration.y << ","
  //   << msg.linear_acceleration.z << std::endl;
}
//   for (uint i = 0; i < topic.imu_data->size(); ++i)
//   {
//     std::cout << topic.imu_data->at(i) << ' ';
//   }
//   std::cout << std::endl;
// }

void publish_img(Topic & topic)
{
  std::string img_dir = "/root/Data/EUROC/MH_01_easy/mav0/cam0/data/";
  std::string img_path = img_dir + topic.img_filename;
  // std::cout << topic.img_filename << std::endl;
  cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);


  std_msgs::Header header;
  ros::Time t;
  t.fromNSec(topic.timestamp);
  header.stamp = t;

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "8UC1", img).toImageMsg();
  pub_img.publish(msg);

  // cv::String windowName = topic.img_filename; //Name of the window
  // cv::namedWindow(windowName); // Create a window
  // cv::imshow(windowName, img); // Show our image inside the created window.
  // cv::waitKey(0); // Wait for any keystroke in the window
  // cv::destroyWindow(windowName); //destroy the created window
}


int main(int argc, char** argv)
{
  // Setting log level
  google::InitGoogleLogging(argv[0]);                // 구글 로그 초기화
  google::SetLogDestination( google::GLOG_INFO, "/root/log_img_publisher/test_" );
  FLAGS_stderrthreshold = 0;
  LOG(INFO) << "Info logging";

  // ROS Init
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  pub_img = it.advertise("/topic/cam0", 10);
  pub_imu = nh.advertise<sensor_msgs::Imu>("/topic/imu0", 1000);


  std::string img_filename = "/root/Data/EUROC/MH_01_easy/mav0/cam0/data.csv";
  std::string imu_filename = "/root/Data/EUROC/MH_01_easy/mav0/imu0/data.csv";

  std::vector< std::vector <std::string> > img_content;
  std::vector< std::vector <std::string> > imu_content;

  csv_load_to(img_content, img_filename);
  csv_load_to(imu_content, imu_filename);

  std::vector<ros::V_string>::iterator img_iter = img_content.begin();
  std::vector<ros::V_string>::iterator imu_iter = imu_content.begin();

  // Timestamp 순서대로 변환
  Topic topic;
  topic = next_topic(img_iter, img_content, imu_iter, imu_content);

  ros::Rate rate = ros::Rate(1000);
  uint64_t topic_tic = topic.timestamp;
  uint64_t topic_toc = topic.timestamp;
  uint64_t topic_start = topic.timestamp;

  uint64_t topic_dif =  topic_toc - topic_tic;
  uint64_t sys_tic = ros::Time::now().toNSec();
  uint64_t sys_toc = ros::Time::now().toNSec();
  uint64_t sys_dif = sys_toc - sys_tic;
  const uint64_t timescale = 1;

  while (ros::ok() && topic.type != END)
  {
    // Process
    // std::cout << double(topic.timestamp - topic_start)/1e9 << ' ';
    if (topic.type == IMAGE)
    {
      publish_img(topic);
    }
    else if (topic.type == IMU)
    {
      publish_imu(topic);
    }

    // Ticking
    do
    {
      rate.sleep();
      sys_toc = ros::Time::now().toNSec();
      sys_dif = sys_toc - sys_tic;
      sys_dif *= timescale;
      // std::cout << "topic_dif: " << topic_dif << ", sys_dif: " << sys_dif << std::endl;
    } while (topic_dif > sys_dif);

    // Picking
    sys_tic = ros::Time::now().toNSec();
    topic_tic = topic_toc;
    topic = next_topic(img_iter, img_content, imu_iter, imu_content);
    topic_toc = topic.timestamp;
    topic_dif = topic_toc - topic_tic;
  }


  ros::spin();
  // print_2d_vector(img_content);
  // print_2d_vector(imu_content);

  return 0;
}
