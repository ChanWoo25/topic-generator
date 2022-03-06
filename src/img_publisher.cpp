#include <ros/ros.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
0
void print_2d_vector(std::vector< std::vector <std::string> > & v)
{
  for (uint i = 0; i < v.size(); ++i)
  {
    for (uint j = 0; j < v[i].size(); ++j)
    {
      std::cout << v[i][j];
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

int main(int argc, char** argv)
{
  // Setting log level
  google::InitGoogleLogging(argv[0]);                // 구글 로그 초기화
  google::SetLogDestination( google::GLOG_INFO, "/root/log_img_publisher/test_" );
  FLAGS_stderrthreshold = 0;
  LOG(INFO) << "Info logging";

  std::string img_filename = "/root/Data/EUROC/MH_01_easy/mav0/cam0/data.csv";
  std::string imu_filename = "/root/Data/EUROC/MH_01_easy/mav0/imu0/data.csv";

  std::vector< std::vector <std::string> > img_content;
  std::vector< std::vector <std::string> > imu_content;

  csv_load_to(img_content, img_filename);
  csv_load_to(imu_content, imu_filename);

  print_2d_vector(img_content);
  print_2d_vector(imu_content);

  // ROS Init
  // ros::init(argc, argv, "image_subscriber");
  // ros::NodeHandle nh("~");
  // ros::Subscriber sub_img = nh.subscribe<sensor_msgs::Image>("/cam0/image_raw", 100, img_callback); //  [sensor_msgs/Image]
  // ros::spin();

  return 0;
}
