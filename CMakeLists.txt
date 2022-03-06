cmake_minimum_required(VERSION 3.0.2)
project(topic_test)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

### Ros Packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge)
include_directories(${catkin_INCLUDE_DIRS})
message(STATUS "catkin include -- ${catkin_INCLUDE_DIRS}")
###

### OPENCV
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Version -- ${OpenCV_VERSION}")
message(STATUS "OpenCV include -- ${OpenCV_INCLUDE_DIRS}")
# message(STATUS "OpenCV libraries -- ${OpenCV_LIBS}") # Too Long to show all opencv libs
include_directories(${OpenCV_INCLUDE_DIRS})
###

### Eigen3
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version -- ${EIGEN3_VERSION_STRING}")
message(STATUS "Eigen include -- ${EIGEN3_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})
###

### Glog
find_package (glog REQUIRED)
message(STATUS "Glog Version -- ${GLOG_VERSION}")
message(STATUS "Glog include -- ${GLOG_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})
# https://ubuntu.pkgs.org/18.04/ubuntu-universe-amd64/libgoogle-glog-dev_0.3.5-1_amd64.deb.html
###

catkin_package()

add_executable(img_subscriber
  src/img_subscriber.cpp
)
target_link_libraries(img_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS} glog::glog)

add_executable(img_publisher
  src/img_publisher.cpp
)
target_link_libraries(img_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS} glog::glog)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
