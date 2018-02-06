#include <iostream>
#include <string>

#include "ros/ros.h"

void print_usage() { 
  std::cout << "Saves a point cloud on head_camera/depth_registered/points to "
               "NAME.bag in the current directory."
            << std::endl;
  std::cout << "Usage: rosrun perception save_cloud NAME" << std::endl;
}

int main(int argc, char** argv) { 
  ros::init(argc, argv, "save_cloud_main");
  if (argc < 2) { 
    print_usage();
    return 1;
  } 
  std::string name(argv[1]);
  std::cout << "Hello, " << name << std::endl;

  return 0;
}
