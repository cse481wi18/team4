#include "crop.h"

Cropper::Cropper() {}

void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  ROS_INFO("Got point cloud");
}