#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"

namespace perception {
class Cropper {
 public:
  Cropper(const ros::Publisher& pub);
  sensor_msgs::PointCloud2 Callback(const sensor_msgs::PointCloud2& msg);
  void CloudCallback(const sensor_msgs::PointCloud2& msg);


 private:
  ros::Publisher pub_;
  tf::TransformListener tf_listener_;
};
}  // namespace perception
