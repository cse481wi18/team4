#include <vector> // lab34

#include "perception/crop.h"
#include "perception/downsample.h"
#include "perception/segmentation.h"
#include "visualization_msgs/Marker.h"

#include "perception/object_recognizer.h" // lab34
#include "perception_msgs/BallPositions.h"


#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;


  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);
  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &perception::Cropper::CloudCallback, &cropper);

  ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  ros::Publisher above_table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("above_table_cloud", 1, true);
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);

   ros::Publisher ball_poses_pub =
      nh.advertise<perception_msgs::BallPositions>("tennis_ball_position_topic", 100);

  // Create the object recognizer.
  std::vector<perception_msgs::ObjectFeatures> dataset;
//  perception::LoadData(data_dir, &dataset);
  perception::ObjectRecognizer recognizer(dataset);

  perception::Segmenter segmenter(table_pub, above_table_pub, marker_pub, ball_poses_pub,
                                  recognizer);
  ros::Subscriber segment_sub = nh.subscribe(
      "cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);

  ros::spin();
  return 0;

}
