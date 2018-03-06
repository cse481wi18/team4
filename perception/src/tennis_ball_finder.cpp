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
//    if (argc >= 2) {
//
//      ROS_INFO("Usage: rosrun perception tennis_ball_finder DATA_DIR"); // TODO milestone 1 read from curr cam
//      ROS_INFO("Reading from hallucinated data"); // TODO milestone 1 read from curr cam
//
//    }
////std::string data_dir(argv[1]);
//
//  ros::init(argc, argv, "tennis_ball_finder");
//  ros::NodeHandle nh;
//
////  // Cropper
//
//  ros::Publisher crop_pub =
//      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
//  //perception::Cropper cropper;
//  perception::Cropper cropper(crop_pub);
//  ros::Subscriber crop_sub =
//      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
//
//// Downsampler
//  ros::Publisher d_pub =
//      nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
//  perception::Downsampler downsampler(d_pub);
//  ros::Subscriber d_sub =
//      nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);
//
//// Segmenter
//  ros::Publisher table_pub =
//      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
//
//  ros::Publisher above_surface_pub =
//      nh.advertise<sensor_msgs::PointCloud2>("above_surface_cloud", 1, true);
//
//  ros::Publisher marker_pub =
//      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
//
//
////     std::vector<std::string> topics;
////      topics.push_back("tennis_ball_position_topic");
////      rosbag::View view(bag, rosbag::TopicQuery(topics));
////
////  ros::Publisher tennis_ball_pub =
////      nh.advertise<perception_msgs::BallPositions>("tennis_ball_position_topic", 1, true);
//
//  // Create the object recognizer.
//  std::vector<perception_msgs::ObjectFeatures> dataset;
////  perception::LoadData(data_dir, &dataset);
//  perception::ObjectRecognizer recognizer(dataset);
//
//
//
//  perception::Segmenter segmenter(table_pub, marker_pub, above_surface_pub, recognizer);
////  perception::Segmenter segmenter(table_pub, marker_pub, above_surface_pub, tennis_ball_pub, recognizer);
//
//
//
//
//   ros::Subscriber sub =
//      nh.subscribe("cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);
//  ros::spin();
//  return 0;

  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;

  if (argc < 2) {
    ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
    ros::spinOnce();
  }
  std::string data_dir("temp dir");

  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);
  ros::Subscriber sub =
      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);

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
