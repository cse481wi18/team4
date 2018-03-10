#include <vector> // lab34

#include "perception/crop.h"
#include "perception/downsample.h"
#include "perception/segmentation.h"
#include "visualization_msgs/Marker.h"


#include "perception/object_recognizer.h" // lab34
#include "perception_msgs/BallPositions.h"
#include "geometry_msgs/Pose.h"


#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"


#define CROPPED_CLOUD_TOPIC "cropped_cloud"

int main(int argc, char** argv) {

  ros::init(argc, argv, "tennis_ball_finder");
  ros::NodeHandle nh;

  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>(CROPPED_CLOUD_TOPIC, 1, true);
  perception::Cropper cropper(crop_pub);

  ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  ros::Publisher above_table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("above_table_cloud", 1, true);
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 100);

   ros::Publisher ball_poses_pub =
      nh.advertise<perception_msgs::BallPositions>("tennis_ball_position_topic", 100);

  // Recognizer doesn't do anything - just passed in
  std::vector<perception_msgs::ObjectFeatures> dataset;
  perception::ObjectRecognizer recognizer(dataset);

  perception::Segmenter segmenter(table_pub, above_table_pub, marker_pub, ball_poses_pub,
                                  recognizer);
  ros::Subscriber segment_sub = nh.subscribe(
      CROPPED_CLOUD_TOPIC, 1, &perception::Segmenter::Callback, &segmenter);

//  ros::Subscriber segmenter_wrapper = handle.subscribe("get_ball_locations", 1, GetBallLocationCallback);

  boost::shared_ptr<const geometry_msgs::Pose> ball_location_message;
  boost::shared_ptr<const sensor_msgs::PointCloud2> cloud_msg;
  while(true) {
  // this should probably just be a subscriver but didnt feel like making all the subsequent subscribers global
    ball_location_message = ros::topic::waitForMessage<geometry_msgs::Pose>("get_ball_locations"); // block until receive
    if (ball_location_message != NULL) {
        ROS_INFO("[tennis_ball_finder.cpp] Got a request for a ball location!");
        // get next message from input camera (cloud_in) and pass it to Cropper callback
        cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_in", ros::Duration(3));
        if (cloud_msg != NULL) {
            ROS_INFO("[tennis_ball_finder.cpp] Got the cloud_msg; calling cropper");
            // for some reason perc
            sensor_msgs::PointCloud2 cropped_msg = cropper.Callback(*cloud_msg);
            segmenter.Callback(cropped_msg);
        } else {
            ROS_INFO("Error - didn't get an input msg from camera!");
        }
        // set reset pointers just in case (boost pointers dont let you just set them to null)
        ball_location_message.reset();
        cloud_msg.reset();
    }
  }

  ros::spin();
  return 0;

}
