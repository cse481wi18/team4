#include <vector> // lab34

#include "perception/crop.h"
#include "perception/downsample.h"
#include "perception/segmentation.h"
#include "visualization_msgs/Marker.h"

#include "perception/object_recognizer.h" // lab34
#include "perception_msgs/BallPositions.h"


#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"


void GetBallLocationCallback(const sensor_msgs::PointCloud2& msg) {
    // TODO call cropper/seg
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;

  if (argc < 2) {
    ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
    ros::spinOnce();
  }

  ros::Publisher crop_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub);
//  ros::Subscriber sub =
//      nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);

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
      "cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);

    // adding a get_ball_locations_callback to Segmenter just because it's easier to build/add
    // takes: sensor_msgs::PointCloud2& msg
  ros::Subscriber segmenter_wrapper = nh.subscribe("get_ball_locations", 1, &perception::Segmenter::GetBallLocationCallback, &segmenter)
  ros::Subscriber segmenter_wrapper = handle.subscribe("get_ball_locations", 1, GetBallLocationCallback);

  while(true) {
    boost::shared_ptr<Output message> ball_location_message = ros::topic::waitForMessage<MyOutputMessage>("get_ball_locations", ros::Duration(2));
    if (ball_location_message != NULL) {
        ROS_INFO("Got a request for a ball location!");
        // get next message from input camera (cloud_in) and pass it to Cropper callback
        boost::shared_ptr<sensor_msgs::PointCloud2> cloud_msg = ros::topic::waitForMessage<MyOutputMessage>("cloud_in", ros::Duration(3));
        cropper.Callback(*cloud_msg);
       // perceptor will be listening for fewer messages and automatically pick up the cropped cloud msg
    }
  }

  ros::spin();
  return 0;

}
