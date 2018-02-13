#include "perception/segmentation.h"
#include "pcl/common/common.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

#include "pcl/filters/extract_indices.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {

void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices) {
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);
  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
  axis << 0, 0, 1;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(10.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  pcl::ModelCoefficients coeff;
  seg.segment(indices_internal, coeff);

  *indices = indices_internal;

  if (indices->indices.size() == 0) {
    ROS_ERROR("Unable to find surface.");
    return;
  }
}
  /*
Notes from lab:
Implement GetAxisAlignedBoundingBox so that it sets the pose and dimensions of a box surrounding the given point cloud. Because we are computing an axis-aligned bounding box, the orientation of the box is just the identity orientation. Making a box that fits more tightly to the data is a refinement that requires extra work.

Implementation hints:

Use pcl::getMinMax3D as discussed in the previous lab.
The center X position is (max.x + min.x) / 2.
The X dimension is max.x - min.x.
You can't return multiple values from a C++ function, but you can pass in pointers and have the function mutate those pointers. Set pointer values like so: pose->orientation.w = 1.
   */

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
  PointC min_pcl;
  PointC max_pcl;
  pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);

  float center_x = (max_pcl.x + min_pcl.x) / 2;
  pose->position.x = center_x;
  pose->position.y = (max_pcl.y + min_pcl.y) / 2;
  pose->position.z =  (max_pcl.z + min_pcl.z) / 2;
  dimensions->x = max_pcl.x - min_pcl.x;
  dimensions->y = max_pcl.y - min_pcl.y;
  dimensions->z = max_pcl.z - min_pcl.z;
  pose->orientation.w = 1;

}

Segmenter::Segmenter(const ros::Publisher& surface_points_pub)
    : surface_points_pub_(surface_points_pub) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  // TODO - all the way until filter
  // Create the segmentation object
  //pcl::PointIndices indices;
  pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
  for (size_t i=0; i<indices->indices.size(); ++i) {
    int index = indices->indices[i];
    const PointC& pt = cloud->points[index];
  }

  // Given these data types:
  PointCloudC::Ptr original_cloud(new PointCloudC);
  PointCloudC::Ptr subset_cloud(new PointCloudC);

  // Extract subset of original_cloud into subset_cloud:
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(original_cloud);
  extract.setIndices(indices);
  extract.filter(*subset_cloud);


  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  SegmentSurface(cloud, table_inliers);
}
}  // namespace perception
