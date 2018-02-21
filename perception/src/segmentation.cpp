#include "perception/segmentation.h"
#include "perception/box_fitter.h"
#include "perception/object.h"

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
#include "pcl/segmentation/extract_clusters.h"
#include "visualization_msgs/Marker.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"


#include <math.h>
#include <sstream>
#include "perception/object_recognizer.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
void SegmentSurface(PointCloudC::Ptr cloud,
					pcl::PointIndices::Ptr indices,
					pcl::ModelCoefficients::Ptr coeff) {
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
	double table_slant_tolerance;
	ros::param::param("table_slant_tolerance", table_slant_tolerance, 10.0);
	seg.setEpsAngle(pcl::deg2rad(table_slant_tolerance));

	// coeff contains the coefficients of the plane:
	// ax + by + cz + d = 0
	ROS_INFO("before segment");
	seg.segment(indices_internal, *coeff);
	ROS_INFO("after segment");

	// get rid of things on top of the table
	double distance_above_plane;
	ros::param::param("distance_above_plane", distance_above_plane, 0.005);

	// Build custom indices that ignores points above the plane.
	for (size_t i = 0; i < cloud->size(); ++i) {
	  const PointC& pt = cloud->points[i];
	  float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
	              coeff->values[2] * pt.z + coeff->values[3];
	  if (val <= distance_above_plane) {
	    indices->indices.push_back(i);
	  }
	}

	if (indices->indices.size() == 0) {
	  ROS_ERROR("Unable to find surface.");
	  return;
	}
}

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
	PointC min_pcl;
	PointC max_pcl;

	pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);

	pose->position.x = (min_pcl.x + max_pcl.x) / 2;
	pose->position.y = (min_pcl.y + max_pcl.y) / 2;
	pose->position.z = (min_pcl.z + max_pcl.z) / 2;

	dimensions->x = (max_pcl.x - min_pcl.x);
	dimensions->y = (max_pcl.y - min_pcl.y);
	dimensions->z = (max_pcl.z - min_pcl.z);
}

void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices) {
	pcl::ExtractIndices<PointC> extract;
	pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
	extract.setInputCloud(cloud);
	extract.setIndices(surface_indices);
	extract.setNegative(true);
	extract.filter(above_surface_indices->indices);

	ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

	double cluster_tolerance;
	int min_cluster_size, max_cluster_size;
	ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
	ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
	ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

	pcl::EuclideanClusterExtraction<PointC> euclid;
	euclid.setInputCloud(cloud);
	euclid.setIndices(above_surface_indices);
	euclid.setClusterTolerance(cluster_tolerance);
	euclid.setMinClusterSize(min_cluster_size);
	euclid.setMaxClusterSize(max_cluster_size);
	euclid.extract(*object_indices);

	// Find the size of the smallest and the largest object,
	// where size = number of points in the cluster
	size_t min_size = std::numeric_limits<size_t>::max();
	size_t max_size = std::numeric_limits<size_t>::min();
	for (size_t i = 0; i < object_indices->size(); ++i) {
	  // TODO: implement this
	  size_t cluster_size = object_indices->at(i).indices.size();
	  if (cluster_size < min_size) {
	  	min_size = cluster_size;
	  }
	  if (cluster_size > max_size) {
	  	max_size = cluster_size;
	  }
	}

	ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
	         object_indices->size(), min_size, max_size);
}

void FindObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          std::vector<Object>* objects) {
	pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
	SegmentSurface(cloud, table_inliers, coeff);

	PointCloudC::Ptr table_cloud(new PointCloudC());

	// Extract subset of original_cloud into table_cloud:
	pcl::ExtractIndices<PointC> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(table_inliers);
	extract.filter(*table_cloud);

	// segmenting surface objects
	std::vector<pcl::PointIndices> object_indices;
	SegmentSurfaceObjects(cloud, table_inliers, &object_indices);


        // find above surface cloud
	PointCloudC::Ptr above_surface_cloud(new PointCloudC);
	extract.setNegative(true);

	extract.filter(*above_surface_cloud);

	for (size_t i = 0; i < object_indices.size(); ++i) {
	  // Reify indices into a point cloud of the object.
	  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	  *indices = object_indices[i];
	  PointCloudC::Ptr object_cloud(new PointCloudC());
	  // TODO: fill in object_cloud using indices
	  extract.setIndices(indices);
	  extract.setNegative(false);
	  extract.filter(*object_cloud);
	  geometry_msgs::Pose object_pose;

      shape_msgs::SolidPrimitive shape;
      FitBox(*object_cloud, coeff, *above_surface_cloud, shape, object_pose);

      Object o;
      o.name = "o";
      o.confidence = 0.0;
      o.cloud = object_cloud;
      o.pose = object_pose;
      if (shape.type == shape_msgs::SolidPrimitive::BOX) {
        o.dimensions.x = shape.dimensions[0];
        o.dimensions.y = shape.dimensions[1];
        o.dimensions.z = shape.dimensions[2];
	  } else {
	    std::cout << "error on BOX" ;
	  }
	  objects->push_back(o);
    }
}

void SegmentTabletopScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          std::vector<Object>* objects,
                          ros::Publisher surface_points_pub_,
                          ros::Publisher marker_pub_,
                          ros::Publisher above_surface_pub_) {

	pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
	SegmentSurface(cloud, table_inliers, coeff);

	PointCloudC::Ptr table_cloud(new PointCloudC());

	// Extract subset of original_cloud into table_cloud:
	pcl::ExtractIndices<PointC> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(table_inliers);
	extract.filter(*table_cloud);

	sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*table_cloud, msg_out);
    surface_points_pub_.publish(msg_out);

    // publish table bounding box
    visualization_msgs::Marker table_marker;
	table_marker.ns = "table";
	table_marker.header.frame_id = "base_link";
	table_marker.type = visualization_msgs::Marker::CUBE;
	// GetAxisAlignedBoundingBox(table_cloud, &table_marker.pose, &table_marker.scale);

	//Use simple_grasping instead of our own function
	PointCloudC::Ptr extract_out(new PointCloudC());
	shape_msgs::SolidPrimitive shape;
	geometry_msgs::Pose table_pose;
//	 simple_grasping::extractShape(*table_cloud, coeff, *extract_out, shape, table_pose);
	FitBox(*table_cloud, coeff, *extract_out, shape, table_pose);

	// segmenting surface objects
	std::vector<pcl::PointIndices> object_indices;
	SegmentSurfaceObjects(cloud, table_inliers, &object_indices);

    // find above surface cloud
	PointCloudC::Ptr above_surface_cloud(new PointCloudC);
	extract.setNegative(true);
	extract.filter(*above_surface_cloud);
	// publish the above surface cloud
	pcl::toROSMsg(*above_surface_cloud, msg_out);
	above_surface_pub_.publish(msg_out);


	// bounding box for objects
	for (size_t i = 0; i < object_indices.size(); ++i) {
	  // Reify indices into a point cloud of the object.
	  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	  *indices = object_indices[i];
	  PointCloudC::Ptr object_cloud(new PointCloudC());
	  // TODO: fill in object_cloud using indices
	  extract.setIndices(indices);
	  extract.setNegative(false);
	  extract.filter(*object_cloud);
	  geometry_msgs::Pose object_pose;
      FitBox(*object_cloud, coeff, *above_surface_cloud, shape, object_pose);

      Object o;
      o.name = "o";
      o.confidence = 0.0;
      o.cloud = object_cloud;
      o.pose = object_pose;
      if (shape.type == shape_msgs::SolidPrimitive::BOX) {
        o.dimensions.x = shape.dimensions[0];
        o.dimensions.y = shape.dimensions[1];
        o.dimensions.z = shape.dimensions[2];
	  } else {
	    std::cout << "error on BOX" ;
	  }
      objects->push_back(o);

	  // Publish a bounding box around it.
//	  visualization_msgs::Marker object_marker;
//	  object_marker.ns = "objects";
//	  object_marker.id = i;
//	  object_marker.header.frame_id = "base_link";
//	  object_marker.type = visualization_msgs::Marker::CUBE;
//	  GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
//	                            &object_marker.scale);
//	  object_marker.color.g = 1;
//	  object_marker.color.a = 0.3;
//	  marker_pub_.publish(object_marker);
	}
}


Segmenter::Segmenter(const ros::Publisher& surface_points_pub,
					 const ros::Publisher& marker_pub,
					 const ros::Publisher& above_surface_pub,
					 const ObjectRecognizer& recognizer)
    : surface_points_pub_(surface_points_pub),
      marker_pub_(marker_pub),
      above_surface_pub_(above_surface_pub),
      recognizer_(recognizer) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
	PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
	pcl::fromROSMsg(msg, *cloud_unfiltered);

    PointCloudC::Ptr cloud(new PointCloudC());

    pcl::fromROSMsg(msg, *cloud);

    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

    std::vector<Object> objects;
    SegmentTabletopScene(cloud, &objects, surface_points_pub_, marker_pub_, above_surface_pub_);

	// bounding box for objects
	for (size_t i = 0; i < objects.size(); ++i) {
	  const Object& object = objects[i];

	  // Publish a bounding box around it.
	  visualization_msgs::Marker object_marker;
	  object_marker.ns = "objects";
	  object_marker.id = i;
	  object_marker.header.frame_id = "base_link";
	  object_marker.type = visualization_msgs::Marker::CUBE;
      object_marker.pose = object.pose;
      object_marker.scale = object.dimensions;
	  object_marker.color.g = 1;
	  object_marker.color.a = 0.3;
	  marker_pub_.publish(object_marker);


	    std::string name;
        double confidence;
        // recognize the object with the recognizer_.
        recognizer_.Recognize(object, &name, &confidence);

	    confidence = round(1000 * confidence) / 1000;

        std::stringstream ss;
        ss << name << " (" << confidence << ")";

        // Publish the recognition result.
        visualization_msgs::Marker name_marker;
        name_marker.ns = "recognition";
        name_marker.id = i;
        name_marker.header.frame_id = "base_link";
        name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        name_marker.pose.position = object.pose.position;
        name_marker.pose.position.z += 0.1;
        name_marker.pose.orientation.w = 1;
        name_marker.scale.x = 0.025;
        name_marker.scale.y = 0.025;
        name_marker.scale.z = 0.025;
        name_marker.color.r = 0;
        name_marker.color.g = 0;
        name_marker.color.b = 1.0;
        name_marker.color.a = 1.0;
        name_marker.text = ss.str();
        marker_pub_.publish(name_marker);
	}
}
}  // namespace perception
