#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"

#include "geometry_msgs/Pose.h"
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/voxel_grid.h>
#include <limits>

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pubm;
ros::Publisher pub_cylinder_marker_array;

tf2_ros::Buffer tf2_buffer;

typedef pcl::PointXYZRGB PointT;

// new variables
visualization_msgs::MarkerArray cylinder_marker_array; // for pub_cylinder_marker_array
std::vector<int> potential_positions_clusters;
std::vector<geometry_msgs::Pose> potential_positions;
int marker_id = 0;

void downsample_pcl_voxel(const pcl::PCLPointCloud2ConstPtr &cloud_blob,
                          const pcl::PointCloud<PointT>::Ptr &cloud)
{
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
  sor.setInputCloud(cloud_blob);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  // sor.setLeafSize(0.005f, 0.005f, 0.005f);
  sor.filter(*cloud_filtered_blob);
  pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud);
}

std::vector<double> get_color(double red, double green, double blue){
	  double r = 255;
    double g = 255;
    double b = 255;
    double a = 1;

  if (red > green && red > blue){
		//Probably red
		//if(abs(red-green) < 20){
        if(abs(red-green) < 15){
			//Probably yellow
        r = 255;
        g = 165;
        b = 0;
        a = 1;
        return std::vector<double> { r, g, b, a};

		}
		
		    r = 1;
        g = 0;
        b = 0;
        a = 1;
        return std::vector<double> { r, g, b, a};

	}else if (green > red && green > blue){
		//Probably green
		//if(abs(red-green) < 20){
        if(abs(red-green) < 15){
			//Probably yellow
        r = 255;
        g = 165;
        b = 0;
        a = 1;
        return std::vector<double> { r, g, b, a};
		}
		
        r = 0;
        g = 1;
        b = 0;
        a = 1;
        return std::vector<double> { r, g, b, a};
	}else if (blue > red && blue > green){
		//Probably blue
        r = 0;
        g = 0;
        b = 1;
        a = 1;
        return std::vector<double> { r, g, b, a};
	}
	
	return std::vector<double> { r, g, b, a};
}

void publish_new_marker(geometry_msgs::Pose pose, double red, double green, double blue)
{
  std::vector<double> color = get_color(red, green, blue);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "cylinder";
  marker.id = marker_id;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose = pose;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1.0f;

  marker.lifetime = ros::Duration();
  marker_id++;
  // pubm.publish(marker);
  cylinder_marker_array.markers.push_back(marker);
  pub_cylinder_marker_array.publish(cylinder_marker_array);
  std::cerr << "Published a new marker. " << std::endl;
}

double euclidean_distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;

  return std::sqrt(dx * dx + dy * dy);
}

void check_potential_cylinder(geometry_msgs::Pose pose, double red, double green, double blue)
{
  double min_distance = std::numeric_limits<double>::infinity();
  int min_distance_index = 0;
  if (potential_positions.size() == 0)
  {
    potential_positions.push_back(pose);
    potential_positions_clusters.push_back(1);
    std::cerr << "First position added" << std::endl;
    return;
  }

  for (int i = 0; i < potential_positions.size(); i++) // check for the closest group
  {
    double eucl_dist = euclidean_distance(potential_positions[i], pose);
    if (eucl_dist < min_distance)
    {
      min_distance = eucl_dist;
      min_distance_index = i;
    }
  }
  std::cerr << "Closest point is: " << min_distance_index << " Eucl dist: " << min_distance << std::endl;

  // if the group is in certain threshold distance to another, add it to its "cluster" and calculate new average coordinates every time a point is computed -> if two cylinders are very close it detects only one?
  if (min_distance < 1.0) // m
  {
    potential_positions_clusters[min_distance_index] += 1;
    potential_positions[min_distance_index].position.x = (potential_positions[min_distance_index].position.x + pose.position.x) / 2;
    potential_positions[min_distance_index].position.y = (potential_positions[min_distance_index].position.y + pose.position.y) / 2;
    potential_positions[min_distance_index].position.z = (potential_positions[min_distance_index].position.z + pose.position.z) / 2;
    std::cerr << "New position for group: " << min_distance_index << " [ " << potential_positions[min_distance_index].position.x << ", " << potential_positions[min_distance_index].position.y << ", " << potential_positions[min_distance_index].position.z << " ]" << std::endl;
  }
  else // if the positions are not close enough create new group
  {
    potential_positions.push_back(pose);
    potential_positions_clusters.push_back(1);
    int new_ix = potential_positions.size() - 1;
    std::cerr << "New added group: " << new_ix << " [ " << potential_positions[new_ix].position.x << ", " << potential_positions[new_ix].position.y << ", " << potential_positions[new_ix].position.z << " ]" << std::endl;
  }

  if (potential_positions_clusters[min_distance_index] == 20)
  {
    publish_new_marker(potential_positions[min_distance_index], red, green, blue);
  }
}

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
  // All the objects needed

  ros::Time time_rec, time_test;
  time_rec = ros::Time::now();

  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  Eigen::Vector4f centroid;

  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

  pcl::PointCloud<PointT>::Ptr cloud_filtered_over_z(new pcl::PointCloud<PointT>);

  // Read in the cloud data
  downsample_pcl_voxel(cloud_blob, cloud);

  // pcl::fromPCLPointCloud2(*cloud_blob, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0, 1.5);
  pass.filter(*cloud_filtered_over_z);

  pass.setInputCloud(cloud_filtered_over_z);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.2, 0.2);
  pass.filter(*cloud_filtered);

  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;
  if (cloud_filtered->points.size() <= 50)
  {
    std::cerr << "PointCloud less than required number of points: " << cloud_filtered->points.size() << " data points." << std::endl;
    return;
  }

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud_filtered);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.03);
  seg.setInputCloud(cloud_filtered);
  seg.setInputNormals(cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment(*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
  extract.filter(*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

  // pcl::PCLPointCloud2 outcloud_plane;
  // pcl::toPCLPointCloud2(*cloud_plane, outcloud_plane);
  // pubx.publish(outcloud_plane);

  // Remove the planar inliers, extract the rest
  extract.setNegative(true);
  extract.filter(*cloud_filtered2);
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals2);

  if (cloud_filtered2->points.size() <= 10)
  {
    return;
  }

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.05);
  seg.setRadiusLimits(0.1, 0.17);
  seg.setInputCloud(cloud_filtered2);
  seg.setInputNormals(cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment(*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  if (coefficients_cylinder->values.size() == 0)
  {
    std::cerr << "Cylinder not found!" << std::endl;
    return;
  }
  // chech if cylinder radius is in the expected range
  if (coefficients_cylinder->values[6] < 0.1 || coefficients_cylinder->values[6] > 0.15)
  {

    return;
  }

  std::cerr << "Inliers_num_cylinder: " << inliers_cylinder->indices.size() << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud(cloud_filtered2);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
  extract.filter(*cloud_cylinder);

  if (cloud_cylinder->points.empty())
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
    std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;

    pcl::compute3DCentroid(*cloud_cylinder, centroid);
    std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << centroid[3] << std::endl;

    // Create a point in the "camera_rgb_optical_frame"
    geometry_msgs::PointStamped point_camera;
    geometry_msgs::PointStamped point_map;
    visualization_msgs::Marker marker;
    geometry_msgs::TransformStamped tss;

    point_camera.header.frame_id = "camera_rgb_optical_frame";
    point_camera.header.stamp = ros::Time::now();

    point_map.header.frame_id = "map";
    point_map.header.stamp = ros::Time::now();

    point_camera.point.x = centroid[0];
    point_camera.point.y = centroid[1];
    point_camera.point.z = centroid[2];

    try
    {
      time_test = ros::Time::now();

      std::cerr << time_rec << std::endl;
      std::cerr << time_test << std::endl;
      tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", time_rec);
      // tf2_buffer.transform(point_camera, point_map, "map", ros::Duration(2));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Transform warning: %s\n", ex.what());
    }

    // std::cerr << tss ;

    tf2::doTransform(point_camera, point_map, tss);

    std::cerr << "point_camera: " << point_camera.point.x << " " << point_camera.point.y << " " << point_camera.point.z << std::endl;

    std::cerr << "Point_map: " << point_map.point.x << " " << point_map.point.y << " " << point_map.point.z << std::endl;

    // calculate average color of the points
    double red = 0;
    double green = 0;
    double blue = 0;

    for (int iii = 0; iii < cloud_cylinder->points.size(); iii++)
    {
      red = red + cloud_cylinder->points[iii].r;
      green = green + cloud_cylinder->points[iii].g;
      blue = blue + cloud_cylinder->points[iii].b;
    }

    red = red / cloud_cylinder->points.size();
    green = green / cloud_cylinder->points.size();
    blue = blue / cloud_cylinder->points.size();

    std::cerr << "RGB: " << red << ", " << green << ", " << blue << std::endl;
    if (abs(red-green) < 10 && abs(red-blue) < 10 && abs(blue-green) < 10)
    {
      std::cerr << "The object is gray!" << std::endl;
      return;
    }
    geometry_msgs::Pose potential_pose;

    potential_pose.position.x = point_map.point.x;
    potential_pose.position.y = point_map.point.y;
    potential_pose.position.z = point_map.point.z;
    potential_pose.orientation.x = 0.0;
    potential_pose.orientation.y = 0.0;
    potential_pose.orientation.z = 0.0;
    potential_pose.orientation.w = 1.0;
    if (std::isnan(point_map.point.x) || std::isnan(point_map.point.y) || std::isnan(point_map.point.z))
    {
      std::cerr << "The point_map values are NaN!" << std::endl;
      return;
    }

    // publish_new_marker(potential_pose);
    check_potential_cylinder(potential_pose, red, green, blue);

    pcl::PCLPointCloud2 outcloud_cylinder;
    pcl::toPCLPointCloud2(*cloud_cylinder, outcloud_cylinder);
    puby.publish(outcloud_cylinder);
  }
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cylinder_segment");
  ros::NodeHandle nh;

  // For transforming between coordinate frames
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubx = nh.advertise<pcl::PCLPointCloud2>("planes", 1);
  puby = nh.advertise<pcl::PCLPointCloud2>("cylinder", 1);

  pubm = nh.advertise<visualization_msgs::Marker>("detected_cylinder", 1);
  pub_cylinder_marker_array = nh.advertise<visualization_msgs::MarkerArray>("detected_cylinders", 1000);
  // Spin
  ros::spin();
}
