#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;
ros::Publisher pub_color;
std::string output;
std::string read_camera;
std::string filtered_field;

float leaf_x;
float leaf_y;
float leaf_z;
float filter_max;
float filter_min;
float distance_threshold;

void cloudCB(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_colored_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  // msg -> raw_cloud
  pcl::fromROSMsg(*cloud_msg, *raw_cloud);

  // Segmentation

  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(raw_cloud);
  pass.setFilterFieldName(filtered_field);
  pass.setFilterLimits(filter_min, filter_max);
  pass.filter(*indices);
  pcl::removeNaNFromPointCloud(*raw_cloud, *indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud(raw_cloud);
  reg.setIndices(indices);
  reg.setDistanceThreshold(distance_threshold);
  reg.setPointColorThreshold(6);
  reg.setRegionColorThreshold(5);
  reg.setMinClusterSize(600);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

  pcl::VoxelGrid<pcl::PointXYZRGB> sor2;
  sor2.setInputCloud(colored_cloud);
  sor2.setLeafSize(leaf_x, leaf_y, leaf_z);
  sor2.filter(*transformed_colored_cloud);

  // Colored cloud -> msg

  sensor_msgs::PointCloud2 colored_output_msg;
  pcl::toROSMsg(*transformed_colored_cloud, colored_output_msg);

  colored_output_msg.header.frame_id = cloud_msg->header.frame_id;
  colored_output_msg.header.stamp = cloud_msg->header.stamp;

  pub_color.publish(colored_output_msg);

  // filter

  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(raw_cloud);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*transformed_cloud);

  // transformed -> msg
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*transformed_cloud, output_msg);

  output_msg.header.frame_id = cloud_msg->header.frame_id;
  output_msg.header.stamp = cloud_msg->header.stamp;

  pub.publish(output_msg);
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "e");

  ROS_INFO("node started");

  ros::NodeHandle nh("~");

  nh.param<std::string>("output_topic", output, "/output");
  nh.param<std::string>("camera_topic", read_camera, "/camera/depth/points");
  nh.param<float>("distance_threshold", distance_threshold, 10.0);
  nh.param<std::string>("filtered_field_name", filtered_field, "z");
  nh.param<float>("filter_min", filter_max, 4.0);
  nh.param<float>("filter_min", filter_min, 0.0);
  nh.param<float>("leaf_x", leaf_x, 0.02);
  nh.param<float>("leaf_y", leaf_y, 0.02);
  nh.param<float>("leaf_z", leaf_z, 0.02);

  // ROS_INFO("%s", output);

  ros::Subscriber sub =
      nh.subscribe<sensor_msgs::PointCloud2>(read_camera, 1, cloudCB);

  pub = nh.advertise<sensor_msgs::PointCloud2>(output, 1);
  pub_color = nh.advertise<sensor_msgs::PointCloud2>("/output2", 1);

  ros::Rate loop(5);

  while (ros::ok()) {
    nh.getParam("/distance_threshold", distance_threshold);

    nh.getParam("/leaf_x", leaf_x);
    nh.getParam("/leaf_y", leaf_y);
    nh.getParam("/leaf_z", leaf_z);

    nh.getParam("/filter_min", filter_min);
    nh.getParam("/filter_max", filter_max);

    nh.getParam("/filtered_field_name", filtered_field);

    loop.sleep();
    ros::spinOnce();
  }

  return 0;
}