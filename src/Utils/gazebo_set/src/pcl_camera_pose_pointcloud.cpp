#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// 添加
#include <pcl/PCLPointCloud2.h>
#include <Eigen/Dense>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace Eigen;

ros::Publisher camera_pos,camera_cloud;
ros::Subscriber pos,point;

#define PI acos(-1)

pcl::PointCloud<pcl::PointXYZ> pc1, pc2, pc3, pc4;
pcl::PointCloud<pcl::PointXYZ> pc_global;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::PointCloud2 camera_sensor_pcd;
int times = 0;
nav_msgs::Odometry odom_;
ros::Time last_odom_stamp = ros::TIME_MAX;


void revOdometryCallback(const nav_msgs::Odometry& odom)
{
  odom_ = odom;
  geometry_msgs::PoseStamped msg1;
  msg1.header = odom.header;
  msg1.header.frame_id = "map";

  Matrix4d body_world = Matrix4d::Identity();

  Eigen::Quaterniond pose;
  pose.x() = odom.pose.pose.orientation.x;
  pose.y() = odom.pose.pose.orientation.y;
  pose.z() = odom.pose.pose.orientation.z;
  pose.w() = odom.pose.pose.orientation.w;
  body_world.block<3, 3>(0, 0) = pose.toRotationMatrix();
  body_world(0, 3) = odom.pose.pose.position.x+0.2;
  body_world(1, 3) = odom.pose.pose.position.y+0.2;
  body_world(2, 3) = odom.pose.pose.position.z;

  Eigen::Matrix4d camera_body;
  camera_body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::Matrix4d camera_world;

  camera_world =body_world *camera_body; 

  last_odom_stamp = odom.header.stamp;

  Eigen::Quaterniond q;
  q = camera_world.block<3, 3>(0, 0);

  msg1.pose.position.x = camera_world(0,3);
  msg1.pose.position.y = camera_world(1,3);
  msg1.pose.position.z = camera_world(2,3);

  msg1.pose.orientation.x  = q.x();
  msg1.pose.orientation.y  = q.y();
  msg1.pose.orientation.z  = q.z();
  msg1.pose.orientation.w  = q.w();

  camera_pos.publish(msg1);
}

void revPointcloudCallback(const sensor_msgs::PointCloud2 pointcloud_camera)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::PointCloud<pcl::PointXYZ> cloud_voxel;
  pcl::fromROSMsg(pointcloud_camera, cloud_input);

  pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  _voxel_sampler.filter(cloud_voxel);

  
  camera_sensor_pcd.is_dense  = true;

  pcl::toROSMsg(cloud_voxel, camera_sensor_pcd);
  camera_sensor_pcd.header.stamp = last_odom_stamp;
  camera_cloud.publish(camera_sensor_pcd); 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_pose");
  ros::NodeHandle nh;
  pos = nh.subscribe("/mavros/local_position/odom", 1, revOdometryCallback);
  point = nh.subscribe("/camera/depth/points",1, revPointcloudCallback); 
  camera_pos = nh.advertise<geometry_msgs::PoseStamped>("/camera_pose", 10);
  camera_cloud = nh.advertise<sensor_msgs::PointCloud2>("/camera_cloud",10);
  ros::spin();
}
