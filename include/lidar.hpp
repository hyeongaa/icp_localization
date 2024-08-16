#ifndef LIDAR_HPP
#define LIDAR_HPP


#include <iostream>
#include <string>
#include <cstdlib> //for servers
#include <thread>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Trigger.h>  // for servers

//PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h> //icp
#include <pcl/registration/gicp.h>//gicp
#include <pcl/registration/ndt.h>//ndt





#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <queue>
#include <vector>
#include <math.h>


#include "frame.hpp"
#include "transformation.hpp"

Eigen::Matrix4d performICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score);
Eigen::Matrix4d performGICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score);
Eigen::Matrix4d performNDT(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, bool& icp_success, double& score);

double estimate_initial_position(bool& success, Eigen::Matrix4d& icp_transformation, int cnt);
pcl::PointCloud<pcl::PointXYZI>::Ptr make_cloud_submap(int index, int submap_size);
void voxelize_pcd(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
void voxelize_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
void voxelize_entire_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);



extern std::vector<frame_pose> frames; //1_frame={image,odom,pointcloud}
extern pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud;
extern std::string map_frame; 
extern ros::Publisher pubinitodom;
extern ros::Publisher pubodom;
extern ros::Publisher pubinit_cloud;
extern ros::Publisher pubcenter_cloud;
extern ros::Publisher pubchange;
extern std::mutex update_pose_mutex;

extern bool changed;
extern Eigen::Matrix4d icp_transformation_result;
extern Eigen::Matrix4d center_trans;

#endif