#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <iostream>
#include <chrono>

// For ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


namespace localmap_utils {
  void apply_original_agf(nav_msgs::OccupancyGrid::Ptr localmap_ptr, int target_idx, double target_yaw, double target_speed, int peak_value);
  void apply_social_agf(nav_msgs::OccupancyGrid::Ptr localmap_ptr, int target_idx, double target_yaw, double target_speed, int peak_value, bool flag_right_hand_side);
  void apply_butterworth_filter(nav_msgs::OccupancyGrid::Ptr localmap_ptr, std::vector<std::vector<int8_t> > &inflation_kernel, int target_idx, int peak_value);
  void butterworth_filter_generate(std::vector<std::vector<int8_t> > &inflation_kernel, double filter_radius, int filter_order, double map_resolution, int peak_value);
  void read_footprint_from_yaml(ros::NodeHandle nh, std::string footprint_topic_name, geometry_msgs::PolygonStamped::Ptr footprint_ptr);
  double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name);
  void GetLineCells(int x0, int x1, int y0, int y1, std::vector<std::pair<int, int> >& pts);
  void GetFillCells(std::vector<std::pair<int, int> >& pts);
  std::vector<std::pair<int, int> > GetFootprintCells(geometry_msgs::PolygonStamped::ConstPtr &footprint_ptr,
                                                      const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr);
}