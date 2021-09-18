#include <chrono>
#include <cmath>
#include <math.h>
#include <signal.h>

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <walker_msgs/Trk3DArray.h>
#include <walker_msgs/Trk3D.h>

// TF
#include <tf/transform_listener.h>

// PCL
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h> // ros2pcl
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

// Custom utils
#include "localmap_utils.hpp"

// using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;


class Scan2LocalmapNode {
public:
    Scan2LocalmapNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    static void sigint_cb(int sig);
    void scan_cb(const sensor_msgs::LaserScan &laser_msg);
    void trk3d_cb(const walker_msgs::Trk3DArray::ConstPtr &msg_ptr);
    void scan_cb_deprecated(const sensor_msgs::LaserScan &laser_msg);

    // ROS related
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_scan_;
    ros::Publisher pub_map_;
    ros::Publisher pub_footprint_;
    std::string localmap_frameid_;                               // Localmap frame_id
    nav_msgs::OccupancyGrid::Ptr localmap_ptr_;             // Localmap msg
    geometry_msgs::PolygonStamped::Ptr footprint_ptr_;      // Robot footprint
    laser_geometry::LaserProjection projector_;             // Projector of laserscan

    // TF listener
    tf::TransformListener* tflistener_ptr_;
    tf::StampedTransform tf_laser2base_;    

    // Inflation filter kernel
    std::vector<std::vector<int8_t> > inflation_kernel_;

    // PCL Cropbox filter
    pcl::CropBox<pcl::PointXYZ> box_filter_; 
    pcl::VoxelGrid<pcl::PointXYZ> vg_filter_;

    // Flag AGF use or not
    int agf_type_;
};


Scan2LocalmapNode::Scan2LocalmapNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh) {
    // Signal handler
    signal(SIGINT, Scan2LocalmapNode::sigint_cb);

    // ROS parameters
    double inflation_radius;
    double map_resolution;
    double localmap_range_x, localmap_range_y;
    std::string scan_src_frameid;
    ros::param::param<double>("~inflation_radius", inflation_radius, 0.2);
    ros::param::param<double>("~map_resolution", map_resolution, 0.1);
    ros::param::param<double>("~localmap_range_x", localmap_range_x, 10.0);     // map_width --> x axis
    ros::param::param<double>("~localmap_range_y", localmap_range_y, 10.0);     // map_height --> y_axis
    ros::param::param<std::string>("~localmap_frameid", localmap_frameid_, "base_link");
    ros::param::param<std::string>("~scan_src_frameid", scan_src_frameid, "laser_link");
    ros::param::param<int>("~agf_type", agf_type_, -1);
    
    // ROS publishers & subscribers
    if(agf_type_ >= 0)
        sub_scan_ = nh_.subscribe("trk3d_result", 1, &Scan2LocalmapNode::trk3d_cb, this);
    else
        sub_scan_ = nh_.subscribe("scan", 1, &Scan2LocalmapNode::scan_cb, this);

    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map", 1);
    pub_footprint_ = nh_.advertise<geometry_msgs::PolygonStamped>("footprint", 1);

    // Prepare the transformation matrix from laser to base
    tflistener_ptr_ = new tf::TransformListener();
    ROS_INFO("Wait for TF from laser_link to %s in 10 seconds...", localmap_frameid_.c_str());
    try{
        tflistener_ptr_->waitForTransform(localmap_frameid_, scan_src_frameid,
                                    ros::Time(), ros::Duration(10.0));
        tflistener_ptr_->lookupTransform(localmap_frameid_, scan_src_frameid,
                                    ros::Time(), tf_laser2base_);
        ROS_INFO("Done.");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("\nCannot get TF from laserscan to %s: %s. Aborting...", localmap_frameid_.c_str(), ex.what());
        exit(-1);
    }
    
    // Initialize localmap meta information
    localmap_ptr_ = nav_msgs::OccupancyGrid::Ptr(new nav_msgs::OccupancyGrid());
    localmap_ptr_->info.width = localmap_range_x * 2 / map_resolution;      // map_width --> x axis
    localmap_ptr_->info.height = localmap_range_y * 2 / map_resolution;     // map_height --> y_axis
    localmap_ptr_->info.resolution = map_resolution;
    localmap_ptr_->info.origin.position.x = -localmap_ptr_->info.resolution * localmap_ptr_->info.width / 2;
    localmap_ptr_->info.origin.position.y = -localmap_ptr_->info.resolution * localmap_ptr_->info.height / 2;
    localmap_ptr_->info.origin.orientation.w = 1.0;
    localmap_ptr_->data.resize(localmap_ptr_->info.width * localmap_ptr_->info.height);
    localmap_ptr_->header.frame_id = localmap_frameid_;
    ROS_INFO("Default range of localmap:+-%.1fx%.1f m, size:%dx%d", 
                localmap_range_x, localmap_range_y, localmap_ptr_->info.width, localmap_ptr_->info.height);
    
    // Footprint generator
    footprint_ptr_ = geometry_msgs::PolygonStamped::Ptr(new geometry_msgs::PolygonStamped());
    localmap_utils::read_footprint_from_yaml(nh_, "footprint", footprint_ptr_);
    footprint_ptr_->header.frame_id = localmap_frameid_;

    // Cropbox filter init
    box_filter_.setMax(Eigen::Vector4f(0.4, 0.50, 5.0, 1.0));
    box_filter_.setMin(Eigen::Vector4f(-1.5, -0.50, -5.0, 1.0));
    box_filter_.setKeepOrganized(false);
    box_filter_.setNegative(true);

    // VoxelGrid filter init
    double voxel_grid_size = 0.2;
    vg_filter_.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);  
}


void Scan2LocalmapNode::trk3d_cb(const walker_msgs::Trk3DArray::ConstPtr &msg_ptr) {
    // Get the transformation from tracking result frame to base frame
    tf::StampedTransform tf_trk2base;
    try{
        tflistener_ptr_->waitForTransform(localmap_frameid_, msg_ptr->header.frame_id,
                                    ros::Time(), ros::Duration(0.1));
        tflistener_ptr_->lookupTransform(localmap_frameid_, msg_ptr->header.frame_id,
                                    ros::Time(), tf_trk2base);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("\nCannot get TF from odom to %s: %s. Aborting...", localmap_frameid_.c_str(), ex.what());
        exit(-1);
    }
    
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    sensor_msgs::LaserScan laser_msg = msg_ptr->scan;

    // Convert laserscan to pointcloud:  laserscan --> ROS PointCloud2 --> PCL PointCloudXYZ
    sensor_msgs::PointCloud2 cloud_msg;
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    PointCloudXYZPtr cloud_transformed(new PointCloudXYZ);
    projector_.projectLaser(laser_msg, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);
    pcl_ros::transformPointCloud(*cloud_raw, *cloud_transformed, tf_laser2base_);

    // Apply cropbox filter
    box_filter_.setInputCloud(cloud_transformed);
    box_filter_.filter(*cloud_transformed);

    // Apply voxel grid filter
    vg_filter_.setInputCloud(cloud_transformed);
    vg_filter_.filter(*cloud_transformed);

    // Localmap init
    std::fill(localmap_ptr_->data.begin(), localmap_ptr_->data.end(), -1);

    double resolution = localmap_ptr_->info.resolution;
    double map_origin_x = localmap_ptr_->info.origin.position.x;
    double map_origin_y = localmap_ptr_->info.origin.position.y;
    int map_width = localmap_ptr_->info.width;
    int map_height = localmap_ptr_->info.height;
    int map_limit = map_width * map_height;

    for(int i = 0; i < localmap_ptr_->data.size(); i++) {
        double grid_real_x = (i % map_width) * resolution + map_origin_x;
        double grid_real_y = (i / map_width) * resolution + map_origin_y;
        // printf("real xy (%.2f, %.2f)\n", grid_real_x, grid_real_y);
        double grid_real_anlge = std::atan2(grid_real_y, grid_real_x);
        double grid_real_distance = std::hypot(grid_real_x, grid_real_y);

        bool is_behind_obstacle = false;
        std::vector<double> distance_list;
        for(int j = 0; j < cloud_transformed->points.size(); j++){
            double obstacle_angle = std::atan2(cloud_transformed->points[j].y, cloud_transformed->points[j].x);
            double obstacle_distance = std::hypot(cloud_transformed->points[j].x, cloud_transformed->points[j].y);
            if(std::abs(obstacle_angle - grid_real_anlge) < (M_PI / 36) && obstacle_distance < grid_real_distance) {
                is_behind_obstacle = true;
                break;
            }
            distance_list.push_back(std::hypot(cloud_transformed->points[j].x - grid_real_x, cloud_transformed->points[j].y - grid_real_y));
        }
        if(is_behind_obstacle) continue;
        double closest_distance = *(std::min_element(distance_list.begin(), distance_list.end()));
        double comfort_value = (closest_distance <= 1.2)? (100.0 / (1 + std::exp(-2.0 * closest_distance)) - 50.0): 
                                                        (210.0 / (1 + std::exp(-0.5 * (closest_distance + 0.4))) - 103.0);
        localmap_ptr_->data[i] = (comfort_value < 80)? 105 - (int8_t)comfort_value: 0;
    }

    // Proxemics generation
    for(int i = 0; i < msg_ptr->trks_list.size(); i++) {
        // Convert object pose from laser coordinate to base coordinate
        tf::Vector3 pt_laser(msg_ptr->trks_list[i].x, msg_ptr->trks_list[i].y, 0);
        // tf::Vector3 pt_base = tf_laser2base_.getBasis() * pt_laser + tf_laser2base_.getOrigin();
        tf::Vector3 pt_base = tf_trk2base.getBasis() * pt_laser + tf_trk2base.getOrigin();
        tf::Quaternion q;
        q.setRPY(0, 0, msg_ptr->trks_list[i].yaw);
        double yaw, pitch, roll;
        tf::Matrix3x3 mat(q);
        // mat = tf_laser2base_.getBasis() * mat;
        mat = tf_trk2base.getBasis() * mat;
        mat.getEulerYPR(yaw, pitch, roll);

        double speed = std::hypot(msg_ptr->trks_list[i].vx, msg_ptr->trks_list[i].vy);

        // Calculate object position in local map
        int map_x = std::floor((pt_base.getX() - map_origin_x) / resolution);
        int map_y = std::floor((pt_base.getY() - map_origin_y) / resolution);
        int idx = map_y * map_width + map_x;

        // Apply AGF
        if(map_x < map_width && map_y < map_height) {
            switch(agf_type_){
                case 0:
                    localmap_utils::apply_original_agf(localmap_ptr_, idx, yaw, speed * 1.2, 100);
                    break;
                case 1:
                    localmap_utils::apply_social_agf(localmap_ptr_, idx, yaw, speed * 1.2, 100, true);
                    break;
                case 2:
                    localmap_utils::apply_social_agf(localmap_ptr_, idx, yaw, speed * 1.2, 100, false);
                    break;
            }
        }
    }

    // Publish localmap
    ros::Time now = ros::Time(0);
    localmap_ptr_->header.stamp = now;
    pub_map_.publish(*localmap_ptr_);

    // Publish footprint
    footprint_ptr_->header.stamp = now;
    pub_footprint_.publish(*footprint_ptr_);

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}


void Scan2LocalmapNode::scan_cb(const sensor_msgs::LaserScan &laser_msg) {
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // Convert laserscan to pointcloud:  laserscan --> ROS PointCloud2 --> PCL PointCloudXYZ
    sensor_msgs::PointCloud2 cloud_msg;
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    PointCloudXYZPtr cloud_transformed(new PointCloudXYZ);
    projector_.projectLaser(laser_msg, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);
    pcl_ros::transformPointCloud(*cloud_raw, *cloud_transformed, tf_laser2base_);

    // Apply cropbox filter
    box_filter_.setInputCloud(cloud_transformed);
    box_filter_.filter(*cloud_transformed);

    // Apply voxel grid filter
    vg_filter_.setInputCloud(cloud_transformed);
    vg_filter_.filter(*cloud_transformed);

    // Localmap init
    std::fill(localmap_ptr_->data.begin(), localmap_ptr_->data.end(), -1);

    double resolution = localmap_ptr_->info.resolution;
    double map_origin_x = localmap_ptr_->info.origin.position.x;
    double map_origin_y = localmap_ptr_->info.origin.position.y;
    int map_width = localmap_ptr_->info.width;
    int map_height = localmap_ptr_->info.height;
    int map_limit = map_width * map_height;

    for(int i = 0; i < localmap_ptr_->data.size(); i++) {
        double grid_real_x = (i % map_width) * resolution + map_origin_x;
        double grid_real_y = (i / map_width) * resolution + map_origin_y;
        // printf("real xy (%.2f, %.2f)\n", grid_real_x, grid_real_y);
        double grid_real_anlge = std::atan2(grid_real_y, grid_real_x);
        double grid_real_distance = std::hypot(grid_real_x, grid_real_y);

        bool is_behind_obstacle = false;
        std::vector<double> distance_list;
        distance_list.push_back(100.0);     // Dummy obstacle to deal with empty localmap problem
        for(int j = 0; j < cloud_transformed->points.size(); j++){
            double obstacle_angle = std::atan2(cloud_transformed->points[j].y, cloud_transformed->points[j].x);
            double obstacle_distance = std::hypot(cloud_transformed->points[j].x, cloud_transformed->points[j].y);
            if(std::abs(obstacle_angle - grid_real_anlge) < (M_PI / 36) && obstacle_distance < grid_real_distance) {
                is_behind_obstacle = true;
                break;
            }
            distance_list.push_back(std::hypot(cloud_transformed->points[j].x - grid_real_x, cloud_transformed->points[j].y - grid_real_y));
        }
        if(is_behind_obstacle) continue;
        double closest_distance = *(std::min_element(distance_list.begin(), distance_list.end()));
        double comfort_value = (closest_distance <= 1.2)? (100.0 / (1 + std::exp(-2.0 * closest_distance)) - 50.0): 
                                                        (210.0 / (1 + std::exp(-0.5 * (closest_distance + 0.4))) - 103.0);
        localmap_ptr_->data[i] = (comfort_value < 80)? 105 - (int8_t)comfort_value: 0;
    }

    // Publish localmap
    ros::Time now = ros::Time(0);
    localmap_ptr_->header.stamp = now;
    pub_map_.publish(*localmap_ptr_);

    // Publish footprint
    footprint_ptr_->header.stamp = now;
    pub_footprint_.publish(*footprint_ptr_);

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}


void Scan2LocalmapNode::sigint_cb(int sig) {
    std::cout << "\nNode name: " << ros::this_node::getName() << " is shutdown." << std::endl;
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "scan2comfortmap_node");
    ros::NodeHandle nh, pnh("~");
    Scan2LocalmapNode node(nh, pnh);    
    ros::spin();
    return 0;
}
