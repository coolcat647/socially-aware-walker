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

    // ROS related
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_scan_;
    ros::Publisher pub_map_;
    ros::Publisher pub_footprint_;
    std::string localmap_frameid_;                          // Localmap frame_id
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
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;  // Voxel grid filter

    // Flag for AGF using or not
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
    // localmap_ptr_->info.origin.position.z = tf_laser2base_.getOrigin().getZ();
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
    // Voxel grid filter init
    voxel_grid_.setLeafSize (map_resolution, map_resolution, map_resolution);

    // Filter kernel generator
    localmap_utils::butterworth_filter_generate(inflation_kernel_, inflation_radius, 2, map_resolution, 100);

    ROS_INFO_STREAM(ros::this_node::getName() + " is ready.");
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

    // Localmap init
    std::fill(localmap_ptr_->data.begin(), localmap_ptr_->data.end(), 0);

    double resolution = localmap_ptr_->info.resolution;
    double map_origin_x = localmap_ptr_->info.origin.position.x;
    double map_origin_y = localmap_ptr_->info.origin.position.y;
    int map_width = localmap_ptr_->info.width;
    int map_height = localmap_ptr_->info.height;
    int map_limit = map_width * map_height - 1;

    // Proxemics generation
    for(int i = 0; i < msg_ptr->trks_list.size(); i++) {
        // Convert object pose from laser coordinate to base coordinate
        tf::Vector3 pt_laser(msg_ptr->trks_list[i].x, msg_ptr->trks_list[i].y, 0);
        tf::Vector3 pt_base = tf_trk2base * pt_laser;
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
                    localmap_utils::apply_original_agf(localmap_ptr_, idx, yaw, speed, 100);
                    break;
                case 1:
                    localmap_utils::apply_social_agf(localmap_ptr_, idx, yaw, speed, 100, true);
                    break;
                case 2:
                    localmap_utils::apply_social_agf(localmap_ptr_, idx, yaw, speed, 100, false);
                    break;
            }
            // Clear points belong human again
            pcl::CropBox<pcl::PointXYZ> human_boxcrop;
            human_boxcrop.setMax(Eigen::Vector4f(pt_base.getX() + 0.4, pt_base.getY() + 0.4, 5.0, 1.0));
            human_boxcrop.setMin(Eigen::Vector4f(pt_base.getX() - 0.4, pt_base.getY() - 0.4, -5.0, 1.0));
            human_boxcrop.setNegative(true);
            human_boxcrop.setInputCloud(cloud_transformed);
            human_boxcrop.filter(*cloud_transformed);
        }
    }

    // Static obstacle inflation
    for(int i = 0; i < cloud_transformed->points.size(); i++) {
        double laser_x = cloud_transformed->points[i].x;
        double laser_y = cloud_transformed->points[i].y;
        if(fabs(laser_x) > map_width * resolution / 2)
            continue;
        else if(fabs(laser_y) > map_height * resolution / 2)
            continue;

        int map_x = std::floor((laser_x - map_origin_x) / resolution);
        int map_y = std::floor((laser_y - map_origin_y) / resolution);
        int idx = map_y * map_width + map_x;
        
        if(map_x < map_width && map_y < map_height && localmap_ptr_->data[idx] < 80) {
            localmap_utils::apply_butterworth_filter(localmap_ptr_, inflation_kernel_, idx, 100);
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
    voxel_grid_.setInputCloud (cloud_transformed);
    voxel_grid_.filter (*cloud_transformed);

    // Localmap init
    std::fill(localmap_ptr_->data.begin(), localmap_ptr_->data.end(), 0);

    double resolution = localmap_ptr_->info.resolution;
    double map_origin_x = localmap_ptr_->info.origin.position.x;
    double map_origin_y = localmap_ptr_->info.origin.position.y;
    int map_width = localmap_ptr_->info.width;
    int map_height = localmap_ptr_->info.height;
    int map_limit = map_width * map_height - 1;

    for(int i = 0; i < cloud_transformed->points.size(); i++) {
        double laser_x = cloud_transformed->points[i].x;
        double laser_y = cloud_transformed->points[i].y;
        if(fabs(laser_x) > map_width * resolution / 2)
            continue;
        else if(fabs(laser_y) > map_height * resolution / 2)
            continue;

        // Add wall(non-walkable) space
        int map_x = std::floor((laser_x - map_origin_x) / resolution);
        int map_y = std::floor((laser_y - map_origin_y) / resolution);
        int idx = map_y * map_width + map_x;
        
        if(map_x < map_width && map_y < map_height && localmap_ptr_->data[idx] < 100) {
            localmap_utils::apply_butterworth_filter(localmap_ptr_, inflation_kernel_, idx, 100);
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


void Scan2LocalmapNode::sigint_cb(int sig) {
    ROS_INFO_STREAM("Node name: " << ros::this_node::getName() << " is shutdown.");
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "laserscan_mapping_node");
    ros::NodeHandle nh, pnh("~");
    Scan2LocalmapNode node(nh, pnh);    
    ros::spin();
    return 0;
}
