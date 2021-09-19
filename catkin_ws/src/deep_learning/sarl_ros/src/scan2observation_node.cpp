#include <chrono>
#include <cmath>
#include <math.h>
#include <signal.h>

#include "ros/ros.h"
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;


class Scan2ObservationNode {
public:
    Scan2ObservationNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    static void sigint_cb(int sig);
    void convert_scan_to_observations(walker_msgs::Trk3DArray::Ptr observation_msg_ptr, PointCloudXYZPtr cloud_baseframe, tf::StampedTransform tf_base2odom);
    void scan_cb(const sensor_msgs::LaserScan &laser_msg);
    void trk3d_cb(const walker_msgs::Trk3DArray::ConstPtr &msg_ptr);

    // ROS related
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_scan_;
    ros::Publisher pub_marker_array_;
    ros::Publisher pub_pc_filtered_;
    ros::Publisher pub_observation_;
    std::string base_frameid_;                              // baselink frame_id
    std::string odom_frameid_;                              // odom frame_id
    laser_geometry::LaserProjection projector_;             // Projector of laserscan

    // TF listener
    tf::TransformListener* tflistener_ptr_;
    tf::StampedTransform tf_laser2base_;    

    // Cropbox filter
    pcl::CropBox<pcl::PointXYZ> box_filter_;
    // Voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;

    double static_obstacle_radius_;
    double dynamic_obstacle_radius_;
};


Scan2ObservationNode::Scan2ObservationNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh) {
    // Signal handler
    signal(SIGINT, Scan2ObservationNode::sigint_cb);

    // ROS parameters
    double inflation_radius;
    double map_resolution;
    double localmap_range_x, localmap_range_y;
    bool flag_only_static;
    std::string scan_src_frameid;
    ros::param::param<double>("~static_obstacle_radius", static_obstacle_radius_, 0.8);
    ros::param::param<double>("~dynamic_obstacle_radius", dynamic_obstacle_radius_, 0.4);
    ros::param::param<std::string>("~base_frameid", base_frameid_, "base_link");
    ros::param::param<std::string>("~odom_frameid", odom_frameid_, "odom");
    ros::param::param<std::string>("~scan_src_frameid", scan_src_frameid, "laser_link");
    ros::param::param<bool>("~flag_only_static", flag_only_static, true);

    // ROS publishers & subscribers
    if(flag_only_static == true)
        sub_scan_ = nh_.subscribe("scan", 1, &Scan2ObservationNode::scan_cb, this);
    else
        sub_scan_ = nh_.subscribe("trk3d_result", 1, &Scan2ObservationNode::trk3d_cb, this);

    // pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>("clustering_result", 1);
    pub_pc_filtered_ = nh.advertise<sensor_msgs::PointCloud2>("pc_filtered", 1);
    pub_observation_ = nh.advertise<walker_msgs::Trk3DArray>("rl_observation_array", 1);

    // Prepare the transformation matrix from laser to base
    tflistener_ptr_ = new tf::TransformListener();
    ROS_INFO("Wait for TF from laser_link to %s in 10 seconds...", base_frameid_.c_str());
    try{
        tflistener_ptr_->waitForTransform(base_frameid_, scan_src_frameid,
                                    ros::Time(), ros::Duration(10.0));
        tflistener_ptr_->lookupTransform(base_frameid_, scan_src_frameid,
                                    ros::Time(), tf_laser2base_);
        ROS_INFO("Done.");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("\nCannot get TF from laserscan to %s: %s. Aborting...", base_frameid_.c_str(), ex.what());
        exit(-1);
    }

    // Cropbox filter init, to skip the laserscan on walker user
    box_filter_.setMax(Eigen::Vector4f(0.4, 0.50, 5.0, 1.0));
    box_filter_.setMin(Eigen::Vector4f(-1.5, -0.50, -5.0, 1.0));
    box_filter_.setKeepOrganized(false);
    box_filter_.setNegative(true);
    // Voxel grid filter init
    voxel_grid_.setLeafSize(static_obstacle_radius_ * 2, static_obstacle_radius_ * 2, static_obstacle_radius_ * 2);

    ROS_INFO_STREAM(ros::this_node::getName() + " is ready.");
}


void Scan2ObservationNode::convert_scan_to_observations(walker_msgs::Trk3DArray::Ptr observation_msg_ptr, 
                                                        PointCloudXYZPtr cloud_ptr_baseframe,
                                                        tf::StampedTransform tf_base2odom) {
    if(!cloud_ptr_baseframe || cloud_ptr_baseframe->points.size() < 1){
        // ROS_WARN("Skip empty scan");
        return;
    }

    if(!observation_msg_ptr){
        throw std::runtime_error("Invaild observation_msg pointer.");
    }

    // Euclidean Clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_ptr_baseframe);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
    extractor.setClusterTolerance(static_obstacle_radius_);
    extractor.setMinClusterSize(1);
    extractor.setMaxClusterSize(200);
    extractor.setSearchMethod(tree);
    extractor.setInputCloud(cloud_ptr_baseframe);
    extractor.extract(cluster_indices);

    // ROS_WARN("num clusters: %ld", cluster_indices.size());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        PointCloudXYZPtr obj_pts(new PointCloudXYZ);
        
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            obj_pts->points.push_back(cloud_ptr_baseframe->points[*pit]);

        pcl::PointXYZ min_point, max_point;
        Eigen::Vector3f center;
        pcl::getMinMax3D(*obj_pts, min_point, max_point);
        center = (min_point.getVector3fMap() + max_point.getVector3fMap()) / 2.0;

        tf::Vector3 vec_baseframe(center[0], center[1], 0.0);
        // Ignore the distant obstacles
        if(vec_baseframe.length() <= 3.0) {
            tf::Vector3 vec_odom = tf_base2odom * vec_baseframe;
            walker_msgs::Trk3D obj_info;
            obj_info.x = vec_odom.getX();
            obj_info.y = vec_odom.getY();
            obj_info.vx = 0.0;
            obj_info.vy = 0.0;
            obj_info.radius = static_obstacle_radius_;
            observation_msg_ptr->trks_list.push_back(obj_info);
        }
    }
}


void Scan2ObservationNode::trk3d_cb(const walker_msgs::Trk3DArray::ConstPtr &msg_ptr) {
    if(msg_ptr->header.frame_id != "odom"){
        throw std::runtime_error("The frame_id of trk3d message must be 'odom'.");
    }

    // Get the transformation from tracking result frame to base frame
    tf::StampedTransform tf_trk2base;
    try{
        tflistener_ptr_->waitForTransform(base_frameid_, msg_ptr->header.frame_id,
                                    ros::Time(), ros::Duration(0.1));
        tflistener_ptr_->lookupTransform(base_frameid_, msg_ptr->header.frame_id,
                                    ros::Time(), tf_trk2base);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("\nCannot get TF from odom to %s: %s. Aborting...", base_frameid_.c_str(), ex.what());
        exit(-1);
    }
    // Get TF from odom2base
    tf::StampedTransform tf_base2odom(tf_trk2base.inverse(), tf_trk2base.stamp_, msg_ptr->header.frame_id, base_frameid_);

    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    sensor_msgs::LaserScan laser_msg = msg_ptr->scan;

    // Convert laserscan to pointcloud:  laserscan --> ROS PointCloud2 --> PCL PointCloudXYZ
    sensor_msgs::PointCloud2 cloud_msg;
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    PointCloudXYZPtr cloud_baseframe(new PointCloudXYZ);
    projector_.projectLaser(laser_msg, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);
    pcl_ros::transformPointCloud(*cloud_raw, *cloud_baseframe, tf_laser2base_);

    // Take human points away from laser scan data 
    for(int i = 0; i < msg_ptr->trks_list.size(); i++) {
        tf::Vector3 vec_trkframe(msg_ptr->trks_list[i].x, msg_ptr->trks_list[i].y, 0);
        tf::Vector3 vec_baseframe = tf_trk2base * vec_trkframe;
        pcl::CropBox<pcl::PointXYZ> human_boxcrop;
        human_boxcrop.setMax(Eigen::Vector4f(vec_baseframe.getX() + dynamic_obstacle_radius_, vec_baseframe.getY() + dynamic_obstacle_radius_, 5.0, 1.0));
        human_boxcrop.setMin(Eigen::Vector4f(vec_baseframe.getX() - dynamic_obstacle_radius_, vec_baseframe.getY() - dynamic_obstacle_radius_, -5.0, 1.0));
        human_boxcrop.setNegative(true);
        human_boxcrop.setInputCloud(cloud_baseframe);
        human_boxcrop.filter(*cloud_baseframe);
    }

    // Apply cropbox filter
    box_filter_.setInputCloud(cloud_baseframe);
    box_filter_.filter(*cloud_baseframe);
    // Apply voxel grid filter
    voxel_grid_.setInputCloud (cloud_baseframe);
    voxel_grid_.filter(*cloud_baseframe);

    // Pass trk3d result to observation msg directly
    walker_msgs::Trk3DArray::Ptr observation_msg_ptr(new walker_msgs::Trk3DArray());
    observation_msg_ptr->trks_list.insert(observation_msg_ptr->trks_list.begin(), msg_ptr->trks_list.begin(), msg_ptr->trks_list.end());
    convert_scan_to_observations(observation_msg_ptr, cloud_baseframe, tf_base2odom);

    observation_msg_ptr->header.stamp = ros::Time(0);
    observation_msg_ptr->header.frame_id = odom_frameid_;
    pub_observation_.publish(observation_msg_ptr);

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}


void Scan2ObservationNode::scan_cb(const sensor_msgs::LaserScan &laser_msg) {
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    tf::StampedTransform tf_base2odom;
    try{
        tflistener_ptr_->waitForTransform(odom_frameid_, base_frameid_,
                                    ros::Time(), ros::Duration(0.1));
        tflistener_ptr_->lookupTransform(odom_frameid_, base_frameid_,
                                    ros::Time(), tf_base2odom);
    }catch (tf::TransformException ex){
        ROS_ERROR("\nCannot get TF from %s to %s: %s. Aborting...", base_frameid_.c_str(), odom_frameid_.c_str(), ex.what());
        exit(-1);
    }

    // Convert laserscan to pointcloud:  laserscan --> ROS PointCloud2 --> PCL PointCloudXYZ
    sensor_msgs::PointCloud2 cloud_msg;
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    PointCloudXYZPtr cloud_baseframe(new PointCloudXYZ);
    projector_.projectLaser(laser_msg, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);
    pcl_ros::transformPointCloud(*cloud_raw, *cloud_baseframe, tf_laser2base_);

    // Apply cropbox filter
    box_filter_.setInputCloud(cloud_baseframe);
    box_filter_.filter(*cloud_baseframe);
    // Apply voxel grid filter
    voxel_grid_.setInputCloud (cloud_baseframe);
    voxel_grid_.filter(*cloud_baseframe);

    if(pub_pc_filtered_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_baseframe, cloud_msg);
        cloud_msg.header.frame_id = base_frameid_;
        pub_pc_filtered_.publish(cloud_msg);
    }

    walker_msgs::Trk3DArray::Ptr observation_msg_ptr(new walker_msgs::Trk3DArray());
    convert_scan_to_observations(observation_msg_ptr, cloud_baseframe, tf_base2odom);

    observation_msg_ptr->header.stamp = ros::Time(0);
    observation_msg_ptr->header.frame_id = odom_frameid_;
    pub_observation_.publish(observation_msg_ptr);

    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}


void Scan2ObservationNode::sigint_cb(int sig) {
    ROS_INFO_STREAM("Node name: " << ros::this_node::getName() << " is shutdown.");
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "scan2observation_node");
    ros::NodeHandle nh, pnh("~");
    Scan2ObservationNode node(nh, pnh);    
    ros::spin();
    return 0;
}
