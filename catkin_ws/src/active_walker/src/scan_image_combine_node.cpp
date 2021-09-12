#include <iostream>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <laser_geometry/laser_geometry.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// Custom msg & srv
#include <walker_msgs/Detection2DTrigger.h>
#include <walker_msgs/Det3D.h>
#include <walker_msgs/Det3DArray.h>

// Message filter
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

// Eigen
#include <Eigen/Dense>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// TF
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h> // Centroid
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h> // ros2pcl
#include <pcl/filters/radius_outlier_removal.h> // RemoveOutlier
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>

// Linear assignment library
#include "Hungarian.h"


typedef message_filters::sync_policies::ApproximateTime<cv_bridge::CvImage, sensor_msgs::LaserScan> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> MySynchronizer;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

// Just for color words display
static const std::string COLOR_RED = "\e[0;31m";
static const std::string COLOR_GREEN = "\e[0;32m";
static const std::string COLOR_YELLOW = "\e[0;33m"; 
static const std::string COLOR_NC = "\e[0m";

static const int kNumOfInterestClass = 1;
static const std::string kInterestClassNames[kNumOfInterestClass] = {"person"};
static const double kMaxDimOfLaserCluster = 1.2;   // Consider the cluster would be merged if two people are too close
static const double kThresholdOfSimilarity = 0.95;
static const double kThresholdOfUnreasonableHeight = 3.0;
static const double kLifetimeOfMarker = 0.1;
static const double kMinLaserClusterTolerance = 0.3;

template <typename T, typename A>
int arg_max(std::vector<T, A> const& vec) {
    return static_cast<int>(std::distance(vec.begin(), max_element(vec.begin(), vec.end())));
}

template <typename T, typename A>
int arg_min(std::vector<T, A> const& vec) {
    return static_cast<int>(std::distance(vec.begin(), min_element(vec.begin(), vec.end())));
}

class ObjInfo {
public:
    ObjInfo(){
        cloud = PointCloudXYZPtr(new PointCloudXYZ);
        radius = 0.0;
    }
    walker_msgs::BBox2D box;        // id, class_name, score, center, size_x, size_y
    PointCloudXYZPtr cloud;

    geometry_msgs::Point location;
    geometry_msgs::Vector3 key_vector;
    double radius;
};


class LaserClusterInfo {
public:
    LaserClusterInfo(){
        cloud = PointCloudXYZPtr(new PointCloudXYZ);
        is_in_fov = false;
    }
    PointCloudXYZPtr cloud;
    bool is_in_fov;
    double dimension_2d;                        // sqrt(W^2 + L^2)
    geometry_msgs::Point location;              // Center of cluster
    geometry_msgs::Vector3 key_vec_imgspace;
    // geometry_msgs::Vector3 key_vec_laserspace;       // deprecated
};


class ScanImageCombineNode {
public:
    ScanImageCombineNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    void img_scan_cb(const cv_bridge::CvImage::ConstPtr &cv_ptr, const sensor_msgs::LaserScan::ConstPtr &laser_msg_ptr);
    void separate_outlier_points(PointCloudXYZPtr cloud_in, PointCloudXYZPtr cloud_out, bool is_far);
    bool is_interest_class(std::string class_name);
    double cosine_similarity_2d(geometry_msgs::Vector3 vec_a, geometry_msgs::Vector3 vec_b);
    double calculate_distance_cost(geometry_msgs::Point location);
    tf::Vector3 point_pixel2laser(double pixel_x, double pixel_y, double depth_from_laser);
    cv::Point2d point_laser2pixel(double x_from_laser, double y_from_laser, double z_from_laser);

    // Transformation
    tf::Matrix3x3 rot_laser2cam_;
    tf::Vector3 tras_laser2cam_;
    tf::Matrix3x3 rot_cam2laser_;
    tf::Vector3 tras_cam2laser_;

    // Elevation angle for object height recovering
    double camera_mount_elevation_angle_;

    // Camera distortion coefficients
    cv::Mat K_;
    cv::Mat D_;

    // ROS related
    ros::NodeHandle nh_, pnh_;
    tf::TransformListener tf_listener_;
    laser_geometry::LaserProjection projector_;
    ros::Publisher pub_combined_image_;
    ros::Publisher pub_detection_image_;
    ros::Publisher pub_marker_array_;
    // ros::Publisher pub_debug_mrk_array_;
    ros::Publisher pub_colored_pc_;
    ros::Publisher pub_detection3d_;
    ros::ServiceClient yolov4_detect_;  // ROS Service client
    // Message filters
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
    message_filters::Subscriber<cv_bridge::CvImage> image_sub_;
    boost::shared_ptr<MySynchronizer> sync_;

    // Object list
    std::vector<ObjInfo> obj_list_;

    bool flag_det_vis_;
};


//    __   __        __  ___  __        __  ___  __   __  
//   /  ` /  \ |\ | /__`  |  |__) |  | /  `  |  /  \ |__) 
//   \__, \__/ | \| .__/  |  |  \ \__/ \__,  |  \__/ |  \ 
//  
ScanImageCombineNode::ScanImageCombineNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh) {
    // ROS parameters
    std::string scan_topic;
    std::string img_topic;
    std::string caminfo_topic;
    std::string yolo_srv_name = "yolov4_node/yolo_detect";
    ros::param::param<std::string>("~scan_topic", scan_topic, "scan");
    ros::param::param<std::string>("~img_topic", img_topic, "usb_cam/image_raw"); 
    ros::param::param<std::string>("~caminfo_topic", caminfo_topic, "usb_cam/camera_info");
    ros::param::param<bool>("~flag_det_vis", flag_det_vis_, false);

    // ROS publisher & subscriber & message filter
    pub_combined_image_ = nh_.advertise<sensor_msgs::Image>("debug_reprojection", 1);
    pub_detection_image_ = nh_.advertise<sensor_msgs::Image>("detection_image", 1);
    if(flag_det_vis_) {
        pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>("obj_marker", 1);
        // pub_debug_mrk_array_ = nh.advertise<visualization_msgs::MarkerArray>("debug_marker", 1);
        pub_colored_pc_ = nh.advertise<sensor_msgs::PointCloud2>("colored_pc", 1);
    }
    pub_detection3d_ = nh.advertise<walker_msgs::Det3DArray>("det3d_result", 1);
    scan_sub_.subscribe(nh_, scan_topic, 1);
    image_sub_.subscribe(nh_, img_topic, 1);
    sync_.reset(new MySynchronizer(MySyncPolicy(10), image_sub_, scan_sub_));
    sync_->registerCallback(boost::bind(&ScanImageCombineNode::img_scan_cb, this, _1, _2));

    // ROS service client
    ROS_INFO_STREAM("Wait for yolo detection service in 20 seconds...");
    if(!ros::service::waitForService(yolo_srv_name, ros::Duration(20.0))) {
        ROS_ERROR("Cannot get the detection service: %s. Aborting...", yolo_srv_name.c_str());
        exit(-1);
    }
    yolov4_detect_ = nh_.serviceClient<walker_msgs::Detection2DTrigger>(yolo_srv_name);

    // To get the image frame_id from an image topic
    std::string image_frame;
    boost::shared_ptr<sensor_msgs::Image const> tmp_img_ptr;
    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] Wait for a image message in 5 seconds");
    tmp_img_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(img_topic, ros::Duration(5.0));
    if(tmp_img_ptr != NULL){
        image_frame = tmp_img_ptr->header.frame_id;
        ROS_INFO("Image topic frame_id: %s", image_frame.c_str());
    }
    else{
        image_frame = "camera_link";
        ROS_WARN("Cannot get any image topic, set default image frame_id: %s", image_frame.c_str());
    }
    tmp_img_ptr.reset();

    // Prepare extrinsic matrix
    tf::StampedTransform stamped_transform;
    try{
        tf_listener_.waitForTransform(image_frame, "laser_link",
                                    ros::Time(0), ros::Duration(10.0));
        tf_listener_.lookupTransform(image_frame, "laser_link", 
                                    ros::Time(0), stamped_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Cannot get TF from camera to laserscan: %s. Aborting...", ex.what());
        exit(-1);
    }
    rot_laser2cam_ = tf::Matrix3x3(stamped_transform.getRotation());
    tras_laser2cam_ = stamped_transform.getOrigin();
    
    // Extrinsic matrix inversion
    rot_cam2laser_ = rot_laser2cam_.transpose();
    tras_cam2laser_ = rot_cam2laser_ * tras_laser2cam_ * (-1);

    // 
    double tmp_roll, tmp_pitch, tmp_yaw;
    rot_laser2cam_.getRPY(tmp_roll, tmp_pitch, tmp_yaw);
    camera_mount_elevation_angle_ = 90.0 - std::fabs(tmp_pitch);

    // Prepare intrinsic matrix
    boost::shared_ptr<sensor_msgs::CameraInfo const> caminfo_ptr;
    double fx, fy, cx, cy;
    double k1, k2, p1, p2;
    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] Wait for a camera_info message in 10 seconds");
    caminfo_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(caminfo_topic, ros::Duration(10.0));
    if(caminfo_ptr != NULL){       
        fx = caminfo_ptr->P[0];
        fy = caminfo_ptr->P[5];
        cx = caminfo_ptr->P[2];
        cy = caminfo_ptr->P[6];

        k1 = caminfo_ptr->D[0];
        k2 = caminfo_ptr->D[1];
        p1 = caminfo_ptr->D[2];
        p2 = caminfo_ptr->D[3];
    }else {
        ROS_WARN_STREAM("[" << ros::this_node::getName() << "] No camera_info received, use default values");
        fx = 518.34283;
        fy = 522.27271;
        cx = 305.42936;
        cy = 244.1336;

        k1 = 0.052152;
        k2 = -0.122459;
        p1 = -0.003933;
        p2 = -0.005683;
    }
    K_ = (cv::Mat_<double>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.);
    D_ = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, 0.0);
    // cout << "K:\n" << K_ << endl;
    // cout << "D:\n" << D_ << endl;

    ROS_INFO_STREAM(COLOR_GREEN << ros::this_node::getName() << " is ready." << COLOR_NC);
}


double ScanImageCombineNode::calculate_distance_cost(geometry_msgs::Point location) {
    return std::floor(std::hypot(location.x, location.y) / 2.0);
}


double ScanImageCombineNode::cosine_similarity_2d(geometry_msgs::Vector3 vec_a, geometry_msgs::Vector3 vec_b) {
    double product_result = vec_a.x * vec_b.x + vec_a.y * vec_b.y;
    double len_a2 = vec_a.x * vec_a.x + vec_a.y * vec_a.y;
    double len_b2 = vec_b.x * vec_b.x + vec_b.y * vec_b.y;

    if(len_a2 == 0.0 || len_b2 == 0.0)
        throw std::runtime_error("cosine similarity is not defined whenever one or both input vectors are zero-vectors.");

    return (product_result / std::sqrt(len_a2 * len_b2));
}


void ScanImageCombineNode::separate_outlier_points(PointCloudXYZPtr cloud_in, PointCloudXYZPtr cloud_out, bool is_far) {
    // Euclidean Cluster Extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_in);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euler_extractor;
    euler_extractor.setClusterTolerance(0.2);
    euler_extractor.setMinClusterSize(1);
    euler_extractor.setMaxClusterSize(100);  // need to check the max pointcloud size of each object
    euler_extractor.setSearchMethod(tree);
    euler_extractor.setInputCloud(cloud_in);
    euler_extractor.extract(cluster_indices);

    // Find the cloud cluster which is closest to ego
    int idx_proper_cloud = 0;
    std::vector<float> candidates;
    for(int i = 0; i < cluster_indices.size(); i++) {
        Eigen::Vector3f centroid;
        centroid << 0, 0, 0;
        for(int j = 0; j < cluster_indices[i].indices.size(); j++) {
            centroid[0] += cloud_in->points[cluster_indices[i].indices[j]].x;
            centroid[1] += cloud_in->points[cluster_indices[i].indices[j]].y;
            centroid[2] += cloud_in->points[cluster_indices[i].indices[j]].z;
        }
        centroid /= cluster_indices[i].indices.size();
        candidates.push_back(centroid.norm());
    }

    // ROS_ERROR("Cluster num: %d, is_far: %s, ", candidates.size(), (is_far)? "true": "false");    
    if(is_far && candidates.size() > 1){
        int idx_closest = arg_min(candidates);
        cluster_indices.erase(cluster_indices.begin() + idx_closest);
        candidates.erase(candidates.begin() + idx_closest);
        idx_proper_cloud = arg_min(candidates);
    }else{
        idx_proper_cloud = arg_min(candidates);
    }
    // ROS_ERROR("Proper candidates: %.2f\n", candidates[idx_proper_cloud]);
    
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices(cluster_indices[idx_proper_cloud]));
    PointCloudXYZPtr cloud_extracted(new PointCloudXYZ);
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(cloud_in);
    extractor.setIndices(inliers_ptr);
    extractor.setNegative(false);
    extractor.filter(*cloud_extracted);

    // Remove outlier
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_extracted);
    outrem.setRadiusSearch(0.2);
    outrem.setMinNeighborsInRadius(2);
    outrem.filter(*(cloud_out));

    pcl::copyPointCloud(*cloud_in, *cloud_out);                  
}


bool ScanImageCombineNode::is_interest_class(std::string class_name){
    for(int i = 0; i < kNumOfInterestClass; i++) {
        if(strcmp(class_name.c_str(), kInterestClassNames[i].c_str()) == 0)
            return true;
    }
    return false;
}


tf::Vector3 ScanImageCombineNode::point_pixel2laser(double pixel_x, double pixel_y, double depth_from_laser) {
    cv::Mat cv_pt = (cv::Mat_<double>(3, 1) << pixel_x, pixel_y, 1.0);
    
    // Transform to camera frame
    cv::Mat pt_camframe = K_.inv() * cv_pt * depth_from_laser;

    // Transform to laser frame
    tf::Vector3 pt_laserframe = tras_cam2laser_ + rot_cam2laser_ * tf::Vector3(pt_camframe.at<double>(0, 0), 
                                                                                pt_camframe.at<double>(1, 0),
                                                                                pt_camframe.at<double>(2, 0));
    return pt_laserframe;
}


cv::Point2d ScanImageCombineNode::point_laser2pixel(double x_from_laser, double y_from_laser, double z_from_laser) {
    // Transform to camera frame
    tf::Vector3 pt_laserframe(x_from_laser, y_from_laser, z_from_laser); 
    tf::Vector3 pt_camframe = rot_laser2cam_ * pt_laserframe + tras_laser2cam_;
    if(pt_camframe.getZ() <= 0.0) // points behind ego
        return cv::Point2d(-1, -1);

    // Normalization: z --> 1
    pt_camframe.setX(pt_camframe.getX() / pt_camframe.getZ());
    pt_camframe.setY(pt_camframe.getY() / pt_camframe.getZ());
    pt_camframe.setZ(1.0);

    // Trasform to pixel frame
    cv::Mat uv = K_ * (cv::Mat_<double>(3, 1) << pt_camframe.getX(), pt_camframe.getY(), pt_camframe.getZ());
    cv::Point2d pt_pixelframe(uv.at<double>(0, 0), uv.at<double>(1, 0));

    return pt_pixelframe;
}


void ScanImageCombineNode::img_scan_cb(const cv_bridge::CvImage::ConstPtr &cv_ptr, const sensor_msgs::LaserScan::ConstPtr &laser_msg_ptr){
    // Object list init
    obj_list_.clear();
    // Visualization msg
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::MarkerArray debug_mrk_array;
    // Detection result message
    walker_msgs::Det3DArray detection_array;

    // Call 2D bounding box detection service
    walker_msgs::Detection2DTrigger srv;
    srv.request.image = *(cv_ptr->toImageMsg());
    if(!yolov4_detect_.call(srv)){
        ROS_ERROR("Failed to call service");
        return;
    }
    
    double kImageWidth = cv_ptr->image.cols;
    double kImageHeight = cv_ptr->image.rows;

    // Collect all interest classes to obj_list_
    // char det_str[200] = {0};
    std::vector<walker_msgs::BBox2D> boxes = srv.response.result.boxes;
    for(int i = 0; i < boxes.size(); i++) {
        if(is_interest_class(boxes[i].class_name)) {
            // Skip the box which is too small
            if(boxes[i].size_y < 80)
                continue;

            ObjInfo obj_info;
            obj_info.box = boxes[i];

            // Prepare key vector for linear assignment later
            obj_info.key_vector.x = boxes[i].center.x - kImageWidth / 2;
            obj_info.key_vector.y = -(boxes[i].center.y - kImageHeight);
            obj_list_.push_back(obj_info);

            // sprintf(det_str, "%s%.2f, ", det_str, std::atan2(obj_info.key_vector.y, obj_info.key_vector.x) / M_PI * 180.0);
        }
    }
    // ROS_INFO("%s", det_str);

    // Reconstruct undistorted cvimage from detection result image
    cv::Mat cvimage;
    cv_bridge::CvImagePtr detected_cv_ptr = cv_bridge::toCvCopy(srv.response.result.result_image);
    cv::undistort(detected_cv_ptr->image, cvimage, K_, D_);
    
    // Convert laserscan to pointcloud:  laserscan --> ROS PointCloud2 --> PCL PointCloudXYZ
    sensor_msgs::PointCloud2 cloud_msg;
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    projector_.projectLaser(*laser_msg_ptr, cloud_msg);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);
 
    // Color pointcloud to visaulize detected points
    PointCloudXYZRGBPtr cloud_colored(new PointCloudXYZRGB);

    // Convert laserscan points to pixel points
    std::vector<cv::Point2d> pts_uv;
    std::vector<int> roi_pts_indices;
    for (int i = 0; i < cloud_raw->points.size(); ++i) {
        cv::Point2d pt_uv = point_laser2pixel(cloud_raw->points[i].x, cloud_raw->points[i].y, cloud_raw->points[i].z); 
        if(pt_uv.x < 0 || pt_uv.y < 0 || pt_uv.x > kImageWidth || pt_uv.y > kImageHeight)
            continue;
        pts_uv.push_back(pt_uv);
        roi_pts_indices.push_back(i);       
    }

    // Check if cloud size > 0, then continue the image and laser fusion process
    if(cloud_raw->points.size() > 0) {
        // Euclidean clustering for laser points
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_raw);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
        extractor.setClusterTolerance(kMinLaserClusterTolerance);     // normal pedestrian dimension
        extractor.setMinClusterSize(2);
        extractor.setMaxClusterSize(1000);      // need to check the max pointcloud size of each object
        extractor.setSearchMethod(tree);
        extractor.setInputCloud(cloud_raw);
        extractor.extract(cluster_indices);

        std::vector<LaserClusterInfo> laser_clusters_list;
        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

            LaserClusterInfo laser_cluster;
            for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
                laser_cluster.cloud->points.push_back(cloud_raw->points[*pit]);

                // Check whether the cluster is in camera FOV
                if(laser_cluster.is_in_fov == false && std::count(roi_pts_indices.begin(), roi_pts_indices.end(), *pit))
                    laser_cluster.is_in_fov = true;
            }

            pcl::PointXYZ min_point, max_point;
            Eigen::Vector3f center;
            pcl::getMinMax3D(*(laser_cluster.cloud), min_point, max_point);
            center = (min_point.getVector3fMap() + max_point.getVector3fMap()) / 2.0;
            laser_cluster.location.x = center[0];
            laser_cluster.location.y = center[1];
            laser_cluster.dimension_2d = std::hypot(min_point.x - max_point.x, min_point.y - max_point.y);
            // laser_cluster.key_vec_laserspace.x = center[1];          // deprecated
            // laser_cluster.key_vec_laserspace.y = -center[0];         // deprecated

            // Ignore too large cloud, it is guessed as background
            if(laser_cluster.dimension_2d > kMaxDimOfLaserCluster)
                laser_cluster.is_in_fov = false;

            if(laser_cluster.is_in_fov == true) {

                // test20201124
                cv::Point2d pt_uv2 = point_laser2pixel(laser_cluster.location.x, laser_cluster.location.y, 0.0);
                laser_cluster.key_vec_imgspace.x = pt_uv2.x - kImageWidth / 2;
                laser_cluster.key_vec_imgspace.y = -(pt_uv2.y - kImageHeight);
                // pts_uv2_list.push_back(pt_uv2);

                laser_clusters_list.push_back(laser_cluster);

                // Visualization 
                // visualization_msgs::Marker marker;
                // marker.header.frame_id = laser_msg_ptr->header.frame_id;
                // marker.header.stamp = ros::Time();
                // marker.ns = "debug1";
                // marker.id = it - cluster_indices.begin();
                // marker.type = visualization_msgs::Marker::LINE_STRIP;
                // marker.lifetime = ros::Duration(kLifetimeOfMarker);
                // marker.action = visualization_msgs::Marker::ADD;
                // geometry_msgs::Point tmp_pt;
                // tmp_pt.x = min_point.x;
                // tmp_pt.y = min_point.y;
                // marker.points.push_back(tmp_pt);
                // tmp_pt.x = max_point.x;
                // tmp_pt.y = max_point.y;
                // marker.points.push_back(tmp_pt);
                // marker.scale.x = 0.1;
                // marker.pose.orientation.x = 0.0;
                // marker.pose.orientation.y = 0.0;
                // marker.pose.orientation.z = 0.0;
                // marker.pose.orientation.w = 1.0;
                // marker.color.a = 0.5;
                // marker.color.r = 1.0;
                // marker.color.b = 1.0;
                // debug_mrk_array.markers.push_back(marker);

                // visualization_msgs::Marker marker2;
                // marker2.header.frame_id = laser_msg_ptr->header.frame_id;
                // marker2.header.stamp = ros::Time();
                // marker2.ns = "debug2";
                // marker2.id = it - cluster_indices.begin();
                // marker2.type = visualization_msgs::Marker::SPHERE;
                // marker2.lifetime = ros::Duration(kLifetimeOfMarker);
                // marker2.action = visualization_msgs::Marker::ADD;
                // marker2.scale.x = 0.5;
                // marker2.scale.y = 0.5;
                // marker2.scale.z = 0.5;
                // marker2.pose.position.x = center[0];
                // marker2.pose.position.y = center[1];
                // marker2.pose.orientation.x = 0.0;
                // marker2.pose.orientation.y = 0.0;
                // marker2.pose.orientation.z = 0.0;
                // marker2.pose.orientation.w = 1.0;
                // marker2.color.a = 0.5;
                // marker2.color.b = 1.0;
                // debug_mrk_array.markers.push_back(marker2);

                // visualization_msgs::Marker marker3;
                // marker3.header.frame_id = laser_msg_ptr->header.frame_id;
                // marker3.header.stamp = ros::Time();
                // marker3.ns = "debug3";
                // marker3.id = it - cluster_indices.begin();
                // marker3.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                // marker3.lifetime = ros::Duration(kLifetimeOfMarker);
                // marker3.action = visualization_msgs::Marker::ADD;
                // marker3.scale.z = 0.5;
                // marker3.pose.position.x = center[0];
                // marker3.pose.position.y = center[1];
                // marker3.pose.position.z = 1.0;
                // marker3.pose.orientation.x = 0.0;
                // marker3.pose.orientation.y = 0.0;
                // marker3.pose.orientation.z = 0.0;
                // marker3.pose.orientation.w = 1.0;
                // marker3.color.a = 1.0;
                // marker3.color.r = 1.0;
                // marker3.color.g = 1.0;
                // marker3.text = std::to_string(laser_cluster.dimension_2d);
                // // marker3.text = std::to_string(std::atan2(laser_cluster.key_vec_imgspace.y, laser_cluster.key_vec_imgspace.x) / M_PI * 180.0);
                // debug_mrk_array.markers.push_back(marker3);
            }
        }
        // pub_debug_mrk_array_.publish(debug_mrk_array);

        

        // Prepare cost matrix for linear assignment
        if(obj_list_.size() != 0 && laser_clusters_list.size() != 0) {
            std::vector< std::vector<double> > cost_matrix(obj_list_.size(), std::vector<double>(laser_clusters_list.size(), 0.0));
            for(int i = 0; i < obj_list_.size(); i++){
                for(int j = 0; j < laser_clusters_list.size(); j++){
                    // Consider the distance 
                    double distance_cost = calculate_distance_cost(laser_clusters_list[j].location);
                    
                    double similarity = cosine_similarity_2d(obj_list_[i].key_vector, laser_clusters_list[j].key_vec_imgspace);
                    double orientation_cost = (similarity >= kThresholdOfSimilarity)? 1.0 - similarity : 100.0; 
                    cost_matrix[i][j] = distance_cost + orientation_cost;
                }
            }

            // Main linear assignment part
            HungarianAlgorithm HungAlgo;
            vector<int> assignment;
            double cost = HungAlgo.Solve(cost_matrix, assignment);
            for (unsigned int i = 0; i < cost_matrix.size(); i++){
                if(assignment[i] != -1 && cost_matrix[i][assignment[i]] < 100)     // -1 means there is no assignment solution for this item 
                    obj_list_[i].cloud = laser_clusters_list[assignment[i]].cloud;
            }

            for(int i = 0; i < obj_list_.size(); i++) {
                if(obj_list_[i].cloud->points.size() >= 1){

                    // Find the center of each object
                    pcl::PointXYZ min_point, max_point;
                    Eigen::Vector3f center;
                    pcl::getMinMax3D(*(obj_list_[i].cloud), min_point, max_point);
                    center = (min_point.getVector3fMap() + max_point.getVector3fMap()) / 2.0;
                    obj_list_[i].location.x = center[0];
                    obj_list_[i].location.y = center[1];

                    // Estimate the object radius
                    double tmp_r1 = sqrt(pow(max_point.x - center[0], 2) + pow(max_point.y - center[1], 2));
                    double tmp_r2 = sqrt(pow(min_point.x - center[0], 2) + pow(min_point.y - center[1], 2));
                    obj_list_[i].radius = std::max(tmp_r1, tmp_r2);

                    // Recover object dimension from image
                    tf::Vector3 lefttop_laserframe = point_pixel2laser(obj_list_[i].box.center.x - obj_list_[i].box.size_x / 2, 
                                                                        obj_list_[i].box.center.y - obj_list_[i].box.size_y / 2,
                                                                        obj_list_[i].location.x);
                    tf::Vector3 righttop_laserframe = point_pixel2laser(obj_list_[i].box.center.x + obj_list_[i].box.size_x / 2, 
                                                                        obj_list_[i].box.center.y - obj_list_[i].box.size_y / 2,
                                                                        obj_list_[i].location.x);
                    tf::Vector3 rightbottom_laserframe = point_pixel2laser(obj_list_[i].box.center.x + obj_list_[i].box.size_x / 2, 
                                                                        obj_list_[i].box.center.y + obj_list_[i].box.size_y / 2,
                                                                        obj_list_[i].location.x);
                    double h_from_image = fabs(rightbottom_laserframe.getZ() - righttop_laserframe.getZ());
                    double w_from_image = fabs(lefttop_laserframe.getY() - righttop_laserframe.getY());
                    // tf::Vector3 center_from_image = point_pixel2laser(obj_list_[i].box.center.x, 
                    //                                                     obj_list_[i].box.center.y,
                    //                                                     obj_list_[i].location.x);
                    double radius_from_image = fabs(lefttop_laserframe.getY() - righttop_laserframe.getY()) / 2;

                    // Skip the match result which has an unreasonable height
                    if(h_from_image > kThresholdOfUnreasonableHeight) continue;

                    // Pack the custom ros package
                    walker_msgs::Det3D det_msg;
                    det_msg.x = obj_list_[i].location.x;
                    det_msg.y = obj_list_[i].location.y;
                    det_msg.z = 0;
                    det_msg.yaw = 0;
                    det_msg.radius = radius_from_image;
                    det_msg.h = h_from_image;           // Additional
                    det_msg.w = w_from_image;           // Additional
                    det_msg.l = w_from_image;           // Additional
                    det_msg.confidence = obj_list_[i].box.score;
                    det_msg.class_name = obj_list_[i].box.class_name;
                    det_msg.class_id = obj_list_[i].box.id;
                    detection_array.dets_list.push_back(det_msg);

                    // Visualization
                    if(flag_det_vis_) {
                        visualization_msgs::Marker marker;
                        marker.header.frame_id = laser_msg_ptr->header.frame_id;
                        marker.header.stamp = ros::Time();
                        marker.ns = "detection_result";
                        marker.id = i;
                        marker.type = visualization_msgs::Marker::CUBE;
                        marker.lifetime = ros::Duration(kLifetimeOfMarker);
                        // marker.lifetime = ros::Duration(10.0);
                        marker.action = visualization_msgs::Marker::ADD;
                        marker.pose.position.x = obj_list_[i].location.x;
                        marker.pose.position.y = obj_list_[i].location.y;
                        marker.pose.position.z = rightbottom_laserframe.getZ() * std::cos(camera_mount_elevation_angle_);
                        marker.pose.orientation.x = 0.0;
                        marker.pose.orientation.y = 0.0;
                        marker.pose.orientation.z = 0.0;
                        marker.pose.orientation.w = 1.0;
                        // marker.scale.x = marker.scale.y = obj_list_[i].radius * 2;
                        marker.scale.x = w_from_image;
                        marker.scale.y = w_from_image;
                        marker.scale.z = h_from_image;
                        marker.color.a = 0.4;
                        marker.color.g = 1.0;
                        marker_array.markers.push_back(marker);
                    }
                }
            }
        }   // end if check prerequisite for the linear assignment
    } // end if check cloud size > 0
    
    // Publish visualization topics
    if(flag_det_vis_ && pub_marker_array_.getNumSubscribers() > 0)
        pub_marker_array_.publish(marker_array);
    if(flag_det_vis_ && pub_colored_pc_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 colored_cloud_msg;
        pcl::toROSMsg(*cloud_colored, colored_cloud_msg);
        colored_cloud_msg.header.frame_id = "laser_link";
        pub_colored_pc_.publish(colored_cloud_msg);
    }

    if(pub_detection_image_.getNumSubscribers() > 0){
        cv_bridge::CvImage result_image(cv_ptr->header, "rgb8", cvimage);
        pub_detection_image_.publish(result_image.toImageMsg());
    }

    if(pub_combined_image_.getNumSubscribers() > 0){
        // Draw points in images
        for (int j = 0; j < pts_uv.size(); ++j)
            cv::circle(cvimage, pts_uv[j], 2, cv::Scalar(255, 0, 0), -1);

        // Classify the human points (green) and static obstacle (red) 
        // for (int i = 0; i < pts_uv.size(); ++i){
        //     bool flag_human_pt = false;
        //     for(int j = 0; j < obj_list_.size(); j++) {
        //         for(int k = 0; k < obj_list_[j].cloud->points.size(); k++){
        //             cv::Point2d obj_pt2d = point_laser2pixel(obj_list_[j].cloud->points[k].x, obj_list_[j].cloud->points[k].y, 0.0);
        //             if(pts_uv[i] == obj_pt2d){
        //                 flag_human_pt = true;
        //                 break;
        //             }
        //         }
        //         if(flag_human_pt == true) break;
        //     }
        //     if(flag_human_pt)
        //         cv::circle(cvimage, pts_uv[i], 2, cv::Scalar(0, 255, 0), -1);
        //     else
        //         cv::circle(cvimage, pts_uv[i], 2, cv::Scalar(255, 0, 0), -1);
        // }

        // for (int j = 0; j < pts_uv2_list.size(); ++j)
        //     cv::circle(cvimage, pts_uv2_list[j], 5, cv::Scalar(255, 0, 0), -1);

        cv_bridge::CvImage result_image(cv_ptr->header, "rgb8", cvimage);
        pub_combined_image_.publish(result_image.toImageMsg());
    }

    // Publish detection result
    if(pub_detection3d_.getNumSubscribers() > 0) {
        detection_array.header.frame_id = laser_msg_ptr->header.frame_id;
        detection_array.header.stamp = ros::Time::now();
        detection_array.scan = *laser_msg_ptr;
        pub_detection3d_.publish(detection_array);
    }

    // Show object infomation
    // if(obj_list_.size() > 0){
    //     std::cout << "Prediction result:" << std::endl;
    //     for(int i = 0; i < obj_list_.size(); i++) {
    //         std::cout << obj_list_[i].box.class_name << ", cloud size: " << obj_list_[i].cloud->points.size() << std::endl; 
    //     }
    //     std::cout << "\n===================" << std::endl;
    // }
}



//            
//   |\/|  /\  | |\ | 
//   |  | /~~\ | | \| 
//  
int main(int argc, char **argv) {
    ros::init(argc, argv, "scan_clustering_node");
    ros::NodeHandle nh, pnh("~");
    ScanImageCombineNode node(nh, pnh);
    ros::spin();
    return 0;
}
