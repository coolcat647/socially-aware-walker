#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>

// For ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "a_star.hpp"

static const std::string FRAME_ID = "base_link";
static const std::string COLOR_RED = "\e[0;31m";
static const std::string COLOR_GREEN = "\e[0;32m";
static const std::string COLOR_YELLOW = "\e[0;33m"; 
static const std::string COLOR_NC = "\e[0m";

static const int kThresObstacleDangerCost = 80;

class FakeMapNode {
public:
    FakeMapNode();
    void timer_cb(const ros::TimerEvent& event);
    void gauss_filter(std::vector<int8_t> &vec, int width, int height, int index, int kernel_size, int peak_value);
    // ~FakeMapNode();
    ros::NodeHandle nh_;                            // Private node handler
    ros::Timer timer_;
    ros::Publisher pub_map_;
    ros::Publisher pub_path_;

    nav_msgs::MapMetaData mapinfo_;
    nav_msgs::OccupancyGrid map_msg_;

    astar::Solver solver_;
};

FakeMapNode::FakeMapNode(){
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("obs_map", 10);
    pub_path_ = nh_.advertise<nav_msgs::Path>("path_test", 10);
    
    // Map metadata
    mapinfo_.resolution = 0.2;
    mapinfo_.width = 40;
    mapinfo_.height = 40;
    mapinfo_.origin.position.x = -mapinfo_.resolution * mapinfo_.width / 2;
    mapinfo_.origin.position.y = -mapinfo_.resolution * mapinfo_.height / 2;
    mapinfo_.origin.orientation.w = 1.0;
    
    map_msg_.header.stamp = ros::Time::now();
    map_msg_.header.frame_id = FRAME_ID;
    map_msg_.info = mapinfo_;
    std::vector<int8_t> tmp_v(mapinfo_.width * mapinfo_.height, 0);
    map_msg_.data = tmp_v;

    solver_ = astar::Solver(nh_, true, kThresObstacleDangerCost, 0.6, 0.6);

    timer_ = nh_.createTimer(ros::Duration(1.0), &FakeMapNode::timer_cb, this);
    ROS_INFO_STREAM(ros::this_node::getName() << " is ready.");
}

void FakeMapNode::gauss_filter(std::vector<int8_t> &vec, int width, int height, int idx, int kernel_size, int peak_value) { 
    // Intialising standard deviation to 1.0 
    double sigma = 1.0; 
    double r, s = 2.0 * sigma * sigma;     

    // Variables for normalization 
    double sum = 0.0; 
    double kernel_peak = 1 / (M_PI * s);
    
    double kernel[kernel_size][kernel_size]; 
    int bound = kernel_size/2;

    // Generate Gaussian kernel 
    for(int x = -bound; x <= bound; x++) { 
        for (int y = -bound; y <= bound; y++) { 
            r = sqrt(x * x + y * y);
            kernel[x + bound][y + bound] = (exp(-(r * r) / s)) / (M_PI * s);
            // sum += kernel[x + 2][y + 2];
        }
    }

    // Normalize
    for(int i = 0; i < kernel_size; ++i){
        for (int j = 0; j < kernel_size; ++j) {
            // kernel[i][j] = kernel[i][j] / sum * peak_value;      // Norm by sum
            kernel[i][j] = kernel[i][j] / kernel_peak * peak_value; // Norm by kernel maximum
        }
    }

    // Apply filter to the input      
    for(int y = -bound; y <= bound; y++) 
        for (int x = -bound; x <= bound; x++) {
            int op_idx = idx + x + width*y;
            if(op_idx < 0 || op_idx > width * height) continue;     // upper and bottom bound
            if(abs(op_idx % width - idx % width) > bound) continue; // left and right bound
            int result = (int)vec[op_idx] + kernel[y + bound][x + bound];
            vec[op_idx] = (result > peak_value)? peak_value: result;
        }
} 

void FakeMapNode::timer_cb(const ros::TimerEvent& event){
    std::fill(map_msg_.data.begin(), map_msg_.data.end(), 0);
    for (int i = 0; i < 40; ++i)
        gauss_filter(map_msg_.data, map_msg_.info.width, map_msg_.info.height, \
                    rand() % (map_msg_.info.width * map_msg_.info.height), 7, 100);

    map_msg_.header.stamp = ros::Time::now();
    pub_map_.publish(map_msg_);

    nav_msgs::Path::Ptr walkable_path_ptr = nav_msgs::Path::Ptr(new nav_msgs::Path());
    walkable_path_ptr->header.frame_id = map_msg_.header.frame_id;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    nav_msgs::OccupancyGrid::ConstPtr map_msg_ptr(new nav_msgs::OccupancyGrid(map_msg_));

    bool flag_success = solver_.FindPathByHashmap(map_msg_ptr, walkable_path_ptr, 0, mapinfo_.width * mapinfo_.height -1, 10.0);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    pub_path_.publish(walkable_path_ptr);
}

void sigint_cb(int sig) {
    std::cout << "\nNode name: " << ros::this_node::getName() << " is shutdown." << std::endl;
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "fake_map_node");   
    FakeMapNode node;
    signal(SIGINT, sigint_cb);
    ros::spin();
    return 0;
}