#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <iostream>
#include <chrono>
// #include <queue.h>

// For ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
 
// using namespace std;

namespace Astar {
    struct Grid2D{
        int x, y;
        bool operator == (const Grid2D& grid_);
        Grid2D operator +(const Grid2D& grid_);         
    };

    struct Node {
        int get_score(void);

        Grid2D grid;
        int g_val;
        int h_val;
        Node *parent;

        // Decision recording
        int8_t decision;

        // c++ define a function-like method to initialize struct data
        Node(Grid2D grid_, Node *parent_ = nullptr);
        friend std::ostream& operator<<(std::ostream& out, const Node& node);
    };

    using HeuristicFunc = std::function<int(Grid2D, Grid2D)>;
    using GridList = std::vector<Grid2D>;


    class Solver {
    public:
        Solver();
        bool solve_ros(nav_msgs::OccupancyGrid::ConstPtr map_msg_ptr, nav_msgs::Path::Ptr path, int start_idx, int goal_idx, double timeout_ms);
        bool is_goal(Grid2D grid);
        bool is_collision(Grid2D grid);
        int against_wall_cost(Grid2D grid);
        void set_diagonal_move(bool enable);
        Node* find_node(std::vector<Node*>& nodes_list, Grid2D grid);
        void set_heuristic(HeuristicFunc h_func);

    private:
        int num_directions_;
        GridList directions_;

        nav_msgs::OccupancyGrid::ConstPtr map_ptr_;

        bool flag_success_;

    
        HeuristicFunc h_func_;
    };

    class Heuristic {
    public:
        static Grid2D getDelta(Grid2D source, Grid2D target);
        static int manhattan(Grid2D source, Grid2D target);
        static int euclidean(Grid2D source, Grid2D target);
        static int octagonal(Grid2D source, Grid2D target);
    };
}