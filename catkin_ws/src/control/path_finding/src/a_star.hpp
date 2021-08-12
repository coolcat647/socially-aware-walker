#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <iostream>
#include <chrono>
#include <ext/hash_map>

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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// using namespace std;

namespace astar {

struct Grid2D {
  int x, y;

  // Grid2D operators
  bool operator == (const Grid2D& grid2) {
    return (x == grid2.x && y == grid2.y);
  }
  Grid2D operator + (const Grid2D& grid2) {
    return {x + grid2.x, y + grid2.y};
  }
};

struct Node {
  float CalcTotalCost(void) const;

  Grid2D grid;
  float g_val;
  float h_val;
  Node *parent;

  // Decision recording
  int8_t decision;

  // C++ define a function-like method to initialize struct data
  Node(Grid2D grid_, Node *parent_ = nullptr);

  // Custom stream out operator
  friend std::ostream& operator << (std::ostream& out, const Node& node) {
    out << "(x:" << node.grid.x << ", y:" << node.grid.y << ")";
    return out;
  }

};

struct node_cost_greater {
  bool operator()(const Node* node1, const Node* node2) const {
    return node1->CalcTotalCost() > node2->CalcTotalCost();
  }
};

struct node_hash_func {
  size_t operator()(const Node* node) const {
    return (node->grid.x << 16) | node->grid.y;
  }
};

struct key_equal_func {
  size_t operator()(const Node* node1, const Node* node2) const {
    return ((node1->grid.x == node2->grid.x) && 
        (node1->grid.y == node2->grid.y));
  }
};


using HeuristicFuncType = std::function<float(Grid2D, Grid2D)>;
using GridList = std::vector<Grid2D>;


class Solver {
  public:
  Solver();
  Solver(ros::NodeHandle& nh,
         bool flag_cost_visualization,
         float max_danger_cost,
         float robot_width,
         float robot_length);
  bool FindPathByHashmap(nav_msgs::OccupancyGrid::ConstPtr map_msg_ptr,
                         nav_msgs::Path::Ptr path,
                         int start_idx,
                         int goal_idx, double timeout_ms);
  bool FindPathByHeap(nav_msgs::OccupancyGrid::ConstPtr map_msg_ptr,
                      nav_msgs::Path::Ptr path,
                      int start_idx,
                      int goal_idx,
                      double timeout_ms);
  bool IsCollision(Grid2D grid);
  float GetPotentialCost(Grid2D grid);
  void SetDiagonalMovement(bool enable);
  Node* find_node(std::vector<Node*>& nodes_list, Grid2D grid);
  void SetHeuristic(HeuristicFuncType h_func);
  void RemoveItemInOpenHash(Node* node);
  void InitVisFunction();
  void AddCostTextToMarkerArray(
    visualization_msgs::MarkerArray& src_mrk_array,
    Grid2D grid,
    float cost);
  void ModifyCostTextInMarkerArray(
    visualization_msgs::MarkerArray& src_mrk_array,
    Grid2D grid,
    float cost);

  private:
  // std::priority_queue<Node*, std::vector<Node*>, node_cost_greater> node_queue_;
  std::vector<Node*> node_queue_;
  __gnu_cxx::hash_map<Node*, Node*, node_hash_func, key_equal_func> open_hashmap_;
  __gnu_cxx::hash_map<Node*, float, node_hash_func, key_equal_func> close_hashmap_;

  int num_directions_;
  GridList directions_;
  HeuristicFuncType GetHeuristic_;
  nav_msgs::OccupancyGrid::ConstPtr map_ptr_;

  float max_danger_cost_ = 80.0;
  float robot_width_ = 0.6;
  float robot_length_ = 0.6;

  ros::Publisher pub_grid_marker_;
  bool flag_cost_visualization_;

  Grid2D* start_grid_ptr_;
  Grid2D* goal_grid_ptr_;
};

class Heuristic {
  public:
  static Grid2D getDelta(Grid2D source, Grid2D target);
  static float manhattan(Grid2D source, Grid2D target);
  static float euclidean(Grid2D source, Grid2D target);
  static float octagonal(Grid2D source, Grid2D target);
};

}