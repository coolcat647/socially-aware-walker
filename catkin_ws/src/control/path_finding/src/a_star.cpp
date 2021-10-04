#include "a_star.hpp"

namespace astar {

// Node struct initialization
Node::Node(Grid2D grid_, Node *parent_) {
  parent = parent_;
  grid = grid_;
  g_val = h_val = 0;
  decision = -1;
}


float Node::CalcTotalCost() const {
  return g_val + h_val;
}


// Constructor
Solver::Solver() {
  directions_ = {
    {0, 1}, {1, 0}, {0, -1}, { -1, 0},
    { -1, 1}, {1, 1}, {1, -1}, { -1, -1}
  };
  // SetHeuristic(&Heuristic::manhattan);
  SetHeuristic(&Heuristic::euclidean);
  SetDiagonalMovement(true);
}


Solver::Solver(ros::NodeHandle& nh,
               bool flag_cost_visualization,
               float max_danger_cost,
               float robot_width,
               float robot_length) {
  directions_ = {
    {0, 1}, {1, 0}, {0, -1}, { -1, 0},
    { -1, 1}, {1, 1}, {1, -1}, { -1, -1}
  };
  SetHeuristic(&Heuristic::euclidean);
  SetDiagonalMovement(true);

  max_danger_cost_ = max_danger_cost;
  float robot_width_ = robot_width;
  float robot_length_ = robot_length;

  flag_cost_visualization_ = flag_cost_visualization;
  if (true || flag_cost_visualization == true){
    pub_grid_marker_ = nh.advertise<visualization_msgs::MarkerArray>("grid_marker", 1);
  }

}


void Solver::SetDiagonalMovement(bool enable) {
  num_directions_ = (enable ? 8 : 4);
}


void Solver::SetHeuristic(HeuristicFuncType h_func) {
  GetHeuristic_ = std::bind(h_func, std::placeholders::_1, std::placeholders::_2);
}


Node* Solver::find_node(std::vector<Node*>& nodes_list, Grid2D grid) {
  for (auto node : nodes_list) {
    if (node->grid == grid) return node;
  }
  return nullptr;
}


void Solver::AddCostTextToMarkerArray(visualization_msgs::MarkerArray& mrk_array,
                                      Grid2D grid,
                                      float cost) {
  visualization_msgs::Marker mrk;
  mrk.header.frame_id = map_ptr_->header.frame_id;
  mrk.ns = "grid_marker";
  mrk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  mrk.action = visualization_msgs::Marker::ADD;
  mrk.id = grid.y * map_ptr_->info.width + (grid.x % map_ptr_->info.width);
  mrk.pose.position.x = grid.x * map_ptr_->info.resolution +
                        map_ptr_->info.origin.position.x;
  mrk.pose.position.y = grid.y * map_ptr_->info.resolution +
                        map_ptr_->info.origin.position.y;
  mrk.pose.orientation.w = 1.0;
  mrk.scale.x = 0.01;
  mrk.scale.z = map_ptr_->info.resolution * 0.5f;
  mrk.color.a = 0.4;
  mrk.lifetime = ros::Duration(20.0);
  char buf[10];
  sprintf(buf, "%.1f", cost);
  mrk.text = std::string(buf);
  mrk.header.stamp = ros::Time();
  mrk_array.markers.push_back(mrk);
}


void Solver::ModifyCostTextInMarkerArray(visualization_msgs::MarkerArray& mrk_array,
                                         Grid2D grid,
                                         float cost) {
  int mrk_id = grid.y * map_ptr_->info.width + (grid.x % map_ptr_->info.width);
  for (auto mrk : mrk_array.markers) {
    if (mrk.id == mrk_id) {
      char buf[10];
      sprintf(buf, "%.1f", cost);
      mrk.text = std::string(buf);
      break;
    }
  }
}


void Solver::RemoveItemInOpenHash(Node* node) {
  auto hashmap_iter = open_hashmap_.find(node);
  if (hashmap_iter != open_hashmap_.end())
    open_hashmap_.erase(hashmap_iter);
}


bool Solver::FindPathByHashmap(nav_msgs::OccupancyGrid::ConstPtr map_msg_ptr,
                       nav_msgs::Path::Ptr path,
                       int start_idx,
                       int goal_idx,
                       double timeout_ms) {
  // Marker initialization
  visualization_msgs::MarkerArray::Ptr mrk_array_ptr = 
      visualization_msgs::MarkerArray::Ptr(new visualization_msgs::MarkerArray());
  if(flag_cost_visualization_){
    visualization_msgs::Marker mrk;
    mrk.action = visualization_msgs::Marker::DELETEALL;
    mrk_array_ptr->markers.push_back(mrk);
    pub_grid_marker_.publish(mrk_array_ptr);
    mrk_array_ptr->markers.clear();
  }

  // Get map information
  map_ptr_ = map_msg_ptr;
  int map_width = (int)map_ptr_->info.width;
  int map_height = (int)map_ptr_->info.height;

  // Initialize the queue and hashmap
  open_hashmap_.clear();
  close_hashmap_.clear();
  node_queue_.clear();
  Node* cur_node = nullptr;

  // Convert (x, y) to occupancy grid index
  start_grid_ptr_ = new Grid2D({start_idx % map_width, start_idx / map_width});
  goal_grid_ptr_ = new Grid2D({goal_idx % map_width, goal_idx / map_width});
  Node* start_node = new Node(*start_grid_ptr_);
  node_queue_.push_back(start_node);
  open_hashmap_[start_node] = 0;

  // Set timeout
  ros::Duration timeout = ros::Duration(timeout_ms / 1000);
  ros::Time begin_time = ros::Time::now();

  // Check goal vaild or not
  if (IsCollision(*goal_grid_ptr_)) {
    path->header.stamp = ros::Time::now();
    return false;
  }

  // Main algorithm loop
  bool flag_success = false;
  while (!node_queue_.empty()) {
    // Check if time is up
    if (ros::Time::now() - begin_time > timeout) {
      flag_success = false;
      break;
    }

    // Extract the lowest cost node from the priority queue as the current node
    cur_node = *node_queue_.begin();
    std::pop_heap(node_queue_.begin(), node_queue_.end(), node_cost_greater());
    node_queue_.pop_back();
    RemoveItemInOpenHash(cur_node);
    close_hashmap_[cur_node] = 0; // Push current node to "visited nodes" list

    if (cur_node->grid == *goal_grid_ptr_) {  // If goal arrival
      flag_success = true;
      break;
    }

    // Explore walkable node
    for (int i = 0; i < num_directions_; ++i) {
      Grid2D tmp_grid(cur_node->grid + directions_[i]);
      Node* tmp_node = new Node(tmp_grid, cur_node);
      // Skip visited node & skip wall
      if (IsCollision(tmp_grid) || 
          close_hashmap_.find(tmp_node) != close_hashmap_.end())
        continue;                       

      float g_cost = (i < 4)? cur_node->g_val + 1.0f : cur_node->g_val + 1.414f;

      auto hashmap_iter = open_hashmap_.find(tmp_node);
      if (hashmap_iter == open_hashmap_.end()) {
        // Expand a new node from current node
        Node* successor_ptr = new Node(tmp_grid, cur_node);
        successor_ptr->g_val = g_cost;
        successor_ptr->h_val = GetHeuristic_(successor_ptr->grid, *goal_grid_ptr_) +
                                GetPotentialCost(tmp_grid) / 5.0;
        successor_ptr->decision = i;

        // Add successor node to open set
        node_queue_.push_back(successor_ptr);
        std::push_heap(node_queue_.begin(), node_queue_.end(), node_cost_greater());
        open_hashmap_[successor_ptr] = successor_ptr;

        // Cost visualization
        if(flag_cost_visualization_)
          AddCostTextToMarkerArray(*mrk_array_ptr,
                                    successor_ptr->grid,
                                    successor_ptr->g_val + successor_ptr->h_val);

      } else if (g_cost < hashmap_iter->second->g_val) {
        // Update non-visited but expanded node if find that has lower cost
        hashmap_iter->second->parent = cur_node;
        hashmap_iter->second->g_val = g_cost;
        hashmap_iter->second->decision = i;

        // Re-heap
        std::push_heap(node_queue_.begin(), node_queue_.end(), node_cost_greater());

        // Cost visualization
        if(flag_cost_visualization_)
          ModifyCostTextInMarkerArray(*mrk_array_ptr,
                                      hashmap_iter->second->grid,
                                      hashmap_iter->second->g_val + hashmap_iter->second->h_val);
      }
    }

    // Show the procedure of path searching 
    // if(flag_cost_visualization_ && pub_grid_marker_.getNumSubscribers() > 0) {
    //   pub_grid_marker_.publish(mrk_array_ptr);
    //   ros::Duration(0.01).sleep();
    // }

  } // while loop end

  if (flag_success) {
    int max_sampling_grid = (int)(robot_length_ / map_ptr_->info.resolution);
    int cnt_sampled_grid = max_sampling_grid;
    while (cur_node != nullptr) {
      // printf("%d ", cur_node->decision);
      if (cnt_sampled_grid == max_sampling_grid || cur_node->decision == -1) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = cur_node->grid.x * map_ptr_->info.resolution +
                               map_ptr_->info.origin.position.x;
        pose.pose.position.y = cur_node->grid.y * map_ptr_->info.resolution +
                               map_ptr_->info.origin.position.y;
        pose.pose.orientation.w = 1.0;
        path->poses.push_back(pose);
        cnt_sampled_grid = 0;
      } else {
        cnt_sampled_grid++;
      }
      cur_node = cur_node->parent;
    }
    // printf("\n");
  }

  // Publish the grid cost markers
  if(flag_cost_visualization_ && pub_grid_marker_.getNumSubscribers() > 0)
    pub_grid_marker_.publish(mrk_array_ptr);

  path->header.stamp = ros::Time::now();
  return flag_success;
}


bool Solver::FindPathByHeap(nav_msgs::OccupancyGrid::ConstPtr map_msg_ptr,
                nav_msgs::Path::Ptr path,
                int start_idx,
                int goal_idx,
                double timeout_ms) {
  // Marker initialization
  visualization_msgs::MarkerArray::Ptr mrk_array_ptr = 
      visualization_msgs::MarkerArray::Ptr(new visualization_msgs::MarkerArray());
  if(flag_cost_visualization_){
    visualization_msgs::Marker mrk;
    mrk.action = visualization_msgs::Marker::DELETEALL;
    mrk_array_ptr->markers.push_back(mrk);
    pub_grid_marker_.publish(mrk_array_ptr);
    mrk_array_ptr->markers.clear();
  }

  // Get map information
  map_ptr_ = map_msg_ptr;
  int map_width = map_ptr_->info.width;
  int map_height = map_ptr_->info.height;

  // Initialize the heap tree
  std::vector<Node*> open_set;
  std::vector<Node*> close_set;
  Node* cur_node = nullptr;

  // Convert (x, y) to occupancy grid index
  Grid2D start = {start_idx % map_width, start_idx / map_width};
  Grid2D goal = {goal_idx % map_width, goal_idx / map_width};
  open_set.push_back(new Node(start));

  /// Set timeout
  ros::Duration timeout = ros::Duration(timeout_ms / 1000);
  ros::Time begin_time = ros::Time::now();

  // Check goal vaild or not
  if(IsCollision(goal)){
    path->header.stamp = ros::Time::now();
    return false;
  }

  // Main algorithm loop
  bool flag_success = false;
  while(!open_set.empty()) {
    // Check if time is up
    if(ros::Time::now() - begin_time > timeout){
      flag_success = false;
      break;
    }

    // Extract the lowest cost node from the priority queue as the current node
    cur_node = *open_set.begin();
    std::pop_heap(open_set.begin(), open_set.end(), node_cost_greater());
    open_set.pop_back();

    if(cur_node->grid == goal) {              // If get goal
      flag_success = true;
      break;
    }

    // Push current node to "visited nodes" list
    close_set.push_back(cur_node);

    // Explore walkable node
    for(int i = 0; i < num_directions_; ++i) {
      Grid2D tmp_grid(cur_node->grid + directions_[i]);
      if(IsCollision(tmp_grid) || find_node(close_set, tmp_grid))
        continue;                       // Skip visited node & skip wall

      float g_cost = (i < 4)? cur_node->g_val + 1.0f : cur_node->g_val + 1.414f;

      Node* successor_ptr = find_node(open_set, tmp_grid);
      if(successor_ptr == nullptr){
        successor_ptr = new Node(tmp_grid, cur_node);       // Expand a new node from current node
        successor_ptr->g_val = g_cost;
        successor_ptr->h_val = GetHeuristic_(successor_ptr->grid, goal) + GetPotentialCost(tmp_grid) / 5.0;  // Calc the heuristic value

        successor_ptr->decision = i;
        open_set.push_back(successor_ptr);
        std::push_heap(open_set.begin(), open_set.end(), node_cost_greater());

        // Cost visualization
        if(flag_cost_visualization_)
          AddCostTextToMarkerArray(*mrk_array_ptr,
                                    successor_ptr->grid,
                                    successor_ptr->g_val + successor_ptr->h_val);

      }else if(g_cost < successor_ptr->g_val){
        // Update non-visited but expanded node if find that has lower cost
        successor_ptr->parent = cur_node;
        successor_ptr->g_val = g_cost;
        successor_ptr->decision = i;

        // Cost visualization
        if(flag_cost_visualization_)
          ModifyCostTextInMarkerArray(*mrk_array_ptr,
                                      successor_ptr->grid,
                                      successor_ptr->g_val + successor_ptr->h_val);
      }
    }
  } // while loop end

  if(flag_success){
    int max_sampling_grid = (int)(robot_length_ / map_ptr_->info.resolution);
    int cnt_sampled_grid = max_sampling_grid;
    while(cur_node != nullptr) {
      // printf("%d ", cur_node->decision);
      if(cnt_sampled_grid == max_sampling_grid || cur_node->decision == -1) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = cur_node->grid.x * map_ptr_->info.resolution \
                    + map_ptr_->info.origin.position.x;
        pose.pose.position.y = cur_node->grid.y * map_ptr_->info.resolution \
                    + map_ptr_->info.origin.position.y;
        pose.pose.orientation.w = 1.0;
        path->poses.push_back(pose);

        cnt_sampled_grid = 0;
      }else{
        cnt_sampled_grid++;
      }
      cur_node = cur_node->parent;
    }
    // printf("\n");
  }

  // Publish the grid cost markers
  if(flag_cost_visualization_ && pub_grid_marker_.getNumSubscribers() > 0)
    pub_grid_marker_.publish(mrk_array_ptr);

  path->header.stamp = ros::Time::now();
  return flag_success;
}


bool Solver::IsCollision(Grid2D grid) {
  int width = map_ptr_->info.width;
  int height = map_ptr_->info.height;

  // Check the (x,y) is valid
  if (grid.x < 0 || grid.y < 0 || grid.x >= width || grid.y >= height)
    return true;

  // Check collision
  int map_idx = grid.y * width + (grid.x % width);
  if (map_ptr_->data[map_idx] >= max_danger_cost_ || map_ptr_->data[map_idx] == -1)
    return true;
  else
    return false;
}

float Solver::GetPotentialCost(Grid2D grid) {
  int width = map_ptr_->info.width;
  int map_idx = grid.y * width + (grid.x % width);
  return static_cast<float>(map_ptr_->data[map_idx]);
}

Grid2D Heuristic::getDelta(Grid2D source, Grid2D target) {
  return{ abs(source.x - target.x),  abs(source.y - target.y) };
}

float Heuristic::manhattan(Grid2D source, Grid2D target) {
  auto delta = std::move(getDelta(source, target));
  // return static_cast<uint>(1 * (delta.x + delta.y));
  return (delta.x + delta.y);
}

float Heuristic::euclidean(Grid2D source, Grid2D target) {
  // auto delta = std::move(getDelta(source, target));
  // return static_cast<uint>(1 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
  // return sqrt(pow(delta.x, 2) + pow(delta.y, 2));
  return std::hypot(source.x - target.x, source.y - target.y);
}

float Heuristic::octagonal(Grid2D source, Grid2D target) {
  auto delta = std::move(getDelta(source, target));
  return 1 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}

}  // namespace