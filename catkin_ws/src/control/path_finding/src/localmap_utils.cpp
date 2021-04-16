#include "localmap_utils.hpp"


template<class T>
constexpr const T& clamp( const T& v, const T& lo, const T& hi )
{
    assert( !(hi < lo) );
    return (v < lo) ? lo : (hi < v) ? hi : v;
}


// Original Asymmetric Gaussian Filter
void localmap_utils::apply_original_agf(nav_msgs::OccupancyGrid::Ptr localmap_ptr, int target_idx, double target_yaw, double target_speed, int peak_value) {
    int map_width = localmap_ptr->info.width;
    int map_height = localmap_ptr->info.height;
    double map_resolution = localmap_ptr->info.resolution;

    int max_proxemics_range = 5;    // +- 5 m
    int kernel_size = max_proxemics_range * 2 / map_resolution;
    kernel_size = (kernel_size % 2 == 0)? kernel_size + 1 : kernel_size;

    int max_map_idx = map_width * map_height - 1;

    // Asymmetric Gaussian Filter kernel
    std::vector<std::vector<int8_t> > agf_kernel(kernel_size, std::vector<int8_t>(kernel_size, 0));
    for(int i = 0; i < kernel_size; i++){
        for(int j = 0; j < kernel_size; j++){
            double sigma_head = std::max(target_speed, 0.5);
            double sigma_side = sigma_head * 2 / 5;
            double sigma_rear = sigma_head / 2;

            double y = -max_proxemics_range + map_resolution * i;
            double x = -max_proxemics_range + map_resolution * j;
            double alpha = std::atan2(-y, -x) - target_yaw + M_PI * 0.5;
            double alpha_normalized = std::atan2(std::sin(alpha), std::cos(alpha));
            double sigma_front = (alpha_normalized > 0)? sigma_head : sigma_rear;
            double sin_pow2 = std::pow(std::sin(target_yaw), 2);
            double cos_pow2 = std::pow(std::cos(target_yaw), 2);
            double sigma_side_pow2 = std::pow(sigma_side, 2);
            double sigma_front_pow2 = std::pow(sigma_front, 2);
            double g_a = cos_pow2 / (2 * sigma_front_pow2) + sin_pow2 / (2 * sigma_side_pow2);
            double g_b = std::sin(2 * target_yaw) / (4 * sigma_front_pow2) - std::sin(2 * target_yaw) / (4 * sigma_side_pow2);
            double g_c = sin_pow2 / (2 * sigma_front_pow2) + cos_pow2 / (2 * sigma_side_pow2);
            double z = 1.0 / std::exp(g_a * std::pow(x, 2) + 2 * g_b * x * y + g_c * std::pow(y, 2)) * peak_value;
            agf_kernel[i][j] = (uint8_t)z;

            // Apply filter
            if(agf_kernel[i][j] == 0) continue;
            int op_idx = target_idx - map_width * (i - kernel_size / 2) - (j - kernel_size / 2);

            //// if(localmap_ptr->data[op_idx] < 0) continue;                         // do not apply filter out of laser range
            if(op_idx < 0 || op_idx > max_map_idx) continue;           // upper and bottom bound
            else if(abs((op_idx % map_width) - (target_idx % map_width)) >= kernel_size / 2) continue;  // left and right bound
            else
                localmap_ptr->data[op_idx] = clamp(agf_kernel[i][j] + localmap_ptr->data[op_idx], 0, peak_value);
        }
    }
}


// Socially-aware Asymmetric Gaussian Filter
void localmap_utils::apply_social_agf(nav_msgs::OccupancyGrid::Ptr localmap_ptr, int target_idx, double target_yaw, double target_speed, int peak_value, bool flag_right_hand_side) {
    int map_width = localmap_ptr->info.width;
    int map_height = localmap_ptr->info.height;
    double map_resolution = localmap_ptr->info.resolution;

    int max_proxemics_range = 4;    // +- 5 m
    int kernel_size = max_proxemics_range * 2 / map_resolution;
    kernel_size = (kernel_size % 2 == 0)? kernel_size + 1 : kernel_size;

    int max_map_idx = map_width * map_height - 1;

    // Asymmetric Gaussian Filter kernel
    std::vector<std::vector<int8_t> > agf_kernel(kernel_size, std::vector<int8_t>(kernel_size, 0));
    for(int i = 0; i < kernel_size; i++){
        for(int j = 0; j < kernel_size; j++){
            double sigma_head = std::max(target_speed * 2, 0.5);
            double sigma_right = (flag_right_hand_side)? sigma_head * 3 / 5 : sigma_head / 5;
            double sigma_left = (flag_right_hand_side)? sigma_head / 5 : sigma_head * 3 / 5;
            double sigma_rear = sigma_head / 2;

            double y = -max_proxemics_range + map_resolution * i;
            double x = -max_proxemics_range + map_resolution * j;
            double alpha = std::atan2(-y, -x) - target_yaw + M_PI * 0.5;
            double alpha_normalized = std::atan2(std::sin(alpha), std::cos(alpha));
            double sigma_front = (alpha_normalized > 0)? sigma_head : sigma_rear;
            double alpha_side = std::atan2(std::sin(alpha + M_PI * 0.5), std::cos(alpha + M_PI * 0.5));
            double sigma_side = (alpha_side > 0)? sigma_right : sigma_left;

            double sin_pow2 = std::pow(std::sin(target_yaw), 2);
            double cos_pow2 = std::pow(std::cos(target_yaw), 2);
            double sigma_side_pow2 = std::pow(sigma_side, 2);
            double sigma_front_pow2 = std::pow(sigma_front, 2);
            double g_a = cos_pow2 / (2 * sigma_front_pow2) + sin_pow2 / (2 * sigma_side_pow2);
            double g_b = std::sin(2 * target_yaw) / (4 * sigma_front_pow2) - std::sin(2 * target_yaw) / (4 * sigma_side_pow2);
            double g_c = sin_pow2 / (2 * sigma_front_pow2) + cos_pow2 / (2 * sigma_side_pow2);
            double z = 1.0 / std::exp(g_a * std::pow(x, 2) + 2 * g_b * x * y + g_c * std::pow(y, 2)) * peak_value;
            agf_kernel[i][j] = (uint8_t)z;

            // Apply filter
            if(agf_kernel[i][j] == 0) continue;
            int op_idx = target_idx - map_width * (i - kernel_size / 2) - (j - kernel_size / 2);

            //// if(localmap_ptr->data[op_idx] < 0) continue;                         // do not apply filter out of laser range
            if(op_idx < 0 || op_idx > max_map_idx) continue;           // upper and bottom bound
            else if(abs((op_idx % map_width) - (target_idx % map_width)) >= kernel_size / 2) continue;  // left and right bound
            else
                localmap_ptr->data[op_idx] = clamp(agf_kernel[i][j] + localmap_ptr->data[op_idx], 0, peak_value);
        }
    }
}


void localmap_utils::apply_butterworth_filter(nav_msgs::OccupancyGrid::Ptr localmap_ptr, std::vector<std::vector<int8_t> > &inflation_kernel, int target_idx, int peak_value) {
    int map_width = localmap_ptr->info.width;
    int map_height = localmap_ptr->info.height;

    int kernel_size = inflation_kernel.size();
    int bound = inflation_kernel.size() / 2;

    int min_bound = -bound;
    int max_bound = (bound + kernel_size % 2);
    int max_map_idx = map_width * map_height - 1;

    // ROS_WARN("kernel_size: %d, bound: %d", kernel_size, bound);

    for(int y = min_bound; y < max_bound; y++) {
        for (int x = min_bound; x < max_bound; x++) {
            int op_idx = target_idx + x + map_width * y;
            int8_t op_kernel_val = inflation_kernel[y + bound][x + bound];

            // ROS_WARN("idx: %d, %d", y + bound, x + bound);
            // if(localmap_ptr->data[op_idx] < 0) continue;                                 // do not apply filter out of laser range
            if(op_kernel_val == 0 || localmap_ptr->data[op_idx] >= op_kernel_val) continue;
            else if(op_idx < 0 || op_idx > max_map_idx) continue;           // upper and bottom bound
            else if(abs((op_idx % map_width) - (target_idx % map_width)) >= bound) continue;  // left and right bound
            else{
                int tmp_val = op_kernel_val + localmap_ptr->data[op_idx];
                localmap_ptr->data[op_idx] = (tmp_val > peak_value)? peak_value : tmp_val;
            }
        }
    }
    // exit(-1);
}


void localmap_utils::butterworth_filter_generate(std::vector<std::vector<int8_t> > &inflation_kernel, double filter_radius, int filter_order, double map_resolution, int peak_value) {
    double kernel_range = filter_radius * 8;
    // std::cout << "Filter kernel: " << std::endl;
    for(double y = -kernel_range / 2 ; y <= kernel_range / 2 * 1.00000001; y += map_resolution){
        std::vector<int8_t> tmp_row;
        for(double x = -kernel_range / 2; x <= kernel_range / 2 * 1.00000001; x += map_resolution){
            double r = sqrt(x * x + y * y);
            double z = (1 / sqrt(1 + pow(r / filter_radius, (2 * filter_order)))) * peak_value;
            tmp_row.push_back(z);
        }
        inflation_kernel.push_back(tmp_row);
    }

    for(int i = 0; i < inflation_kernel.size(); i++){
        for(int j = 0; j < inflation_kernel[i].size(); j++){
            // if(i == inflation_kernel.size() / 2 && j == inflation_kernel[0].size() / 2)
                // inflation_kernel[i][j] = peak_value;
            printf("%3d,", inflation_kernel[i][j]);
        }
        std::cout << std::endl;
    }

    ROS_INFO_STREAM("Inflation kernel size: (" << inflation_kernel.size() << ", " << inflation_kernel[0].size() << ")");
}

void localmap_utils::read_footprint_from_yaml(ros::NodeHandle nh, std::string footprint_topic_name, geometry_msgs::PolygonStamped::Ptr footprint_ptr) {
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    nh.getParam(footprint_topic_name, footprint_xmlrpc);
    if(footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString &&
        footprint_xmlrpc != "" &&
        footprint_xmlrpc != "[]") {
        ROS_FATAL("Reading footprint from string is not implement!");
        exit(-1);
    }
    else if(footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        // Make sure we have an array of at least 3 elements.
        if(footprint_xmlrpc.size() < 3) {
            ROS_FATAL("The footprint must be specified as list of lists on the parameter server");
            throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                                        "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
        }
        for (int i = 0; i < footprint_xmlrpc.size(); i++) {
            // Make sure each element of the list is an array of size 2. (x and y coordinates)
            XmlRpc::XmlRpcValue point = footprint_xmlrpc[i];
            if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() != 2) {
                ROS_FATAL("The footprint must be specified as list of lists on the parameter server eg: "
                            "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.");
                throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                                            "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
            }
            geometry_msgs::Point32 pt;

            pt.x = localmap_utils::getNumberFromXMLRPC(point[0], footprint_topic_name);
            pt.y = localmap_utils::getNumberFromXMLRPC(point[1], footprint_topic_name);
            footprint_ptr->polygon.points.push_back(pt);
        }
    }
}


double localmap_utils::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name) {
    // Make sure that the value we're looking at is either a double or an int.
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
        value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
        std::string& value_string = value;
        ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
                   full_param_name.c_str(), value_string.c_str());
        throw std::runtime_error("Values in the footprint specification must be numbers");
    }
    return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}