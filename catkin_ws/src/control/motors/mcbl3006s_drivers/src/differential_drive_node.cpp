
#include "differential_drive_node.hpp"

using namespace std;

static const string COLOR_RED = "\e[0;31m";
static const string COLOR_GREEN = "\e[0;32m";
static const string COLOR_YELLOW = "\e[0;33m"; 
static const string COLOR_NC = "\e[0m";
static const double kMaxLinearVelocity = 0.6;
static const double kMaxAngularVelocity = 0.4;

// Custom operators
namespace geometry_msgs {
    bool operator== (const Twist &cmd1, const Twist &cmd2) {
        double epsilon = 1e-6;
        return std::fabs(cmd1.linear.x - cmd2.linear.x) < epsilon && 
                std::fabs(cmd1.angular.z - cmd2.angular.z) < epsilon;
    }
    bool operator!= (const Twist &cmd1, const Twist &cmd2) {
        return !(cmd1 == cmd2);
    }
}


mutex serial_mutex;



//    __   __        __  ___  __        __  ___  __   __  
//   /  ` /  \ |\ | /__`  |  |__) |  | /  `  |  /  \ |__) 
//   \__, \__/ | \| .__/  |  |  \ \__/ \__,  |  \__/ |  \ 
//  
DiffDriveNode::DiffDriveNode(){
    cout <<setprecision(3) << setiosflags(ios::fixed);
    signal(SIGINT, sigint_cb);

    // ROS publisher & subscriber
    pub_odom_ = nh_.advertise<nav_msgs::Odometry>("wheel_odom", 50);
    sub_cmd_ = nh_.subscribe("cmd_vel", 1, &DiffDriveNode::cmd_cb, this);
    srv_rst_odom_ = nh_.advertiseService("reset_odom", 
                                                   &DiffDriveNode::rst_odom_cb,
                                                   this);
    srv_motor_disable_ = nh_.advertiseService("motor_disable", 
                                                   &DiffDriveNode::motor_disable_cb,
                                                   this);
    int baudrate;
    flag_motor_disable_ = false;
    string serial_device;
    // User-accessible parameters
    ros::param::param<bool>("~motor_disable", flag_motor_disable_, false);
    ros::param::param<std::string>("~serial_device", serial_device, "/dev/ttyUSB0");  // /dev/walker_motor_left
    
    // Fixed parameters
    ros::param::param<int>("~baud", baudrate, 115200);
    ros::param::param<double>("~wheel_radius", wheel_radius_, 0.0625);          // 0.105
    ros::param::param<double>("~wheels_distance", wheels_distance_, 0.6);       // 0.59
    ros::param::param<double>("~gear_ratio", gear_ratio_, 14.0);                // 13.69863
    
    // Timer related
    double command_interval;                                               
    ros::param::param<double>("~command_interval", command_interval, 0.1);     // Car command time interval
    ros::param::param<double>("~watchdog_interval", watchdog_interval_, 0.5);   // Watchdog for robot safety

    motors_init(serial_device, baudrate, flag_motor_disable_);
    
    
    double odom_timer_interval = 0.025;
    max_timer_counter_ = (int)std::round(command_interval / odom_timer_interval);

    // Timer setup
    cmd_timer_cnt_ = 0;
    timer_ = nh_.createTimer(ros::Duration(odom_timer_interval), &DiffDriveNode::timer_cb, this);
    cout << COLOR_GREEN << "Differential drive node is ready." << COLOR_NC << endl;
}


bool DiffDriveNode::rst_odom_cb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp) {
    robot_x_ = 0;
    robot_y_ = 0;
    robot_theta_ = 0;
    is_first_odom_ = true;

    string message = "Reset wheel odometry.";
    cout << COLOR_GREEN << message << COLOR_NC << endl;
    resp.success = true; // boring, but valid response info
    resp.message = message;
    return true;
}


bool DiffDriveNode::motor_disable_cb(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& resp){
    string message;
    if(req.data == true){
        send_motor_cmd(serial_port_ptr_, "1V0");
        send_motor_cmd(serial_port_ptr_, "2V0");
        send_motor_cmd(serial_port_ptr_, "1DI");
        send_motor_cmd(serial_port_ptr_, "2DI");
        flag_motor_disable_ = true;
        message = "Motor disable";
    }else{
        // send_motor_cmd(serial_port_ptr_, "1V0");
        // send_motor_cmd(serial_port_ptr_, "2V0");
        send_motor_cmd(serial_port_ptr_, "1EN");
        send_motor_cmd(serial_port_ptr_, "2EN");
        flag_motor_disable_ = false;
        message = "Motor enable";
    }

    resp.success = true;
    resp.message = message;
    return true;
}


void DiffDriveNode::send_motor_cmd(serial::Serial* ser, const char* cmd){
    string cmd_plus_end = string(cmd) + "\r\n"; 
    string motor_feedback;

    // Critical section start
    serial_mutex.lock();
    ser->write(cmd_plus_end);
    usleep(40);
    try{
        motor_feedback = ser->readline();
    } catch(serial::SerialException& e){
        ROS_ERROR_STREAM("Cannot read the response from motor controller, emergency stop");
        emergency_stop();
        serial_mutex.unlock();
        exit(-1);
    }

    if(motor_feedback.find("OK\r\n") == -1){
        ROS_ERROR_STREAM("CMD: " + std::string(cmd) + ", do not get \"OK\" response but get \"" + motor_feedback + "\" from the motor.");
        emergency_stop();
        serial_mutex.unlock();
        exit(-1);
    }
    usleep(10);
    serial_mutex.unlock();
    // Critical section end
}


int DiffDriveNode::ask_motor_feedback(serial::Serial* ser, const char* cmd){
    string ret_str;
    string cmd_plus_end = string(cmd) + "\r\n"; 

    // Critical section start
    serial_mutex.lock();

    ser->write(cmd_plus_end);
    usleep(40);

    // Note that there is a useless emergency_stop because the serial port error is showing it
    // cannot write/read serial port at that time. We cannot send any zero-velocity command to it.
    // TODO: Solve the critical problem from hardware.
    try{
        ret_str = ser->readline();
    } catch(serial::SerialException& e){
        cout << COLOR_RED << "Cannot read the response from motor controller, emergency stop" << COLOR_NC << endl;
        emergency_stop();
        serial_mutex.unlock();
        exit(-1);
    }
    usleep(10);
    serial_mutex.unlock();
    // Critical section end

    return atoi(ret_str.c_str());
}


void DiffDriveNode::emergency_stop(void) {
    // serial_mutex.lock();
    serial_port_ptr_->write("1V0\r\n");
    serial_port_ptr_->write("2V0\r\n");
    // serial_mutex.unlock();
}


void DiffDriveNode::timer_cb(const ros::TimerEvent& event) {
    // Send rpm command to motors
    if(++cmd_timer_cnt_ >= max_timer_counter_ && !flag_motor_disable_){
        // Watchdog for robot safety
        if(ros::Time::now() - last_cmd_time_ >= ros::Duration(watchdog_interval_)){
            cmd_msg_ = geometry_msgs::Twist();      // zero v, zero omega
            last_cmd_time_ = ros::Time::now();
            // cout << "Stop robot automatically by watchdog counter." << endl;
        }
        double     v = cmd_msg_.linear.x;
        double omega = cmd_msg_.angular.z;
        double desire_rpml = std::round((2 * v - omega * wheels_distance_) / (2 * wheel_radius_) / M_PI / 2 * 60 * gear_ratio_);
        double desire_rpmr = std::round((2 * v + omega * wheels_distance_) / (2 * wheel_radius_) / M_PI / 2 * 60 * gear_ratio_);

        // Send cmd to motor & get motor pulse
        char cmd[15];
        sprintf(cmd, "1V%d", (int)desire_rpml);
        send_motor_cmd(serial_port_ptr_, cmd);
        sprintf(cmd, "2V%d", -(int)desire_rpmr);     // notice the "minus" note due to right motor setup
        send_motor_cmd(serial_port_ptr_, cmd);

        // Reset timer counter
        cmd_timer_cnt_ = 0;
    }

    // Read pulse from motors
    int pulsel = ask_motor_feedback(serial_port_ptr_, "1POS");
    int pulser = ask_motor_feedback(serial_port_ptr_, "2POS");
    
    // Wheel odometry process
    if(is_first_odom_) {
        is_first_odom_ = false;
        robot_x_ = robot_y_ = robot_theta_ = 0.0;
        // last_pulsel_ = pulsel;
        // last_pulser_ = pulser;
        last_feedback_time_ = ros::Time::now();
    }
    ros::Time current_time = ros::Time::now();
    double time_diff = (current_time - last_feedback_time_).toSec();
    // double omega_l = (double)(pulsel - last_pulsel_) / 3000 / gear_ratio_ * 2 * M_PI / time_diff;
    // double omega_r = -(double)(pulser - last_pulser_) / 3000 / gear_ratio_ * 2 * M_PI / time_diff;     // notice the "minus"
    double omega_l = (double)ask_motor_feedback(serial_port_ptr_, "1GN") / 3000 / gear_ratio_ * 2 * M_PI / time_diff;
    double omega_r = -(double)ask_motor_feedback(serial_port_ptr_, "2GN") / 3000 / gear_ratio_ * 2 * M_PI / time_diff;
    // cout << "wheel -> desire (rpm_l, rpm_r): " << desire_rpml / gear_ratio_ << ", " << desire_rpmr / gear_ratio_;
    // cout << "\nwheel ->   real (rpm_l, rpm_r): " <<  omega_l * 60 / 2 / M_PI << ", " << omega_r * 60 / 2 / M_PI << "\n" << endl;
    
    double real_v = wheel_radius_ / 2 * (omega_r + omega_l);
    double real_omega = wheel_radius_ / wheels_distance_ * (omega_r - omega_l);
    // cout << "real (v, w) = " << real_v << ", " << real_omega << endl;

    robot_x_ += real_v * cos(robot_theta_) * time_diff;
    robot_y_ += real_v * sin(robot_theta_) * time_diff;
    robot_theta_ += real_omega * time_diff;

    // Odom message preparing
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_theta_);
    geometry_msgs::TransformStamped odom_tf_msg;
    odom_tf_msg.header.stamp = current_time;
    odom_tf_msg.header.frame_id = "odom";
    odom_tf_msg.child_frame_id = "base_link";
    odom_tf_msg.transform.translation.x = robot_x_;
    odom_tf_msg.transform.translation.y = robot_y_;
    odom_tf_msg.transform.rotation = odom_quat;
    // odom_broadcaster_.sendTransform(odom_tf_msg);

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x = robot_x_;
    odom_msg.pose.pose.position.y = robot_y_;
    odom_msg.pose.pose.orientation = odom_quat;
    // odom_msg.pose.covariance[0]  = 1e2;    // covariance of x 
    // odom_msg.pose.covariance[7]  = 1e2;    // covariance of y
    // odom_msg.pose.covariance[35] = 1e2;    // covariance of yaw
    // odom_msg.pose.covariance[14] = 1e10;    // yaw x axis
    // odom_msg.pose.covariance[21] = 1e10;    // yaw y axis
    // odom_msg.pose.covariance[28] = 1e10;    // yaw z axis
    for(int i = 0; i < 36; i++)
        odom_msg.pose.covariance[i]  = 1e5;

    odom_msg.pose.covariance[0]  = 1e2;    // covariance of x 
    odom_msg.pose.covariance[7]  = 1e2;    // covariance of y
    odom_msg.pose.covariance[5]  = 1e2;    // covariance of xxx 
    odom_msg.pose.covariance[11]  = 1e2;    // covariance of yyy

    odom_msg.pose.covariance[35] = 1e2;    // covariance of yaw
    odom_msg.pose.covariance[30] = 1e2;    // covariance of yaw
    odom_msg.pose.covariance[31] = 1e2;    // covariance of yaw
    odom_msg.pose.covariance[14] = 1e2;    // yaw x axis
    odom_msg.pose.covariance[21] = 1e2;    // yaw y axis
    odom_msg.pose.covariance[28] = 1e2;    // yaw z axis
    odom_msg.twist.twist.linear.x = real_v;
    odom_msg.twist.twist.angular.z = real_omega;

    pub_odom_.publish(odom_msg);

    // last_pulsel_ = pulsel;
    // last_pulser_ = pulser;
    last_feedback_time_ = current_time;
} 


void DiffDriveNode::cmd_cb(const geometry_msgs::Twist& msg) {
    cmd_msg_ = msg;
    last_cmd_time_ = ros::Time::now();
}


void DiffDriveNode::motors_init(string serial_device, int baudrate, bool flag_motor_disable) {
    // Motors port init
    try {
        serial_port_ptr_ = new serial::Serial(serial_device, baudrate, serial::Timeout::simpleTimeout(1000)); // timeout: 1ms
        serial_port_ptr_->readline();   // Just clear buffer
    } catch(exception & e){
        cout << COLOR_RED << "Can not open serial port: " << serial_device << ", please check the port is available." << COLOR_NC << endl;
    }

    // Set Acceleration and deceleration maximum
    ask_motor_feedback(serial_port_ptr_, "1GNODEADR");
    ask_motor_feedback(serial_port_ptr_, "2GNODEADR");
    send_motor_cmd(serial_port_ptr_, "1AC5");
    send_motor_cmd(serial_port_ptr_, "1DEC10");
    send_motor_cmd(serial_port_ptr_, "2AC5");
    send_motor_cmd(serial_port_ptr_, "2DEC10");

    if(flag_motor_disable == true) {
        send_motor_cmd(serial_port_ptr_, "1DI");
        send_motor_cmd(serial_port_ptr_, "2DI");
    }else{
        send_motor_cmd(serial_port_ptr_, "1EN");
        send_motor_cmd(serial_port_ptr_, "2EN");
    }

    cout << COLOR_GREEN << "Motor drivers are ready." << COLOR_NC << endl;
}

void DiffDriveNode::sigint_cb(int sig) {
    cout << "\nNode name: " << ros::this_node::getName() << " is shutdown." << endl;
    // serial_mutex.unlock();
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


//            
//   |\/|  /\  | |\ | 
//   |  | /~~\ | | \| 
//  
int main (int argc, char** argv) {
    ros::init(argc, argv, "differential_drive_node"); 
    DiffDriveNode node;
    ros::spin();

    // Safety concern
    // serial_mutex.lock();
    node.send_motor_cmd(node.serial_port_ptr_, "1V0");
    node.send_motor_cmd(node.serial_port_ptr_, "2V0");
    // node.serial_port_ptr_->write("1V0\r\n");
    // node.serial_port_ptr_->write("2V0\r\n");
    // serial_mutex.unlock();
    return 0;
}
