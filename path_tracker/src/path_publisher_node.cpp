#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

// PID Controller class
class PIDController
{
public:
    PIDController(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd)
    {
        integral_ = 0.0;
        last_error_ = 0.0;
    }
    
    double compute(double error, double dt)
    {
        integral_ += error * dt;
        double derivative = (error - last_error_) / dt;
        
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        
        last_error_ = error;
        return output;
    }
    
    void reset()
    {
        integral_ = 0.0;
        last_error_ = 0.0;
    }
    
private:
    double kp_, ki_, kd_;
    double integral_;
    double last_error_;
};

// Utility function to get yaw from quaternion
double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher_node")
    {
        // Parameters
        this->declare_parameter<std::string>("path_directory", "/root/ros2_ws/src/path_tracker/path");
        this->declare_parameter<std::string>("path_file", "");
        this->declare_parameter<double>("control_rate", 100.0); // Hz
        this->declare_parameter<double>("path_publish_rate", 1.0); // Hz
        this->declare_parameter<double>("distance_threshold", 0.3);
        this->declare_parameter<double>("angle_threshold", 0.1);
        this->declare_parameter<double>("kp_x", 1.0);
        this->declare_parameter<double>("kp_y", 1.0);
        this->declare_parameter<double>("kp_yaw", 2.0);
        this->declare_parameter<double>("max_linear_velocity", 0.5);
        this->declare_parameter<double>("max_angular_velocity", 1.0);
        this->declare_parameter<int>("pose_increment", 1);
        
        // Get parameters
        this->get_parameter("path_directory", path_directory_);
        this->get_parameter("path_file", path_file_);
        this->get_parameter("control_rate", control_rate_);
        this->get_parameter("path_publish_rate", path_publish_rate_);
        this->get_parameter("distance_threshold", distance_threshold_);
        this->get_parameter("angle_threshold", angle_threshold_);
        this->get_parameter("kp_x", kp_x_);
        this->get_parameter("kp_y", kp_y_);
        this->get_parameter("kp_yaw", kp_yaw_);
        this->get_parameter("max_linear_velocity", max_linear_velocity_);
        this->get_parameter("max_angular_velocity", max_angular_velocity_);
        this->get_parameter("pose_increment", pose_increment_);
        if (pose_increment_ < 1) {
            RCLCPP_WARN(this->get_logger(), "pose_increment must be >= 1. Overriding to 1 (was %d)", pose_increment_);
            pose_increment_ = 1;
        }
        
        // Check if we have either path_file or path_directory
        if (path_file_.empty() && path_directory_.empty()) {
            RCLCPP_FATAL(this->get_logger(), "Either 'path_file' or 'path_directory' parameter must be set!");
            return;
        }
        
        // Initialize TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        navigation_feedback_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot_navigation_feedback", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path_visualization", 10);
        
        // Subscriber for navigation commands
        navigation_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/navigation_cmd", 10, std::bind(&PathPublisher::navigation_cmd_callback, this, std::placeholders::_1));
        
        // Initialize PID controllers
        pid_x_ = std::make_unique<PIDController>(kp_x_, 0.0, 0.0);
        pid_y_ = std::make_unique<PIDController>(kp_y_, 0.0, 0.0);
        pid_yaw_ = std::make_unique<PIDController>(kp_yaw_, 0.0, 0.0);
        
        // Load path on startup if path_file is provided
        if (!path_file_.empty())
        {
            if (load_path_from_csv())
            {
                RCLCPP_INFO(this->get_logger(), "Path loaded successfully. Finding closest pose...");
                // Publish path for visualization
                publish_path();
                
                // Find closest pose to current robot position
                geometry_msgs::msg::PoseStamped current_pose;
                if (get_current_robot_pose(current_pose))
                {
                    current_pose_idx_ = find_closest_pose_index(current_pose);
                    RCLCPP_INFO(this->get_logger(), "Starting from closest pose index: %zu", current_pose_idx_);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Could not get current robot pose, starting from index 0");
                    current_pose_idx_ = 0;
                }
                
                is_tracking_ = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load path: [%s]. Node will not function.", path_file_.c_str());
                // return;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No initial path file provided. Waiting for navigation command...");
        }
        
        // Timer for control loop
        double timer_period = 1.0 / control_rate_;
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(timer_period * 1000)), 
            std::bind(&PathPublisher::control_loop, this));

        // Timer to publish path visualization at configured rate (default 1 Hz)
        double path_timer_period = 1.0 / std::max(0.1, path_publish_rate_);
        path_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(path_timer_period * 1000)),
            std::bind(&PathPublisher::publish_path, this));
        
        RCLCPP_INFO(this->get_logger(), "Path Tracker Node started. Path file: [%s]", path_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Control rate: %.1f Hz", control_rate_);
    }

private:
    // Navigation command callback
    void navigation_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string path_alias = msg->data;
        
        // Ignore if the same path alias is received
        if (path_alias == last_path_alias_) {
            RCLCPP_DEBUG(this->get_logger(), "Ignoring duplicate navigation command for path alias: [%s]", path_alias.c_str());
            return;
        }
        
        last_path_alias_ = path_alias;
        RCLCPP_INFO(this->get_logger(), "Received navigation command for path alias: [%s]", path_alias.c_str());
        
        // Stop current tracking
        is_tracking_ = false;
        aligning_ = false;
        aligning_ticks_remaining_ = 0;
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.linear.y = 0.0;
        stop_cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(stop_cmd);
        
        // Construct new path file
        if (!path_directory_.empty())
        {
            path_file_ = path_directory_ + "/" + path_alias + ".csv";
            RCLCPP_INFO(this->get_logger(), "New path file: %s", path_file_.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Path directory not set! Cannot load path alias: %s", path_alias.c_str());
            // std_msgs::msg::Bool feedback_msg;
            // feedback_msg.data = true; // "Failed to load path"
            // navigation_feedback_pub_->publish(feedback_msg);
            return;
        }
        
        // Load new path
        if (load_path_from_csv())
        {
            RCLCPP_INFO(this->get_logger(), "New path loaded successfully. Finding closest pose...");
            // Publish path for visualization
            publish_path();
            
            // Find closest pose to current robot position
            geometry_msgs::msg::PoseStamped current_pose;
            if (get_current_robot_pose(current_pose))
            {
                current_pose_idx_ = find_closest_pose_index(current_pose);
                RCLCPP_INFO(this->get_logger(), "Starting from closest pose index: %zu", current_pose_idx_);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Could not get current robot pose, starting from index 0");
                current_pose_idx_ = 0;
            }
            
            // Reset PID controllers
            pid_x_->reset();
            pid_y_->reset();
            pid_yaw_->reset();
            
            // Start tracking
            is_tracking_ = true;
            
            // Send feedback
            std_msgs::msg::Bool feedback_msg;
            feedback_msg.data = false; // "Tracking in progress"
            navigation_feedback_pub_->publish(feedback_msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load path: [%s]. Task aborted.", path_file_.c_str());
            
            // // Send feedback
            // std_msgs::msg::Bool feedback_msg;
            // feedback_msg.data = true; // "Failed to load path"
            // navigation_feedback_pub_->publish(feedback_msg);
        }
    }
    
    // Get current robot pose from TF
    bool get_current_robot_pose(geometry_msgs::msg::PoseStamped& pose)
    {
        try
        {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            pose.header = t.header;
            pose.pose.position.x = t.transform.translation.x;
            pose.pose.position.y = t.transform.translation.y;
            pose.pose.position.z = t.transform.translation.z;
            pose.pose.orientation = t.transform.rotation;
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform from map to base_link: %s", ex.what());
            return false;
        }
    }
    
    // Find closest pose index in the path
    size_t find_closest_pose_index(const geometry_msgs::msg::PoseStamped& current_pose)
    {
        if (poses_.empty()) {
            return 0;
        }
        
        size_t closest_index = 0;
        double min_distance = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < poses_.size(); ++i) {
            const auto& path_pose = poses_[i];
            double distance = std::hypot(
                path_pose.pose.position.x - current_pose.pose.position.x,
                path_pose.pose.position.y - current_pose.pose.position.y
            );
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Closest pose found at index %zu, distance: %.3f m", closest_index, min_distance);
        return closest_index;
    }
    
    // Main control loop
    void control_loop()
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Control loop started");
        if ((!is_tracking_ && !aligning_) || poses_.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Not tracking or no poses loaded");
            return;
        }
        
        // Get current robot pose
        geometry_msgs::msg::PoseStamped current_pose;
        if (!get_current_robot_pose(current_pose))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Could not get current robot pose");
            return;
        }
        
        // Get target pose
        geometry_msgs::msg::PoseStamped target_pose = poses_[current_pose_idx_];
        target_pose.header.stamp = this->get_clock()->now();
        pose_pub_->publish(target_pose);
        
        // Calculate errors
        double x_error = target_pose.pose.position.x - current_pose.pose.position.x;
        double y_error = target_pose.pose.position.y - current_pose.pose.position.y;
        
        double current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation);
        double target_yaw = get_yaw_from_quaternion(target_pose.pose.orientation);
        double yaw_error = target_yaw - current_yaw;
        
        // Normalize angle error
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
        
        // Calculate distance and angle errors
        double distance_error = std::hypot(x_error, y_error);
        double angle_error = std::abs(yaw_error);
        
        // Check if we reached the current target
        if (distance_error < distance_threshold_ && angle_error < angle_threshold_)
        {
            // Move to next pose
            if (current_pose_idx_ < poses_.size() - 1)
            {
                const size_t next_index = std::min(current_pose_idx_ + static_cast<size_t>(pose_increment_), poses_.size() - 1);
                current_pose_idx_ = next_index;
                RCLCPP_INFO(this->get_logger(), "Target reached! Moving to next pose #%zu (increment %d)", current_pose_idx_, pose_increment_);
                
                // Reset PID controllers
                pid_x_->reset();
                pid_y_->reset();
                pid_yaw_->reset();
            }
            else
            {
                // Begin or continue a 3-second alignment phase at the final waypoint
                if (!aligning_)
                {
                    RCLCPP_INFO(this->get_logger(), "Path completed! Starting 3s alignment phase.");
                    aligning_ = true;
                    is_tracking_ = false;
                    aligning_ticks_remaining_ = static_cast<int>(std::ceil(3.0 * control_rate_));
                }
                // else if (this->get_clock()->now() >= align_until_)
                // {
                //     RCLCPP_INFO(this->get_logger(), "Alignment phase finished. Stopping robot and sending completion feedback.");
                //     aligning_ = false;
                    
                //     // Stop robot
                //     geometry_msgs::msg::Twist stop_cmd;
                //     stop_cmd.linear.x = 0.0;
                //     stop_cmd.linear.y = 0.0;
                //     stop_cmd.angular.z = 0.0;
                //     cmd_vel_pub_->publish(stop_cmd);

                //     // ===================================================
                //     // ===== MissionController에 완료 신호(true) 전송 =====
                //     // ===================================================
                //     std_msgs::msg::Bool feedback_msg;
                //     feedback_msg.data = true;
                //     navigation_feedback_pub_->publish(feedback_msg);

                //     return;

                // }
            }
        }
        
        if(aligning_)
        {
            if(aligning_ticks_remaining_ <= 0){
                RCLCPP_INFO(this->get_logger(), "Alignment phase finished. Stopping robot and sending completion feedback.");
                aligning_ = false;

                geometry_msgs::msg::Twist stop_cmd;
                stop_cmd.linear.x = 0.0;
                stop_cmd.linear.y = 0.0;
                stop_cmd.angular.z = 0.0;
                cmd_vel_pub_->publish(stop_cmd);

                std_msgs::msg::Bool feedback_msg;
                feedback_msg.data = true;
                navigation_feedback_pub_->publish(feedback_msg);

                return;
            } else {
                aligning_ticks_remaining_ = std::max(0, aligning_ticks_remaining_ - 1);
                const double time_left_sec = static_cast<double>(aligning_ticks_remaining_) / control_rate_;
                RCLCPP_INFO(this->get_logger(), "Alignment phase not finished yet. Continuing alignment. Time left: %.3f s", time_left_sec);
            }
        }
        // Transform errors to robot frame
        double x_error_robot = x_error * cos(current_yaw) + y_error * sin(current_yaw);
        double y_error_robot = -x_error * sin(current_yaw) + y_error * cos(current_yaw);
        
        // Compute PID outputs
        double dt = 1.0 / control_rate_;
        double x_velocity = pid_x_->compute(x_error_robot, dt);
        double y_velocity = pid_y_->compute(y_error_robot, dt);
        double yaw_velocity = pid_yaw_->compute(yaw_error, dt);
        
        // Apply velocity limits
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = std::clamp(x_velocity, -max_linear_velocity_, max_linear_velocity_);
        cmd_vel.linear.y = std::clamp(y_velocity, -max_linear_velocity_, max_linear_velocity_);
        cmd_vel.angular.z = std::clamp(yaw_velocity, -max_angular_velocity_, max_angular_velocity_);
        
        cmd_vel_pub_->publish(cmd_vel);
        
        // Log every 100 iterations to avoid spam
        static int log_counter = 0;
        if (++log_counter % 100 == 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Target: %zu/%zu, Dist: %.3f, Angle: %.3f, Cmd: [%.3f, %.3f, %.3f]", 
                current_pose_idx_, poses_.size(), distance_error, angle_error,
                cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
        }
    }
    
    bool load_path_from_csv()
    {
        if (path_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Path file name is empty.");
            return false;
        }
        
        std::ifstream file(path_file_);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open path file: %s", path_file_.c_str());
            return false;
        }
        
        poses_.clear();
        
        std::string line;
        std::getline(file, line); // Skip header
        
        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string value;
            std::vector<std::string> all_values;
            while(std::getline(ss, value, ',')) { 
                all_values.push_back(value); 
            }
            if (all_values.size() < 10) continue;
            
            try {
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header.stamp = this->get_clock()->now();
                pose_stamped.header.frame_id = "map";
                pose_stamped.pose.position.x = std::stod(all_values[3]);
                pose_stamped.pose.position.y = std::stod(all_values[4]);
                pose_stamped.pose.position.z = std::stod(all_values[5]);
                pose_stamped.pose.orientation.x = std::stod(all_values[6]);
                pose_stamped.pose.orientation.y = std::stod(all_values[7]);
                pose_stamped.pose.orientation.z = std::stod(all_values[8]);
                pose_stamped.pose.orientation.w = std::stod(all_values[9]);
                poses_.push_back(pose_stamped);
            } catch (const std::invalid_argument& e) {
                RCLCPP_ERROR(this->get_logger(), "Error parsing values from CSV line. Check data format.");
                continue;
            }
        }
        
        if (poses_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Path loaded from %s is empty.", path_file_.c_str());
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Loaded path with %zu poses from %s", poses_.size(), path_file_.c_str());
        return true;
    }
    
    // Publish the loaded path as nav_msgs/Path for visualization
    void publish_path()
    {
        if (poses_.empty()) {
            return;
        }
        if (!path_pub_) {
            return;
        }
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";
        path_msg.poses = poses_;
        path_pub_->publish(path_msg);
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr navigation_feedback_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_cmd_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr path_timer_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::vector<geometry_msgs::msg::PoseStamped> poses_;
    bool is_tracking_ = false;
    bool aligning_ = false;
    int aligning_ticks_remaining_ = 0;
    size_t current_pose_idx_ = 0;
    std::string last_path_alias_;
    
    // PID controllers
    std::unique_ptr<PIDController> pid_x_;
    std::unique_ptr<PIDController> pid_y_;
    std::unique_ptr<PIDController> pid_yaw_;
    
    // Parameters
    double control_rate_;
    double path_publish_rate_;
    double distance_threshold_;
    double angle_threshold_;
    double kp_x_, kp_y_, kp_yaw_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    std::string path_file_;
    std::string path_directory_;
    int pose_increment_ = 1;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPublisher>());
    rclcpp::shutdown();
    return 0;
}
