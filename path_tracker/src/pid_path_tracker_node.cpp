// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/path.hpp>
// #include <std_msgs/msg/empty.hpp> // Empty 메시지 타입 추가
// #include <std_msgs/msg/string.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <cmath>
// #include <algorithm>
// #include <fstream>
// #include <sstream>
// #include <string>
// #include <vector>

// // 쿼터니언에서 Yaw 각도를 추출하는 유틸리티 함수
// double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
// {
//     tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
//     tf2::Matrix3x3 m(tf_q);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);
//     return yaw;
// }


// class PidPathTracker : public rclcpp::Node
// {
// public:
//     PidPathTracker() : Node("pid_path_tracker_node")
//     {
//         // 파라미터 선언
//         this->declare_parameter<std::string>("path_file", "");
//         this->declare_parameter<double>("kp_linear", 1.0);
//         this->declare_parameter<double>("ki_linear", 0.0);
//         this->declare_parameter<double>("kd_linear", 0.1);
//         this->declare_parameter<double>("kp_angular", 2.0);
//         this->declare_parameter<double>("ki_angular", 0.0);
//         this->declare_parameter<double>("kd_angular", 0.2);
//         this->declare_parameter<double>("max_linear_velocity", 0.5);
//         this->declare_parameter<double>("max_angular_velocity", 1.0);
//         this->declare_parameter<double>("goal_tolerance", 0.1);
//         this->declare_parameter<double>("waypoint_arrival_dist", 0.3);
//         this->declare_parameter<double>("goal_angular_tolerance", 0.05); // 이 파라미터는 더 이상 사용되지 않지만, 호환성을 위해 남겨둡니다.

//         // 파라미터 값 가져오기
//         this->get_parameter("path_file", path_file_);
//         this->get_parameter("kp_linear", kp_linear_);
//         this->get_parameter("ki_linear", ki_linear_);
//         this->get_parameter("kd_linear", kd_linear_);
//         this->get_parameter("kp_angular", kp_angular_);
//         this->get_parameter("ki_angular", ki_angular_);
//         this->get_parameter("kd_angular", kd_angular_);
//         this->get_parameter("max_linear_velocity", max_linear_velocity_);
//         this->get_parameter("max_angular_velocity", max_angular_velocity_);
//         this->get_parameter("goal_tolerance", goal_tolerance_);
//         this->get_parameter("waypoint_arrival_dist", waypoint_arrival_dist_);
//         this->get_parameter("goal_angular_tolerance", goal_angular_tolerance_);

//         // TF 리스너 및 버퍼 초기화
//         tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         // Publisher/Subscriber 설정
//         cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
//         target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
//         // finish_pub_ = this->create_publisher<std_msgs::msg::Empty>("/robot_finish", 10);
//         // start_sub_ = this->create_subscription<std_msgs::msg::Empty>(
//         //     "/robot_start", 10, std::bind(&PidPathTracker::start_tracking_callback, this, std::placeholders::_1));

//         navigation_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
//             "/navigation_cmd", 10, std::bind(&PidPathTracker::navigation_cmd_callback, this, std::placeholders::_1));

//         // 매니저 노드에게 상태를 보고하는 Publisher
//         navigation_feedback_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot_navigation_feedback", 10);
        
//         // 경로 파일 로드
//         if (!load_path_from_csv())
//         {
//             RCLCPP_ERROR(this->get_logger(), "Failed to load path, node will not function.");
//             return;
//         }

//         // 주기적으로 제어 로직을 실행할 타이머
//         control_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(50), std::bind(&PidPathTracker::update_control, this));

//         RCLCPP_INFO(this->get_logger(), "PID Path Tracker Node started. Waiting for command on /navigation_cmd...");
//     }

// private:
//     // 매니저로부터 내비게이션 명령(경로 파일)을 받는 콜백
//     void navigation_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
//     {
//         RCLCPP_INFO(this->get_logger(), "Received navigation command for path: %s", msg->data.c_str());
        
//         // 현재 진행중인 작업이 있다면 중지
//         is_tracking_active_ = false;
//         cmd_vel_pub_->publish(geometry_msgs::msg::Twist());

//         // 새 경로 파일명으로 업데이트하고 로드 시도
//         path_file_ = msg->data;
//         if (load_path_from_csv())
//         {
//             // 경로 로드 성공 시, 추적 시작 및 피드백(false) 발행
//             RCLCPP_INFO(this->get_logger(), "New path loaded. Starting tracking immediately.");
//             is_tracking_active_ = true;
//             tracking_completed_ = false;
//             current_waypoint_idx_ = 0;
//             integral_linear_ = 0.0;
//             last_linear_error_ = 0.0;
//             integral_angular_ = 0.0;
//             last_angular_error_ = 0.0;

//             std_msgs::msg::Bool feedback_msg;
//             feedback_msg.data = false; // "임무 수행 중"
//             navigation_feedback_pub_->publish(feedback_msg);
//         }
//         else
//         {
//             // 경로 로드 실패 시, 에러 로그 출력 및 피드백(true) 발행
//             RCLCPP_ERROR(this->get_logger(), "Failed to load path: %s. Task aborted.", path_file_.c_str());
//             std_msgs::msg::Bool feedback_msg;
//             feedback_msg.data = true; // "임무 실패, 대기 상태로 복귀"
//             navigation_feedback_pub_->publish(feedback_msg);
//         }
//     }
//     // void start_tracking_callback(const std_msgs::msg::Empty::SharedPtr msg)
//     // {
//     //     (void)msg;
//     //     RCLCPP_INFO(this->get_logger(), "Received /robot_start signal. Starting path tracking.");

//     //     is_tracking_active_ = true;
//     //     tracking_completed_ = false;
//     //     current_waypoint_idx_ = 0;
        
//     //     integral_linear_ = 0.0;
//     //     last_linear_error_ = 0.0;
//     //     integral_angular_ = 0.0;
//     //     last_angular_error_ = 0.0;
//     // }

//     bool load_path_from_csv()
//     {
//         if (path_file_.empty()) {
//             RCLCPP_ERROR(this->get_logger(), "Parameter 'path_file' is not set.");
//             return false;
//         }

//         std::ifstream file(path_file_);
//         if (!file.is_open())
//         {
//             RCLCPP_ERROR(this->get_logger(), "Failed to open path file: %s", path_file_.c_str());
//             return false;
//         }
        
//         current_path_.poses.clear();
//         current_path_.header.stamp = this->get_clock()->now();
//         current_path_.header.frame_id = "map";

//         std::string line;
//         std::getline(file, line);

//         while (std::getline(file, line))
//         {
//             std::stringstream ss(line);
//             std::string value;
//             std::vector<std::string> all_values;
//             while(std::getline(ss, value, ',')) { all_values.push_back(value); }
//             if (all_values.size() < 10) continue;
        
//             try {
//                 geometry_msgs::msg::PoseStamped pose_stamped;
//                 pose_stamped.header.stamp = this->get_clock()->now();
//                 pose_stamped.header.frame_id = "map";
//                 pose_stamped.pose.position.x = std::stod(all_values[3]);
//                 pose_stamped.pose.position.y = std::stod(all_values[4]);
//                 pose_stamped.pose.position.z = std::stod(all_values[5]);
//                 pose_stamped.pose.orientation.x = std::stod(all_values[6]);
//                 pose_stamped.pose.orientation.y = std::stod(all_values[7]);
//                 pose_stamped.pose.orientation.z = std::stod(all_values[8]);
//                 pose_stamped.pose.orientation.w = std::stod(all_values[9]);
//                 current_path_.poses.push_back(pose_stamped);
//             } catch (const std::invalid_argument& e) {
//                 RCLCPP_ERROR(this->get_logger(), "Error parsing specific values from CSV line. Check data format and indices.");
//                 continue;
//             }
//         }
        
//         if (current_path_.poses.empty())
//         {
//             RCLCPP_WARN(this->get_logger(), "Path loaded from %s is empty.", path_file_.c_str());
//             path_loaded_ = false;
//             return false;
//         }
        
//         path_loaded_ = true;
//         RCLCPP_INFO(this->get_logger(), "Loaded path with %zu poses from %s", current_path_.poses.size(), path_file_.c_str());
//         return true;
//     }

//     void update_control()
//     {
//         if (!path_loaded_ || !is_tracking_active_ || tracking_completed_)
//         {
//             return;
//         }
    
//         geometry_msgs::msg::PoseStamped current_pose;
//         try
//         {
//             geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
//             current_pose.header = t.header;
//             current_pose.pose.position.x = t.transform.translation.x;
//             current_pose.pose.position.y = t.transform.translation.y;
//             current_pose.pose.orientation = t.transform.rotation;
//         }
//         catch (const tf2::TransformException &ex)
//         {
//             RCLCPP_WARN(this->get_logger(), "Could not get transform from map to base_link: %s", ex.what());
//             return;
//         }
    
//         // 최종 목표 도착 여부를 최우선으로 확인
//         const auto& final_goal_pose = current_path_.poses.back();
//         double dist_to_final_goal = std::hypot(
//             final_goal_pose.pose.position.x - current_pose.pose.position.x,
//             final_goal_pose.pose.position.y - current_pose.pose.position.y);

//         if (current_waypoint_idx_ == current_path_.poses.size() - 1 && dist_to_final_goal < goal_tolerance_)
//         {
//             RCLCPP_INFO(this->get_logger(), "Final goal position reached! Tracking completed.");
            
//             // 로봇 정지
//             geometry_msgs::msg::Twist stop_cmd;
//             stop_cmd.linear.x = 0.0;
//             stop_cmd.angular.z = 0.0;
//             cmd_vel_pub_->publish(stop_cmd);
            
//             // 완료 신호 발행
//             std_msgs::msg::Bool feedback_msg;
//             feedback_msg.data = true; // "임무 완료"
//             navigation_feedback_pub_->publish(feedback_msg);
            
//             // 상태 업데이트
//             is_tracking_active_ = false;
//             tracking_completed_ = true;
//             return; // 제어 루프 종료
//         }
    
//         // 일반 경로 추적 로직
//         geometry_msgs::msg::PoseStamped target_pose = current_path_.poses[current_waypoint_idx_];
        
//         double dist_to_target = std::hypot(
//             target_pose.pose.position.x - current_pose.pose.position.x,
//             target_pose.pose.position.y - current_pose.pose.position.y);

//         // 현재 웨이포인트에 도착하면 다음 웨이포인트로 인덱스 업데이트
//         if (dist_to_target < waypoint_arrival_dist_) {
//             if (current_waypoint_idx_ < current_path_.poses.size() - 1) {
//                 current_waypoint_idx_++;
//                 RCLCPP_INFO(this->get_logger(), "Waypoint reached. Moving to next waypoint #%zu", current_waypoint_idx_);
//                 integral_linear_ = 0.0;
//                 integral_angular_ = 0.0;
//             }
//         }
        
//         // 제어에 사용할 목표 지점 업데이트
//         target_pose = current_path_.poses[current_waypoint_idx_];
//         target_pose.header.stamp = this->get_clock()->now();
//         target_pose_pub_->publish(target_pose);

//         // 거리 및 각도 오차 계산
//         double distance_error = dist_to_target;
//         double angle_to_target = std::atan2(
//             target_pose.pose.position.y - current_pose.pose.position.y,
//             target_pose.pose.position.x - current_pose.pose.position.x);
        
//         double current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation);
//         double angle_error = angle_to_target - current_yaw;
//         while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
//         while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

//         // PID 제어
//         integral_linear_ += distance_error;
//         double derivative_linear = distance_error - last_linear_error_;
//         double linear_velocity = kp_linear_ * distance_error + ki_linear_ * integral_linear_ + kd_linear_ * derivative_linear;
//         last_linear_error_ = distance_error;

//         integral_angular_ += angle_error;
//         double derivative_angular = angle_error - last_angular_error_;
//         double angular_velocity = kp_angular_ * angle_error + ki_angular_ * integral_angular_ + kd_angular_ * derivative_angular;
//         last_angular_error_ = angle_error;

//         // 속도 제한 및 발행
//         geometry_msgs::msg::Twist cmd_vel_msg;
//         cmd_vel_msg.linear.x = std::clamp(linear_velocity, 0.0, max_linear_velocity_);
//         cmd_vel_msg.angular.z = std::clamp(angular_velocity, -max_angular_velocity_, max_angular_velocity_);
    
//         cmd_vel_pub_->publish(cmd_vel_msg);
//         RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Publishing cmd_vel -> Linear: %.3f m/s, Angular: %.3f rad/s", 
//             cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
//     }

//     // 멤버 변수
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
//     rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
//     rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr finish_pub_;
//     rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_cmd_sub_;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr navigation_feedback_pub_;
//     rclcpp::TimerBase::SharedPtr control_timer_;

//     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

//     nav_msgs::msg::Path current_path_;
//     bool path_loaded_ = false;
//     bool is_tracking_active_ = false;
//     bool tracking_completed_ = false;
//     // bool final_orientation_alignment_ = false; // 더 이상 필요 없음

//     size_t current_waypoint_idx_ = 0;

//     double kp_linear_, ki_linear_, kd_linear_;
//     double kp_angular_, ki_angular_, kd_angular_;
    
//     double integral_linear_ = 0.0;
//     double last_linear_error_ = 0.0;
//     double integral_angular_ = 0.0;
//     double last_angular_error_ = 0.0;

//     double max_linear_velocity_;
//     double max_angular_velocity_;
//     double goal_tolerance_;
//     double goal_angular_tolerance_;
//     double waypoint_arrival_dist_;
//     std::string path_file_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<PidPathTracker>());
//     rclcpp::shutdown();
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// 쿼터니언에서 Yaw 각도를 추출하는 유틸리티 함수
double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}


class PidPathTracker : public rclcpp::Node
{
public:
    PidPathTracker() : Node("pid_path_tracker_node")
    {
        // 파라미터 선언
        this->declare_parameter<std::string>("path_directory", ""); // 경로 파일이 저장된 디렉토리
        // X축 PID 파라미터
        this->declare_parameter<double>("kp_x", 1.0);
        this->declare_parameter<double>("ki_x", 0.0);
        this->declare_parameter<double>("kd_x", 0.1);
        // Y축 PID 파라미터
        this->declare_parameter<double>("kp_y", 1.0);
        this->declare_parameter<double>("ki_y", 0.0);
        this->declare_parameter<double>("kd_y", 0.1);
        // Yaw PID 파라미터
        this->declare_parameter<double>("kp_yaw", 2.0);
        this->declare_parameter<double>("ki_yaw", 0.0);
        this->declare_parameter<double>("kd_yaw", 0.2);
        // 속도 제한 파라미터
        this->declare_parameter<double>("max_linear_velocity", 0.5);
        this->declare_parameter<double>("max_angular_velocity", 1.0);
        this->declare_parameter<double>("goal_tolerance", 0.1);
        this->declare_parameter<double>("waypoint_arrival_dist", 0.3);

        // 파라미터 값 가져오기
        this->get_parameter("path_directory", path_directory_);
        // X축 PID 파라미터
        this->get_parameter("kp_x", kp_x_);
        this->get_parameter("ki_x", ki_x_);
        this->get_parameter("kd_x", kd_x_);
        // Y축 PID 파라미터
        this->get_parameter("kp_y", kp_y_);
        this->get_parameter("ki_y", ki_y_);
        this->get_parameter("kd_y", kd_y_);
        // Yaw PID 파라미터
        this->get_parameter("kp_yaw", kp_yaw_);
        this->get_parameter("ki_yaw", ki_yaw_);
        this->get_parameter("kd_yaw", kd_yaw_);
        // 속도 제한 파라미터
        this->get_parameter("max_linear_velocity", max_linear_velocity_);
        this->get_parameter("max_angular_velocity", max_angular_velocity_);
        this->get_parameter("goal_tolerance", goal_tolerance_);
        this->get_parameter("waypoint_arrival_dist", waypoint_arrival_dist_);

        // path_directory 파라미터가 비어 있으면 노드가 동작할 수 없으므로 확인 후 종료
        if (path_directory_.empty()) {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'path_directory' is not set! This is required.");
            // rclcpp::shutdown(); // 필요 시 노드를 강제 종료
            return;
        }

        // TF 리스너 및 버퍼 초기화
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher/Subscriber 설정
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
        
        navigation_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/navigation_cmd", 10, std::bind(&PidPathTracker::navigation_cmd_callback, this, std::placeholders::_1));

        navigation_feedback_pub_ = this->create_publisher<std_msgs::msg::Bool>("/robot_navigation_feedback", 10);

        // 주기적으로 제어 로직을 실행할 타이머
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&PidPathTracker::update_control, this));

        RCLCPP_INFO(this->get_logger(), "PID Path Tracker Node started. Path directory: [%s]", path_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "Waiting for command on /navigation_cmd...");
    }

private:
    // 매니저로부터 내비게이션 명령(경로 별칭)을 받는 콜백
    void navigation_cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string path_alias = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received navigation command for path alias: [%s]", path_alias.c_str());
        
        // 동일 경로에 대한 중복 명령은 무시하여 재시작/리셋을 방지
        if (is_tracking_active_ && path_loaded_ && !current_path_alias_.empty() && current_path_alias_ == path_alias) {
            RCLCPP_INFO(this->get_logger(), "Already tracking the same path [%s]. Ignoring duplicate command.", path_alias.c_str());
            return;
        }
        
        // 현재 진행중인 작업이 있다면 중지
        is_tracking_active_ = false;
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;  // dx
        stop_cmd.linear.y = 0.0;  // dy
        stop_cmd.angular.z = 0.0;  // yaw
        cmd_vel_pub_->publish(stop_cmd);

        // 현재 경로 별칭 저장 및 경로 별칭과 디렉토리를 조합하여 전체 파일 경로 생성
        current_path_alias_ = path_alias;
        path_file_ = path_directory_ + "/" + path_alias + ".csv";
        // RCLCPP_INFO(this->get_logger(), "Constructed full path: [%s]", path_file_.c_str());
        

        // 생성된 전체 경로로 CSV 파일 로드 시도
        if (load_path_from_csv())
        {
            // 경로 로드 성공 시, 추적 시작 및 피드백(false) 발행
            // RCLCPP_INFO(this->get_logger(), "New path loaded successfully. Starting tracking.");
            is_tracking_active_ = true;
            tracking_completed_ = false;
            // 가장 가까운 노드를 찾아서 시작점으로 설정
            geometry_msgs::msg::PoseStamped current_pose;
            try {
                geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
                current_pose.header = t.header;
                current_pose.pose.position.x = t.transform.translation.x;
                current_pose.pose.position.y = t.transform.translation.y;
                current_pose.pose.orientation = t.transform.rotation;
                
                current_waypoint_idx_ = find_closest_node_index(current_pose);
                RCLCPP_INFO(this->get_logger(), "Starting from closest node index: %zu", current_waypoint_idx_);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get transform for initial position, starting from index 0");
                current_waypoint_idx_ = 0;
            }
            
            // PID 변수들 초기화
            integral_x_ = 0.0;
            last_x_error_ = 0.0;
            integral_y_ = 0.0;
            last_y_error_ = 0.0;
            integral_yaw_ = 0.0;
            last_yaw_error_ = 0.0;

            // std_msgs::msg::Bool feedback_msg;
            // feedback_msg.data = false; // "임무 수행 중"
            // navigation_feedback_pub_->publish(feedback_msg);
        }
        else
        {
            // 경로 로드 실패 시, 에러 로그 출력 및 피드백(true) 발행
            RCLCPP_ERROR(this->get_logger(), "Failed to load path: [%s]. Task aborted.", path_file_.c_str());
            // std_msgs::msg::Bool feedback_msg;
            // feedback_msg.data = true; // "임무 실패, 대기 상태로 복귀"
            // navigation_feedback_pub_->publish(feedback_msg);
        }
    }

    bool load_path_from_csv()
    {
        if (path_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Constructed path file name is empty.");
            return false;
        }

        std::ifstream file(path_file_);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open path file: %s", path_file_.c_str());
            path_loaded_ = false;
            return false;
        }
        
        current_path_.poses.clear();
        current_path_.header.stamp = this->get_clock()->now();
        current_path_.header.frame_id = "map";

        std::string line;
        std::getline(file, line); // 헤더 라인 스킵

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string value;
            std::vector<std::string> all_values;
            while(std::getline(ss, value, ',')) { all_values.push_back(value); }
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
                current_path_.poses.push_back(pose_stamped);
            } catch (const std::invalid_argument& e) {
                RCLCPP_ERROR(this->get_logger(), "Error parsing values from CSV line. Check data format.");
                continue;
            }
        }
        
        if (current_path_.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Path loaded from %s is empty.", path_file_.c_str());
            path_loaded_ = false;
            return false;
        }
        
        path_loaded_ = true;
        RCLCPP_INFO(this->get_logger(), "Loaded path with %zu poses from %s", current_path_.poses.size(), path_file_.c_str());
        return true;
    }

    // 현재 로봇 위치에서 가장 가까운 노드의 인덱스를 찾는 함수
    size_t find_closest_node_index(const geometry_msgs::msg::PoseStamped& current_pose)
    {
        if (current_path_.poses.empty()) {
            return 0;
        }

        size_t closest_index = 0;
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < current_path_.poses.size(); ++i) {
            const auto& node_pose = current_path_.poses[i];
            double distance = std::hypot(
                node_pose.pose.position.x - current_pose.pose.position.x,
                node_pose.pose.position.y - current_pose.pose.position.y
            );
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Closest node found at index %zu, distance: %.3f m", closest_index, min_distance);
        return closest_index;
    }

    void update_control()
    {
        if (!path_loaded_ || !is_tracking_active_ || tracking_completed_)
        {
            return;
        }
    
        // 현재 로봇 위치 가져오기
        geometry_msgs::msg::PoseStamped current_pose;
        try
        {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            current_pose.header = t.header;
            current_pose.pose.position.x = t.transform.translation.x;
            current_pose.pose.position.y = t.transform.translation.y;
            current_pose.pose.orientation = t.transform.rotation;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform from map to base_link: %s", ex.what());
            return;
        }

        // 목표 노드 설정 (현재 웨이포인트 인덱스 사용)
        size_t target_node_idx = current_waypoint_idx_;
        
        // 현재 목표 노드에 도착했는지 확인
        const auto& current_target_pose = current_path_.poses[target_node_idx];
        double dist_to_current_target = std::hypot(
            current_target_pose.pose.position.x - current_pose.pose.position.x,
            current_target_pose.pose.position.y - current_pose.pose.position.y);

        RCLCPP_INFO(this->get_logger(), "dist_to_current_target: %.3f", dist_to_current_target);

        // 현재 목표에 도착했으면 다음 노드로 이동
        if (dist_to_current_target < waypoint_arrival_dist_) {
            if (current_waypoint_idx_ < current_path_.poses.size() - 1) {
                current_waypoint_idx_++;
                RCLCPP_INFO(this->get_logger(), "Waypoint reached. Moving to next waypoint #%zu", current_waypoint_idx_);
                // PID 변수들 초기화
                integral_x_ = 0.0;
                integral_y_ = 0.0;
                integral_yaw_ = 0.0;
                target_node_idx = current_waypoint_idx_;
            } else {
                // 마지막 노드에 도달한 경우
                if (dist_to_current_target < goal_tolerance_) {
                    RCLCPP_INFO(this->get_logger(), "Final goal position reached! Task completed.");
                    
                    // 로봇 정지
                    geometry_msgs::msg::Twist stop_cmd;
                    stop_cmd.linear.x = 0.0;  // dx
                    stop_cmd.linear.y = 0.0;  // dy
                    stop_cmd.angular.z = 0.0;  // yaw
                    cmd_vel_pub_->publish(stop_cmd);
                    
                    // 완료 피드백 발행
                    std_msgs::msg::Bool feedback_msg;
                    feedback_msg.data = true; // "임무 완료"
                    navigation_feedback_pub_->publish(feedback_msg);
                    
                    // 상태 업데이트
                    is_tracking_active_ = false;
                    tracking_completed_ = true;
                    return;
                }
            }
        }

        // 목표 노드 설정
        geometry_msgs::msg::PoseStamped target_pose = current_path_.poses[target_node_idx];
        target_pose.header.stamp = this->get_clock()->now();
        target_pose_pub_->publish(target_pose);

        // 전역 좌표계에서의 오차 계산
        double x_error_global = target_pose.pose.position.x - current_pose.pose.position.x;
        double y_error_global = target_pose.pose.position.y - current_pose.pose.position.y;
        
        // 현재 로봇의 yaw 각도
        double current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation);
        
        // 전역 좌표 오차를 로봇 기준 좌표로 변환 (회전 변환)
        double x_error_robot = x_error_global * cos(current_yaw) + y_error_global * sin(current_yaw);
        double y_error_robot = -x_error_global * sin(current_yaw) + y_error_global * cos(current_yaw);
        
        // 목표 yaw와 현재 yaw 오차 계산
        double target_yaw = get_yaw_from_quaternion(target_pose.pose.orientation);
        double yaw_error = target_yaw - current_yaw;
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

        // X축 PID 제어 (로봇 기준)
        integral_x_ += x_error_robot;
        double derivative_x = x_error_robot - last_x_error_;
        double x_velocity = kp_x_ * x_error_robot + ki_x_ * integral_x_ + kd_x_ * derivative_x;
        last_x_error_ = x_error_robot;

        // Y축 PID 제어 (로봇 기준)
        integral_y_ += y_error_robot;
        double derivative_y = y_error_robot - last_y_error_;
        double y_velocity = kp_y_ * y_error_robot + ki_y_ * integral_y_ + kd_y_ * derivative_y;
        last_y_error_ = y_error_robot;

        // Yaw PID 제어
        integral_yaw_ += yaw_error;
        double derivative_yaw = yaw_error - last_yaw_error_;
        double yaw_velocity = kp_yaw_ * yaw_error + ki_yaw_ * integral_yaw_ + kd_yaw_ * derivative_yaw;
        last_yaw_error_ = yaw_error;

        // 속도 제한 및 발행
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = std::clamp(x_velocity, -max_linear_velocity_, max_linear_velocity_);  // dx
        cmd_vel_msg.linear.y = std::clamp(y_velocity, -max_linear_velocity_, max_linear_velocity_);  // dy
        cmd_vel_msg.angular.z = std::clamp(yaw_velocity, -max_angular_velocity_, max_angular_velocity_);  // yaw

        cmd_vel_msg.angular.z *= -1.0;
    
        cmd_vel_pub_->publish(cmd_vel_msg);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            "Current: %zu, Target: %zu, Current_yaw: %.3f, Global errors -> x: %.3f, y: %.3f, Robot errors -> x: %.3f, y: %.3f, yaw: %.3f, cmd_vel -> dx: %.3f, dy: %.3f, yaw: %.3f", 
            current_waypoint_idx_, target_node_idx, current_yaw, x_error_global, y_error_global, x_error_robot, y_error_robot, yaw_error,
            cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z);
    }

    // 멤버 변수
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr navigation_feedback_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    nav_msgs::msg::Path current_path_;
    bool path_loaded_ = false;
    bool is_tracking_active_ = false;
    bool tracking_completed_ = false;

    size_t current_waypoint_idx_ = 0;

    // X축 PID 파라미터
    double kp_x_, ki_x_, kd_x_;
    // Y축 PID 파라미터
    double kp_y_, ki_y_, kd_y_;
    // Yaw PID 파라미터
    double kp_yaw_, ki_yaw_, kd_yaw_;
    
    // X축 PID 제어 변수
    double integral_x_ = 0.0;
    double last_x_error_ = 0.0;
    // Y축 PID 제어 변수
    double integral_y_ = 0.0;
    double last_y_error_ = 0.0;
    // Yaw PID 제어 변수
    double integral_yaw_ = 0.0;
    double last_yaw_error_ = 0.0;

    double max_linear_velocity_;
    double max_angular_velocity_;
    double goal_tolerance_;
    double waypoint_arrival_dist_;
    std::string path_file_;
    std::string path_directory_;
    std::string current_path_alias_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PidPathTracker>());
    rclcpp::shutdown();
    return 0;
}