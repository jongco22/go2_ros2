#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>

class MultiPathTopicPublisher : public rclcpp::Node {
public:
  MultiPathTopicPublisher()
  : rclcpp::Node("multi_path_topic_publisher") {
    // Parameters
    const std::string default_share = ament_index_cpp::get_package_share_directory("path_tracker");
    const std::string source_path_dir = "/root/ros2_ws/src/path_tracker/path/";
    
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 1.0);

    // Define CSV files and their corresponding topics
    csv_files_ = {
      source_path_dir + "path_3_1.csv",
      source_path_dir + "path_3_1_modified_1.csv",
      source_path_dir + "path_3_1_modified_2.csv", 
      source_path_dir + "path_3_1_modified_3.csv",
      source_path_dir + "path_3_1_modified_4.csv"
    };
    
    topic_names_ = {
      "/test_path",
      "/test_path_modified_1",
      "/test_path_modified_2",
      "/test_path_modified_3", 
      "/test_path_modified_4"
    };

    // Load all CSV files
    for (size_t i = 0; i < csv_files_.size(); ++i) {
      nav_msgs::msg::Path path_msg;
      if (loadPathFromCsv(csv_files_[i], path_msg)) {
        path_msgs_.push_back(path_msg);
        RCLCPP_INFO(this->get_logger(), "Loaded path %zu with %zu poses from %s", 
                    i, path_msg.poses.size(), csv_files_[i].c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load CSV file: %s", csv_files_[i].c_str());
        // Create empty path for failed loads
        nav_msgs::msg::Path empty_path;
        empty_path.header.frame_id = frame_id_;
        path_msgs_.push_back(empty_path);
      }
    }

    // Create publishers for each topic
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    
    for (const auto& topic_name : topic_names_) {
      auto publisher = this->create_publisher<nav_msgs::msg::Path>(topic_name, qos);
      publishers_.push_back(publisher);
    }

    // Timer for periodic republish
    if (publish_rate_hz <= 0.0) {
      publish_rate_hz = 1.0;
    }
    using namespace std::chrono_literals;
    const auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz));
    timer_ = this->create_wall_timer(period_ms, [this]() {
      for (size_t i = 0; i < publishers_.size() && i < path_msgs_.size(); ++i) {
        if (!path_msgs_[i].poses.empty()) {
          path_msgs_[i].header.stamp = this->get_clock()->now();
          publishers_[i]->publish(path_msgs_[i]);
        }
      }
    });

    RCLCPP_INFO(this->get_logger(), "Multi-path publisher initialized with %zu paths", path_msgs_.size());
  }

private:
  bool loadPathFromCsv(const std::string &filepath, nav_msgs::msg::Path &path) {
    std::ifstream in(filepath);
    if (!in.is_open()) {
      return false;
    }

    path.header.frame_id = frame_id_;
    path.header.stamp = this->get_clock()->now();

    std::string line;
    while (std::getline(in, line)) {
      if (line.empty()) {
        continue;
      }
      std::stringstream ss(line);
      std::string cell;
      std::vector<std::string> cols;
      while (std::getline(ss, cell, ',')) {
        cols.emplace_back(cell);
      }

      // Expecting columns: ... x(3), y(4), z(5), qx(6), qy(7), qz(8), qw(9)
      if (cols.size() < 10) {
        continue;
      }

      try {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->get_clock()->now();
        pose_stamped.header.frame_id = frame_id_;
        pose_stamped.pose.position.x = std::stod(cols[3]);
        pose_stamped.pose.position.y = std::stod(cols[4]);
        pose_stamped.pose.position.z = std::stod(cols[5]);
        pose_stamped.pose.orientation.x = std::stod(cols[6]);
        pose_stamped.pose.orientation.y = std::stod(cols[7]);
        pose_stamped.pose.orientation.z = std::stod(cols[8]);
        pose_stamped.pose.orientation.w = std::stod(cols[9]);

        path.poses.emplace_back(std::move(pose_stamped));
      } catch (const std::exception &e) {
        // Skip malformed line
        continue;
      }
    }

    in.close();
    return true;
  }

  std::string frame_id_;
  std::vector<std::string> csv_files_;
  std::vector<std::string> topic_names_;
  std::vector<nav_msgs::msg::Path> path_msgs_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiPathTopicPublisher>());
  rclcpp::shutdown();
  return 0;
}
