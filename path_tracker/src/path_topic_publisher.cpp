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

class PathTopicPublisher : public rclcpp::Node {
public:
  PathTopicPublisher()
  : rclcpp::Node("path_topic_publisher") {
    // Parameters
    const std::string default_share = ament_index_cpp::get_package_share_directory("path_tracker");
    const std::string default_csv = default_share + "/path/path_3_1.csv";

    csv_file_path_ = this->declare_parameter<std::string>("csv_file", default_csv);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/test_path");
    double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 1.0);

    // Create modified CSV file with y-0.2 in source directory
    std::string source_path_dir = "/root/ros2_ws/src/path_tracker/path/";
    std::string original_filename = "path_3_1.csv";
    std::string modified_csv_path = source_path_dir + "path_3_1_modified_4.csv";
    
    if (writeModifiedCsv(csv_file_path_, modified_csv_path)) {
      RCLCPP_INFO(this->get_logger(), "Created modified CSV file: %s", modified_csv_path.c_str());
    }

    if (!loadPathFromCsv(csv_file_path_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load CSV file: %s", csv_file_path_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Loaded path with %zu poses from %s", path_msg_.poses.size(), csv_file_path_.c_str());
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    // Even though transient_local could work, RViz subscribers are typically volatile.
    // We'll republish periodically to ensure RViz receives the message.

    publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_name_, qos);

    // Timer for periodic republish so RViz always catches it
    if (publish_rate_hz <= 0.0) {
      publish_rate_hz = 1.0;
    }
    using namespace std::chrono_literals;
    const auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz));
    timer_ = this->create_wall_timer(period_ms, [this]() {
      if (!path_msg_.poses.empty()) {
        path_msg_.header.stamp = this->get_clock()->now();
        publisher_->publish(path_msg_);
      }
    });
  }

private:
  bool loadPathFromCsv(const std::string &filepath) {
    std::ifstream in(filepath);
    if (!in.is_open()) {
      return false;
    }

    nav_msgs::msg::Path path;
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
    path_msg_ = std::move(path);
    return true;
  }

  bool writeModifiedCsv(const std::string &input_filepath, const std::string &output_filepath) {
    std::ifstream in(input_filepath);
    if (!in.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open input CSV file: %s", input_filepath.c_str());
      return false;
    }

    std::ofstream out(output_filepath);
    if (!out.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create output CSV file: %s", output_filepath.c_str());
      in.close();
      return false;
    }

    std::string line;
    int line_count = 0;
    while (std::getline(in, line)) {
      if (line.empty()) {
        out << line << std::endl;
        continue;
      }

      std::stringstream ss(line);
      std::string cell;
      std::vector<std::string> cols;
      while (std::getline(ss, cell, ',')) {
        cols.emplace_back(cell);
      }

      // If we have enough columns, modify the y value (column 4)
      if (cols.size() >= 10) {
        try {
          double y_value = std::stod(cols[4]);
          double x_value = std::stod(cols[3]);
          x_value -= 0.03;
          cols[3] = std::to_string(x_value);
          y_value += 0.1;
          cols[4] = std::to_string(y_value);
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->get_logger(), "Failed to parse y value in line %d, keeping original", line_count);
        }
      }

      // Write the modified line
      for (size_t i = 0; i < cols.size(); ++i) {
        if (i > 0) out << ",";
        out << cols[i];
      }
      out << std::endl;
      line_count++;
    }

    in.close();
    out.close();
    
    RCLCPP_INFO(this->get_logger(), "Successfully created modified CSV file: %s (%d lines processed)", 
                output_filepath.c_str(), line_count);
    return true;
  }

  std::string csv_file_path_;
  std::string frame_id_;
  std::string topic_name_;

  nav_msgs::msg::Path path_msg_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathTopicPublisher>());
  rclcpp::shutdown();
  return 0;
}


