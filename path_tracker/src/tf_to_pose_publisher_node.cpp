#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

class TfToPosePublisher : public rclcpp::Node
{
public:
    TfToPosePublisher() : Node("tf_to_pose_publisher_node")
    {
        // 파라미터 선언
        this->declare_parameter<std::string>("source_frame", "map");
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<std::string>("topic_name", "/current_pose");
        this->declare_parameter<double>("publish_rate", 50.0); // 발행 주기 (Hz)

        // 파라미터 가져오기
        this->get_parameter("source_frame", source_frame_);
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("topic_name", topic_name_);
        this->get_parameter("publish_rate", publish_rate_);

        // TF 리스너 및 버퍼: TF를 읽어오기 위함
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // PoseStamped 메시지를 발행할 퍼블리셔
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name_, 10);
        
        // 설정된 주기에 따라 타이머 생성
        auto period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            period,
            std::bind(&TfToPosePublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "TF to Pose Publisher started.");
        RCLCPP_INFO(this->get_logger(), "Listening for TF from '%s' to '%s'.", 
                    source_frame_.c_str(), target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to topic '%s' at %.1f Hz.", 
                    topic_name_.c_str(), publish_rate_);
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            // TF 버퍼에서 가장 최신의 변환 정보를 가져옴
            transform_stamped = tf_buffer_->lookupTransform(
                source_frame_, target_frame_,
                tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), 
                *this->get_clock(), 
                5000, // 5초에 한 번만 경고 메시지 출력
                "Could not lookup transform from '%s' to '%s': %s",
                source_frame_.c_str(), target_frame_.c_str(), ex.what());
            return;
        }

        // TransformStamped 메시지를 PoseStamped 메시지로 변환
        geometry_msgs::msg::PoseStamped pose_msg;

        // 헤더 정보 복사 (stamp와 frame_id가 설정됨)
        pose_msg.header = transform_stamped.header;

        // 위치(translation) 정보 복사
        pose_msg.pose.position.x = transform_stamped.transform.translation.x;
        pose_msg.pose.position.y = transform_stamped.transform.translation.y;
        pose_msg.pose.position.z = transform_stamped.transform.translation.z;

        // 방향(rotation) 정보 복사
        pose_msg.pose.orientation = transform_stamped.transform.rotation;

        // PoseStamped 토픽 발행
        pose_publisher_->publish(pose_msg);
    }

    // 멤버 변수
    std::string source_frame_;
    std::string target_frame_;
    std::string topic_name_;
    double publish_rate_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfToPosePublisher>());
    rclcpp::shutdown();
    return 0;
}