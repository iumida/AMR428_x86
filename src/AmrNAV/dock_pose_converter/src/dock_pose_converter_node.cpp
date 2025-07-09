#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class DockPosePublisher : public rclcpp::Node
{
public:
  DockPosePublisher() : Node("dock_pose_publisher")
  {
    subscription_ = this->create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      "tag_detections", 10, std::bind(&DockPosePublisher::detectionCallback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", 10);

    use_first_detection_ = this->declare_parameter<bool>("use_first_detection", true);
    dock_tag_family_ = this->declare_parameter<std::string>("dock_tag_family", "tag36h11");
    dock_tag_id_ = this->declare_parameter<int>("dock_tag_id", 0);

    RCLCPP_INFO(this->get_logger(), "DockPosePublisher initialized, use_first_detection: %s, family: %s, id: %d",
                use_first_detection_ ? "true" : "false", dock_tag_family_.c_str(), dock_tag_id_);
  }

private:
  void detectionCallback(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    if (msg->detections.empty()) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;

    if (use_first_detection_) {
      pose_msg.header = msg->header;
      pose_msg.pose = msg->detections[0].pose.pose.pose;
      publisher_->publish(pose_msg);
      RCLCPP_DEBUG(this->get_logger(), "Published first detection as dock pose.");
      return;
    }

    for (const auto & detection : msg->detections) {
      if (detection.family == dock_tag_family_ && detection.id == dock_tag_id_) {
        pose_msg.header = msg->header;
        pose_msg.pose = detection.pose.pose.pose;
        publisher_->publish(pose_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published matched detection as dock pose.");
        return;
      }
    }
  }

  bool use_first_detection_;
  std::string dock_tag_family_;
  int dock_tag_id_;
  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockPosePublisher>());
  rclcpp::shutdown();
  return 0;
}

