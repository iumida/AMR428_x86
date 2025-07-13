#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class DockPosePublisher : public rclcpp::Node
{
public:
  DockPosePublisher()
  : Node("dock_pose_publisher")
  {
    // 訂閱正確的 AprilTag 偵測結果 topic
    subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/camera_fork/detections", 10,
      std::bind(&DockPosePublisher::detectionCallback, this, _1));

    // 發布檢測到的 Dock 位姿
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "detected_dock_pose", 10);

    use_first_detection_ = this->declare_parameter<bool>("use_first_detection", true);
    dock_tag_id_ = this->declare_parameter<int>("dock_tag_id", 0);

    RCLCPP_INFO(this->get_logger(),
      "DockPosePublisher initialized: use_first_detection=%s, dock_tag_id=%d",
      use_first_detection_ ? "true" : "false", dock_tag_id_);
  }

private:
  void detectionCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    if (msg->detections.empty()) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;

    if (use_first_detection_) {
      const auto & det = msg->detections[0];
      assignPoseFromDetection(det, pose_msg);
      publisher_->publish(pose_msg);
      return;
    }

    for (const auto & det : msg->detections) {
      if (det.id == dock_tag_id_) {
        pose_msg.header.stamp = msg->header.stamp;
        assignPoseFromDetection(det, pose_msg);
        publisher_->publish(pose_msg);
        return;
      }
    }
  }

  void assignPoseFromDetection(
    const apriltag_msgs::msg::AprilTagDetection & det,
    geometry_msgs::msg::PoseStamped & pose_msg)
  {
    pose_msg.pose.position.x = det.centre.x;
    pose_msg.pose.position.y = det.centre.y;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.w = 1.0;
    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = 0.0;
  }

  bool use_first_detection_;
  int dock_tag_id_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockPosePublisher>());
  rclcpp::shutdown();
  return 0;
}

