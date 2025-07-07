#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class DockPosePublisher : public rclcpp::Node
{
public:
  DockPosePublisher() : Node("dock_pose_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    subscription_ = this->create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      "tag_detections", 10, std::bind(&DockPosePublisher::detectionCallback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&DockPosePublisher::timerCallback, this));

    use_first_detection_ = this->declare_parameter<bool>("use_first_detection", true);
    dock_tag_family_ = this->declare_parameter<std::string>("dock_tag_family", "tag36h11");
    dock_tag_id_ = this->declare_parameter<int>("dock_tag_id", 0);

    last_detection_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
    has_last_tag_global_pose_ = false;
    RCLCPP_INFO(this->get_logger(), "DockPosePublisher initialized, use_first_detection: %s, family: %s, id: %d",
                use_first_detection_ ? "true" : "false", dock_tag_family_.c_str(), dock_tag_id_);
  }

private:
  void detectionCallback(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    rclcpp::Clock clock(RCL_SYSTEM_TIME);

    if (msg->detections.empty()) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;

    if (use_first_detection_) {
      pose_msg.header = msg->header;
      pose_msg.pose = msg->detections[0].pose.pose.pose;
      publisher_->publish(pose_msg);
      storeTagGlobalPose(pose_msg);
      last_detection_time_ = rclcpp::Time(msg->header.stamp, clock.get_clock_type());
      return;
    }

    for (const auto & detection : msg->detections) {
      if (detection.family == dock_tag_family_ && detection.id == dock_tag_id_) {
        pose_msg.header = msg->header;
        pose_msg.pose = detection.pose.pose.pose;
        publisher_->publish(pose_msg);
        storeTagGlobalPose(pose_msg);
        last_detection_time_ = rclcpp::Time(msg->header.stamp, clock.get_clock_type());
        return;
      }
    }
  }

  void timerCallback()
  {
    rclcpp::Clock clock(RCL_SYSTEM_TIME);

    if ((clock.now() - last_detection_time_).seconds() > 1.0 && has_last_tag_global_pose_) {
      try {
        geometry_msgs::msg::TransformStamped tf_camera = tf_buffer_.lookupTransform(
          "odom", "camera_fork", tf2::TimePointZero);

        tf2::Transform T_odom_camera;
        tf2::fromMsg(tf_camera.transform, T_odom_camera);

        tf2::Transform T_odom_tag;
        tf2::fromMsg(last_tag_global_pose_.pose, T_odom_tag);

        tf2::Transform T_camera_tag = T_odom_camera.inverse() * T_odom_tag;

        geometry_msgs::msg::PoseStamped tag_in_camera;
        tag_in_camera.header.frame_id = "camera_fork";
        tag_in_camera.header.stamp = clock.now();
        tf2::toMsg(T_camera_tag, tag_in_camera.pose);

        publisher_->publish(tag_in_camera);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform tag pose: %s", ex.what());
      }
    }
  }

  void storeTagGlobalPose(const geometry_msgs::msg::PoseStamped & tag_pose_in_camera)
  {
    try {
      geometry_msgs::msg::TransformStamped tf_camera = tf_buffer_.lookupTransform(
        "odom", tag_pose_in_camera.header.frame_id, tf2::TimePointZero);

      tf2::Transform T_odom_camera;
      tf2::fromMsg(tf_camera.transform, T_odom_camera);

      tf2::Transform T_camera_tag;
      tf2::fromMsg(tag_pose_in_camera.pose, T_camera_tag);

      tf2::Transform T_odom_tag = T_odom_camera * T_camera_tag;

      last_tag_global_pose_.header.frame_id = "odom";
      last_tag_global_pose_.header.stamp = this->now();
      tf2::toMsg(T_odom_tag, last_tag_global_pose_.pose);
      has_last_tag_global_pose_ = true;

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not update global tag pose: %s", ex.what());
    }
  }

  bool use_first_detection_;
  std::string dock_tag_family_;
  int dock_tag_id_;
  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_detection_time_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::PoseStamped last_tag_global_pose_;
  bool has_last_tag_global_pose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
