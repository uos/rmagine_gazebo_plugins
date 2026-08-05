#include "rmagine_gazebo_plugins/ros/gt_localization.hpp"

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rmagine_gazebo_plugins
{

GTLocalizationNode::GTLocalizationNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("gt_localization_node", options)
{
  this->declare_parameter("gz_parent_frame", "world");
  this->declare_parameter("gz_child_frame", "robot");
  this->declare_parameter("ros_parent_frame", "map");
  this->declare_parameter("ros_child_frame", "base_footprint");
  this->declare_parameter("ros_odom_frame", "odom");

  gz_parent_frame_ = this->get_parameter("gz_parent_frame").as_string();
  gz_child_frame_ = this->get_parameter("gz_child_frame").as_string();
  ros_parent_frame_ = this->get_parameter("ros_parent_frame").as_string();
  ros_child_frame_ = this->get_parameter("ros_child_frame").as_string();
  ros_odom_frame_ = this->get_parameter("ros_odom_frame").as_string();
  skip_odom_frame_ = !ros_odom_frame_.empty();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // subscribe to ground-truth pose messages bridged from Gazebo
  sub_tf_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_gt", 10,
    [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr & msgs) -> void
    {
      tf_cb(msgs);
    });

  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Searching for Gazebo transform " << gz_child_frame_ << " -> " << gz_parent_frame_);
}

void GTLocalizationNode::tf_cb(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msgs)
{
  for (const geometry_msgs::msg::TransformStamped & T : msgs->transforms) {
    if (T.header.frame_id != gz_parent_frame_ || T.child_frame_id != gz_child_frame_) {
      continue;
    }

    // NR convention: Tmb = map <- base_footprint (ground truth from Gazebo)
    geometry_msgs::msg::TransformStamped Tmb = T;

    if (!skip_odom_frame_) {
      // publish map -> odom as the correction of the odometry:
      // Tmo = Tmb * Tob^-1  with Tob = odom <- base_footprint
      geometry_msgs::msg::TransformStamped Tob;
      try {
        Tob = tf_buffer_->lookupTransform(
          ros_odom_frame_, ros_child_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000, "Could not transform %s to %s: %s",
          ros_child_frame_.c_str(), ros_odom_frame_.c_str(), ex.what());
        return;
      }

      tf2::Transform Tmb_tf;
      tf2::fromMsg(Tmb.transform, Tmb_tf);

      tf2::Transform Tob_tf;
      tf2::fromMsg(Tob.transform, Tob_tf);

      const tf2::Transform Tmo_tf = Tmb_tf * Tob_tf.inverse();

      geometry_msgs::msg::TransformStamped Tmo;
      Tmo.header.frame_id = ros_parent_frame_;
      // use the odometry time, since this transform corrects the odometry
      Tmo.header.stamp = Tob.header.stamp;
      Tmo.child_frame_id = ros_odom_frame_;
      Tmo.transform = tf2::toMsg(Tmo_tf);

      tf_broadcaster_->sendTransform(Tmo);
    } else {
      // no odom frame configured: publish the ground truth directly
      Tmb.header.frame_id = ros_parent_frame_;
      Tmb.child_frame_id = ros_child_frame_;
      tf_broadcaster_->sendTransform(Tmb);
    }
  }
}

}  // namespace rmagine_gazebo_plugins

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmagine_gazebo_plugins::GTLocalizationNode)