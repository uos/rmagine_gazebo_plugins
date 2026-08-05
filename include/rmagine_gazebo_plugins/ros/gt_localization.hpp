#ifndef RMAGINE_GAZEBO_PLUGINS_ROS_GT_LOCALIZATION_HPP_
#define RMAGINE_GAZEBO_PLUGINS_ROS_GT_LOCALIZATION_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace rmagine_gazebo_plugins
{

/**
 * Publishes the map -> odom correction transform from the ground-truth model
 * pose reported by Gazebo (via a bridged pose topic, by default /tf_gt).
 *
 * Robot-agnostic: the Gazebo world / model frames and the ROS frames are
 * parameters, so any robot can reuse this node.
 */
class GTLocalizationNode : public rclcpp::Node
{
public:
  explicit GTLocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void tf_cb(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msgs);

  std::string gz_parent_frame_;
  std::string gz_child_frame_;
  std::string ros_parent_frame_;
  std::string ros_child_frame_;

  // odom frame to correct; when empty the ground truth is published directly
  // as ros_parent_frame -> ros_child_frame
  std::string ros_odom_frame_;
  bool skip_odom_frame_;

  // TF stuff
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
};

}  // namespace rmagine_gazebo_plugins

#endif  // RMAGINE_GAZEBO_PLUGINS_ROS_GT_LOCALIZATION_HPP_