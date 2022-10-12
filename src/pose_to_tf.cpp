#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

const static size_t count_threshold {10}; // Rest tf if no messages received for 1s

class PoseToTf : public rclcpp::Node
{
public:
    PoseToTf()
    : Node("laser_link_frame_broadcaster"),
      count_(0)
    {
        // Declare and acquire `turtlename` parameter
        parent_frame_ = this->declare_parameter<std::string>("parent_frame", "odom");
        child_frame_ = this->declare_parameter<std::string>("child_frame", "base_link");
        pose_topic_ = this->declare_parameter<std::string>("pose_topic", "laser_pose");

        // Initialize reset time
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PoseToTf::timer_callback, this));

        // Initialize the transform broadcaster
        tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to pose topic
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, 10,
        std::bind(&PoseToTf::pose_callback, this, std::placeholders::_1));
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        count_ = {0}; // Re-initialize count

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = parent_frame_;
        t.child_frame_id = child_frame_;

        t.transform.translation.x = msg->pose.position.x;
        t.transform.translation.y = msg->pose.position.y;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = msg->pose.orientation.x;
        t.transform.rotation.y = msg->pose.orientation.y;
        t.transform.rotation.z = msg->pose.orientation.z;
        t.transform.rotation.w = msg->pose.orientation.w;

        tf_broadcaster_->sendTransform(t);
    }

    void timer_callback()
    {
        // Publish identity tf if no messags recived for a while
        if (count_ > count_threshold)
        {
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = parent_frame_;
            t.child_frame_id = child_frame_;

            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;

            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(t);
        }

        count_++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string parent_frame_;
    std::string child_frame_;
    std::string pose_topic_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToTf>());
  rclcpp::shutdown();
  return 0;
}