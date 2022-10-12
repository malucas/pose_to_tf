#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PosePublisher : public rclcpp::Node
{
  public:
    PosePublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("laser_pose", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&PosePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "laser_link";
      msg.pose.position.x = 1.0;
      msg.pose.position.y = 2.0;
      msg.pose.position.z = 3.0;
      RCLCPP_INFO(this->get_logger(), "Publishing pose");
      publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePublisher>());
  rclcpp::shutdown();
  return 0;
}