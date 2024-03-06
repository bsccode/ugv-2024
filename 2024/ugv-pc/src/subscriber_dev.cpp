#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/range.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("ugv_subscriber")
  {
    range_front_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/range/front", 10, std::bind(&MinimalSubscriber::range_front_topic_callback, this, _1));
    range_rear_sub = this->create_subscription<sensor_msgs::msg::Range>(
      "/range/rear", 10, std::bind(&MinimalSubscriber::range_rear_topic_callback, this, _1));
    rpm_left_sub = this->create_subscription<std_msgs::msg::Float32>(
      "/rpm/left", 10, std::bind(&MinimalSubscriber::rpm_left_topic_callback, this, _1));
    rpm_right_sub = this->create_subscription<std_msgs::msg::Float32>(
      "/rpm/right", 10, std::bind(&MinimalSubscriber::rpm_right_topic_callback, this, _1));
  }

private:
  void range_front_topic_callback(const sensor_msgs::msg::Range::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "range front: '%f'", msg->range);
  }

  void range_rear_topic_callback(const sensor_msgs::msg::Range::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "range rear: '%f'", msg->range);
  }

  void rpm_left_topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "rpm left: '%f'", msg->data);
  }

  void rpm_right_topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "rpm right: '%f'", msg->data);
  }


  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_front_sub;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_rear_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rpm_left_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rpm_right_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
