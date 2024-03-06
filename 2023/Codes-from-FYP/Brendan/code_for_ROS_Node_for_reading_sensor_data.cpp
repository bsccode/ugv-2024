#include <memory>
#include “rclcpp/rclcpp.hpp”
#include “sensor_msgs/msg/range.hpp”
using std::placeholders::_1;
#define NODE_NAME “sensor_data_node”
#define FRONT_TOPIC_NAME “microROS/tof/front”
#define REAR_TOPIC_NAME “microROS/tof/rear”
int front;
int rear;
class TOFSubscriber : public rclcpp::Node
{
public:
TOFSubscriber()
: Node(NODE_NAME)
{
front_subscription_ = this->create_subscription<sensor_msgs::msg::Range>(
FRONT_TOPIC_NAME, 10, std::bind(&TOFSubscriber::front_topic_callback, this, _1));
rear_subscription_ = this->create_subscription<sensor_msgs::msg::Range>(
REAR_TOPIC_NAME, 10, std::bind(&TOFSubscriber::rear_topic_callback, this, _1));
RCLCPP_INFO(this->get_logger(), “Sensor data node has been initialised”);
}
private:
void front_topic_callback(const sensor_msgs::msg::Range::SharedPtr msg) const
{
// RCLCPP_INFO(this->get_logger(), “Front: %.2f mm”, msg->range);
front = msg->range;
}
void rear_topic_callback(const sensor_msgs::msg::Range::SharedPtr msg) const
{
// RCLCPP_INFO(this->get_logger(), “Front: %.2f mm”, msg->range);
rear = msg->range;
}
rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr front_subscription_;
rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rear_subscription_;
};
int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::Node::SharedPtr node = std::make_shared<TOFSubscriber>();
while(1){
rclcpp::spin_some(node);
// RCLCPP_INFO(node.get_logger(), “Front: %.2f mm Rear: %.2f mm”, front, rear);
}
rclcpp::shutdown();
return 0;
}