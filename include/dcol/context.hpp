#pragma once
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace dcol
{
struct Context
{
    using PublisherPtr = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;
    
    std::shared_ptr<rclcpp::Node> node;
    PublisherPtr iot_node;
};
}  // namespace dcol