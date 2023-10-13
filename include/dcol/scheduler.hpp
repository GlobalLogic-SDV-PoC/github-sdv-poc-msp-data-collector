#pragma once

#include <chrono>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <string>

#include "context.hpp"
#include "nlohmann/json.hpp"
#include "scoped_handle.hpp"
#include "std_msgs/msg/string.hpp"

namespace dcol
{
class Scheduler
{
public:
    using CollectFn = std::function<std::optional<nlohmann::json>()>;

private:
    using PublisherPtr = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;
    using SubscriptionPtr = rclcpp::Subscription<std_msgs::msg::String>::SharedPtr;
    using TimerPtr = rclcpp::TimerBase::SharedPtr;
    struct CollectorInfo
    {
        PublisherPtr publisher_ptr;
        std::string publish_topic;
        SubscriptionPtr subscriber_ptr;
        std::string query_topic;
        CollectFn collector;
        std::optional<std::chrono::milliseconds> trigger_interval;
        std::optional<TimerPtr> trigger_timer;
    };
    // TODO: (Andrew) publish and subscribe should be to the same global message topic 
public:
    explicit Scheduler(const std::shared_ptr<Context>& context);
    auto register_automatic_collector(const std::string& publish_topic,
                                      const std::string& query_topic,
                                      std::chrono::milliseconds trigger_interval,
                                      const CollectFn& collector) -> std::shared_ptr<ScopedHandle>;
    auto register_query_collector(const std::string& publish_topic,
                                  const std::string& query_topic,
                                  const CollectFn& collector) -> std::shared_ptr<ScopedHandle>;

private:
    std::shared_ptr<Context> m_ctx;
    std::list<std::shared_ptr<CollectorInfo>> m_messages;
};
}  // namespace dcol