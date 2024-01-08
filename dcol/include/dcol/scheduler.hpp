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

namespace dcol
{
class Scheduler
{
public:
    using CollectFn = std::function<std::optional<nlohmann::json>()>;
    using SendFn = std::function<void(std::string /*topic*/, std::string /*payload*/)>;

private:
    using TimerPtr = rclcpp::TimerBase::SharedPtr;
    struct CollectorInfo
    {
        std::string publish_topic;
        std::string query_topic;
        CollectFn collector;
        SendFn sender;
        std::function<void()> collect_and_send;
        std::optional<std::chrono::milliseconds> trigger_interval;
        std::optional<TimerPtr> trigger_timer;
    };

public:
    explicit Scheduler(const std::shared_ptr<Context>& context);
    auto register_automatic_collector(const std::string& publish_topic,
                                      const std::string& query_topic,
                                      std::chrono::milliseconds trigger_interval,
                                      const CollectFn& collector,
                                      const SendFn& sender) -> std::shared_ptr<ScopedHandle>;
    auto register_query_collector(const std::string& publish_topic,
                                  const std::string& query_topic,
                                  const CollectFn& collector,
                                  const SendFn& sender) -> std::shared_ptr<ScopedHandle>;
    void trigger_collect_event_for_topic(const std::string& topic);

private:
    std::shared_ptr<Context> m_ctx;
    std::unordered_map<std::string, std::list<std::shared_ptr<CollectorInfo>>> m_messages;
};
}  // namespace dcol