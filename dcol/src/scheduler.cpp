#include "dcol/scheduler.hpp"

#include <rclcpp/logging.hpp>
namespace dcol
{
Scheduler::Scheduler(const std::shared_ptr<Context>& context)
    : m_ctx(context)
{
    assert(m_ctx);
}
auto Scheduler::register_query_collector(const std::string& publish_topic,
                                         const std::string& query_topic,
                                         const CollectFn& collector,
                                         const SendFn& sender) -> std::shared_ptr<ScopedHandle>
{
    auto new_collector = std::make_shared<CollectorInfo>();
    new_collector->collector = collector;
    new_collector->query_topic = query_topic;
    new_collector->publish_topic = publish_topic;
    new_collector->sender = sender;
    new_collector->collector = collector;
    const auto send_data_fn = [ctx = m_ctx, weak = std::weak_ptr<CollectorInfo>(new_collector)]()
    {
        if (weak.expired())
        {
            return;
        }
        auto info = weak.lock();
        const auto data = info->collector();
        if (data)
        {
            RCLCPP_INFO(ctx->node->get_logger(), "Sending msg: %s", data->dump().c_str());
            info->sender(info->publish_topic, data->dump());
        }
        else
        {
            RCLCPP_INFO(ctx->node->get_logger(), "Failed to send msg to topic: %s", info->publish_topic.c_str());
        }
    };
    new_collector->collect_and_send = send_data_fn;
    m_messages[query_topic].push_front(std::move(new_collector));

    const auto it = std::begin(m_messages);
    const auto unsubscribe_fn = [this, it]()
    {
        m_messages.erase(it);
    };

    return std::make_shared<ScopedHandle>(unsubscribe_fn);
}

auto Scheduler::register_automatic_collector(const std::string& publish_topic,
                                             const std::string& query_topic,
                                             std::chrono::milliseconds trigger_interval,
                                             const CollectFn& collector,
                                             const SendFn& sender) -> std::shared_ptr<ScopedHandle>
{
    const auto scoped_handle = register_query_collector(publish_topic, query_topic, collector, sender);
    auto& new_collector = *m_messages[query_topic].begin();
    new_collector->trigger_interval = trigger_interval;
    new_collector->trigger_timer = m_ctx->node->create_wall_timer(trigger_interval, new_collector->collect_and_send);
    return scoped_handle;
}

void Scheduler::trigger_collect_event_for_topic(const std::string& topic)
{
    const auto it = m_messages.find(topic);
    if (it == m_messages.end())
    {
        return;
    }
    for (const auto& collector_info : it->second)
    {
        collector_info->collect_and_send();
    }
}
}  // namespace dcol