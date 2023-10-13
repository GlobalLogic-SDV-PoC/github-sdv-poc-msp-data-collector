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
                                         const CollectFn& collector) -> std::shared_ptr<ScopedHandle>
{
    auto new_collector = std::make_shared<CollectorInfo>();
    new_collector->collector = collector;
    new_collector->publisher_ptr = m_ctx->node->create_publisher<std_msgs::msg::String>(publish_topic, 5);
    const auto send_data_fn = [weak = std::weak_ptr<CollectorInfo>(new_collector)]()
    {
        if (weak.expired())
        {
            return;
        }
        auto info = weak.lock();
        const auto data = info->collector();
        if (data)
        {
            auto message = std_msgs::msg::String();
            message.data = data->dump();
            info->publisher_ptr->publish(message);
        }
    };
    new_collector->subscriber_ptr = m_ctx->node->create_subscription<std_msgs::msg::String>(query_topic, 5, [send_data_fn](const std_msgs::msg::String&)
                                                                                            { send_data_fn(); });
    m_messages.push_front(std::move(new_collector));

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
                                             const CollectFn& collector) -> std::shared_ptr<ScopedHandle>
{
    auto new_collector = std::make_shared<CollectorInfo>();
    new_collector->trigger_interval = trigger_interval;
    new_collector->collector = collector;
    new_collector->publisher_ptr = m_ctx->node->create_publisher<std_msgs::msg::String>(publish_topic, 5);
    const auto send_data_fn = [ctx = m_ctx, weak = std::weak_ptr<CollectorInfo>(new_collector)]()
    {
        if (weak.expired())
        {
            return;
        }
        const auto info = weak.lock();
        const auto data = info->collector();
        if (data)
        {
            RCLCPP_INFO(ctx->node->get_logger(), "Sending msg: %s", data->dump().c_str());
            auto message = std_msgs::msg::String();
            message.data = data->dump();
            info->publisher_ptr->publish(message);
        }
    };
    new_collector->trigger_timer = m_ctx->node->create_wall_timer(trigger_interval, send_data_fn);
    new_collector->subscriber_ptr = m_ctx->node->create_subscription<std_msgs::msg::String>(query_topic, 5, [send_data_fn](const std_msgs::msg::String&)
                                                                                            { send_data_fn(); });
    m_messages.push_front(std::move(new_collector));

    const auto it = std::begin(m_messages);
    const auto unsubscribe_fn = [this, it]()
    {
        m_messages.erase(it);
    };

    return std::make_shared<ScopedHandle>(unsubscribe_fn);
}
}  // namespace dcol