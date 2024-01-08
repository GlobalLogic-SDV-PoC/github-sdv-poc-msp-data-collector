#include "dcol/app.hpp"

#include <chrono>
#include <cstring>
#include <exception>
#include <fstream>
#include <initializer_list>
#include <ios>
#include <ipc/client.hpp>
#include <memory>
#include <string>
#include <tuple>

#include "dcol/extractors.hpp"
#include "nlohmann/json.hpp"

namespace dcol
{
App::App()
{
    init();
    parse_config();
    configure_scheduler();
}
void App::start()
{
    using std::placeholders::_1;
    m_ipc_client->connect(m_config["server_address"], "dcol_response", m_ctx->node, std::bind(&App::on_ipc_received, this, _1), [this]()
                          {
                            for(const auto& topic : m_subbed_topics)
                            {
                                m_ipc_client->subscribe(topic);
                            } });
    rclcpp::spin(m_ctx->node);
}
void App::stop()
{
    rclcpp::shutdown();
}
void App::init()
{
    m_ctx = std::make_shared<dcol::Context>();
    m_ctx->node = std::make_shared<rclcpp::Node>("dcol");
    m_ctx->node->declare_parameter("config_path", "");
    m_scheduler = std::make_shared<dcol::Scheduler>(m_ctx);
    m_ipc_client = std::make_shared<ipc::Client>("dcol_receive");
}

void App::on_ipc_received(const std::string& payload)
{
    nlohmann::json msg = nlohmann::json::parse(payload);
    const auto* _topic = msg["topic"].get_ptr<const std::string*>();
    if (!_topic)
    {
        return;
    }
    m_scheduler->trigger_collect_event_for_topic(*_topic);
}

void App::configure_scheduler()
{
    const std::string root_send = m_config["root_send"];
    const std::string root_query = m_config["root_query"];
    using args_t = std::initializer_list<std::tuple<const char*, std::optional<nlohmann::json> (*)(const std::string&, const std::shared_ptr<Context>&)>>;
    const auto send_to_iot_fn = [this](auto&& topic, auto&& payload)
    {
        m_ipc_client->forward_payload(topic, payload);
    };
    for (const auto& e : args_t{{"temp", &get_temperature},
                                {"storage", &get_storage_space},
                                {"ram", &get_mem_info}})
    {
        const auto name = std::get<0>(e);
        const auto fn = std::get<1>(e);

        const auto& collector_config = m_config[name];
        const std::string& extract_path = collector_config["extract_path"];
        const auto collect_fn = [fn, extract_path, this]()
        {
            return fn(extract_path, m_ctx);
        };

        if (collector_config["enabled"])
        {
            RCLCPP_INFO(m_ctx->node->get_logger(), "enabled %s", name);
            m_subbed_topics.push_back(root_query + name);
            const auto trigger_interval = std::chrono::seconds(collector_config["interval"]);
            m_subscriptions.push_back(
                m_scheduler->register_automatic_collector(root_send + name, root_query + name, trigger_interval, collect_fn, send_to_iot_fn));
        }
        else
        {
            RCLCPP_INFO(m_ctx->node->get_logger(), "disabled %s", name);

            m_subscriptions.push_back(
                m_scheduler->register_query_collector(root_send + name, root_query + name, collect_fn, send_to_iot_fn));
        }
    }
}
void App::init_default_config()
{
    static constexpr auto DEFAULT_CONFIG = R"(
    {
        "server_address": "iot_server",
        "root_send": "/data_collection/send_data",
        "root_query": "/data_collection/query_data",
        "temp": {
            "enabled": true,
            "interval": 15,
            "extract_path": "/sys/class/thermal/thermal_zone0/temp",
            "topic": "/temp"
        },
        "storage": {
            "enabled": true,
            "interval": 10,
            "extract_path": "/tmp",
            "topic": "/storage"
        },
        "ram": {
            "enabled": true,
            "interval": 5,
            "extract_path": "/proc/meminfo",
            "topic": "/ram"
        }
    }
    )";
    m_config = nlohmann::json::parse(DEFAULT_CONFIG);
}
void App::parse_config()
{
    std::string config_path;
    if (m_ctx->node->get_parameter("config_path", config_path))
    {
        try
        {
            std::ifstream config(config_path);
            config.seekg(std::ios::beg);
            m_config = nlohmann::json::parse(config);
        }
        catch (std::exception& e)
        {
            RCLCPP_ERROR(m_ctx->node->get_logger(), "Failed to used config from parameter %s due to %s Using default", config_path.c_str(), e.what());
            init_default_config();
        }
    }
    else
    {
        RCLCPP_WARN(m_ctx->node->get_logger(), "Parameter 'config_path' not found. Using default");
        init_default_config();
    }
}
}  // namespace dcol