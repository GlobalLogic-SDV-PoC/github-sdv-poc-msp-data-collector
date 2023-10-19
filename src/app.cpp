#include "dcol/app.hpp"

#include <chrono>
#include <cstring>
#include <exception>
#include <fstream>
#include <initializer_list>
#include <ios>
#include <string>
#include <tuple>

#include "dcol/extractors.hpp"
#include "fmt/core.h"

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
}
void App::configure_scheduler()
{
    const std::string root_send = m_config["root_send"];
    const std::string root_query = m_config["root_query"];
    using args_t = std::initializer_list<std::tuple<const char*, std::optional<nlohmann::json> (*)(const std::string&, const std::shared_ptr<Context>&)>>;

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

            const auto trigger_interval = std::chrono::seconds(collector_config["interval"]);
            m_subscriptions.push_back(
                m_scheduler->register_automatic_collector(root_send + name, root_query + name, trigger_interval, collect_fn));
        }
        else
        {
            RCLCPP_INFO(m_ctx->node->get_logger(), "disabled %s", name);

            m_subscriptions.push_back(
                m_scheduler->register_query_collector(root_send + name, root_query + name, collect_fn));
        }
    }
}
void App::init_default_config()
{
    static constexpr auto DEFAULT_CONFIG = R"(
    {
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