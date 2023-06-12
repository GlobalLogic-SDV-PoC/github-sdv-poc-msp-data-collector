#include "dcol/app.hpp"

#include <fstream>
#include <memory>
#include <vector>

#include "dcol/extractors.hpp"
#include "dcol/fs.hpp"
#include "ipc/util.hpp"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/spdlog.h"

#ifdef __ANDROID__
#include "spdlog/sinks/android_sink.h"
#else
#include "spdlog/sinks/stdout_color_sinks.h"
#endif

using namespace std::literals;

namespace
{
bool topic_matches(std::string_view topic1, std::string_view topic2)
{
    // TODO: wildcards
    return topic1 == topic2;
}
}  // namespace
namespace dcol
{
App::App(nlohmann::json config)
    : m_work_guard(net::make_work_guard(m_context))
    , m_config(std::move(config))
{
}
App::~App()
{
    stop();
}
void App::init()
{
    SPDLOG_DEBUG("[dcol] App initialization: starting...");
    initIpc();
    initDataCollection();
    SPDLOG_DEBUG("[dcol] App initialization: done.");
}

void App::initIpc()
{
    auto& ipc_config = m_config["ipc"];
    ipc::Client::Config client_config;
    client_config.host = ipc_config["host"];
    client_config.service = ipc_config["service"];
    client_config.header_buffer_size = ipc_config["header_buffer_size"];
    client_config.body_buffer_size = ipc_config["body_buffer_size"];
    client_config.on_receive_handler = ipc::bind_front(&App::onIpcReceived, this);
    client_config.on_connected_handler = std::bind(&App::onIpcConnected, this);

    m_ipc_client = std::make_shared<ipc::Client>(m_context, std::move(client_config));
}

void App::initDefaultLogger(std::string_view filepath,
                            size_t max_size,
                            size_t max_files,
                            std::chrono::seconds flush_interval)
{
#ifdef __ANDROID__
    auto console_logger = std::make_shared<spdlog::sinks::android_sink_mt>("dcol");
#else
    auto console_logger = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
#endif
    console_logger->set_level(spdlog::level::trace);

    auto file_logger
        = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(filepath.data(),
                                                                 max_size,
                                                                 max_files);
    file_logger->set_level(spdlog::level::trace);

    m_log_dir = fs::absolute(
        fs::path(
            file_logger->filename())
            .parent_path());

    auto new_logger = std::make_shared<spdlog::logger>("file_console",
                                                       spdlog::sinks_init_list{std::move(file_logger),
                                                                               std::move(console_logger)});

    spdlog::set_default_logger(std::move(new_logger));
    spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
    spdlog::set_level(static_cast<spdlog::level::level_enum>(SPDLOG_ACTIVE_LEVEL));
    spdlog::flush_every(flush_interval);
}
void App::onIpcConnected()
{
    SPDLOG_DEBUG("[dcol] IPC connected.");
    auto subscribe_packet = std::make_shared<ipc::Packet>();
    subscribe_packet->header["action"] = "subscribe";

    auto& collectors_info = m_config["collectors"];
    std::string root_query = collectors_info["root_query"];
    if (collectors_info["temp"]["enabled"])
    {
        subscribe_packet->header["topic"] = root_query + collectors_info["temp"]["topic"].get<std::string>();
        m_ipc_client->post(subscribe_packet);
    }
    if (collectors_info["storage"]["enabled"])
    {
        subscribe_packet->header["topic"] = root_query + collectors_info["storage"]["topic"].get<std::string>();
        m_ipc_client->post(subscribe_packet);
    }
    if (collectors_info["ram"]["enabled"])
    {
        subscribe_packet->header["topic"] = root_query + collectors_info["ram"]["topic"].get<std::string>();
        m_ipc_client->post(subscribe_packet);
    }
}

void App::onIpcReceived(std::shared_ptr<ipc::Packet> packet)
{
    SPDLOG_DEBUG("[dcol] IPC received.");
    if (packet->header["action"] != "forward")
    {
        return;
    }
    // TODO: proper protocol handling
    const auto topic = packet->header["topic"].get<std::string>();
    const auto slash = topic.rfind('\\');
    if (slash == std::string::npos)
    {
        SPDLOG_ERROR("[dcol] invalid topic {}.", topic);
        return;
    }
    const auto root_query = m_config["collectors"]["root_query"].get<std::string>();
    const auto topic_root = topic.substr(0, slash);
    if (!topic_matches(topic_root, root_query))
    {
        SPDLOG_ERROR("[dcol] invalid topic root_query: {} != {} ", root_query, topic_root);
        return;
    }
    queryData(topic.substr(slash));
}

void App::queryData(const std::string& topic)
{
    SPDLOG_DEBUG("[dcol] Query data: starting...");
    const auto it = m_collect_handlers.find(topic);
    if (it == std::end(m_collect_handlers))
    {
        SPDLOG_ERROR("[dcol] no topic handlers for topic: {} ", topic);
        return;
    }
    sendCollectionData(m_config["collectors"]["root_send"].get<std::string>() + it->first, it->second());
    SPDLOG_DEBUG("[dcol] Query data: done.");
}

void App::initDataCollection()
{
    SPDLOG_DEBUG("[dcol] Register Infinite Data Collection: starting...");
    m_event_scheduler = std::make_shared<EventScheduler>();
    const auto register_collector = [this](auto collect_functor, const nlohmann::json& collector_config) {
        const auto collet_wrapper = [this](auto&& callable, std::string topic, auto&&... args) {
            sendCollectionData(m_config["collectors"]["root_send"].get<std::string>() + topic,
                               std::invoke(std::forward<decltype(callable)>(callable), std::forward<decltype(args)>(args)...));
        };
        if (collector_config["enabled"].get<bool>())
        {
            m_event_scheduler->schedule(std::make_unique<InfiniteEvent>(
                std::bind(collet_wrapper, collect_functor, collector_config["topic"].get<std::string>(), fs::path(collector_config["extract_path"].get<std::string>())),
                std::chrono::seconds(collector_config["interval"])));
        }
        m_collect_handlers[collector_config["topic"]] = std::bind(std::move(collect_functor), fs::path(collector_config["extract_path"].get<std::string>()));
    };
    auto& collectors_info = m_config["collectors"];
    {
        auto& temp = collectors_info["temp"];
        register_collector(&getTemperature, temp);
    }
    {
        auto& storage = collectors_info["storage"];
        register_collector(&getStorageSpace, storage);
    }
    {
        auto& ram = collectors_info["ram"];
        register_collector(&getMemInfo, ram);
    }
    SPDLOG_DEBUG("[dcol] Register Infinite Data Collection: done.");
}

void App::sendCollectionData(const std::string& topic, const nlohmann::json& collection_data)
{
    SPDLOG_DEBUG("[dcol] Send collection data: starting...");
    auto logs_packet = std::make_shared<ipc::Packet>();
    logs_packet->header["action"] = "forward";
    logs_packet->header["topic"] = topic;
    logs_packet->payload = collection_data.dump();
    m_ipc_client->post(logs_packet);
    SPDLOG_DEBUG("[dcol] Send collection data: done.");
}

void App::start()
{
    SPDLOG_DEBUG("[dcol] Starting app");
    m_ipc_client->start();
    m_ipc_thread = std::thread([this]() { m_context.run(); });
    auto start = std::chrono::steady_clock::now();
    while (!m_canceled)
    {
        auto now = std::chrono::steady_clock::now();
        m_event_scheduler->update(std::chrono::duration_cast<std::chrono::milliseconds>(now - start));
        start = std::move(now);
        std::this_thread::sleep_for(500ms);
    }
}
void App::stop()
{
    SPDLOG_DEBUG("[dcol] Stopping app.");
    m_canceled = true;
    m_ipc_client->stop();
    m_context.stop();
    m_ipc_thread.join();
}
}  // namespace dcol
