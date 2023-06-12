#pragma once
#include <string_view>
#include <thread>
#include <unordered_map>

#include "event_scheduler.hpp"
#include "extractors.hpp"
#include "ipc/client.hpp"
#include "nlohmann/json.hpp"

namespace dcol
{
namespace net = ipc::net;
class App
{
    using work_guard = ipc::net::executor_work_guard<ipc::net::io_context::executor_type>;

public:
    App(nlohmann::json config);
    ~App();
    void init();
    void start();
    void stop();
    void initDefaultLogger(std::string_view filepath,
                           size_t max_size,
                           size_t max_files,
                           std::chrono::seconds flush_interval);

private:
    void initDataCollection();
    void initIpc();
    void onIpcConnected();
    void onIpcReceived(std::shared_ptr<ipc::Packet> packet);
    void queryData(const std::string& topic);
    void sendCollectionData(const std::string& topic, const nlohmann::json& collection_data);

private:
    net::io_context m_context;
    work_guard m_work_guard;
    nlohmann::json m_config;
    std::shared_ptr<ipc::Client> m_ipc_client;
    std::shared_ptr<EventScheduler> m_event_scheduler;
    std::thread m_ipc_thread;
    std::atomic_bool m_canceled = false;
    std::unordered_map<std::string, std::function<nlohmann::json()>> m_collect_handlers;
    std::string m_log_dir;
};
}  // namespace dcol