#pragma once
#include <memory>

#include "dcol/context.hpp"
#include "nlohmann/json.hpp"
#include "scheduler.hpp"
#include "ipc/client.hpp"

namespace dcol
{
class App
{
public:
    App();
    void start();
    void stop();

private:
    void init();
    void configure_scheduler();
    void init_default_config();
    void parse_config();
    void on_ipc_received(const std::string& payload);

private:
    nlohmann::json m_config;

    std::shared_ptr<Context> m_ctx;
    std::shared_ptr<Scheduler> m_scheduler;
    std::shared_ptr<ipc::Client> m_ipc_client;
    std::vector<std::string> m_subbed_topics;
    std::vector<std::shared_ptr<ScopedHandle>> m_subscriptions;
};
}  // namespace dcol