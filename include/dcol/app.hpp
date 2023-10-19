#pragma once
#include <memory>

#include "dcol/context.hpp"
#include "ipc/nlohmann/json.hpp"
#include "scheduler.hpp"

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

private:
    nlohmann::json m_config;

    std::shared_ptr<Context> m_ctx;
    std::shared_ptr<Scheduler> m_scheduler;
    std::vector<std::shared_ptr<ScopedHandle>> m_subscriptions;
};
}  // namespace dcol