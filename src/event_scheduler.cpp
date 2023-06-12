#include "dcol/event_scheduler.hpp"

#include <cassert>

#include "spdlog/spdlog.h"

using std::chrono::milliseconds;

namespace dcol
{
InfiniteEvent::InfiniteEvent(const std::function<void()>& callback, const milliseconds& period)
    : m_callback(callback)
    , m_period(period)
{
    assert(callback);
}

void InfiniteEvent::update(const milliseconds& elapsed)
{
    SPDLOG_DEBUG("[dcol] Update InfiniteEvent: starting...");
    m_current_time += elapsed;
    while (m_current_time >= m_period)
    {
        m_callback();
        m_current_time -= m_period;
    }
    SPDLOG_DEBUG("[dcol] Update InfiniteEvent: done.");
}

Event::Event(const std::function<void()>& callback, const milliseconds& period, uint32_t times_to_repeat)
    : m_callback(callback)
    , m_period(period)
    , m_times_to_repeat(times_to_repeat)
{
    assert(times_to_repeat);
    assert(callback);
}

void Event::update(const milliseconds& elapsed)
{
    SPDLOG_DEBUG("[dcol] Update Event: starting...");
    if (is_finished())
    {
        return;
    }
    m_current_time += elapsed;
    while (!is_finished() && m_current_time >= m_period)
    {
        m_callback();
        m_current_time -= m_period;
        set_finished(!(--m_times_to_repeat));
    }
    SPDLOG_DEBUG("[dcol] Update Event: done.");
}
void EventScheduler::update(const milliseconds& elapsed)
{
    SPDLOG_DEBUG("[dcol] Update events: starting...");
    const std::scoped_lock lock(m_mutex);
    for (auto it = m_events.begin(); it != m_events.end();)
    {
        it->second->update(elapsed);
        if (it->second->is_finished())
        {
            it = m_events.erase(it);
        }
        else
        {
            ++it;
        }
    };
    SPDLOG_DEBUG("[dcol] Update events: done.");
}
size_t EventScheduler::schedule(std::unique_ptr<IEvent>&& event)
{
    SPDLOG_DEBUG("[dcol] Schedule event: starting...");
    assert(event);
    const std::scoped_lock lock(m_mutex);
    static size_t descriptor = 0;
    m_events.emplace(descriptor, std::move(event));
    SPDLOG_DEBUG("[dcol] Schedule event: done.");
    return descriptor++;
}

void EventScheduler::unschedule(size_t descriptor)
{
    SPDLOG_DEBUG("[dcol] Schedule event: starting...");
    const std::scoped_lock lock(m_mutex);
    m_events.erase(descriptor);
    SPDLOG_DEBUG("[dcol] Unschedule event: done.");
}
}  // namespace dcol