#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace dcol
{
class IEvent
{
public:
    virtual void update(const std::chrono::milliseconds& elapsed) = 0;
    bool is_finished() const { return m_finished; }
    void set_finished(bool finished) { m_finished = finished; }

    virtual ~IEvent() = default;

private:
    bool m_finished{false};
};

class InfiniteEvent : public IEvent
{
public:
    InfiniteEvent(const std::function<void()>& callback, const std::chrono::milliseconds& period);
    void update(const std::chrono::milliseconds& elapsed) override;

private:
    std::function<void()> m_callback;
    std::chrono::milliseconds m_period{};
    std::chrono::milliseconds m_current_time{};
};
class Event : public IEvent
{
public:
    Event(const std::function<void()>& callback, const std::chrono::milliseconds& period, uint32_t times_to_repeat = 1);
    void update(const std::chrono::milliseconds& elapsed) override;

private:
    std::function<void()> m_callback;
    std::chrono::milliseconds m_period{};
    std::chrono::milliseconds m_current_time{};
    uint32_t m_times_to_repeat{};
};

class EventScheduler
{
public:
    void update(const std::chrono::milliseconds& elapsed);
    size_t schedule(std::unique_ptr<IEvent>&& event);
    void unschedule(size_t uuid);

private:
    std::unordered_map<size_t, std::unique_ptr<IEvent>> m_events;
    std::mutex m_mutex;
};
}  // namespace dcol
