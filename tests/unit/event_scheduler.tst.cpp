#include "dcol/event_scheduler.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using namespace std::chrono_literals;

class TestableFunctor
{
public:
    MOCK_METHOD(void, handle, (), ());
};

class EventSchedulerTest : public ::testing::Test
{
protected:
    TestableFunctor m_functor;
    dcol::EventScheduler m_scheduler;
};

TEST_F(EventSchedulerTest, SingleEvent)
{
    EXPECT_CALL(m_functor, handle()).Times(1);
    dcol::EventScheduler scheduler;
    m_scheduler.schedule(std::make_unique<dcol::Event>([this]() { m_functor.handle(); }, 1s));
    m_scheduler.update(1min);
}

TEST_F(EventSchedulerTest, NTimesRepeatedEvent)
{
    EXPECT_CALL(m_functor, handle()).Times(5);
    m_scheduler.schedule(std::make_unique<dcol::Event>([this]() { m_functor.handle(); }, 5s, 5u));
    m_scheduler.update(1min);
}

TEST_F(EventSchedulerTest, InfinitelyRepeatedEvent)
{
    EXPECT_CALL(m_functor, handle()).Times(1min / 2s);
    m_scheduler.schedule(std::make_unique<dcol::InfiniteEvent>([this]() { m_functor.handle(); }, 2s));
    m_scheduler.update(1min);
}