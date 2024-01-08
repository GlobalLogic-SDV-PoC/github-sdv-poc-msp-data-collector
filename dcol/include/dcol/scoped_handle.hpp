#pragma once

#include <functional>
#include <type_traits>

namespace dcol
{
class ScopedHandle
{
public:
    using OnScopeExitFn = std::function<void()>;

public:
    template <typename T, typename = std::enable_if_t<std::is_constructible_v<OnScopeExitFn, T>>>
    explicit ScopedHandle(T&& exit_fn)
        : m_exit_fn(std::forward<T>(exit_fn))
    {
    }
    ~ScopedHandle()
    {
        m_exit_fn();
    }
    ScopedHandle(const ScopedHandle&) = delete;
    void operator=(const ScopedHandle&) = delete;

private:
    OnScopeExitFn m_exit_fn;
};
}  // namespace dcol