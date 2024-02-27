#pragma once

#if defined(_MSC_VER) && _MSC_VER < 1914
#include <experimental/filesystem>
#elif defined(__has_include)
#if __has_include(<filesystem>)
#include <filesystem>
namespace dcol
{
namespace fs = ::std::filesystem;
}
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace dcol
{
namespace fs = ::std::experimental::filesystem;
}
#endif
#endif
