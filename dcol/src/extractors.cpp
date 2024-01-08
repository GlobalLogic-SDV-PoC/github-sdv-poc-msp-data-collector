#include "dcol/extractors.hpp"

#include <sys/statvfs.h>

#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>

namespace dcol
{
namespace detail
{
auto get_data_from_file(const std::string& path) -> std::optional<std::string>
{
    std::ifstream file(path);

    if (!file)
    {
        return {};
    }

    std::stringstream buffer;
    buffer << file.rdbuf();

    if (file.fail())
    {
        return {};
    }

    file.close();
    return buffer.str();
}

auto split_string(const std::string& str) -> std::vector<std::string>
{
    std::vector<std::string> tokens;

    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, '\n'))
    {
        tokens.push_back(token);
    }

    return tokens;
}
auto parse_mem_info(const std::string& data) -> std::optional<nlohmann::json>
{
    auto split_data = detail::split_string(data);
    std::string label;
    std::int64_t memTotal = -1, memAvail = -1, value;
    for (const auto& line : split_data)
    {
        std::stringstream ss{line};
        ss >> label >> value;

        if (label == "MemTotal:")
        {
            memTotal = value;
            continue;
        }

        if (label == "MemAvailable:")
        {
            memAvail = value;
            continue;
        }
    }
    if (memTotal == -1 || memAvail == -1)
    {
        return {};
    }

    nlohmann::json result;
    result["total"] = memTotal;
    result["available"] = memAvail;
    result["used"] = memTotal - memAvail;
    return result;
}
auto parse_temperature(const std::string& data) -> std::optional<nlohmann::json>
{
    try
    {
        int cpuTemp = std::stoi(data);
        cpuTemp /= 1000;
        nlohmann::json result;
        result["temp"] = cpuTemp;
        return result;
    }
    catch (std::exception& e)
    {
        return {};
    }
}
}  // namespace detail
auto get_mem_info(const std::string& path, const std::shared_ptr<Context>& ctx) -> std::optional<nlohmann::json>
{
    auto data = detail::get_data_from_file(path);
    if (!data)
    {
        RCLCPP_INFO(ctx->node->get_logger(), "Failed to load file");
        return {};
    }
    auto result = detail::parse_mem_info(*data);
    if (!result)
    {
        RCLCPP_ERROR(ctx->node->get_logger(), "Failed to parse mem_info");
    }
    return result;
}

auto get_temperature(const std::string& path, const std::shared_ptr<Context>& ctx) -> std::optional<nlohmann::json>
{
    const auto data = detail::get_data_from_file(path);
    if (!data)
    {
        RCLCPP_INFO(ctx->node->get_logger(), "Failed to load file");
        return {};
    }
    auto result = detail::parse_temperature(*data);
    if (!result)
    {
        RCLCPP_ERROR(ctx->node->get_logger(), "Failed to parse temperature");
    }
    return result;
}

auto get_storage_space(const std::string& path, const std::shared_ptr<Context>& ctx) -> std::optional<nlohmann::json>
{
    struct statvfs stat;
    if (statvfs(path.c_str(), &stat) == 0)
    {
        const auto totalSpace = stat.f_blocks * stat.f_frsize;
        const auto freeSpace = stat.f_bfree * stat.f_frsize;
        const auto availableSpace = stat.f_bavail * stat.f_frsize;

        nlohmann::json result;
        result["available"] = availableSpace / (1024 * 1024);
        result["capacity"] = totalSpace / (1024 * 1024);
        result["free"] = freeSpace / (1024 * 1024);
        return result;
    }
    RCLCPP_ERROR(ctx->node->get_logger(), "Failed to get storage space from: %s", path.c_str());

    return {};
}
}  // namespace dcol