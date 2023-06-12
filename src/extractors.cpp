#include "dcol/extractors.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "spdlog/spdlog.h"

namespace dcol
{
namespace
{
std::string getDataFromFile(const fs::path& path)
{
    std::ifstream file(path);

    if (!file)
    {
        throw std::runtime_error("Failed to open the file: " + path.string());
    }

    std::stringstream buffer;
    buffer << file.rdbuf();

    if (file.fail())
    {
        throw std::runtime_error("Failed to read the file: " + path.string());
    }

    file.close();
    return buffer.str();
}

std::vector<std::string> splitString(const std::string& str)
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
}  // namespace
nlohmann::json getMemInfo(const fs::path& path)
{
    auto data = getDataFromFile(path);
    auto split_data = splitString(data);
    std::string label;
    std::int64_t memTotal = -1, memAvail = -1, memUsed = -1, value;
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
    if (memTotal != -1 || memAvail != -1)
    {
        memUsed = memTotal - memAvail;
    }
    SPDLOG_INFO("[dcol] Total Memory: {} Available Memory: {} Used Memory: {}", memTotal, memAvail, memUsed);

    nlohmann::json result;
    result["total"] = memTotal;
    result["available"] = memAvail;
    result["used"] = memUsed;
    return result;
}

nlohmann::json getTemperature(const fs::path& path)
{
    try
    {
        std::string tempData = getDataFromFile(path);
        int cpuTemp = std::stoi(tempData);
        cpuTemp /= 1000;
        SPDLOG_INFO("[dcol] CPU Temperature: {}", cpuTemp);

        nlohmann::json result;
        result["temp"] = cpuTemp;
        return result;
    }
    catch (const std::runtime_error& e)
    {
        SPDLOG_ERROR("[dcol] runtime_error: {}", e.what());
    }
    catch (const std::invalid_argument& e)
    {
        SPDLOG_ERROR("[dcol] invalid_argument: {}", e.what());
    }
    catch (const std::out_of_range& e)
    {
        SPDLOG_ERROR("[dcol] out_of_range: {}", e.what());
    }
    return {};
}

nlohmann::json getStorageSpace(const fs::path& path)
{
    std::error_code ec;
    const fs::space_info si = fs::space(path, ec);
    SPDLOG_INFO("[dcol] {} Available Storage Space: {} Total Storage Space: {}", path.c_str(), static_cast<std::intmax_t>(si.available), static_cast<std::intmax_t>(si.capacity));

    nlohmann::json result;
    result["available"] = si.available;
    result["capacity"] = si.capacity;
    return result;
}
}  // namespace dcol