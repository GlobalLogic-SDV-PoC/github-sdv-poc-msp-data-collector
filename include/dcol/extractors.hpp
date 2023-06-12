#pragma once

#include <string>
#include <vector>

#include "dcol/fs.hpp"
#include "nlohmann/json.hpp"

namespace dcol
{
nlohmann::json getTemperature(const fs::path& path);
nlohmann::json getStorageSpace(const fs::path& path);
nlohmann::json getMemInfo(const fs::path& path);
}  // namespace dcol