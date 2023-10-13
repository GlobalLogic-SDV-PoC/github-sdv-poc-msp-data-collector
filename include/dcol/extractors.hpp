#pragma once

#include <optional>
#include <string>
#include <vector>

#include "context.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"

namespace dcol
{
auto get_temperature(const std::string& path, const std::shared_ptr<Context>& ctx) -> std::optional<nlohmann::json>;
auto get_storage_space(const std::string& path, const std::shared_ptr<Context>& ctx) -> std::optional<nlohmann::json>;
auto get_mem_info(const std::string& path, const std::shared_ptr<Context>& ctx) -> std::optional<nlohmann::json>;

namespace detail
{
auto get_data_from_file(const std::string& path) -> std::optional<std::string>;
auto split_string(const std::string& str) -> std::vector<std::string>;
auto parse_temperature(const std::string& data) -> std::optional<nlohmann::json>;
auto parse_mem_info(const std::string& data) -> std::optional<nlohmann::json>;
}  // namespace detail
}  // namespace dcol