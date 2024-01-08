#include "dcol/extractors.hpp"

#include <gtest/gtest.h>

class ParserTest : public ::testing::Test
{
protected:
    nlohmann::json expected_result;
    std::string input;
};

TEST_F(ParserTest, ValidTempData)
{
    input = "45050";
    expected_result["temp"] = 45;

    const auto result = dcol::detail::parse_temperature(input);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(expected_result, *result);
}

TEST_F(ParserTest, InvalidTempData)
{
    input = "asd";

    const auto result = dcol::detail::parse_temperature(input);
    ASSERT_FALSE(result.has_value());
}

TEST_F(ParserTest, OutOfRangeData)
{
    input = "1234567890987654321";
    const auto result = dcol::detail::parse_temperature(input);

    ASSERT_FALSE(result.has_value());
}

TEST_F(ParserTest, ValidMemoryData)
{
    input = "MemAvailable:        12369528 kB\nMemTotal:       32247680 kB\n";
    expected_result["total"] = 32247680;
    expected_result["available"] = 12369528;
    expected_result["used"] = 32247680 - 12369528;

    const auto result = dcol::detail::parse_mem_info(input);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(expected_result, *result);
}

TEST_F(ParserTest, InValidMemoryData)
{
    input = "some text here\n";
    const auto result = dcol::detail::parse_mem_info(input);

    ASSERT_FALSE(result.has_value());
}
