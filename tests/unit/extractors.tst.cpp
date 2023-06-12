#include <fstream>

#include "dcol/extractors.hpp"
#include "dcol/fs.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
namespace dcol
{
class ExtractTemperatureTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        path_to_file = fs::temp_directory_path()
                       / "somefile.txt";
        ofs.open(path_to_file);
    }

    void TearDown() override
    {
        remove(path_to_file);
    }

    nlohmann::json expected_result;
    std::ofstream ofs;
    fs::path path_to_file;
};

TEST_F(ExtractTemperatureTest, ValidData)
{
    ofs << "45050";
    ofs.close();

    expected_result["temp"] = 45;

    const auto result = dcol::getTemperature(path_to_file);

    EXPECT_EQ(expected_result, result);
}

TEST_F(ExtractTemperatureTest, InvalidData)
{
    ofs << "asd";
    ofs.close();

    const auto result = dcol::getTemperature(path_to_file);

    EXPECT_EQ(expected_result, result);
}

TEST_F(ExtractTemperatureTest, OutOfRangeData)
{
    ofs << "1234567890987654321";
    ofs.close();

    const auto result = dcol::getTemperature(path_to_file);

    EXPECT_EQ(expected_result, result);
}

TEST_F(ExtractTemperatureTest, FileNotExist)
{
    ofs << "asd";
    ofs.close();

    const auto result = dcol::getTemperature(path_to_file / "wrong");

    EXPECT_EQ(expected_result, result);
}

class ExtractStorageSpaceTest : public ::testing::Test
{
protected:
    fs::path path;
    nlohmann::json expected_result;
};

TEST_F(ExtractStorageSpaceTest, ValidData)
{
    path = "/tmp";
    std::error_code ec;
    const fs::space_info si = fs::space(path, ec);

    expected_result["available"] = si.available;
    expected_result["capacity"] = si.capacity;

    const auto result = dcol::getStorageSpace(path);

    EXPECT_EQ(expected_result, result);
}

class ExtractMemoryTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        path_to_file = fs::temp_directory_path()
                       / "somefile.txt";
        ofs.open(path_to_file);
    }

    void TearDown() override
    {
        remove(path_to_file);
    }

    nlohmann::json expected_result;
    std::ofstream ofs;
    fs::path path_to_file;
};

TEST_F(ExtractMemoryTest, ValidData)
{
    ofs << "MemAvailable:        12369528 kB\nMemTotal:       32247680 kB\n";

    ofs.close();

    expected_result["total"] = 32247680;
    expected_result["available"] = 12369528;
    expected_result["used"] = 32247680 - 12369528;

    const auto result = dcol::getMemInfo(path_to_file);

    EXPECT_EQ(expected_result, result);
}

TEST_F(ExtractMemoryTest, InValidData)
{
    ofs << "some text here\n";

    ofs.close();

    expected_result["total"] = -1;
    expected_result["available"] = -1;
    expected_result["used"] = -1;

    const auto result = dcol::getMemInfo(path_to_file);

    EXPECT_EQ(expected_result, result);
}
}  // namespace dcol
