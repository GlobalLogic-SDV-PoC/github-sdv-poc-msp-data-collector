cmake_minimum_required(VERSION 3.5.2)
project(
    data_collection
    LANGUAGES CXX
    VERSION 1.0.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

option(DCOL_ENABLE_TESTING "Specify if testing should be enabled" OFF)
option(DCOL_ENABLE_CODE_COVERAGE "Specify if code coverage should be enabled"
       OFF)
option(DCOL_EXTERNAL_SPDLOG "Specify whether to use an external spdlog library"
       OFF)

if(DCOL_ENABLE_TESTING)
    if(DCOL_ENABLE_CODE_COVERAGE)
        include(CodeCoverage)
        append_coverage_compiler_flags()
    endif()
    add_subdirectory(external/gtest)
endif()
if(NOT DCOL_EXTERNAL_SPDLOG)
    add_subdirectory(external/spdlog)
endif()
add_subdirectory(ipc)

find_package(Filesystem REQUIRED COMPONENTS Final Experimental)

add_library(data_collection STATIC src/extractors.cpp src/event_scheduler.cpp
                                   src/app.cpp)
target_include_directories(data_collection PUBLIC include)
target_link_libraries(data_collection PUBLIC ipc spdlog::spdlog std::filesystem)
target_compile_features(data_collection PUBLIC cxx_std_17)

if(DCOL_ENABLE_TESTING)
    add_subdirectory(tests/unit)
endif()
