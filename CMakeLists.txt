cmake_minimum_required(VERSION 3.10)
project(dcol)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(dep/external/json)
add_subdirectory(dep/external/fmt)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib src/scheduler.cpp src/extractors.cpp src/app.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp std_msgs)
target_link_libraries(${PROJECT_NAME}_lib nlohmann_json fmt)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_lib)

add_subdirectory(test/unit)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
