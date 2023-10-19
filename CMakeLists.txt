cmake_minimum_required(VERSION 3.10)
project(dcol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ipc REQUIRED)

add_library(${PROJECT_NAME}_lib STATIC)
target_sources(${PROJECT_NAME}_lib PRIVATE src/scheduler.cpp src/extractors.cpp
                                           src/app.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC include)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_14)
ament_target_dependencies(${PROJECT_NAME}_lib ipc rclcpp std_msgs)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_lib)

add_subdirectory(test/unit)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
