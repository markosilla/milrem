cmake_minimum_required(VERSION 3.8)
project(logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(messages REQUIRED)

include_directories(
        include
        ${rclcpp_INCLUDE_DIRS})

file(GLOB_RECURSE SOURCES "src/*.cpp")
add_library(logger ${SOURCES})

add_executable(logger_node src/loggerNode.cpp)

target_link_libraries(logger_node logger)

ament_target_dependencies(logger
        rclcpp
        messages)

ament_target_dependencies(logger_node
        rclcpp
        messages)

include_directories("include")

install(
        TARGETS logger_node
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY params
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
