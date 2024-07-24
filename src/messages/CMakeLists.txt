cmake_minimum_required(VERSION 3.5)
project(messages)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB_RECURSE MESSAGES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
set(msg_files ${MESSAGES})

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES std_msgs
        ADD_LINTER_TESTS
        )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
