cmake_minimum_required(VERSION 3.16)
project(homework)

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(Boost REQUIRED COMPONENTS system thread program_options)

include_directories("src/logger/include")
include_directories("src/udp/include")
include_directories("src/common/include")
include_directories("install/include/")

include_directories(/opt/ros/foxy/include)

file(GLOB_RECURSE SOURCES "*.cpp" "*.hpp")

add_library(dummy STATIC ${SOURCES})
target_link_libraries(dummy ${Boost_LIBRARIES})

add_executable(${PROJECT_NAME} ${SOURCES})
