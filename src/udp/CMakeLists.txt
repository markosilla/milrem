cmake_minimum_required(VERSION 3.8)
project(udp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(messages REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
        include
        ${std_msgs_INCLUDE_DIRS}
        ${std_srvs_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS})

add_library(udp src/udp.cpp)

add_executable(udp_node src/udp_node.cpp)

target_link_libraries(udp_node udp)

ament_target_dependencies(udp_node
        rclcpp
        messages)


ament_target_dependencies(udp
        rclcpp
        messages)

target_include_directories(udp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(udp_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(
        TARGETS udp_node
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY params
        DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
