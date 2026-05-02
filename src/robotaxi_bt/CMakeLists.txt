cmake_minimum_required(VERSION 3.8)
project(robotaxi_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_library(init_mission_action_node SHARED
  src/init_mission_action.cpp
)

target_include_directories(init_mission_action_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(init_mission_action_node
  behaviortree_cpp_v3
  rclcpp
  geometry_msgs
)

install(TARGETS init_mission_action_node
  EXPORT export_init_mission_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ── Test executable: ros2 run robotaxi_bt bt_test ──
add_executable(bt_test src/main.cpp)
target_link_libraries(bt_test init_mission_action_node)
ament_target_dependencies(bt_test
  behaviortree_cpp_v3
  rclcpp
  geometry_msgs
)

install(TARGETS bt_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_init_mission_action_node HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp_v3 rclcpp geometry_msgs)
ament_package()