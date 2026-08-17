cmake_minimum_required(VERSION 3.8)
project(robotaksi_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 gerekli (std::filesystem)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(nlohmann_json REQUIRED)

include_directories(include)



# ── Yeni Modüler BT node'ları kütüphanesi ──
add_library(segment_bt_nodes SHARED
  src/segment_loop_nodes.cpp
  src/vehicle_control_nodes.cpp
  src/traffic_logic_nodes.cpp
  src/mission_utility_nodes.cpp
  src/stub_nodes.cpp
)

target_include_directories(segment_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(segment_bt_nodes
  behaviortree_cpp_v3
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(segment_bt_nodes nlohmann_json::nlohmann_json)

install(TARGETS segment_bt_nodes
  EXPORT export_robotaksi_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ── Test executable: ros2 run robotaksi_bt bt_test ──
add_executable(bt_test src/main.cpp)
target_link_libraries(bt_test
  segment_bt_nodes
)
ament_target_dependencies(bt_test
  behaviortree_cpp_v3
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS bt_test
  DESTINATION lib/${PROJECT_NAME}
)

# ── Behaviour tree XML'leri kur ──
install(DIRECTORY behaviour_trees/
  DESTINATION share/${PROJECT_NAME}/behaviour_trees
  FILES_MATCHING PATTERN "*.xml"
)

# ── Scripts kur ──
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
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

ament_export_targets(export_robotaksi_bt_nodes HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp_v3 rclcpp rclcpp_action geometry_msgs std_msgs nav_msgs nav2_msgs tf2 tf2_geometry_msgs nlohmann_json)
ament_package()