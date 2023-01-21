cmake_minimum_required(VERSION 3.8)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


set(srv_files
  "srv/Reset.srv"
  "srv/Teleport.srv")

rosidl_generate_interfaces(${PROJECT_NAME}_srv
  ${srv_files} LIBRARY_NAME ${PROJECT_NAME})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

add_executable(nusim src/nusim.cpp)
ament_target_dependencies(
  nusim
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
install(
  TARGETS nusim
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(nusim "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
