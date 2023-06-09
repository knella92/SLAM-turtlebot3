cmake_minimum_required(VERSION 3.8)
project(nuslam)
include(CTest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(nuturtle_control REQUIRED)

add_executable(nuslam src/slam.cpp)
target_link_libraries(nuslam turtlelib::turtlelib)
add_executable(landmarks src/landmarks.cpp)
target_link_libraries(landmarks turtlelib::turtlelib)
ament_target_dependencies(
  nuslam
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nuturtlebot_msgs
  sensor_msgs
  nav_msgs
  nuturtle_control
  turtlelib)
  
ament_target_dependencies(
  landmarks
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nuturtlebot_msgs
  sensor_msgs
  nav_msgs
  nuturtle_control
  turtlelib)

  install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})
  install(
    TARGETS nuslam landmarks
    DESTINATION lib/${PROJECT_NAME})

  find_package(Catch2 3 REQUIRED)
  enable_testing()
  add_executable(circle_tests tests/circle_tests.cpp)
  target_link_libraries(circle_tests Catch2::Catch2WithMain turtlelib::turtlelib)
  # target_link_libraries(circle_tests turtlelib::turtlelib)
  add_test(NAME Test_of_Circle_Detection COMMAND circle_tests)

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
