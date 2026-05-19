cmake_minimum_required(VERSION 3.20)
project(finger_vision2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Armadillo REQUIRED)
find_package(fingerlib REQUIRED)
find_package(modern_robotics REQUIRED)

add_executable(vision src/vision.cpp)
add_executable(fake_vision src/fake_vision.cpp)

target_link_libraries(vision
  rclcpp::rclcpp
  
  ${cv_bridge_TARGETS}
  ${OpenCV_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_ros_TARGETS}
  ${rcl_interfaces_TARGETS}
)

target_link_libraries(fake_vision
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${tf2_ros_TARGETS}
  ${ARMADILLO_LIBRARIES}
  fingerlib::fingerlib
)

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

install(
  TARGETS vision fake_vision
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
