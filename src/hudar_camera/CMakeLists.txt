cmake_minimum_required(VERSION 3.8)
project(hudar_camera)

# 海康sdk编译的警告
add_compile_options(-Wno-missing-field-initializers)
add_compile_options(-Wno-sign-compare)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
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

include_directories(
  /opt/MVS/include
  ${cv_bridge_INCLUDE_DIRS}

)
link_directories(
  /opt/MVS/lib/64/
)

add_executable(radar_camera src/main.cpp src/Camera.cpp)
ament_target_dependencies(radar_camera rclcpp OpenCV std_msgs sensor_msgs cv_bridge)

add_executable(publish_video src/publish_video.cpp)
ament_target_dependencies(publish_video rclcpp OpenCV std_msgs sensor_msgs cv_bridge)
target_link_libraries(radar_camera
  -lMvCameraControl
)

install(TARGETS
  radar_camera
  publish_video
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

