cmake_minimum_required(VERSION 3.8)
project(yolov7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(DEPENDENCIES
  rclcpp
  OpenCV
  cv_bridge
  sensor_msgs
  rclcpp_components
  message_filters
  geometry_msgs
  std_msgs
  nav_msgs
  art_perception_msgs
  art_msgs
)

foreach (dependency IN LISTS DEPENDENCIES)
  find_package(${dependency} REQUIRED)
endforeach()

ament_auto_find_build_dependencies(REQUIRED ${DEPENDENCIES})

set(LIBRARIES
  ${OpenCV_LIBRARIES}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE 
  include
)

target_link_libraries(${PROJECT_NAME} ${LIBRARIES})

# rclcpp_components_register_nodes(${PROJECT_NAME} "image_pipeline::YOLOv7")

ament_auto_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# the node must go into the project specific lib directory or else
# the launch file will not find it

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}/
)

# the shared library goes into the global lib dir so it can
# be used as a composable node by other projects

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
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

ament_package()
