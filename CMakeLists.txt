cmake_minimum_required(VERSION 3.8)
project(merge_cloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# message_filters removed: using latest-cache model instead of ApproximateTime synchronizer
find_package(livox_ros_driver2 REQUIRED)  # 用于订阅Livox LiDAR数据
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)  # 用于构建变换矩阵

# 添加可执行文件
add_executable(merge_cloud_node src/merge_cloud_code.cpp)

# 链接依赖库
ament_target_dependencies(merge_cloud_node rclcpp pcl_conversions pcl_ros sensor_msgs nav_msgs tf2_ros geometry_msgs Eigen3 livox_ros_driver2)

# 安装可执行文件
install(TARGETS
  merge_cloud_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

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

