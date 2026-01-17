cmake_minimum_required(VERSION 3.28)
project(fast_planner)

# 关键：设置编译类型为Debug，生成调试符号
set(CMAKE_BUILD_TYPE Debug)  # 优先级最高，强制开启Debug模式
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")  # -g生成调试符号，-O0关闭优化
set(CMAKE_CXX_FLAGS_RELEASE "-g -O0")  # 防止误切换到Release模式，仍保留调试符号

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system chrono)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)


include_directories(${Boost_INCLUDE_DIRS})
include_directories(./planner/localmap_esdf/include)
include_directories(./planner/trajopt_bsplane/include)
add_subdirectory(planner/localmap_esdf)
add_subdirectory(planner/trajopt_bspline)

# 编译节点
add_executable(fast_planner_plan src/trajectory_publisher.cpp)
ament_target_dependencies(
  fast_planner_plan
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
)

target_link_libraries(
  fast_planner_plan
  trajopt
  local_map
)

# 安装节点
install(TARGETS
  fast_planner_plan
  DESTINATION lib/${PROJECT_NAME})

ament_package()