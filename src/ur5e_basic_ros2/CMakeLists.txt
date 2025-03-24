cmake_minimum_required(VERSION 3.8)
project(ur5e_basic_ros2)

IF (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
ENDIF ()

# 设置变量
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

add_compile_options(-Wno-maybe-uninitialized)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# 寻找ros2库/系统库/第三方库/功能包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(backward_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ur_msgs REQUIRED)

find_package(orocos_kdl REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(NLopt REQUIRED)

# 头文件路径 全局
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${allDir}
  ${orocos_kdl_INCLUDE_DIRS}
  ${trac_ik_lib_INCLUDE_DIRS}
)

# 设置依赖
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  controller_manager_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  std_srvs
  tf2_geometry_msgs
  ur_client_library
  ur_dashboard_msgs
  ur_msgs
  trac_ik_lib
  NLopt
)

# 生成库 自建库
add_library(ur5e_basic
  SHARED
  src/UR5e.cpp
  src/kin_dyn.cpp
)

target_include_directories(
  ur5e_basic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${NLOPT_INCLUDE_DIRS}
)

# target 使下游文件可以访问
ament_export_targets(ur5e_basic HAS_LIBRARY_TARGET)

# target 添加 系统库/自定义路径库 依赖
target_link_libraries(
  ur5e_basic
  # ${sysCoreLibs}
  ${NLOPT_LIBRARIES}
)

# target ROS2库
ament_target_dependencies(
  ur5e_basic
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# 安装 自己编写的库 库文件
install (
  TARGETS ur5e_basic
  EXPORT ur5e_basic
  DESTINATION lib
)

# 安装 自己编写的库 头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 生成可执行文件
add_executable(ur5e_demo ${CMAKE_CURRENT_SOURCE_DIR}/src/UR5e.cpp)
target_link_libraries(ur5e_demo ur5e_basic ${orocos_kdl_LIBRARIES} ${NLOPT_INCLUDE_DIRS})
ament_target_dependencies(ur5e_demo ${THIS_PACKAGE_INCLUDE_DEPENDS} trac_ik_lib)


# include_directories(${orocos_kdl_INCLUDE_DIRS})
add_executable(kdl_test ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(kdl_test ${orocos_kdl_LIBRARIES})

install(TARGETS ur5e_demo kdl_test
  DESTINATION lib/${PROJECT_NAME}
)

# 设置和启用自动化代码检查和测试的配置
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