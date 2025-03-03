cmake_minimum_required(VERSION 3.8)
project(taisei)

set(BOOST_ROOT "/usr")         
set(BOOST_INCLUDEDIR "/usr/include")
set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tachimawari_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)

add_executable(robot_wrapper
  src/wrapper/robot_wrapper.cpp
  src/node.cpp)

  
target_include_directories(robot_wrapper PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robot_wrapper
  pinocchio::pinocchio
  Eigen3::Eigen)

ament_target_dependencies(robot_wrapper
    rclcpp
    tf2_ros
    geometry_msgs
    pinocchio
    Eigen3
    tachimawari_interfaces)

install(TARGETS
    robot_wrapper
    DESTINATION lib/${PROJECT_NAME}/
  )

ament_package()