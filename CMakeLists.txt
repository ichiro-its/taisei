cmake_minimum_required(VERSION 3.8)
project(taisei)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(tachimawari_interfaces REQUIRED)
find_package(tachimawari  REQUIRED)
find_package(keisan REQUIRED)
find_package(jitsuyo REQUIRED)
find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
   "src/${PROJECT_NAME}/node/taisei_node.cpp"
   "src/${PROJECT_NAME}/robot_wrapper/robot_wrapper.cpp"
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  pinocchio::pinocchio
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    tf2_ros
    geometry_msgs
    pinocchio
    tachimawari_interfaces
    tachimawari
    keisan
    jitsuyo)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin"
)

add_executable(robot_wrapper
  src/main.cpp
)

target_include_directories(robot_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_wrapper PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  ${PROJECT_NAME}
)

install(TARGETS
    robot_wrapper
    DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  tf2_ros
  geometry_msgs
  pinocchio
  tachimawari_interfaces
  tachimawari
  keisan
  jitsuyo
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
