cmake_minimum_required(VERSION 3.8)
project(dofbot)

include(GNUInstallDirs)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
ament_cmake
rclcpp
hardware_interface
pluginlib
rclcpp_lifecycle
)

add_executable(dofbot src/dofbot.cpp)
target_include_directories(dofbot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(dofbot PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
ament_target_dependencies(
  dofbot
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS dofbot
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# dofbot_arm is responsible for reading joint angles and sending new joint angles to the servos
add_library(dofbot_arm SHARED 
include/arm.hpp
src/arm.cpp
)
target_link_libraries(dofbot atomic i2c)
target_link_libraries(dofbot_arm i2c)
target_include_directories(
  dofbot_arm 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  dofbot_arm
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface dofbot.xml)

ament_export_targets(
  dofbot_arm_targets
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  hardware_interface
)
ament_export_libraries(
  dofbot_arm
)
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(TARGETS dofbot_arm
  EXPORT dofbot_arm_targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED) 
  find_package(ros2_control_test_assets REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_inst_hardwares test/test_load_dofbot_arm.cpp)
  target_include_directories(test_inst_hardwares 
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(
    test_inst_hardwares
    hardware_interface
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
    ros2_control_test_assets
  )
endif()

add_custom_target(format
  COMMAND ament_clang_format
  --filter ${CMAKE_CURRENT_SOURCE_DIR}/include
  --filter ${CMAKE_CURRENT_SOURCE_DIR}/src
  --filter ${CMAKE_CURRENT_SOURCE_DIR}/test
  --style=file
  -i `find include src test -name *.hpp -o -name *.cpp`
)
add_custom_target(tidy
  COMMAND ament_clang_tidy
  --filter ${CMAKE_CURRENT_SOURCE_DIR}/include
  --filter ${CMAKE_CURRENT_SOURCE_DIR}/src
  --filter ${CMAKE_CURRENT_SOURCE_DIR}/test
  --clang-tidy-binary clang-tidy
  --header-filter=.*
  -p `pwd`/compile_commands.json
)

ament_package()
