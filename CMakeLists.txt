cmake_minimum_required(VERSION 3.5)
project(ipa_ros2_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(soem_ros2 REQUIRED)
find_package(tl_expected REQUIRED)

unset( CATKIN_INSTALL_INTO_PREFIX_ROOT )

include_directories(include)

link_directories(/usr/local/lib)
find_library(mini_cheetah_motor_driver REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/mcDSAE25.cpp
  src/ethercat_wrapper.cpp
  src/mvp_hardware_interface.cpp
  src/curt_mini_hardware_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  soem_ros2
  tl_expected
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
#target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")

# prevent pluginlib from using boost
#target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface mvp_driver.xml)

pluginlib_export_plugin_description_file(hardware_interface curt_mini_driver.xml)

target_link_libraries(
  ${PROJECT_NAME} mini_cheetah_motor_driver
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(
#   TARGETS ${PROJECT_NAME}
#   DESTINATION lib
# )

install(
  DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
#   PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}/
# )

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
# endif()

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".git" EXCLUDE)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME}
)

# ament_export_dependencies(
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
#   soem_ros2
#   tl_expected
# )
ament_package()