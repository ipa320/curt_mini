cmake_minimum_required(VERSION 3.5)
project(curt_mini_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models config DESTINATION share/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
