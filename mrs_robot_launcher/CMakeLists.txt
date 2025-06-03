cmake_minimum_required(VERSION 3.8)
project(mrs_robot_launcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# installation stuff: 
install(
  DIRECTORY launch config urdf meshes worlds  # these are the directories I need
  DESTINATION share/${PROJECT_NAME}           # this is the destination
)

ament_package()
