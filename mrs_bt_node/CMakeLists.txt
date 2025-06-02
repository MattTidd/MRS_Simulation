cmake_minimum_required(VERSION 3.8)
project(mrs_bt_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies:
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# include the headers:
include_directories(
  include
)

# make the BT node executable:
add_executable(bt_node
  src/main.cpp 
)

target_include_directories(bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# dependencies:
ament_target_dependencies(bt_node
)

# install everything:
install(TARGETS bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include trees launch params maps src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


