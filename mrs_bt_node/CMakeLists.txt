cmake_minimum_required(VERSION 3.8)
project(mrs_bt_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies:
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)

# include the headers:
include_directories(
  include
)

# make the BT node executable:
add_executable(bt_node
  src/main.cpp
  src/auctioneer/send_bid_request_action.cpp
  src/auctioneer/bids_received_condition.cpp
  src/auctioneer/wait_for_bids_action.cpp
  src/auctioneer/allocation_action.cpp
)

target_include_directories(bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# dependencies:
ament_target_dependencies(bt_node
  rclcpp
  behaviortree_cpp
)

# install everything:
install(TARGETS bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include trees launch src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


