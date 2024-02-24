cmake_minimum_required(VERSION 3.8)
project(action_forward_turn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_forward_turn_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  action_forward_turn_interfaces
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/action_forward_turn/ActionClientForward.cpp
  src/action_forward_turn/ActionClientTurn.cpp
  src/action_forward_turn/ActionServer.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(action_client_forward_main src/action_client_forward_main.cpp)
ament_target_dependencies(action_client_forward_main ${dependencies})
target_link_libraries(action_client_forward_main ${PROJECT_NAME})

add_executable(action_client_turn_main src/action_client_turn_main.cpp)
ament_target_dependencies(action_client_turn_main ${dependencies})
target_link_libraries(action_client_turn_main ${PROJECT_NAME})

add_executable(action_server_main src/action_server_main.cpp)
ament_target_dependencies(action_server_main ${dependencies})
target_link_libraries(action_server_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  action_client_forward_main
  action_client_turn_main
  action_server_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()