cmake_minimum_required(VERSION 3.5)
project(plansys2_ariac)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_executor
    ariac_msgs 
    std_srvs
)

# pickup_part_action_node
add_executable(pickup_part_action_node src/pickup_part_action_node.cpp)
ament_target_dependencies(pickup_part_action_node ${dependencies})

# pickup_tray_action_node
add_executable(pickup_tray_action_node src/pickup_tray_action_node.cpp)
ament_target_dependencies(pickup_tray_action_node ${dependencies})

# change_to_part_gripper_action_node
add_executable(change_to_part_gripper_action_node src/change_to_part_gripper_action_node.cpp)
ament_target_dependencies(change_to_part_gripper_action_node ${dependencies})

# change_to_tray_gripper_action_node
add_executable(change_to_tray_gripper_action_node src/change_to_tray_gripper_action_node.cpp)
ament_target_dependencies(change_to_tray_gripper_action_node ${dependencies})

# putdown_part_action_node
add_executable(putdown_part_action_node src/putdown_part_action_node.cpp)
ament_target_dependencies(putdown_part_action_node ${dependencies})

# putdown_tray_action_node
add_executable(putdown_tray_action_node src/putdown_tray_action_node.cpp)
ament_target_dependencies(putdown_tray_action_node ${dependencies})

# start_competition_action_node
add_executable(start_competition_action_node src/start_competition_action_node.cpp)
ament_target_dependencies(start_competition_action_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  pickup_part_action_node
  pickup_tray_action_node
  change_to_part_gripper_action_node
  change_to_tray_gripper_action_node
  putdown_part_action_node
  putdown_tray_action_node
  start_competition_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()