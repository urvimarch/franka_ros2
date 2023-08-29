cmake_minimum_required(VERSION 3.5)
project(franka_description)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

#add_executable(talker src/1_controller_test.py)
#ament_target_dependencies(talker rclpy std_msgs)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_urdf_tests test/urdf_tests.py)

endif()

# Install launch files.
install(DIRECTORY
  launch robots meshes rviz worlds config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()