cmake_minimum_required(VERSION 3.8)
project(ur5e_hand_integration)
find_package(ament_cmake REQUIRED)
install(PROGRAMS
  scripts/moveit_center_ik.py
  scripts/moveit_fake_arm_controller.py
  scripts/hand_joint_state_publisher.py
  scripts/joint_state_combiner.py
  scripts/start_state_latcher.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})
ament_package()
