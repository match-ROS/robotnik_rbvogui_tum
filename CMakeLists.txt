cmake_minimum_required(VERSION 3.8)
project(robotnik_rbvogui_tum)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  dependencies
  docs
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/current_pose_from_tf.py
  scripts/rbvogui_swerve_controller.py
  scripts/tf_model_pose_to_pose_stamped.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
