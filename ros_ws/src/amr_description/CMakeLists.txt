cmake_minimum_required(VERSION 3.5)
project(amr_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  urdf
  worlds
  meshes
  resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
