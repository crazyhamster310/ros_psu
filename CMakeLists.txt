cmake_minimum_required(VERSION 3.8)
project(eng_description)

# Find ament_cmake and other required packages
find_package(ament_cmake REQUIRED)

# Install directories to the share folder
install(DIRECTORY launch meshes config urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()