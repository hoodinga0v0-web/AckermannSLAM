cmake_minimum_required(VERSION 3.8)
project(ackermann_car_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/normalize_meshes.py
    scripts/generate_geometry.py
    scripts/validate_yaml.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_simulation_launch.py TIMEOUT 120)
endif()

ament_package()
