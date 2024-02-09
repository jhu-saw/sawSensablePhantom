cmake_minimum_required(VERSION 3.10)
project(saw_sensable_phantom_models VERSION 1.1.0)

find_package(catkin QUIET COMPONENTS urdf)

if (catkin_FOUND)
  catkin_package()
endif (catkin_FOUND)
