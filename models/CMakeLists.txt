cmake_minimum_required(VERSION 3.10)
project(sensable_phantom_models VERSION 1.1.0)

# search for ROS1 first
find_package(catkin QUIET COMPONENTS urdf)
if (catkin_FOUND)
  catkin_package()
else (catkin_FOUND)

  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)

    set (_to_be_installed meshes urdf ros2/rviz ros2/launch)

    foreach (_dir ${_to_be_installed})
      install (DIRECTORY
        ${_dir}
        DESTINATION share/${PROJECT_NAME})
    endforeach ()

    ament_package ()
  endif (ament_cmake_FOUND)

endif (catkin_FOUND)
