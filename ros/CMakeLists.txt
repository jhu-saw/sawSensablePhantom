#
# (C) Copyright 2017-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sensable_phantom VERSION 1.1.0)

# first test for ROS1
find_package (catkin QUIET
  COMPONENTS
  cisst_ros_crtk
  roscpp
  roslib)

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    find_package (ament_cmake_python REQUIRED)
    set (ROS_VERSION 2)
    set (REQUIRED_AMENT_PACKAGES
      rclcpp std_msgs std_srvs geometry_msgs sensor_msgs diagnostic_msgs
      tf2_ros tf2_msgs cisst_msgs crtk_msgs cisst_ros_bridge cisst_ros_crtk)
    foreach (_pkg ${REQUIRED_AMENT_PACKAGES})
      find_package (${_pkg} REQUIRED)
    endforeach ()

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

# find cisst and make sure the required libraries have been compiled
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonQt
  cisstVector
  cisstVectorQt
  cisstOSAbstraction
  cisstMultiTask
  cisstMultiTaskQt
  cisstParameterTypes
  cisstParameterTypesQt
  cisstQt)

find_package (cisst 1.2.1 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # set paths
  cisst_set_output_path ()

  # sawSensablePhantom has been compiled within cisst, we should find it automatically
  find_package (sawSensablePhantom 1.1.0)

  if (sawSensablePhantom_FOUND)

    if (ROS_VERSION EQUAL 1)
      file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")
      catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
        CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)
      include_directories (${catkin_INCLUDE_DIRS})
      # for python CRTK client
      catkin_python_setup ()
    endif ()

    # sawSensablePhantom configuration
    include_directories (${sawSensablePhantom_INCLUDE_DIR} ${sensable_phantom_SOURCE_DIR})
    link_directories (${sawSensablePhantom_LIBRARY_DIR})

    add_executable (sensable_phantom src/sensable_phantom.cpp)
    target_link_libraries (sensable_phantom
      ${sawSensablePhantom_LIBRARIES}
      ${catkin_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sensable_phantom ${REQUIRED_CISST_LIBRARIES})

    install (TARGETS sensable_phantom
      LIBRARY DESTINATION lib/${PROJECT_NAME}
      RUNTIME DESTINATION lib/${PROJECT_NAME}
      LIBRARY DESTINATION lib
      INCLUDES DESTINATION include)

    if (ROS_VERSION EQUAL 2)
      # Python
      add_subdirectory (python)
      install (PROGRAMS
        scripts/xyz-motions.py
        scripts/cartesian_impedance.py
        DESTINATION lib/${PROJECT_NAME})
      # C++
      ament_target_dependencies (
        sensable_phantom
        ${REQUIRED_AMENT_PACKAGES}
        )
      ament_package ()
    endif ()

  else (sawSensablePhantom_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawSensablePhantom")
  endif (sawSensablePhantom_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
