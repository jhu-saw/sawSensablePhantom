#
# (C) Copyright 2017-2021 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonQt
                              cisstVector
                              cisstVectorQt
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstMultiTaskQt
                              cisstParameterTypes
                              cisstParameterTypesQt)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.1.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawSensablePhantomIGTL_IS_CATKIN_BUILT)
  if (sawSensablePhantomIGTL_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # sawSensablePhantom has been compiled within cisst, we should find it automatically
  find_package (sawSensablePhantom 1.0.0)
  find_package (sawOpenIGTLink 1.2.0)

  if (sawSensablePhantom_FOUND AND sawOpenIGTLink_FOUND AND CISST_HAS_QT)

    # sawSensablePhantom configuration
    include_directories (${sawSensablePhantom_INCLUDE_DIR} ${sawOpenIGTLink_INCLUDE_DIR})
    link_directories (${sawSensablePhantom_LIBRARY_DIR} ${sawOpenIGTLink_LIBRARY_DIR})

    add_executable (sawSensablePhantomQtIGTL main.cpp)
    set_property (TARGET sawSensablePhantomQtIGTL PROPERTY FOLDER "sawSensablePhantom")

    # link against non cisst libraries and saw components
    target_link_libraries (sawSensablePhantomQtIGTL
                           ${sawSensablePhantom_LIBRARIES}
                           ${sawSensablePhantomQt_LIBRARIES}
                           ${sawOpenIGTLink_LIBRARIES}
                           ${QT_LIBRARIES}
                           )

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawSensablePhantomQtIGTL ${REQUIRED_CISST_LIBRARIES})

  else ()
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawSensablePhantom and cisstQt")
  endif ()

endif (cisst_FOUND_AS_REQUIRED)
