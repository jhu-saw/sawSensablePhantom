#
# (C) Copyright 2011-2022 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawSensablePhantom VERSION 1.0.0)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes)

find_package (cisst 1.1.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # make sure CISST_USE_SI_UNITS is defined and set to 1
  if (NOT DEFINED CISST_USE_SI_UNITS)
    message (FATAL_ERROR "You need to upgrade to a version of cisst that defines CISST_USE_SI_UNITS (and set CISST_USE_SI_UNITS to ON)")
  endif ()
  if (NOT CISST_USE_SI_UNITS)
    message (FATAL_ERROR "You need to make sure CISST_USE_SI_UNITS to ON")
  endif ()

  # catkin/ROS paths
  cisst_set_output_path ()

  # recent drivers install headers and libraries in standard path, i.e. /usr/include
  if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_path (SENSABLE_INCLUDE_DIR
               NAMES "HD/hd.h")
    if (EXISTS "${SENSABLE_INCLUDE_DIR}/HD/hd.h")
      set (SENSABLE_FOUND 1)
      set (SENSABLE_LIBRARIES HD)
    endif ()
  endif ()

  # if not found this way, try using FindSensable.cmake
  if (NOT SENSABLE_FOUND)
    set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
    find_package (Sensable)
  endif ()

  # for the cartesian impedence controller
  find_package (sawControllers 2.0.0 REQUIRED)

  if (SENSABLE_FOUND AND sawControllers_FOUND)

    # create/configure file for find_package (sawSensablePhantom)
    set (sawSensablePhantom_INCLUDE_DIR
      "${sawSensablePhantom_SOURCE_DIR}/include"
      "${CMAKE_CURRENT_BINARY_DIR}/include")
    set (sawSensablePhantom_HEADER_DIR "${sawSensablePhantom_SOURCE_DIR}/include/sawSensablePhantom")
    set (sawSensablePhantom_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set (sawSensablePhantom_LIBRARIES sawSensablePhantom ${SENSABLE_LIBRARIES})

    include_directories (
      ${SENSABLE_INCLUDE_DIR}
      ${sawSensablePhantom_INCLUDE_DIR}
      ${sawControllers_INCLUDE_DIR})

    link_directories (
      ${sawControllers_LIBRARY_DIR})

    # add Qt code
    if (CISST_HAS_QT)
      add_subdirectory (code/Qt)
      set (sawSensablePhantom_LIBRARIES ${sawSensablePhantom_LIBRARIES} sawSensablePhantomQt)
    endif (CISST_HAS_QT)

    # add all config files for this component
    cisst_add_config_files (sawSensablePhantom)

    add_library (sawSensablePhantom ${IS_SHARED}
                 ${sawSensablePhantom_HEADER_DIR}/sawSensablePhantomExport.h
                 ${sawSensablePhantom_HEADER_DIR}/mtsSensableHD.h
                 code/mtsSensableHD.cpp
                 )
    set_target_properties (sawSensablePhantom PROPERTIES
                           VERSION ${sawSensablePhantom_VERSION}
                           FOLDER "sawSensablePhantom")
    target_link_libraries (sawSensablePhantom ${SENSABLE_LIBRARIES} ${sawControllers_LIBRARIES})
    cisst_target_link_libraries (sawSensablePhantom ${REQUIRED_CISST_LIBRARIES})

    # Install target for headers and library
    install (DIRECTORY
             "${sawSensablePhantom_SOURCE_DIR}/include/sawSensablePhantom"
             DESTINATION include
             COMPONENT sawSensablePhantom-dev
             PATTERN .svn EXCLUDE)

    install (TARGETS sawSensablePhantom COMPONENT sawSensablePhantom
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

  else ()
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} requires Sensable.")
  endif ()

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
