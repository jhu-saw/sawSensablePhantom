#
# (C) Copyright 2011-2017 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (sawSensablePhantom)

cmake_minimum_required (VERSION 2.8)

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstOSAbstraction
     cisstMultiTask
     cisstParameterTypes)

find_package (cisst 1.0.6 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawSensablePhantom_IS_CATKIN_BUILT)
  if (sawSensablePhantom_IS_CATKIN_BUILT)
    set (sawSensablePhantom_CONFIG_FILE_DIR
         "${CATKIN_DEVEL_PREFIX}/share/sawSensablePhantom/cmake")
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
    set (LIBRARY_OUTPUT_PATH    "${CATKIN_DEVEL_PREFIX}/lib")
  else ()
    set (sawSensablePhantom_CONFIG_FILE_DIR "${sawSensablePhantom_BINARY_DIR}")
  endif ()

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

  if (SENSABLE_FOUND)

    # create/configure file for find_package (sawSensablePhantom)
    set (sawSensablePhantom_INCLUDE_DIR "${sawSensablePhantom_SOURCE_DIR}/include"
                                         "${CMAKE_CURRENT_BINARY_DIR}/include")
    set (sawSensablePhantom_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set (sawSensablePhantom_LIBRARIES sawSensablePhantom ${SENSABLE_LIBRARIES})

    # Set the version number
    set (sawSensablePhantom_VERSION_MAJOR "1")
    set (sawSensablePhantom_VERSION_MINOR "0")
    set (sawSensablePhantom_VERSION_PATCH "0")
    set (sawSensablePhantom_VERSION "${sawSensablePhantom_VERSION_MAJOR}.${sawSensablePhantom_VERSION_MINOR}.${sawSensablePhantom_VERSION_PATCH}")

    # Generate sawSensablePhantomRevision.h
    configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/code/sawSensablePhantomRevision.h.in"
                    "${CMAKE_CURRENT_BINARY_DIR}/include/sawSensablePhantom/sawSensablePhantomRevision.h")

    include_directories (${SENSABLE_INCLUDE_DIR} ${sawSensablePhantom_INCLUDE_DIR})

    add_library (sawSensablePhantom ${IS_SHARED}
                 include/sawSensablePhantom/sawSensablePhantomExport.h
                 include/sawSensablePhantom/mtsSensableHD.h
                 code/mtsSensableHD.cpp
                 )
    set_property (TARGET sawSensablePhantom PROPERTY FOLDER "sawSensablePhantom")
    target_link_libraries (sawSensablePhantom ${SENSABLE_LIBRARIES})
    cisst_target_link_libraries (sawSensablePhantom ${REQUIRED_CISST_LIBRARIES})

    set (sawSensablePhantom_CMAKE_CONFIG_FILE
         "${sawSensablePhantom_CONFIG_FILE_DIR}/sawSensablePhantomConfig.cmake")

    configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawSensablePhantomConfig.cmake.in"
                    "${sawSensablePhantom_CMAKE_CONFIG_FILE}"
                    @ONLY)

    cisst_add_config_version ("${sawSensablePhantom_CONFIG_FILE_DIR}/sawSensablePhantomConfigVersion.cmake"
                              VERSION ${sawSensablePhantom_VERSION}
                              DESTINATION ${CISST_CMAKE_INSTALL_SUFFIX}/saw
                              COMPONENT sawSensablePhantom)

    # Install target for headers and library
    install (DIRECTORY
             "${sawSensablePhantom_SOURCE_DIR}/include/sawSensablePhantom"
             DESTINATION include
             PATTERN .svn EXCLUDE)

    install (TARGETS sawSensablePhantom
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

    install (FILES "${sawSensablePhantom_BINARY_DIR}/sawSensablePhantomConfig.cmake"
             DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

  else (SENSABLE_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} requires Sensable.")
  endif (SENSABLE_FOUND)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
