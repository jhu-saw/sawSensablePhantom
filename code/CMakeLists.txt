#
#
# (C) Copyright 2011-2012 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
  find_package (Sensable REQUIRED)

  if (SENSABLE_FOUND)

    set (sawSensablePhantom_INCLUDE_DIR "${sawSensablePhantom_SOURCE_DIR}/include")
    set (sawSensablePhantom_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
    set (sawSensablePhantom_LIBRARIES sawSensablePhantom ${SENSABLE_LIBRARIES})

    include_directories (${SENSABLE_INCLUDE_DIR} ${sawSensablePhantom_INCLUDE_DIR})
    set (sawSensablePhantom_HEADER_DIR "${sawSensablePhantom_SOURCE_DIR}/include/sawSensablePhantom")
    add_library (sawSensablePhantom ${IS_SHARED}
                 ${sawSensablePhantom_HEADER_DIR}/sawSensablePhantomExport.h
                 mtsSensableHD.cpp
                 ${sawSensablePhantom_HEADER_DIR}/mtsSensableHD.h
                 )
    set_property (TARGET sawSensablePhantom PROPERTY FOLDER "sawSensablePhantom")
    target_link_libraries (sawSensablePhantom ${SENSABLE_LIBRARIES})

    configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/sawSensablePhantomConfig.cmake.in"
                    "${sawSensablePhantom_BINARY_DIR}/sawSensablePhantomConfig.cmake"
                     @ONLY@)

    cisst_target_link_libraries (sawSensablePhantom ${REQUIRED_CISST_LIBRARIES})

    # Install target for headers and library
    install (DIRECTORY
             ${sawSensablePhantom_INCLUDE_DIR}/sawSensablePhantom
             DESTINATION include
             PATTERN .svn EXCLUDE)

    install (TARGETS sawSensablePhantom
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

    install (FILES "${sawSensablePhantom_BINARY_DIR}/sawSensablePhantomConfig.cmake"
             DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

  else (SENSABLE_FOUND)
    message (SEND_ERROR "code in ${CMAKE_CURRENT_SOURCE_DIR} requires Sensable.")
  endif (SENSABLE_FOUND)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
