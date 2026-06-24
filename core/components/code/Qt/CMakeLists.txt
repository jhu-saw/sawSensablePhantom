#
# (C) Copyright 2014-2026 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstVector
  cisstVectorQt
  cisstOSAbstraction
  cisstMultiTask
  cisstMultiTaskQt
  cisstParameterTypes
  cisstParameterTypesQt)

# cisst
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # for Qt generated header files
  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  set (CMAKE_AUTOMOC ON)

  add_library (
    sawSensablePhantomQt
    ${sawSensablePhantom_HEADER_DIR}/mtsSensableHDQtWidget.h
    mtsSensableHDQtWidget.cpp)

  set_target_properties (
    sawSensablePhantomQt PROPERTIES
    VERSION ${sawSensablePhantom_VERSION}
    FOLDER "sawSensablePhantom")

  target_link_libraries (
    sawSensablePhantomQt
    ${REQUIRED_CISST_LIBRARIES})

  install (TARGETS sawSensablePhantomQt COMPONENT sawSensablePhantom-Qt
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND)
  message ("Information: code will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND)
