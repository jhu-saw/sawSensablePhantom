#
#
# (C) Copyright 2005-2012 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# create a list of libraries needed for this project
set (REQUIRED_CISST_LIBRARIES cisstCommon cisstVector cisstOSAbstraction
                              cisstMultiTask cisstParameterTypes cisstFLTK)

# find cisst and make sure the required libraries have been compiled
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # sawSensablePhantom has been compiled within cisst, we should find it automatically
  cisst_find_saw_component (sawSensablePhantom REQUIRED)

  if (sawSensablePhantom_FOUND)

    # sawSensablePhantom configuration
    include_directories (${sawSensablePhantom_INCLUDE_DIR})
    link_directories (${sawSensablePhantom_LIBRARY_DIR})

    # CMake command, generate .cxx and .h from .fl files
    fltk_wrap_ui (sawSensableHDExampleFLTK displayUI.fl)

    # name the main executable and specifies with source files to use
    add_executable (sawSensableHDExampleFLTK
                    displayTask.cpp
                    main.cpp
                    displayTask.h
                    ${sawSensableHDExampleFLTK_FLTK_UI_SRCS}
                    )
    set_property (TARGET sawSensableHDExampleFLTK PROPERTY FOLDER "sawSensablePhantom")
    target_link_libraries (sawSensableHDExampleFLTK ${sawSensablePhantom_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawSensableHDExampleFLTK ${REQUIRED_CISST_LIBRARIES})

  endif (sawSensablePhantom_FOUND)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
