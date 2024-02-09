#
# $Id$
#
# Author(s):  Anton Deguet
# Created on: 2008-04-04
#
# (C) Copyright 2008 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---
#
# Try to find Sensable HD library (aka 3DTouch)
#
# The following values are defined
#
# SENSABLE_INCLUDE_DIR          - include directories to use
# SENSABLE_LIBRARIES            - libraries needed
# SENSABLE_LIBRARIES_RELEASE    - libraries needed in release mode (deprecated)
# SENSABLE_LIBRARIES_DEBUG      - libraries needed in debug mode (deprecated)
#
# Important notes:
# - Only uses the environment variable 3DTOUCH_BASE, could search in
#   c:\program files in future releases.
# - Should use SENSABLE_LIBRARIES, which already includes the debug
#   and optimized CMake keywords, e.g.,:
#      TARGET_LINK_LIBRARIES(myTarget ${SENSABLE_LIBRARIES})
#   instead of:
#      TARGET_LINK_LIBRARIES(myTarget debug ${SENSABLE_LIBRARIES_DEBUG} optimized ${SENSABLE_LIBRARIES_RELEASE})
#   The latter syntax does not work correctly because the debug and optimized keywords only apply to the
#   next library in the list.

if (WIN32)
  set (SENSABLE_BASE_ENV $ENV{3DTOUCH_BASE})

  if (DEFINED SENSABLE_BASE_ENV)
    set (SENSABLE_FOUND true)
    set (SENSABLE_INCLUDE_DIR
         ${SENSABLE_BASE_ENV}/include
         ${SENSABLE_BASE_ENV}/utilities/include)

    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
      SET(LIBRARY_PREFIX "x64")
    else()
      SET(LIBRARY_PREFIX "Win32")
    endif()

    #set(SENSABLE_LIB_HD "SENSABLE_LIB_HD-NOTFOUND")
    find_library (SENSABLE_LIB_HD hd PATHS ${SENSABLE_BASE_ENV}/lib/${LIBRARY_PREFIX}/Release)
    find_library (SENSABLE_LIB_HL hl PATHS ${SENSABLE_BASE_ENV}/lib/${LIBRARY_PREFIX}/Release)
    find_library (SENSABLE_LIB_HDD hd PATHS ${SENSABLE_BASE_ENV}/lib/${LIBRARY_PREFIX}/Debug)
    find_library (SENSABLE_LIB_HLD hl PATHS ${SENSABLE_BASE_ENV}/lib/${LIBRARY_PREFIX}/Debug)

    find_library (SENSABLE_LIB_HDU hdu PATHS ${SENSABLE_BASE_ENV}/utilities/lib/${LIBRARY_PREFIX}/Release)
    find_library (SENSABLE_LIB_HLU hlu PATHS ${SENSABLE_BASE_ENV}/utilities/lib/${LIBRARY_PREFIX}/Release)
    find_library (SENSABLE_LIB_HDUD hdu PATHS ${SENSABLE_BASE_ENV}/utilities/lib/${LIBRARY_PREFIX}/Debug)
    find_library (SENSABLE_LIB_HLUD hlu PATHS ${SENSABLE_BASE_ENV}/utilities/lib/${LIBRARY_PREFIX}/Debug)

    set (SENSABLE_LIBRARIES_RELEASE
         ${SENSABLE_LIB_HD} ${SENSABLE_LIB_HDU}
         ${SENSABLE_LIB_HL} ${SENSABLE_LIB_HLU})
    set (SENSABLE_LIBRARIES_DEBUG
         ${SENSABLE_LIB_HDD} ${SENSABLE_LIB_HDUD}
         ${SENSABLE_LIB_HLD} ${SENSABLE_LIB_HLUD})
    set (SENSABLE_LIBRARIES
         optimized ${SENSABLE_LIB_HD} debug ${SENSABLE_LIB_HDD}
         optimized ${SENSABLE_LIB_HL} debug ${SENSABLE_LIB_HLD}
         optimized ${SENSABLE_LIB_HDU} debug ${SENSABLE_LIB_HDUD}
         optimized ${SENSABLE_LIB_HLU} debug ${SENSABLE_LIB_HLUD})

    file (TO_CMAKE_PATH "${SENSABLE_INCLUDE_DIR}" SENSABLE_INCLUDE_DIR)
    file (TO_CMAKE_PATH "${SENSABLE_LIBRARIES}" SENSABLE_LIBRARIES)

    mark_as_advanced (SENSABLE_LIB_HD
                      SENSABLE_LIB_HL
                      SENSABLE_LIB_HDU
                      SENSABLE_LIB_HDUD
                      SENSABLE_LIB_HLU
                      SENSABLE_LIB_HLUD
                      SENSABLE_BASE_ENV
                      SENSABLE_INCLUDE_DIR
                      SENSABLE_LIBRARIES_RELEASE
                      SENSABLE_LIBRARIES_DEBUG
                      SENSABLE_LIBRARIES)

  else (DEFINED SENSABLE_BASE_ENV)
    message ("Make sure the environment variable 3DTOUCH_BASE is defined")
  endif (DEFINED SENSABLE_BASE_ENV)
endif (WIN32)

if (APPLE)
  set (SENSABLE_FRAMEWORKS)
  include (CMakeFindFrameworks)
  cmake_find_frameworks (HD)
  if (HD_FRAMEWORKS)
    set (SENSABLE_INCLUDE_DIR ${SENSABLE_INCLUDE_DIR} ${HD_FRAMEWORKS}/Headers)
    set (SENSABLE_FRAMEWORKS "-framework HD" ${SENSABLE_FRAMEWORKS})
  endif (HD_FRAMEWORKS)
  cmake_find_frameworks (HL)
  if (HL_FRAMEWORKS)
    set (SENSABLE_INCLUDE_DIR ${SENSABLE_INCLUDE_DIR} ${HL_FRAMEWORKS}/Headers)
    set (SENSABLE_FRAMEWORKS "-framework HL" ${SENSABLE_FRAMEWORKS})
  endif (HL_FRAMEWORKS)
  cmake_find_frameworks (HDU)
  if (HDU_FRAMEWORKS)
    set (SENSABLE_INCLUDE_DIR ${SENSABLE_INCLUDE_DIR} ${HDU_FRAMEWORKS}/Headers)
    set (SENSABLE_FRAMEWORKS "-framework HDU" ${SENSABLE_FRAMEWORKS})
  endif (HDU_FRAMEWORKS)
  cmake_find_frameworks (HLU)
  if (HLU_FRAMEWORKS)
    set (SENSABLE_INCLUDE_DIR ${SENSABLE_INCLUDE_DIR} ${HLU_FRAMEWORKS}/Headers)
    set (SENSABLE_FRAMEWORKS "-framework HLU" ${SENSABLE_FRAMEWORKS})
  endif (HLU_FRAMEWORKS)

  if (SENSABLE_FRAMEWORKS)
    set (SENSABLE_LIBRARIES_RELEASE ${SENSABLE_FRAMEWORKS})
    set (SENSABLE_LIBRARIES_DEBUG ${SENSABLE_FRAMEWORKS})
    set (SENSABLE_LIBRARIES ${SENSABLE_FRAMEWORKS})
  endif (SENSABLE_FRAMEWORKS)

  mark_as_advanced (SENSABLE_FRAMEWORKS
                    SENSABLE_INCLUDE_DIR
                    SENSABLE_LIBRARIES_RELEASE
                    SENSABLE_LIBRARIES_DEBUG
                    SENSABLE_LIBRARIES)
endif (APPLE)

if ("${CMAKE_SYSTEM}" MATCHES "Linux")
  set (SENSABLE_SEARCH_PATH /usr /usr/local)
  find_path (SENSABLE_DIR include/HD/hd.h ${SENSABLE_SEARCH_PATH})
  if (SENSABLE_DIR)
    set(SENSABLE_INCLUDE_DIR ${SENSABLE_DIR}/include)
    set(SENSABLE_FOUND true)
    find_library (SENSABLE_LIB_HD HD  ${SENSABLE_DIR}/lib ${SENSABLE_DIR}/lib64)
    find_library (SENSABLE_LIB_HDU HDU  ${SENSABLE_DIR}/lib ${SENSABLE_DIR}/lib64)
    find_library (SENSABLE_LIB_HL HL  ${SENSABLE_DIR}/lib ${SENSABLE_DIR}/lib64)
    find_library (SENSABLE_LIB_HLU HLU  ${SENSABLE_DIR}/lib ${SENSABLE_DIR}/lib64)

    set (SENSABLE_LIBRARIES
         ${SENSABLE_LIB_HD}
         ${SENSABLE_LIB_HDU}
         ${SENSABLE_LIB_HL}
         ${SENSABLE_LIB_HLU})

    mark_as_advanced (SENSABLE_LIB_HD
                      SENSABLE_LIB_HDU
                      SENSABLE_LIB_HL
                      SENSABLE_LIB_HLU)
  endif (SENSABLE_DIR)
endif ("${CMAKE_SYSTEM}" MATCHES "Linux")
