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

cmake_minimum_required(VERSION 3.10)
project (sawSensablePhantomAll VERSION 1.0.0)

find_package (cisst REQUIRED)
include (${CISST_USE_FILE})
cisst_cpack_settings (
  VENDOR "JHU"
  MAINTAINER "anton.deguet@jhu.edu")

add_subdirectory (components)
add_subdirectory (share)

set (sawSensablePhantom_DIR "${sawSensablePhantomAll_BINARY_DIR}/components")
add_subdirectory (examples)
add_subdirectory (igtl)

include (CPack)
cpack_add_component (sawSensablePhantom)
cpack_add_component (sawSensablePhantom-dev
                     DEPENDS sawSensablePhantom)
cpack_add_component (sawSensablePhantom-Share
                     DEPENDS sawSensablePhantom)
cpack_add_component (sawSensablePhantom-Qt
                     DEPENDS sawSensablePhantom)
cpack_add_component (sawSensablePhantom-Examples
                     DEPENDS sawSensablePhantom sawSensablePhantom-Qt)
