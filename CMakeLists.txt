#
# (C) Copyright 2014-2016 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

project (sawConstraintController)
add_subdirectory (code)
option (sawConstraintController_EXAMPLES "Compile sawConstraintController examples")

if (sawConstraintController_EXAMPLES)
  add_subdirectory (examples)
endif ()
