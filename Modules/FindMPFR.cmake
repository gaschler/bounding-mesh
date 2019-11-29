# Try to find the GMP librairies
#  MPFR_FOUND - system has GMP lib
#  MPFR_INCLUDE_DIR - the GMP include directory
#  MPFR_LIBRARIES - Libraries needed to use GMP

# Copyright (c) 2006, Laurent Montel, <montel@kde.org>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

if (MPFR_INCLUDE_DIR AND MPFR_LIBRARIES)
  # Already in cache, be silent
  set(MPFR_FIND_QUIETLY TRUE)
endif (MPFR_INCLUDE_DIR AND MPFR_LIBRARIES)

find_path(MPFR_INCLUDE_DIR NAMES mpfr.h )
find_library(MPFR_LIBRARIES NAMES mpfr libpmfr )
MESSAGE(STATUS "MPFR libs: " ${MPFR_LIBRARIES} )

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(MPFR DEFAULT_MSG MPFR_INCLUDE_DIR MPFR_LIBRARIES)

mark_as_advanced(MPFR_INCLUDE_DIR MPFR_LIBRARIES)
