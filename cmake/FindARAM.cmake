# Locate ARAM library
# This module defines
#  ARAM_FOUND, if false, do not try to link to ARAM
#  ARAM_LIBRARY
#  ARAM_INCLUDE_DIR, where to find ARAMException.hpp
#  ARAM_DIR - Can be set to ARAM install path or Windows build path

find_path(ARAM_INCLUDE_DIR ARAM/ARAMException.hpp
  HINTS ${ARAM_DIR}
  PATH_SUFFIXES include
  PATHS
  ~/Library/Frameworks
  /Library/Frameworks
  /usr/local
  /usr
  /sw # Fink
  /opt/local # DarwinPorts
  /opt/csw # Blastwave
  /opt
)

find_library(ARAM_LIBRARY 
  NAMES ARAM
  HINTS ${ARAM_DIR}
  PATH_SUFFIXES lib64 lib
  PATHS
  ~/Library/Frameworks
  /Library/Frameworks
  /usr/local
  /usr
  /sw
  /opt/local
  /opt/csw
  /opt
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ARAM_FOUND to TRUE if 
# all listed variables are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ARAM DEFAULT_MSG ARAM_LIBRARY ARAM_INCLUDE_DIR)

mark_as_advanced(ARAM_INCLUDE_DIR ARAM_LIBRARY)

