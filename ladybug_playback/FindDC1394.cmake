# - Try to find libdc1394 v2
# Once done this will define
#  DC1394_FOUND - System has libdc1394
#  DC1394_INCLUDE_DIRS - The libdc1394 include directories
#  DC1394_LIBRARIES - The libraries needed to use libdc1394
#  DC1394_DEFINITIONS - Compiler switches required for using libdc1394

find_package(PkgConfig)
pkg_check_modules(PC_DC1394 QUIET libdc1394-2)
set(DC1394_DEFINITIONS ${PC_DC1394_CFLAGS_OTHER})

find_path(DC1394_INCLUDE_DIR dc1394/dc1394.h
          HINTS ${PC_DC1394_INCLUDEDIR} ${PC_DC1394_INCLUDE_DIRS}
          PATH_SUFFIXES dc1394)

find_library(DC1394_LIBRARY NAMES dc1394
             HINTS ${PC_DC1394_LIBDIR} ${PC_DC1394_LIBRARY_DIRS} )

set(DC1394_LIBRARIES ${DC1394_LIBRARY} )
set(DC1394_INCLUDE_DIRS ${DC1394_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set DC1394_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(DC1394 DEFAULT_MSG
                                  DC1394_LIBRARY DC1394_INCLUDE_DIR)

mark_as_advanced(DC1394_INCLUDE_DIR DC1394_LIBRARY )
