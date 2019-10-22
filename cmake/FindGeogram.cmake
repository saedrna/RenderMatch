# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#.rst:
# FindGeogram
# --------
#
# Find the Geogram library (libGeogram).
#
# Imported targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets:
#
# ``Geogram::Geogram``
#   The Geogram library, if found.
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project:
#
# ``Geogram_FOUND``
#   true if the Geogram headers and libraries were found
# ``Geogram_INCLUDE_DIR``
#   the directory containing the Geogram headers
# ``Geogram_INCLUDE_DIRS``
#   the directory containing the Geogram headers
# ``Geogram_LIBRARIES``
#   Geogram libraries to be linked
#
# Cache variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``Geogram_INCLUDE_DIR``
#   the directory containing the Geogram headers
# ``Geogram_LIBRARY``
#   the path to the Geogram library

find_path(Geogram_INCLUDE_DIR "geogram/api/defs.h" PATHS ${CMAKE_PREFIX_PATH}/include ${CMAKE_PREFIX_PATH}/include/geogram1)

set(Geogram_NAMES ${Geogram_NAMES} geogram libgeogram)
foreach(name ${Geogram_NAMES})
  list(APPEND Geogram_NAMES_DEBUG "${name}" "${name}d")
endforeach()

if(NOT Geogram_LIBRARY)
  find_library(Geogram_LIBRARY_RELEASE NAMES ${Geogram_NAMES})
  find_library(Geogram_LIBRARY_DEBUG NAMES ${Geogram_NAMES_DEBUG})
  include(SelectLibraryConfigurations)
  select_library_configurations(Geogram)
  mark_as_advanced(Geogram_LIBRARY_RELEASE Geogram_LIBRARY_DEBUG)
endif()
unset(Geogram_NAMES)
unset(Geogram_NAMES_DEBUG)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Geogram
                                  REQUIRED_VARS Geogram_LIBRARY Geogram_INCLUDE_DIR
                                  VERSION_VAR Geogram_VERSION_STRING)

if(Geogram_FOUND)
  set(Geogram_LIBRARIES ${Geogram_LIBRARY})
  set(Geogram_INCLUDE_DIRS "${Geogram_INCLUDE_DIR}")

  if(NOT TARGET Geogram::Geogram)
    add_library(Geogram::Geogram UNKNOWN IMPORTED)
    if(Geogram_INCLUDE_DIRS)
      set_target_properties(Geogram::Geogram PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${Geogram_INCLUDE_DIRS}")
    endif()
    if(EXISTS "${Geogram_LIBRARY}")
      set_target_properties(Geogram::Geogram PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "C"
        IMPORTED_LOCATION "${Geogram_LIBRARY}")
    endif()
    if(EXISTS "${Geogram_LIBRARY_RELEASE}")
      set_property(TARGET Geogram::Geogram APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(Geogram::Geogram PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
        IMPORTED_LOCATION_RELEASE "${Geogram_LIBRARY_RELEASE}")
    endif()
    if(EXISTS "${Geogram_LIBRARY_DEBUG}")
      set_property(TARGET Geogram::Geogram APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(Geogram::Geogram PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C"
        IMPORTED_LOCATION_DEBUG "${Geogram_LIBRARY_DEBUG}")
    endif()
  endif()
endif()

mark_as_advanced(Geogram_INCLUDE_DIR Geogram_LIBRARY)
