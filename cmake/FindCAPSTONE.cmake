# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#.rst:
# FindCAPSTONE
# --------
#
# Find the CAPSTONE library (libCAPSTONE).
#
# Imported targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets:
#
# ``CAPSTONE::CAPSTONE``
#   The CAPSTONE library, if found.
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project:
#
# ``CAPSTONE_FOUND``
#   true if the CAPSTONE headers and libraries were found
# ``CAPSTONE_INCLUDE_DIR``
#   the directory containing the CAPSTONE headers
# ``CAPSTONE_INCLUDE_DIRS``
#   the directory containing the CAPSTONE headers
# ``CAPSTONE_LIBRARIES``
#   CAPSTONE libraries to be linked
#
# Cache variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``CAPSTONE_INCLUDE_DIR``
#   the directory containing the CAPSTONE headers
# ``CAPSTONE_LIBRARY``
#   the path to the CAPSTONE library

find_path(CAPSTONE_INCLUDE_DIR "capstone/capstone.h")

set(CAPSTONE_NAMES ${CAPSTONE_NAMES} capstone)

foreach(name ${CAPSTONE_NAMES})
  list(APPEND CAPSTONE_NAMES_DEBUG  "${name}" "${name}d")
endforeach()

if(NOT CAPSTONE_LIBRARY)
  find_library(CAPSTONE_LIBRARY_RELEASE NAMES ${CAPSTONE_NAMES})
  find_library(CAPSTONE_LIBRARY_DEBUG NAMES ${CAPSTONE_NAMES_DEBUG})
  include(SelectLibraryConfigurations)
  select_library_configurations(CAPSTONE)
  mark_as_advanced(CAPSTONE_LIBRARY_RELEASE CAPSTONE_LIBRARY_DEBUG)
endif()
unset(CAPSTONE_NAMES)
unset(CAPSTONE_NAMES_DEBUG)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(CAPSTONE
                                  REQUIRED_VARS CAPSTONE_LIBRARY CAPSTONE_INCLUDE_DIR
                                  VERSION_VAR CAPSTONE_VERSION_STRING)

if(CAPSTONE_FOUND)
  set(CAPSTONE_LIBRARIES ${CAPSTONE_LIBRARY})
  set(CAPSTONE_INCLUDE_DIRS "${CAPSTONE_INCLUDE_DIR}")

  if(NOT TARGET CAPSTONE::CAPSTONE)
    add_library(CAPSTONE::CAPSTONE UNKNOWN IMPORTED)
    if(CAPSTONE_INCLUDE_DIRS)
      set_target_properties(CAPSTONE::CAPSTONE PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${CAPSTONE_INCLUDE_DIRS}")
    endif()
    if(EXISTS "${CAPSTONE_LIBRARY}")
      set_target_properties(CAPSTONE::CAPSTONE PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "C"
        IMPORTED_LOCATION "${CAPSTONE_LIBRARY}")
    endif()
    if(EXISTS "${CAPSTONE_LIBRARY_RELEASE}")
      set_property(TARGET CAPSTONE::CAPSTONE APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(CAPSTONE::CAPSTONE PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
        IMPORTED_LOCATION_RELEASE "${CAPSTONE_LIBRARY_RELEASE}")
    endif()
    if(EXISTS "${CAPSTONE_LIBRARY_DEBUG}")
      set_property(TARGET CAPSTONE::CAPSTONE APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(CAPSTONE::CAPSTONE PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C"
        IMPORTED_LOCATION_DEBUG "${CAPSTONE_LIBRARY_DEBUG}")
    endif()
  endif()
endif()

mark_as_advanced(CAPSTONE_INCLUDE_DIR CAPSTONE_LIBRARY)
