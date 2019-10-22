# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#.rst:
# FindEmbree3
# --------
#
# Find the Embree3 library (libEmbree3).
#
# Imported targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets:
#
# ``Embree3::Embree3``
#   The Embree3 library, if found.
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project:
#
# ``Embree3_FOUND``
#   true if the Embree3 headers and libraries were found
# ``Embree3_INCLUDE_DIR``
#   the directory containing the Embree3 headers
# ``Embree3_INCLUDE_DIRS``
#   the directory containing the Embree3 headers
# ``Embree3_LIBRARIES``
#   Embree3 libraries to be linked
#
# Cache variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``Embree3_INCLUDE_DIR``
#   the directory containing the Embree3 headers
# ``Embree3_LIBRARY``
#   the path to the Embree3 library

find_path(Embree3_INCLUDE_DIR "embree3/rtcore.h")

set(Embree3_NAMES ${Embree3_NAMES} embree3 libembree3)
foreach(name ${Embree3_NAMES})
  list(APPEND Embree3_NAMES_DEBUG "${name}" "${name}d")
endforeach()

if(NOT Embree3_LIBRARY)
  find_library(Embree3_LIBRARY_RELEASE NAMES ${Embree3_NAMES})
  find_library(Embree3_LIBRARY_DEBUG NAMES ${Embree3_NAMES_DEBUG})
  include(SelectLibraryConfigurations)
  select_library_configurations(Embree3)
  mark_as_advanced(Embree3_LIBRARY_RELEASE Embree3_LIBRARY_DEBUG)
endif()
unset(Embree3_NAMES)
unset(Embree3_NAMES_DEBUG)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Embree3
                                  REQUIRED_VARS Embree3_LIBRARY Embree3_INCLUDE_DIR
                                  VERSION_VAR Embree3_VERSION_STRING)

if(Embree3_FOUND)
  set(Embree3_LIBRARIES ${Embree3_LIBRARY})
  set(Embree3_INCLUDE_DIRS "${Embree3_INCLUDE_DIR}")

  if(NOT TARGET Embree3::Embree3)
    add_library(Embree3::Embree3 UNKNOWN IMPORTED)
    if(Embree3_INCLUDE_DIRS)
      set_target_properties(Embree3::Embree3 PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${Embree3_INCLUDE_DIRS}")
    endif()
    if(EXISTS "${Embree3_LIBRARY}")
      set_target_properties(Embree3::Embree3 PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "C"
        IMPORTED_LOCATION "${Embree3_LIBRARY}")
    endif()
    if(EXISTS "${Embree3_LIBRARY_RELEASE}")
      set_property(TARGET Embree3::Embree3 APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(Embree3::Embree3 PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
        IMPORTED_LOCATION_RELEASE "${Embree3_LIBRARY_RELEASE}")
    endif()
    if(EXISTS "${Embree3_LIBRARY_DEBUG}")
      set_property(TARGET Embree3::Embree3 APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(Embree3::Embree3 PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C"
        IMPORTED_LOCATION_DEBUG "${Embree3_LIBRARY_DEBUG}")
    endif()
  endif()
endif()

mark_as_advanced(Embree3_INCLUDE_DIR Embree3_LIBRARY)
