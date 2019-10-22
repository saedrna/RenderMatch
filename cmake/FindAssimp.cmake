# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#.rst:
# FindAssimp
# --------
#
# Find the Assimp library (libAssimp).
#
# Imported targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets:
#
# ``Assimp::Assimp``
#   The Assimp library, if found.
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project:
#
# ``Assimp_FOUND``
#   true if the Assimp headers and libraries were found
# ``Assimp_INCLUDE_DIR``
#   the directory containing the Assimp headers
# ``Assimp_INCLUDE_DIRS``
#   the directory containing the Assimp headers
# ``Assimp_LIBRARIES``
#   Assimp libraries to be linked
#
# Cache variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``Assimp_INCLUDE_DIR``
#   the directory containing the Assimp headers
# ``Assimp_LIBRARY``
#   the path to the Assimp library

find_path(Assimp_INCLUDE_DIR "assimp/scene.h")

set(Assimp_NAMES ${Assimp_NAMES} assimp-vc140-mt)
foreach(name ${Assimp_NAMES})
  list(APPEND Assimp_NAMES_DEBUG "${name}d")
endforeach()

if(NOT Assimp_LIBRARY)
  find_library(Assimp_LIBRARY_RELEASE NAMES ${Assimp_NAMES})
  find_library(Assimp_LIBRARY_DEBUG NAMES ${Assimp_NAMES_DEBUG})
  include(SelectLibraryConfigurations)
  select_library_configurations(Assimp)
  mark_as_advanced(Assimp_LIBRARY_RELEASE Assimp_LIBRARY_DEBUG)
endif()
unset(Assimp_NAMES)
unset(Assimp_NAMES_DEBUG)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Assimp
                                  REQUIRED_VARS Assimp_LIBRARY Assimp_INCLUDE_DIR
                                  VERSION_VAR Assimp_VERSION_STRING)

if(Assimp_FOUND)
  set(Assimp_LIBRARIES ${Assimp_LIBRARY})
  set(Assimp_INCLUDE_DIRS "${Assimp_INCLUDE_DIR}")

  if(NOT TARGET Assimp::Assimp)
    add_library(Assimp::Assimp UNKNOWN IMPORTED)
    if(Assimp_INCLUDE_DIRS)
      set_target_properties(Assimp::Assimp PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${Assimp_INCLUDE_DIRS}")
    endif()
    if(EXISTS "${Assimp_LIBRARY}")
      set_target_properties(Assimp::Assimp PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "C"
        IMPORTED_LOCATION "${Assimp_LIBRARY}")
    endif()
    if(EXISTS "${Assimp_LIBRARY_RELEASE}")
      set_property(TARGET Assimp::Assimp APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(Assimp::Assimp PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
        IMPORTED_LOCATION_RELEASE "${Assimp_LIBRARY_RELEASE}")
    endif()
    if(EXISTS "${Assimp_LIBRARY_DEBUG}")
      set_property(TARGET Assimp::Assimp APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(Assimp::Assimp PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "C"
        IMPORTED_LOCATION_DEBUG "${Assimp_LIBRARY_DEBUG}")
    endif()
  endif()
endif()

mark_as_advanced(Assimp_INCLUDE_DIR Assimp_LIBRARY)
