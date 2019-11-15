# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

# .rst: FindTBB
# --------
#
# Find the TBB library (libTBB).
#
# Imported targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets:
#
# ``TBB::TBB`` The TBB library, if found.
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project:
#
# ``TBB_FOUND`` true if the TBB headers and libraries were found
# ``TBB_INCLUDE_DIR`` the directory containing the TBB headers
# ``TBB_INCLUDE_DIRS`` the directory containing the TBB headers
# ``TBB_LIBRARIES`` TBB libraries to be linked
#
# Cache variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``TBB_INCLUDE_DIR`` the directory containing the TBB headers ``TBB_LIBRARY``
# the path to the TBB library

find_path(TBB_INCLUDE_DIR "tbb/tbb.h")

function(add_tbb_library target)
  if(NOT TBB_${target}_LIBRARY)
    find_library(TBB_${target}_LIBRARY_RELEASE NAMES ${target})
    find_library(TBB_${target}_LIBRARY_DEBUG NAMES ${target})
    include(SelectLibraryConfigurations)
    select_library_configurations(TBB_${target})
    mark_as_advanced(TBB_${target}_LIBRARY_RELEASE TBB_${target}_LIBRARY_DEBUG)
  endif()

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(TBB_${target}
                                    REQUIRED_VARS
                                    TBB_${target}_LIBRARY
                                    TBB_INCLUDE_DIR
                                    VERSION_VAR
                                    TBB_VERSION_STRING)

  if(TBB_${target}_FOUND)
    if(NOT TARGET TBB::${target})
      add_library(TBB::${target} UNKNOWN IMPORTED)
      if(TBB_INCLUDE_DIR)
        set_target_properties(TBB::${target}
                              PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                         "${TBB_INCLUDE_DIR}")
      endif()
      if(EXISTS "${TBB_${target}_LIBRARY}")
        set_target_properties(TBB::${target}
                              PROPERTIES IMPORTED_LOCATION
                                         "${TBB_${target}_LIBRARY}")
      endif()
      if(EXISTS "${TBB_${target}_LIBRARY_RELEASE}")
        set_property(TARGET TBB::${target}
                     APPEND
                     PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
        set_target_properties(
          TBB::${target}
          PROPERTIES IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE
                     "${TBB_${target}_LIBRARY_RELEASE}")
      endif()
      if(EXISTS "${TBB_${target}_LIBRARY_DEBUG}")
        set_property(TARGET TBB::${target}
                     APPEND
                     PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
        set_target_properties(TBB::${target}
                              PROPERTIES IMPORTED_LOCATION_DEBUG
                                         "${TBB_${target}_LIBRARY_DEBUG}")
      endif()
    endif()
  endif()
  mark_as_advanced(TBB_INCLUDE_DIR TBB_${target}_LIBRARY)
endfunction(add_tbb_library ${target})

set(TBB_COMPONENTS tbb tbbmalloc tbbmalloc_proxy)
foreach(target ${TBB_COMPONENTS})
  add_tbb_library(${target})
endforeach(target ${TBB_COMPONENTS})
