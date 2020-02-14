# * Find Intel IPP Find the IPP libraries Options:
#
# IPP_STATIC: true if using static linking IPP_MULTI_THREADED: true if using
# multi-threaded static linking
#
# This module defines the following variables:
#
# IPP_FOUND       : True if IPP_INCLUDE_DIR are found IPP_INCLUDE_DIR : where to
# find ipp.h, etc. IPP_INCLUDE_DIRS: set when IPP_INCLUDE_DIR found
# IPP_LIBRARIES   : the library to link against.

include(FindPackageHandleStandardArgs)

set(IPP_STATIC ON CACHE BOOL "use static or dynamic IPP")

# Find header file dir
find_path(IPP_INCLUDE_DIR ipp.h
          PATHS ${CMAKE_PREFIX_PATH}/include ${CMAKE_PREFIX_PATH}/include/ipp)
mark_as_advanced("${IPP_INCLUDE_DIR}")

# Find libraries
set(IPP_COMPONENTS I S CORE)

if(WIN32)
  set(CMAKE_FIND_LIBRARY_SUFFIXES .lib)
else()
  if(IPP_STATIC)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a)
  else()
    set(CMAKE_FIND_LIBRARY_SUFFIXES .so)
  endif()
endif()

if(IPP_STATIC AND MSVC)
  set(IPP_LIBNAME_SUFFIX mt)
else()
  set(IPP_LIBNAME_SUFFIX "")
endif()

macro(FIND_IPP_LIBRARY libname)
  string(TOLOWER ${libname} libname_lower)

  find_library(IPP_LIB_${libname}
               NAMES ipp${libname_lower}${IPP_LIBNAME_SUFFIX})
  mark_as_advanced("IPP_LIB_${libname}")
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(IPP
                                    REQUIRED_VARS
                                    IPP_LIB_${libname}
                                    IPP_INCLUDE_DIR)

  if(NOT TARGET "IPP::${libname}")
    add_library("IPP::${libname}" UNKNOWN IMPORTED)
    if(IPP_INCLUDE_DIR)
      set_target_properties("IPP::${libname}"
                            PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                       "${IPP_INCLUDE_DIR}")
    endif()
    if(EXISTS "${IPP_LIB_${libname}}")
      set_target_properties("IPP::${libname}"
                            PROPERTIES IMPORTED_LOCATION "${IPP_LIB_${libname}}")
    endif()
  endif()
endmacro(FIND_IPP_LIBRARY libname)

# Handle suffix
set(_IPP_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})

# IPP components Core
set(IPP_LIBRARIES "")
foreach(name ${IPP_COMPONENTS})
    find_ipp_library(${name})
    list(APPEND IPP_LIBRARIES IPP::${name})
endforeach(name ${IPP_COMPONENTS})

set(CMAKE_FIND_LIBRARY_SUFFIXES ${_IPP_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})
