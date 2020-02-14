# Mark variables as used so cmake doesn't complain about them
mark_as_advanced(CMAKE_TOOLCHAIN_FILE)

if(VCPKG_CHAINLOAD_TOOLCHAIN_FILE)
    include("${VCPKG_CHAINLOAD_TOOLCHAIN_FILE}")
endif()

if(TOOLCHAIN_LOADED)
  return()
endif()
set(TOOLCHAIN_LOADED ON)

set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_LIST_DIR}/Library)
mark_as_advanced(CMAKE_PREFIX_PATH)

function(add_executable name)
  _add_executable(${ARGV})
  list(FIND ARGV "IMPORTED" IMPORTED_IDX)
  list(FIND ARGV "ALIAS" ALIAS_IDX)
  if(IMPORTED_IDX EQUAL -1 AND ALIAS_IDX EQUAL -1)
    add_custom_command(
      TARGET ${name} POST_BUILD
      COMMAND
        powershell -noprofile -executionpolicy Bypass -file
        ${CMAKE_PREFIX_PATH}/../applocal.ps1 -targetBinary
        $<TARGET_FILE:${name}> -installedDir
        "${CMAKE_PREFIX_PATH}/bin"
        -OutVariable out)
  endif()
endfunction()

function(add_library name)
  _add_library(${ARGV})
  list(FIND ARGV "IMPORTED" IMPORTED_IDX)
  list(FIND ARGV "INTERFACE" INTERFACE_IDX)
  list(FIND ARGV "ALIAS" ALIAS_IDX)
  if(IMPORTED_IDX EQUAL -1 AND INTERFACE_IDX EQUAL -1 AND ALIAS_IDX EQUAL -1)
    get_target_property(IS_LIBRARY_SHARED ${name} TYPE)
    if((IS_LIBRARY_SHARED STREQUAL "SHARED_LIBRARY") OR (IS_LIBRARY_SHARED STREQUAL "MODULE_LIBRARY"))
        add_custom_command(
            TARGET ${name} POST_BUILD
            COMMAND
            powershell -noprofile -executionpolicy Bypass -file
            ${CMAKE_PREFIX_PATH}/../applocal.ps1 -targetBinary
            $<TARGET_FILE:${name}> -installedDir
            "${CMAKE_PREFIX_PATH}/bin"
            -OutVariable out)
    endif()
  endif()
endfunction()
