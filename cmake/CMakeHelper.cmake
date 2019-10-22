# wrapper for test
macro(H2O_ADD_TEST TARGET_NAME TARGET_SRC)
  if(H2O_BUILD_TEST)
    # ${ARGN} will store the list of source files passed to this function.
    add_definitions(-DH2O_DATA_DIR="${H2O_DATA_DIR}")
    add_executable(${TARGET_NAME} ${TARGET_SRC})
    target_link_libraries(${TARGET_NAME} ${ARGN})
    enable_testing()
    add_test("test/${TARGET_NAME}" ${TARGET_NAME}
             WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set_target_properties(${TARGET_NAME} PROPERTIES FOLDER "h2o_tests/")
  endif()
endmacro(H2O_ADD_TEST)

macro(H2O_ADD_LIBRARY TARGET_NAME)
  add_library(${TARGET_NAME} ${ARGN})
  set_target_properties(${TARGET_NAME} PROPERTIES FOLDER "libs/")
  set_target_properties(${TARGET_NAME} PROPERTIES DEBUG_POSTFIX "d")
  target_include_directories(${TARGET_NAME}
                             PUBLIC $<INSTALL_INTERFACE:include/h2o>)

  # install targets
  install(TARGETS ${TARGET_NAME}
          EXPORT "${TARGETS_EXPORT_NAME}"
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin
          INCLUDES
          DESTINATION include)

  # install headers
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
          DESTINATION include/h2o
          FILES_MATCHING
          PATTERN "*.h"
          PATTERN "*.hpp")

  # install pdb files
  install(FILES $<$<CONFIG:Debug>:$<TARGET_PDB_FILE:${TARGET_NAME}>>
          DESTINATION bin)
  set_target_properties(${TARGET_NAME} PROPERTIES FOLDER "libs")
endmacro(H2O_ADD_LIBRARY)

macro(H2O_ADD_EXECUTABLE TARGET_NAME)
  add_executable(${TARGET_NAME} ${ARGN})
  set_target_properties(${TARGET_NAME} PROPERTIES FOLDER "libs/")
  set_target_properties(${TARGET_NAME} PROPERTIES DEBUG_POSTFIX "d")

  # install targets
  install(TARGETS ${TARGET_NAME}
          EXPORT "${TARGETS_EXPORT_NAME}"
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin
          INCLUDES
          DESTINATION include)

  # install pdb files
  install(FILES $<$<CONFIG:Debug>:$<TARGET_PDB_FILE:${TARGET_NAME}>>
          DESTINATION bin)
  set_target_properties(${TARGET_NAME} PROPERTIES FOLDER "apps")
endmacro(H2O_ADD_EXECUTABLE)
