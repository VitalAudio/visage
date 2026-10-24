if (VISAGE_BUILD_TESTS AND NOT EMSCRIPTEN)
  message(STATUS "VISAGE: Downloading testing dependencies")

  include(FetchContent)
  FetchContent_Declare(Catch2 GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_SHALLOW TRUE
    GIT_TAG 4e8d92bf02f7d1c8006a0e7a5ecabd8e62d98502
    EXCLUDE_FROM_ALL
  )
  FetchContent_MakeAvailable(Catch2)
  list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
  include(CTest)
  include(Catch)

  set_target_properties(Catch2 PROPERTIES FOLDER "visage/third_party/Catch2")
  set_target_properties(Catch2WithMain PROPERTIES FOLDER "visage/third_party/Catch2")
endif ()

function(visage_add_test_target)
  set(single_options TARGET TEST_DIRECTORY)
  cmake_parse_arguments(PARSE "" "${single_options}" "${multi_options}" ${ARGN})

  if (VISAGE_BUILD_TESTS AND NOT EMSCRIPTEN)
    file(GLOB_RECURSE HEADERS ${PARSE_TEST_DIRECTORY}/*.h)
    file(GLOB_RECURSE SOURCE_FILES ${PARSE_TEST_DIRECTORY}/*.cpp)
    if (APPLE)
      file(GLOB_RECURSE OBJC_SOURCE_FILES ${PARSE_TEST_DIRECTORY}/*.mm)
      list(APPEND SOURCE_FILES ${OBJC_SOURCE_FILES})
    endif ()
    add_executable(${PARSE_TARGET} ${HEADERS} ${SOURCE_FILES} ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/test_main.cpp)
    target_link_libraries(${PARSE_TARGET} PRIVATE Catch2::Catch2 visage bgfx)
    catch_discover_tests(${PARSE_TARGET} TEST_SPEC "~[x11_selection]" PROPERTIES SKIP_RETURN_CODE 4)
    catch_discover_tests(${PARSE_TARGET} TEST_SPEC "[x11_selection]"
                         PROPERTIES SKIP_RETURN_CODE 4 RESOURCE_LOCK visage_x11_selection)
    set_target_properties(${PARSE_TARGET} PROPERTIES FOLDER "visage/tests")
  endif ()
endfunction()
