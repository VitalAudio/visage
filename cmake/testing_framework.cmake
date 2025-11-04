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
    add_executable(${PARSE_TARGET} ${HEADERS} ${SOURCE_FILES})
    target_link_libraries(${PARSE_TARGET} PRIVATE Catch2::Catch2WithMain visage)
    catch_discover_tests(${PARSE_TARGET})
    set_target_properties(${PARSE_TARGET} PROPERTIES FOLDER "visage/tests")
  endif ()
endfunction()
