cmake_minimum_required(VERSION 3.17)
project(visage VERSION 0.1.0)

if (NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()

if (APPLE)
  enable_language(OBJC)
  enable_language(OBJCXX)
endif ()

option(VISAGE_AMALGAMATED_BUILD "Compile sources together" ON)
option(VISAGE_ENABLE_WIDGETS "Add widgets library" ON)
option(VISAGE_ENABLE_BACKGROUND_GRAPHICS_THREAD "Offloads graphics rendering to a background thread" ON)
option(VISAGE_ENABLE_GRAPHICS_DEBUG_LOGGING "Shows graphics debug log in console in debug mode" ON)
option(VISAGE_BUILD_TESTS "Build tests" OFF)
set(VISAGE_APPLICATION_NAME ${CMAKE_PROJECT_NAME} CACHE STRING "Application name used for default window title")

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  option(VISAGE_BUILD_EXAMPLES "Build examples" ON)
else ()
  option(VISAGE_BUILD_EXAMPLES "Build examples" OFF)
endif ()

set(VISAGE_INCLUDE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

if (MSVC)
  # set(SANITIZER_FLAGS "/fsanitize=address")
elseif (NOT EMSCRIPTEN)
  # set(SANITIZER_FLAGS "-fsanitize=address")
  # set(SANITIZER_FLAGS "-fsanitize=thread")
endif ()

add_compile_options("$<$<CONFIG:Debug>:${SANITIZER_FLAGS}>")
add_link_options("$<$<CONFIG:Debug>:${SANITIZER_FLAGS}>")

include(cmake/compile_flags.cmake)
include(cmake/testing_framework.cmake)

function(amalgamate_sources AMALGAMATED_SOURCE FILE_SOURCES)
  file(WRITE "${AMALGAMATED_SOURCE}" "// Auto generated amalgamated source.\n")
  foreach (FILE ${FILE_SOURCES})
    message(${FILE})
    file(APPEND "${AMALGAMATED_SOURCE}" "#include \"${FILE}\"\n")
  endforeach ()
endfunction()

add_subdirectory(visage_file_embed)
add_subdirectory(visage_graphics)
add_subdirectory(visage_ui)
add_subdirectory(visage_utils)
add_subdirectory(visage_widgets)
add_subdirectory(visage_windowing)
add_subdirectory(visage_app)

set(DUMMY_FILE ${CMAKE_CURRENT_BINARY_DIR}/dummy.cpp)
if (NOT EXISTS ${DUMMY_FILE})
  file(WRITE ${DUMMY_FILE} "")
endif ()

add_library(visage STATIC
  ${DUMMY_FILE}
  $<TARGET_OBJECTS:VisageApp>
  $<TARGET_OBJECTS:VisageGraphics>
  $<TARGET_OBJECTS:VisageUtils>
  $<TARGET_OBJECTS:VisageUi>
  $<TARGET_OBJECTS:VisageWidgets>
  $<TARGET_OBJECTS:VisageWindowing>
)

target_link_libraries(visage
  PRIVATE
  ${VisageGraphicsDependencies}
  ${VisageWindowingDependencies}
)

target_include_directories(visage SYSTEM PUBLIC ${VISAGE_INCLUDE_PATH})
set_target_properties(visage PROPERTIES FOLDER "visage")

if (VISAGE_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif ()
