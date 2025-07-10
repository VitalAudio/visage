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
option(VISAGE_ENABLE_BACKGROUND_GRAPHICS_THREAD "Offloads graphics rendering to a background thread" OFF)
option(VISAGE_ENABLE_GRAPHICS_DEBUG_LOGGING "Shows graphics debug log in console in debug mode" OFF)
option(VISAGE_ADDRESS_SANITIZER "Enable AddressSanitizer" OFF)
if (VISAGE_ADDRESS_SANITIZER)
  if (MSVC)
    add_compile_options(/fsanitize=address /Zi)
    add_link_options(/fsanitize=address)
  else ()
    add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address)
  endif ()
endif ()

set(VISAGE_APPLICATION_NAME ${CMAKE_PROJECT_NAME} CACHE STRING "Application name used for default window title")

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  option(VISAGE_BUILD_EXAMPLES "Build examples" ON)
  option(VISAGE_BUILD_TESTS "Build tests" ON)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
else ()
  option(VISAGE_BUILD_EXAMPLES "Build examples" OFF)
  option(VISAGE_BUILD_TESTS "Build tests" OFF)
endif ()

set(VISAGE_INCLUDE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

include(cmake/compile_flags.cmake)
include(cmake/testing_framework.cmake)

set(VISAGE_INCLUDE ${CMAKE_CURRENT_BINARY_DIR}/include)

function(amalgamate_headers AMALGAMATED_HEADER_NAME FILE_HEADERS)
  set(FILE_CONTENTS "// Auto generated amalgamated header.\n")
  foreach (FILE ${FILE_HEADERS})
    set(FILE_CONTENTS "${FILE_CONTENTS}#include \"${FILE}\"\n")
  endforeach ()
  file(CONFIGURE OUTPUT "${AMALGAMATED_HEADER_NAME}" CONTENT "${FILE_CONTENTS}")
endfunction()

function(amalgamate_sources AMALGAMATED_SOURCE FILE_SOURCES)
  set(FILE_CONTENTS "// Auto generated amalgamated source.\n")
  foreach (FILE ${FILE_SOURCES})
    set_source_files_properties("${FILE}" PROPERTIES HEADER_FILE_ONLY TRUE)
    set(FILE_CONTENTS "${FILE_CONTENTS}#include \"${FILE}\"\n")
  endforeach ()
  file(CONFIGURE OUTPUT "${AMALGAMATED_SOURCE}" CONTENT "${FILE_CONTENTS}")
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
)

target_include_directories(visage SYSTEM PUBLIC ${VISAGE_INCLUDE_PATH} ${VISAGE_INCLUDE})
target_link_libraries(visage PRIVATE
  VisageApp
  VisageGraphics
  VisageUtils
  VisageUi
  VisageWidgets
  VisageWindowing
)

set_target_properties(visage PROPERTIES FOLDER "visage")

if (VISAGE_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif ()
