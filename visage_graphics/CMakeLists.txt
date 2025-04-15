file(GLOB HEADERS *.h)
file(GLOB SOURCE_FILES *.cpp)
if (WIN32)
  list(APPEND SOURCE_FILES win32/emoji_win32.cpp)
  set(WIN_LIBS d2d1 dwrite Windowscodecs)
elseif (APPLE)
  list(APPEND SOURCE_FILES macos/emoji_macos.cpp)
elseif (UNIX)
  list(APPEND SOURCE_FILES linux/emoji_linux.cpp)
endif ()

if (VISAGE_AMALGAMATED_BUILD)
  amalgamate_sources(${CMAKE_CURRENT_BINARY_DIR}/visage_graphics_amalgamated.cpp "${SOURCE_FILES}")
  list(APPEND SOURCE_FILES ${CMAKE_CURRENT_BINARY_DIR}/visage_graphics_amalgamated.cpp)
endif ()

amalgamate_headers(${VISAGE_INCLUDE}/visage/graphics.h "${HEADERS}")
list(APPEND HEADERS ${VISAGE_INCLUDE}/visage/graphics.h)

add_library(VisageGraphics OBJECT ${SOURCE_FILES} ${HEADERS})

if (APPLE)
  find_library(METAL_LIBRARY Metal)
  find_library(METALKIT_LIBRARY MetalKit)
  set(METAL_LIBS ${METAL_LIBRARY} ${METALKIT_LIBRARY})
elseif (UNIX)
  add_definitions(-D_LARGEFILE64_SOURCE)
endif ()

set(BGFX_BUILD_EXAMPLES OFF CACHE INTERNAL "" FORCE)
set(BGFX_BUILD_EXAMPLE_COMMON OFF CACHE INTERNAL "" FORCE)
set(BGFX_BUILD_TOOLS OFF CACHE INTERNAL "" FORCE)
set(BGFX_BUILD_TESTS OFF CACHE INTERNAL "" FORCE)
set(BGFX_INSTALL OFF CACHE INTERNAL "" FORCE)
set(BX_AMALGAMATED ON CACHE INTERNAL "" FORCE)
set(BX_CONFIG_DEBUG OFF CACHE INTERNAL "" FORCE)
set(BGFX_CONFIG_DEBUG_ANNOTATION ${VISAGE_ENABLE_GRAPHICS_DEBUG_LOGGING} CACHE INTERNAL "" FORCE)
set(BGFX_AMALGAMATED ON CACHE INTERNAL "" FORCE)
set(BGFX_CONFIG_MAX_FRAME_BUFFERS 1024 CACHE INTERNAL "" FORCE)
set(BGFX_CONFIG_MAX_VIEWS 256 CACHE INTERNAL "" FORCE)
set(BGFX_CONFIG_MAX_DRAW_CALLS 8192 CACHE INTERNAL "" FORCE)
set(BGFX_WITH_WAYLAND OFF CACHE INTERNAL "" FORCE)
set(FT_DISABLE_PNG ON CACHE BOOL "" FORCE)
set(FT_DISABLE_HARFBUZZ ON CACHE BOOL "" FORCE)
set(FT_DISABLE_BROTLI ON CACHE BOOL "" FORCE)
set(FT_DISABLE_ZLIB ON CACHE BOOL "" FORCE)
set(FT_DISABLE_BZIP2 ON CACHE BOOL "" FORCE)

if (VISAGE_ENABLE_GRAPHICS_DEBUG_LOGGING)
  target_compile_definitions(VisageGraphics PUBLIC VISAGE_GRAPHICS_DEBUG_LOGGING=1)
endif ()

set(BGFX_CONFIG_MULTITHREADED 0 CACHE INTERNAL "" FORCE)
if (NOT EMSCRIPTEN AND VISAGE_ENABLE_BACKGROUND_GRAPHICS_THREAD)
  set(BGFX_CONFIG_MULTITHREADED 1 CACHE INTERNAL "" FORCE)
  target_compile_definitions(VisageGraphics PUBLIC VISAGE_BACKGROUND_GRAPHICS_THREAD=1)
endif ()

include(FetchContent)
message(STATUS "Downloading graphics dependencies")
set(CMAKE_MESSAGE_LOG_LEVEL "ERROR")
FetchContent_Declare(bgfx GIT_REPOSITORY https://github.com/bkaradzic/bgfx.cmake.git
  GIT_SHALLOW TRUE
  GIT_TAG v1.129.8863-490
  EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(bgfx)


FetchContent_Declare(freetype GIT_REPOSITORY https://gitlab.freedesktop.org/freetype/freetype.git
  GIT_SHALLOW TRUE
  GIT_TAG VER-2-13-3
  EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(freetype)
set(CMAKE_MESSAGE_LOG_LEVEL "NOTICE")

add_subdirectory(third_party/nanosvg EXCLUDE_FROM_ALL)

if (MSVC)
  set(DISABLE_WARNINGS /W0)
else ()
  set(DISABLE_WARNINGS -w)
endif ()

target_compile_options(freetype PRIVATE ${DISABLE_WARNINGS})
target_compile_options(nanosvg PRIVATE ${DISABLE_WARNINGS})
target_compile_options(nanosvgrast PRIVATE ${DISABLE_WARNINGS})
target_compile_options(bgfx PRIVATE ${DISABLE_WARNINGS})
target_compile_options(bx PRIVATE ${DISABLE_WARNINGS})
target_compile_options(bimg_decode PRIVATE ${DISABLE_WARNINGS})
target_compile_options(bimg_encode PRIVATE ${DISABLE_WARNINGS})
target_compile_options(bimg PRIVATE ${DISABLE_WARNINGS})

target_link_libraries(
  VisageGraphics
  PRIVATE
  bx
  bgfx
  bx
  bimg_decode
  bimg_encode
  freetype
  nanosvg
  nanosvgrast
  ${WIN_LIBS}
  ${METAL_LIBS}
  ${FONTCONFIG_LIBRARIES}
)

include(embedded.cmake)

target_include_directories(VisageGraphics
  PRIVATE
  ${VISAGE_INCLUDE_PATH}
  ${freetype_SOURCE_DIR}/include
  ${bgfx_SOURCE_DIR}/bgfx/include
  ${bgfx_SOURCE_DIR}/bgfx/3rdparty
  ${bgfx_SOURCE_DIR}/bx/include
  ${bgfx_SOURCE_DIR}/bimg/include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

if(WIN32)
  target_compile_definitions(VisageGraphics PRIVATE
    WINVER=0x0A00
    _WIN32_WINNT=0x0A00
    NTDDI_VERSION=0x0A000000
  )
endif()

target_include_directories(VisageGraphics
  SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bgfx/bgfx/3rdparty
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bgfx/bgfx/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bgfx/bgfx/tools
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bgfx/bx/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bgfx/bimg/include
)

set_target_properties(VisageGraphics PROPERTIES FOLDER "visage")
set_target_properties(nanosvg PROPERTIES FOLDER "visage/third_party/nanosvg")
set_target_properties(nanosvgrast PROPERTIES FOLDER "visage/third_party/nanosvg")
set_target_properties(bgfx PROPERTIES FOLDER "visage/third_party/bgfx")
set_target_properties(bx PROPERTIES FOLDER "visage/third_party/bgfx")
set_target_properties(bimg PROPERTIES FOLDER "visage/third_party/bgfx")
set_target_properties(bimg_decode PROPERTIES FOLDER "visage/third_party/bgfx")
set_target_properties(bimg_encode PROPERTIES FOLDER "visage/third_party/bgfx")

add_test_target(
  TARGET VisageGraphicsTests
  TEST_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests
)
set_target_properties(freetype PROPERTIES FOLDER "visage/third_party")
