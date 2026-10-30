file(GLOB HEADERS *.h)
file(GLOB SOURCE_FILES *.cpp)

set(GPU_FILES gpu/gpu.cpp gpu/gpu_shader.cpp)
if (WIN32)
  list(APPEND SOURCE_FILES win32/emoji_win32.cpp)
  list(APPEND GPU_FILES gpu/gpu_d3d11.cpp)
  set(WIN_LIBS d2d1 dwrite Windowscodecs d3d11 dxgi d3dcompiler)
elseif (APPLE)
  list(APPEND SOURCE_FILES macos/emoji_macos.cpp)
  list(APPEND PLATFORM_FILES gpu/gpu_metal.mm)
elseif (EMSCRIPTEN)
  list(APPEND SOURCE_FILES linux/emoji_linux.cpp)
  list(APPEND GPU_FILES gpu/gpu_gles.cpp)
elseif (UNIX)
  list(APPEND SOURCE_FILES linux/emoji_linux.cpp)
  list(APPEND GPU_FILES gpu/gpu_vulkan.cpp)
endif ()
list(APPEND PLATFORM_FILES ${GPU_FILES})
file(GLOB GPU_HEADERS gpu/*.h)

if (VISAGE_AMALGAMATED_BUILD)
  amalgamate_sources(AMALGAMATED ${CMAKE_CURRENT_BINARY_DIR}/visage_graphics_amalgamated "${SOURCE_FILES}")
  list(APPEND SOURCE_FILES ${AMALGAMATED})
endif ()

amalgamate_headers(${VISAGE_INCLUDE}/visage/graphics.h "${HEADERS}")
list(APPEND HEADERS ${VISAGE_INCLUDE}/visage/graphics.h)

add_library(VisageGraphics OBJECT ${SOURCE_FILES} ${PLATFORM_FILES} ${HEADERS} ${GPU_HEADERS})

if (APPLE)
  find_library(METAL_LIBRARY Metal)
  find_library(METALKIT_LIBRARY MetalKit)
  find_library(QUARTZCORE_LIBRARY QuartzCore)
  find_library(APPKIT_LIBRARY AppKit)
  set(METAL_LIBS ${METAL_LIBRARY} ${METALKIT_LIBRARY} ${QUARTZCORE_LIBRARY} ${APPKIT_LIBRARY})
elseif (UNIX AND NOT EMSCRIPTEN)
  add_definitions(-D_LARGEFILE64_SOURCE)
  find_path(VISAGE_VULKAN_INCLUDE_DIR vulkan/vulkan.h)
  if (NOT VISAGE_VULKAN_INCLUDE_DIR)
    FetchContent_Declare(vulkan_headers GIT_REPOSITORY https://github.com/KhronosGroup/Vulkan-Headers.git
      GIT_TAG v1.4.341
      GIT_SHALLOW TRUE
      EXCLUDE_FROM_ALL
    )
    FetchContent_MakeAvailable(vulkan_headers)
    set(VISAGE_VULKAN_INCLUDE_DIR ${vulkan_headers_SOURCE_DIR}/include)
  endif ()
  target_include_directories(VisageGraphics SYSTEM PRIVATE ${VISAGE_VULKAN_INCLUDE_DIR})
endif ()

set(FT_DISABLE_PNG ON CACHE BOOL "" FORCE)
set(FT_DISABLE_HARFBUZZ ON CACHE BOOL "" FORCE)
set(FT_DISABLE_BROTLI ON CACHE BOOL "" FORCE)
set(FT_DISABLE_ZLIB ON CACHE BOOL "" FORCE)
set(FT_DISABLE_BZIP2 ON CACHE BOOL "" FORCE)

if (VISAGE_ENABLE_GRAPHICS_DEBUG_LOGGING)
  target_compile_definitions(VisageGraphics PUBLIC VISAGE_GRAPHICS_DEBUG_LOGGING=1)
endif ()

message(STATUS "VISAGE: Downloading graphics dependencies")
set(CMAKE_MESSAGE_LOG_LEVEL "ERROR")
find_package(Git REQUIRED)
FetchContent_Declare(freetype GIT_REPOSITORY https://gitlab.freedesktop.org/freetype/freetype.git
  GIT_SHALLOW TRUE
  GIT_TAG VER-2-14-1
  EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(freetype)
set(CMAKE_MESSAGE_LOG_LEVEL "NOTICE")

if (MSVC)
  set(DISABLE_WARNINGS /W0)
else ()
  set(DISABLE_WARNINGS -w)
endif ()

target_compile_options(freetype PRIVATE ${DISABLE_WARNINGS})

target_link_libraries(
  VisageGraphics
  PRIVATE
  freetype
  ${WIN_LIBS}
  ${METAL_LIBS}
  ${FONTCONFIG_LIBRARIES}
)

include(embedded.cmake)

target_include_directories(VisageGraphics
  PRIVATE
  ${VISAGE_INCLUDE_PATH}
  ${freetype_SOURCE_DIR}/include
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
)

set_target_properties(VisageGraphics PROPERTIES FOLDER "visage")

visage_add_test_target(
  TARGET VisageGraphicsTests
  TEST_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests
)

if (VISAGE_BUILD_TESTS AND UNIX AND NOT APPLE AND NOT EMSCRIPTEN)
  add_library(VisageGraphicsTestModule MODULE test_module/canvas_test_module.cpp)
  target_link_libraries(VisageGraphicsTestModule PRIVATE visage)
  set_target_properties(VisageGraphicsTestModule PROPERTIES FOLDER "visage/tests")
  add_dependencies(VisageGraphicsTests VisageGraphicsTestModule)
  target_compile_definitions(VisageGraphicsTests PRIVATE
                             VISAGE_GRAPHICS_TEST_MODULE="$<TARGET_FILE:VisageGraphicsTestModule>")
endif ()
set_target_properties(freetype PROPERTIES FOLDER "visage/third_party")

if (APPLE)
  target_compile_options(VisageGraphics PRIVATE -fobjc-arc)
endif ()
