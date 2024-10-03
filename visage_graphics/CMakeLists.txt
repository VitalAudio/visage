file(GLOB HEADERS *.h)
file(GLOB SOURCE_FILES *.cpp)
add_library(VisageGraphics OBJECT ${SOURCE_FILES} ${HEADERS})

if (UNIX AND NOT APPLE)
  add_definitions(-D_LARGEFILE64_SOURCE)
endif ()

set(BGFX_BUILD_EXAMPLES OFF CACHE INTERNAL "" FORCE)
set(BGFX_BUILD_EXAMPLE_COMMON OFF CACHE INTERNAL "" FORCE)
set(BGFX_BUILD_TOOLS OFF CACHE INTERNAL "" FORCE)
set(BGFX_BUILD_TESTS OFF CACHE INTERNAL "" FORCE)
set(BGFX_INSTALL OFF CACHE INTERNAL "" FORCE)
set(BGFX_AMALGAMATED ON CACHE INTERNAL "" FORCE)
set(BGFX_CONFIG_MAX_FRAME_LATENCY 1 CACHE INTERNAL "" FORCE)
set(BGFX_CONFIG_MAX_FRAME_BUFFERS 1024 CACHE INTERNAL "" FORCE)
set(BGFX_CONFIG_MAX_VIEWS 256 CACHE INTERNAL "" FORCE)
set(BGFX_CONFIG_MAX_DRAW_CALLS 8192 CACHE INTERNAL "" FORCE)

set(BX_CONFIG_DEBUG 0 CACHE INTERNAL "" FORCE)
set(BGFX_CONFIG_DEBUG_ANNOTATION 0 CACHE INTERNAL "" FORCE)
set(BGFX_WITH_WAYLAND OFF CACHE INTERNAL "" FORCE)
if (VISAGE_ENABLE_GRAPHICS_DEBUG_LOGGING)
  set(BX_CONFIG_DEBUG 1 CACHE INTERNAL "" FORCE)
  set(BGFX_CONFIG_DEBUG_ANNOTATION 1 CACHE INTERNAL "" FORCE)
endif ()

set(BGFX_CONFIG_MULTITHREADED 0 CACHE INTERNAL "" FORCE)
if (NOT EMSCRIPTEN AND VISAGE_ENABLE_BACKGROUND_GRAPHICS_THREAD)
  set(BGFX_CONFIG_MULTITHREADED 1 CACHE INTERNAL "" FORCE)
  target_compile_definitions(VisageGraphics PUBLIC VISAGE_BACKGROUND_GRAPHICS_THREAD=1)
endif ()

message(STATUS "Downloading bgfx for low level graphics APIs")
include(FetchContent)
FetchContent_Declare(bgfx GIT_REPOSITORY https://github.com/bkaradzic/bgfx.cmake.git
  GIT_SHALLOW TRUE
  GIT_TAG v1.128.8808-482
  EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(bgfx)

add_subdirectory(third_party/nanosvg EXCLUDE_FROM_ALL)

if (MSVC)
  set(DISABLE_WARNINGS /W0)
else()
  set(DISABLE_WARNINGS -w)
endif()

target_compile_options(nanosvg PRIVATE ${DISABLE_WARNINGS})
target_compile_options(nanosvgrast PRIVATE ${DISABLE_WARNINGS})
target_compile_options(bgfx PRIVATE ${DISABLE_WARNINGS})
target_compile_options(bx PRIVATE ${DISABLE_WARNINGS})
target_compile_options(bimg_decode PRIVATE ${DISABLE_WARNINGS})
target_compile_options(bimg_encode PRIVATE ${DISABLE_WARNINGS})
target_compile_options(bimg PRIVATE ${DISABLE_WARNINGS})

target_link_libraries(VisageGraphics PRIVATE bx)

set(VisageGraphicsDependencies
  VisageGraphicsEmbeds
  bgfx
  bx
  nanosvg
  nanosvgrast
  PARENT_SCOPE
)

include(embedded.cmake)

target_include_directories(VisageGraphics
  PRIVATE
  ${VISAGE_INCLUDE_PATH}
  ${bgfx_SOURCE_DIR}/bgfx/include
  ${bgfx_SOURCE_DIR}/bgfx/3rdparty
  ${bgfx_SOURCE_DIR}/bx/include
  ${bgfx_SOURCE_DIR}/bimg/include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

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
