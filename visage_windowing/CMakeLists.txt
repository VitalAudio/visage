if (EMSCRIPTEN)
  file(GLOB PLATFORM_SOURCE_FILES emscripten/*.cpp)
  file(GLOB PLATFORM_HEADER_FILES emscripten/*.h)
elseif (WIN32)
  file(GLOB PLATFORM_SOURCE_FILES win32/*.cpp)
  file(GLOB PLATFORM_HEADER_FILES win32/*.h)
  set(WIN_LIBS dxgi Shell32 Comctl32)
  file(GLOB CURSOR_FILES cursors/*.cur)
elseif (APPLE)
  file(GLOB PLATFORM_SOURCE_FILES macos/*.mm)
  file(GLOB PLATFORM_HEADER_FILES macos/*.h)
elseif (UNIX)
  file(GLOB PLATFORM_SOURCE_FILES linux/*.cpp)
  file(GLOB PLATFORM_HEADER_FILES linux/*.h)

  find_package(X11 REQUIRED COMPONENTS xcb)
  find_library(VISAGE_XCB_PRESENT_LIB NAMES xcb-present)
  find_path(VISAGE_XCB_PRESENT_INCLUDE_DIR xcb/present.h)
  if (NOT VISAGE_XCB_PRESENT_LIB OR NOT VISAGE_XCB_PRESENT_INCLUDE_DIR)
    message(FATAL_ERROR "VISAGE: xcb-present is required on Linux (libxcb-present-dev on Debian and Ubuntu)")
  endif ()
  set(X11_LIBS ${X11_LIBRARIES} ${X11_Xrandr_LIB} ${CMAKE_DL_LIBS} ${X11_xcb_LIB} ${VISAGE_XCB_PRESENT_LIB})
  set(X11_INCLUDES ${X11_INCLUDE_DIR} ${Xrandr_INCLUDE_DIR} ${X11_xcb_INCLUDE_PATH}
                   ${VISAGE_XCB_PRESENT_INCLUDE_DIR})
  file(GLOB CURSOR_FILES cursors/*.cur)

  option(VISAGE_WAYLAND "Build the Wayland windowing backend on Linux" ON)
  if (VISAGE_WAYLAND)
    find_package(PkgConfig)
    if (PKG_CONFIG_FOUND)
      pkg_check_modules(VISAGE_WAYLAND_DEPS IMPORTED_TARGET wayland-client>=1.22 wayland-cursor xkbcommon)
      pkg_check_modules(VISAGE_WAYLAND_PROTOCOLS wayland-protocols>=1.35)
      pkg_get_variable(VISAGE_WAYLAND_PROTOCOLS_DIR wayland-protocols pkgdatadir)
      pkg_get_variable(VISAGE_WAYLAND_SCANNER wayland-scanner wayland_scanner)
    endif ()
    if (NOT VISAGE_WAYLAND_DEPS_FOUND OR NOT VISAGE_WAYLAND_PROTOCOLS_FOUND OR NOT VISAGE_WAYLAND_PROTOCOLS_DIR OR
        NOT VISAGE_WAYLAND_SCANNER)
      message(WARNING "VISAGE: building without Wayland support (needs libwayland-dev, libxkbcommon-dev and wayland-protocols on Debian and Ubuntu)")
      set(VISAGE_WAYLAND OFF)
    endif ()
  endif ()

  if (VISAGE_WAYLAND)
    set(VISAGE_WAYLAND_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/wayland)
    file(MAKE_DIRECTORY ${VISAGE_WAYLAND_GENERATED_DIR})
    set(VISAGE_WAYLAND_PROTOCOL_FILES
      stable/xdg-shell/xdg-shell.xml
      stable/viewporter/viewporter.xml
      stable/tablet/tablet-v2.xml
      staging/fractional-scale/fractional-scale-v1.xml
      staging/cursor-shape/cursor-shape-v1.xml
      unstable/xdg-decoration/xdg-decoration-unstable-v1.xml
      unstable/pointer-constraints/pointer-constraints-unstable-v1.xml
      unstable/relative-pointer/relative-pointer-unstable-v1.xml
    )
    if (EXISTS ${VISAGE_WAYLAND_PROTOCOLS_DIR}/staging/pointer-warp/pointer-warp-v1.xml)
      list(APPEND VISAGE_WAYLAND_PROTOCOL_FILES staging/pointer-warp/pointer-warp-v1.xml)
      set(VISAGE_WAYLAND_POINTER_WARP ON)
    endif ()
    foreach (PROTOCOL_FILE ${VISAGE_WAYLAND_PROTOCOL_FILES})
      get_filename_component(PROTOCOL_NAME ${PROTOCOL_FILE} NAME_WE)
      set(PROTOCOL_XML ${VISAGE_WAYLAND_PROTOCOLS_DIR}/${PROTOCOL_FILE})
      set(PROTOCOL_HEADER ${VISAGE_WAYLAND_GENERATED_DIR}/${PROTOCOL_NAME}-client-protocol.h)
      set(PROTOCOL_SOURCE ${VISAGE_WAYLAND_GENERATED_DIR}/${PROTOCOL_NAME}-protocol.c)
      add_custom_command(OUTPUT ${PROTOCOL_HEADER}
        COMMAND ${VISAGE_WAYLAND_SCANNER} client-header ${PROTOCOL_XML} ${PROTOCOL_HEADER}
        DEPENDS ${PROTOCOL_XML})
      add_custom_command(OUTPUT ${PROTOCOL_SOURCE}
        COMMAND ${VISAGE_WAYLAND_SCANNER} private-code ${PROTOCOL_XML} ${PROTOCOL_SOURCE}
        DEPENDS ${PROTOCOL_XML})
      list(APPEND VISAGE_WAYLAND_PROTOCOL_SOURCES ${PROTOCOL_HEADER} ${PROTOCOL_SOURCE})
    endforeach ()
    list(APPEND PLATFORM_SOURCE_FILES ${VISAGE_WAYLAND_PROTOCOL_SOURCES})
    list(APPEND X11_LIBS PkgConfig::VISAGE_WAYLAND_DEPS)
    list(APPEND X11_INCLUDES ${VISAGE_WAYLAND_GENERATED_DIR})
  endif ()
endif ()

file(GLOB HEADERS *.h)
file(GLOB SOURCE_FILES *.cpp)
file(GLOB CURSOR_HEADERS cursors/*.h)

amalgamate_headers(${VISAGE_INCLUDE}/visage/windowing.h "${HEADERS}")
list(APPEND HEADERS ${VISAGE_INCLUDE}/visage/windowing.h)

add_library(VisageWindowing OBJECT ${SOURCE_FILES} ${PLATFORM_SOURCE_FILES} ${HEADERS} ${PLATFORM_HEADER_FILES}
            ${CURSOR_HEADERS})
target_include_directories(VisageWindowing
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${VISAGE_INCLUDE_PATH}
  ${X11_INCLUDES}
)
target_link_libraries(VisageWindowing PRIVATE ${X11_LIBS} ${WIN_LIBS})

if (CURSOR_FILES)
  add_embedded_resources(VisageEmbeddedCursors "cursors.h" "visage::cursors" "${CURSOR_FILES}")
  target_link_libraries(VisageWindowing PRIVATE VisageEmbeddedCursors)
  set_target_properties(VisageEmbeddedCursors PROPERTIES FOLDER "visage")
endif ()

if (VISAGE_WAYLAND)
  target_compile_definitions(VisageWindowing PRIVATE VISAGE_WAYLAND=1)
  if (VISAGE_WAYLAND_POINTER_WARP)
    target_compile_definitions(VisageWindowing PRIVATE VISAGE_WAYLAND_POINTER_WARP=1)
  endif ()
endif ()

set_target_properties(VisageWindowing PROPERTIES FOLDER "visage")
if (APPLE)
  target_compile_options(VisageWindowing PRIVATE -fobjc-arc)
endif ()

