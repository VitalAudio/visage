set(VISAGE_WAYLAND 0)

if (EMSCRIPTEN)
  file(GLOB PLATFORM_SOURCE_FILES emscripten/*.cpp)
  file(GLOB PLATFORM_HEADER_FILES emscripten/*.h)
elseif (WIN32)
  file(GLOB PLATFORM_SOURCE_FILES win32/*.cpp)
  file(GLOB PLATFORM_HEADER_FILES win32/*.h)
  set(WIN_LIBS dxgi Shell32)
elseif (APPLE)
  file(GLOB PLATFORM_SOURCE_FILES macos/*.mm)
  file(GLOB PLATFORM_HEADER_FILES macos/*.h)
elseif (UNIX)
  file(GLOB PLATFORM_SOURCE_FILES linux/*.cpp)
  file(GLOB PLATFORM_HEADER_FILES linux/*.h)

  find_package(X11 REQUIRED)
  set(X11_LIBS ${X11_LIBRARIES} ${X11_Xrandr_LIB})
  set(X11_INCLUDES ${X11_INCLUDE_DIR} ${Xrandr_INCLUDE_DIR})

  find_package(PkgConfig REQUIRED)
  pkg_check_modules(WAYLAND wayland-client)
  if (WAYLAND_FOUND)
    set(VISAGE_WAYLAND 1)
    pkg_check_modules(WaylandProtocols REQUIRED wayland-protocols)
    pkg_get_variable(WAYLAND_PROTOCOLS_DATADIR wayland-protocols pkgdatadir)
    set(XDG_SHELL_XML ${WAYLAND_PROTOCOLS_DATADIR}/stable/xdg-shell/xdg-shell.xml)
    set(XDG_SHELL_HEADER ${CMAKE_CURRENT_BINARY_DIR}/xdg-shell-client-protocol.h)
    set(XDG_SHELL_SOURCE ${CMAKE_CURRENT_BINARY_DIR}/xdg-shell-protocol.c)

    add_custom_command(
      OUTPUT ${XDG_SHELL_HEADER} ${XDG_SHELL_SOURCE}
      COMMAND wayland-scanner client-header ${XDG_SHELL_XML} ${XDG_SHELL_HEADER}
      COMMAND wayland-scanner private-code ${XDG_SHELL_XML} ${XDG_SHELL_SOURCE}
      DEPENDS ${XDG_SHELL_XML}
      COMMENT "Generating xdg-shell protocol files"
    )
  endif ()
endif ()

file(GLOB HEADERS windowing.h ${PLATFORM_HEADER_FILES})
file(GLOB SOURCE_FILES windowing.cpp ${PLATFORM_SOURCE_FILES})

add_library(VisageWindowing OBJECT ${SOURCE_FILES} ${HEADERS} ${XDG_SHELL_HEADER} ${XDG_SHELL_SOURCE})
target_compile_definitions(VisageWindowing PUBLIC VISAGE_WAYLAND=${VISAGE_WAYLAND})
target_include_directories(VisageWindowing
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${VISAGE_INCLUDE_PATH}
  ${X11_INCLUDES}
  ${CMAKE_CURRENT_BINARY_DIR}
)
set_target_properties(VisageWindowing PROPERTIES FOLDER "visage")
if (APPLE)
  set_target_properties(VisageWindowing PROPERTIES COMPILE_FLAGS "-fobjc-arc")
endif ()

set(VisageWindowingDependencies ${X11_LIBS} ${WAYLAND_LIBRARIES} ${WIN_LIBS} PARENT_SCOPE)
