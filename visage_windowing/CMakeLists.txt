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
endif ()

file(GLOB HEADERS *.h)
file(GLOB SOURCE_FILES *.cpp)

amalgamate_headers(${VISAGE_INCLUDE}/visage/windowing.h "${HEADERS}")
list(APPEND HEADERS ${VISAGE_INCLUDE}/visage/windowing.h)

add_library(VisageWindowing OBJECT ${SOURCE_FILES} ${PLATFORM_SOURCE_FILES} ${HEADERS} ${PLATFORM_HEADER_FILES})
target_include_directories(VisageWindowing
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${VISAGE_INCLUDE_PATH}
  ${X11_INCLUDES}
)
target_link_libraries(VisageWindowing PRIVATE ${X11_LIBS} ${WIN_LIBS})

set_target_properties(VisageWindowing PROPERTIES FOLDER "visage")
if (APPLE)
  target_compile_options(VisageWindowing PRIVATE -fobjc-arc)
endif ()

