if (EMSCRIPTEN)
  file(GLOB PLATFORM_SOURCE_FILES emscripten/*.cpp)
  file(GLOB PLATFORM_HEADER_FILES emscripten/*.h)
elseif (WIN32)
  file(GLOB PLATFORM_SOURCE_FILES win32/*.cpp)
  file(GLOB PLATFORM_HEADER_FILES win32/*.h)
  set(DXGI_LIB dxgi)
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

file(GLOB HEADERS windowing.h ${PLATFORM_HEADER_FILES})
file(GLOB SOURCE_FILES windowing.cpp ${PLATFORM_SOURCE_FILES})

add_library(VisageWindowing OBJECT ${SOURCE_FILES} ${HEADERS})
target_include_directories(VisageWindowing
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${VISAGE_INCLUDE_PATH}
  ${X11_INCLUDES}
)
set_target_properties(VisageWindowing PROPERTIES FOLDER "visage")
if (APPLE)
  set_target_properties(VisageWindowing PROPERTIES COMPILE_FLAGS "-fobjc-arc")
endif ()

set(VisageWindowingDependencies ${X11_LIBS} ${DXGI_LIB} PARENT_SCOPE)
