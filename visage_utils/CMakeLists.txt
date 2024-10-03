if (WIN32)
  file(GLOB PLATFORM_HEADER_FILES win32/*.h)
  file(GLOB PLATFORM_SOURCE_FILES win32/*.cpp)
elseif (APPLE)
  file(GLOB PLATFORM_HEADER_FILES macos/*.h)
  file(GLOB PLATFORM_SOURCE_FILES macos/*.cpp)
elseif (UNIX)
  file(GLOB PLATFORM_HEADER_FILES linux/*.h)
  file(GLOB PLATFORM_SOURCE_FILES linux/*.cpp)
endif ()

file(GLOB HEADERS *.h ${PLATFORM_HEADER_FILES})
file(GLOB SOURCE_FILES *.cpp ${PLATFORM_SOURCE_FILES})
add_library(VisageUtils OBJECT ${SOURCE_FILES} ${HEADERS})
target_include_directories(VisageUtils PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
add_library(VisageUtilsDefinitions INTERFACE)
set_target_properties(VisageUtils PROPERTIES FOLDER "visage")

add_test_target(
  TARGET VisageUtilsTests
  TEST_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests
  DEPENDENCIES VisageUtils
)
