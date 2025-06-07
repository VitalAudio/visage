if (APPLE)
  list(APPEND PLATFORM_FILES macos/menu_bar.mm)
endif ()

file(GLOB HEADERS *.h)
file(GLOB SOURCE_FILES *.cpp)
if (VISAGE_AMALGAMATED_BUILD)
  amalgamate_sources(${CMAKE_CURRENT_BINARY_DIR}/visage_ui_amalgamated.cpp "${SOURCE_FILES}")
  list(APPEND SOURCE_FILES ${CMAKE_CURRENT_BINARY_DIR}/visage_ui_amalgamated.cpp)
endif ()

amalgamate_headers(${VISAGE_INCLUDE}/visage/ui.h "${HEADERS}")
list(APPEND HEADERS ${VISAGE_INCLUDE}/visage/ui.h)

add_library(VisageUi OBJECT ${SOURCE_FILES} ${HEADERS} ${PLATFORM_FILES})
target_include_directories(VisageUi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${VISAGE_INCLUDE_PATH})
target_link_libraries(VisageUi PRIVATE VisageGraphicsEmbeds)
set_target_properties(VisageUi PROPERTIES FOLDER "visage")

add_test_target(
  TARGET VisageUiTests
  TEST_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests
)
