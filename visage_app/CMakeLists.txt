file(GLOB HEADERS *.h)
file(GLOB SOURCE_FILES *.cpp)
if (VISAGE_AMALGAMATED_BUILD)
  amalgamate_sources(${CMAKE_CURRENT_BINARY_DIR}/visage_app_amalgamated.cpp "${SOURCE_FILES}")
  list(APPEND SOURCE_FILES ${CMAKE_CURRENT_BINARY_DIR}/visage_app_amalgamated.cpp)
endif ()

amalgamate_headers(${VISAGE_INCLUDE}/visage/app.h "${HEADERS}")
list(APPEND HEADERS ${VISAGE_INCLUDE}/visage/app.h)

add_library(VisageApp OBJECT ${SOURCE_FILES} ${HEADERS})
target_include_directories(VisageApp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${VISAGE_INCLUDE_PATH})
set_target_properties(VisageApp PROPERTIES FOLDER "visage")

add_test_target(
  TARGET VisageIntegrationTests
  TEST_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests
)
