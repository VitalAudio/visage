if (VISAGE_ENABLE_WIDGETS)
  file(GLOB HEADERS *.h)
  file(GLOB SOURCE_FILES *.cpp)
  if (VISAGE_AMALGAMATED_BUILD)
    amalgamate_sources(${CMAKE_CURRENT_BINARY_DIR}/visage_widgets_amalgamated.cpp "${SOURCE_FILES}")
    list(APPEND SOURCE_FILES ${CMAKE_CURRENT_BINARY_DIR}/visage_widgets_amalgamated.cpp)
  endif ()

  amalgamate_headers(${VISAGE_INCLUDE}/visage/widgets.h "${HEADERS}")
  list(APPEND HEADERS ${VISAGE_INCLUDE}/visage/widgets.h)

  add_library(VisageWidgets OBJECT ${SOURCE_FILES} ${HEADERS})

  target_include_directories(VisageWidgets PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${VISAGE_INCLUDE_PATH})
  target_link_libraries(VisageWidgets PRIVATE VisageGraphicsEmbeds)
  set_target_properties(VisageWidgets PROPERTIES FOLDER "visage")

  add_test_target(
    TARGET VisageWidgetsTests
    TEST_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests
  )
endif ()
