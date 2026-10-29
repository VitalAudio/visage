if (VISAGE_ENABLE_WIDGETS)
  file(GLOB HEADERS *.h)
  file(GLOB SOURCE_FILES *.cpp)
  if (VISAGE_AMALGAMATED_BUILD)
    amalgamate_sources(AMALGAMATED ${CMAKE_CURRENT_BINARY_DIR}/visage_widgets_amalgamated "${SOURCE_FILES}")
    list(APPEND SOURCE_FILES ${AMALGAMATED})
  endif ()

  amalgamate_headers(${VISAGE_INCLUDE}/visage/widgets.h "${HEADERS}")
  list(APPEND HEADERS ${VISAGE_INCLUDE}/visage/widgets.h)

  add_library(VisageWidgets OBJECT ${SOURCE_FILES} ${HEADERS})

  target_include_directories(VisageWidgets PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${VISAGE_INCLUDE_PATH})
  target_link_libraries(VisageWidgets PRIVATE VisageGraphicsEmbeds)
  if (VISAGE_LIVE_SHADERS AND NOT EMSCRIPTEN)
    if (NOT TARGET VisageShaderCompiler)
      add_subdirectory(${VISAGE_SHADER_COMPILER_DIR} ${CMAKE_BINARY_DIR}/visage_shader_compiler EXCLUDE_FROM_ALL)
    endif ()
    target_link_libraries(VisageWidgets PRIVATE VisageShaderCompiler)
    target_compile_definitions(VisageWidgets PRIVATE VISAGE_LIVE_SHADERS=1)
  endif ()
  set_target_properties(VisageWidgets PROPERTIES FOLDER "visage")

  visage_add_test_target(
    TARGET VisageWidgetsTests
    TEST_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tests
  )
  if (TARGET VisageWidgetsTests AND VISAGE_LIVE_SHADERS AND NOT EMSCRIPTEN)
    target_compile_definitions(VisageWidgetsTests PRIVATE VISAGE_LIVE_SHADERS=1)
  endif ()
endif ()
