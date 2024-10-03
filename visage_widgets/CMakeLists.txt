if (VISAGE_ENABLE_WIDGETS)
  file(GLOB HEADERS *.h)
  file(GLOB SOURCE_FILES *.cpp)
  add_library(VisageWidgets OBJECT ${SOURCE_FILES} ${HEADERS})

  target_include_directories(VisageWidgets PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${VISAGE_INCLUDE_PATH})
  target_link_libraries(VisageWidgets PRIVATE VisageGraphicsEmbeds)
  set_target_properties(VisageWidgets PROPERTIES FOLDER "visage")
endif ()
