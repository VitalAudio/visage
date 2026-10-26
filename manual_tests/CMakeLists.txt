set(EXAMPLES_DIRECTORY ${PROJECT_SOURCE_DIR}/examples)

file(GLOB MANUAL_TEST_SHADERS ${EXAMPLES_DIRECTORY}/shaders/[vf]s_*.sc)
file(GLOB MANUAL_TEST_FONTS ${EXAMPLES_DIRECTORY}/fonts/*.ttf)
file(GLOB MANUAL_TEST_ICONS ${EXAMPLES_DIRECTORY}/icons/*.svg)
file(GLOB MANUAL_TEST_IMAGES ${EXAMPLES_DIRECTORY}/images/*.png)

visage_embed_shaders(ManualTestShaders "example_shaders.h" "resources::shaders" "${MANUAL_TEST_SHADERS}")
add_embedded_resources(ManualTestFonts "example_fonts.h" "resources::fonts" "${MANUAL_TEST_FONTS}")
add_embedded_resources(ManualTestIcons "example_icons.h" "resources::icons" "${MANUAL_TEST_ICONS}")
add_embedded_resources(ManualTestImages "example_images.h" "resources::images" "${MANUAL_TEST_IMAGES}")

file(GLOB SOURCE_FILES *.cpp)
file(GLOB HEADER_FILES *.h)
add_executable(VisageManualTests ${SOURCE_FILES} ${HEADER_FILES})

if (MSVC)
  target_compile_options(VisageManualTests PRIVATE /utf-8)
endif ()

if (WIN32)
  set_target_properties(VisageManualTests PROPERTIES WIN32_EXECUTABLE YES)
elseif (EMSCRIPTEN)
  target_link_options(VisageManualTests PRIVATE
    --shell-file ${EXAMPLES_DIRECTORY}/emscripten_template.html
    -sGL_ENABLE_GET_PROC_ADDRESS
    -sALLOW_MEMORY_GROWTH
    -lidbfs.js
    --bind
    "-sEXPORTED_FUNCTIONS=['_main', '_pasteCallback']"
    "-sEXPORTED_RUNTIME_METHODS=['ccall', 'cwrap', 'UTF8ToString']"
  )
  set_target_properties(VisageManualTests PROPERTIES
    SUFFIX ".html"
    OUTPUT_NAME "index"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/web"
  )
elseif (APPLE)
  set(EXAMPLE_NAME VisageManualTests)
  configure_file(${EXAMPLES_DIRECTORY}/Info.plist.in ${CMAKE_CURRENT_BINARY_DIR}/Info.plist)
  set_target_properties(VisageManualTests PROPERTIES
    MACOSX_BUNDLE TRUE
    MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_BINARY_DIR}/Info.plist
  )
endif ()

target_link_libraries(VisageManualTests PRIVATE
  visage
  ManualTestShaders
  ManualTestFonts
  ManualTestIcons
  ManualTestImages
)

set_target_properties(VisageManualTests PROPERTIES FOLDER "visage/tests")
foreach (target ManualTestShaders ManualTestFonts ManualTestIcons ManualTestImages)
  set_target_properties(${target} PROPERTIES FOLDER "visage/tests")
endforeach ()
