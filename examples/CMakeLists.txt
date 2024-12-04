file(GLOB EXAMPLES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */)

file(GLOB_RECURSE EXAMPLE_SHADERS shaders/[vf]s_*.sc)

visage_embed_shaders(EmbeddedShaderResources "example_shaders.h" "resources::shaders" "${EXAMPLE_SHADERS}")
file(GLOB_RECURSE FONT_TTF_FILES fonts/*.ttf)
file(GLOB_RECURSE ICON_FILES icons/*.svg)

add_embedded_resources(EmbeddedFontResources "example_fonts.h" "resources::fonts" "${FONT_TTF_FILES}")
add_embedded_resources(EmbeddedIconResources "example_icons.h" "resources::icons" "${ICON_FILES}")

if (WIN32)
  file(COPY "${CMAKE_SOURCE_DIR}/visage_graphics/bin/win32/shaderc.exe" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
elseif (APPLE)
  file(COPY "${CMAKE_SOURCE_DIR}/visage_graphics/bin/macos/shaderc" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
else (UNIX)
  file(COPY "${CMAKE_SOURCE_DIR}/visage_graphics/bin/linux/shaderc" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif ()

foreach (EXAMPLE ${EXAMPLES})
  file(GLOB SOURCE_FILES ${EXAMPLE}/*.cpp)
  file(GLOB HEADER_FILES ${EXAMPLE}/*.h)

  if (IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${EXAMPLE}" AND SOURCE_FILES)
    add_executable(Example${EXAMPLE} main.cpp ${SOURCE_FILES} ${HEADER_FILES})
    if (WIN32)
      set_target_properties(Example${EXAMPLE} PROPERTIES WIN32_EXECUTABLE YES)
    elseif (APPLE)
      set(EXAMPLE_NAME Example${EXAMPLE})
      configure_file(Info.plist.in ${CMAKE_CURRENT_BINARY_DIR}/${EXAMPLE}/ExampleInfo.plist)
      set_target_properties(Example${EXAMPLE} PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_BINARY_DIR}/${EXAMPLE}/ExampleInfo.plist
      )
    elseif (EMSCRIPTEN)
      target_link_options(Example${EXAMPLE}
        PRIVATE
        --shell-file ${CMAKE_CURRENT_SOURCE_DIR}/emscripten_template.html
        -sGL_ENABLE_GET_PROC_ADDRESS
        -sALLOW_MEMORY_GROWTH
        --bind
        "-sEXPORTED_FUNCTIONS=['_main']"
        "-sEXPORTED_RUNTIME_METHODS=['ccall', 'cwrap']"
      )

      set_target_properties(Example${EXAMPLE} PROPERTIES
        SUFFIX ".html"
        OUTPUT_NAME "index"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/builds/${EXAMPLE}"
      )
    endif ()

    target_link_libraries(Example${EXAMPLE} PRIVATE
      visage
      EmbeddedShaderResources
      EmbeddedFontResources
      EmbeddedIconResources
    )
  endif ()
endforeach ()

