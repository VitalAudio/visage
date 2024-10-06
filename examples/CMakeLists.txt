file(GLOB EXAMPLES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */)

file(GLOB_RECURSE EXAMPLE_SHADERS shaders/[vf]s_*.sc)

visage_embed_shaders(ExampleEmbeddedShaders "example_shaders.h" "resources::shaders" "${EXAMPLE_SHADERS}")
file(GLOB_RECURSE FONT_TTF_FILES fonts/*.ttf)
file(GLOB_RECURSE ICON_FILES icons/*.svg)

add_embedded_resources(ExampleEmbeddedFonts "fonts.h" "resources::fonts" "${FONT_TTF_FILES}")
add_embedded_resources(ExampleEmbeddedIcons "icons.h" "resources::icons" "${ICON_FILES}")

if (WIN32)
  file(COPY "${CMAKE_SOURCE_DIR}/visage_graphics/bin/win32/shaderc.exe" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
elseif (APPLE)
  file(COPY "${CMAKE_SOURCE_DIR}/visage_graphics/bin/macos/shaderc" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
else(UNIX)
  file(COPY "${CMAKE_SOURCE_DIR}/visage_graphics/bin/linux/shaderc" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

foreach(EXAMPLE ${EXAMPLES})
  file(GLOB SOURCE_FILES ${EXAMPLE}/*.cpp)
  file(GLOB HEADER_FILES ${EXAMPLE}/*.h)

  if (SOURCE_FILES)
    add_executable(Example${EXAMPLE} main.cpp ${SOURCE_FILES} ${HEADERS_FILE})
    if (WIN32)
      set_target_properties(Example${EXAMPLE} PROPERTIES WIN32_EXECUTABLE YES)
    elseif(EMSCRIPTEN)
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
      )
    endif()

    target_link_libraries(Example${EXAMPLE} PRIVATE
      visage
      ExampleEmbeddedShaders
      ExampleEmbeddedFonts
      ExampleEmbeddedIcons
    )
  endif()
endforeach()

