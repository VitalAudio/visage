file(GLOB EXAMPLES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} */)

file(GLOB_RECURSE EXAMPLE_SHADERS shaders/[vf]s_*.sc)
add_compile_definitions(SHADERS_FOLDER=\"${CMAKE_CURRENT_SOURCE_DIR}/shaders\")

visage_embed_shaders(EmbeddedShaderResources "example_shaders.h" "resources::shaders" "${EXAMPLE_SHADERS}")
file(GLOB_RECURSE FONT_TTF_FILES fonts/*.ttf)
file(GLOB_RECURSE ICON_FILES icons/*.svg)
file(GLOB_RECURSE IMAGE_FILES images/*.png)

add_embedded_resources(EmbeddedFontResources "example_fonts.h" "resources::fonts" "${FONT_TTF_FILES}")
add_embedded_resources(EmbeddedIconResources "example_icons.h" "resources::icons" "${ICON_FILES}")
add_embedded_resources(EmbeddedImageResources "example_images.h" "resources::images" "${IMAGE_FILES}")

if (WIN32)
  file(COPY "${CMAKE_SOURCE_DIR}/visage_graphics/bin/win32/shaderc.exe" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
elseif (APPLE)
  file(COPY "${CMAKE_SOURCE_DIR}/visage_graphics/bin/macos/shaderc" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
else (UNIX)
  file(COPY "${CMAKE_SOURCE_DIR}/visage_graphics/bin/linux/shaderc" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif ()

if (NOT EMSCRIPTEN)
  message(STATUS "Downloading CLAP SDK for plugin example")
  FetchContent_Declare(clap GIT_REPOSITORY https://github.com/free-audio/clap.git
    GIT_TAG main
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL
  )

  FetchContent_Declare(clap_helpers GIT_REPOSITORY https://github.com/free-audio/clap-helpers.git
    GIT_TAG main
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL
  )

  FetchContent_MakeAvailable(clap)
  FetchContent_MakeAvailable(clap_helpers)
  set(CLAP_LIBRARIES clap clap-helpers)
endif ()

foreach (EXAMPLE ${EXAMPLES})
  file(GLOB SOURCE_FILES ${EXAMPLE}/*.cpp)
  file(GLOB HEADER_FILES ${EXAMPLE}/*.h)

  set(SHOULD_BUILD ON)
  if (EMSCRIPTEN AND EXAMPLE STREQUAL "ClapPlugin")
    set(SHOULD_BUILD OFF)
  endif ()

  if (IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${EXAMPLE}" AND SOURCE_FILES AND SHOULD_BUILD)
    if (EXAMPLE STREQUAL "ClapPlugin")
      add_library(Example${EXAMPLE} MODULE ${SOURCE_FILES} ${HEADER_FILES})
      set_target_properties(Example${EXAMPLE} PROPERTIES SUFFIX ".clap" PREFIX "")
      set_target_properties(Example${EXAMPLE} PROPERTIES POSITION_INDEPENDENT_CODE ON)
    else ()
      add_executable(Example${EXAMPLE} main.cpp ${SOURCE_FILES} ${HEADER_FILES})
    endif ()

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
        "-sEXPORTED_FUNCTIONS=['_main', '_pasteCallback']"
        "-sEXPORTED_RUNTIME_METHODS=['ccall', 'cwrap', 'UTF8ToString']"
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
      EmbeddedImageResources
      ${CLAP_LIBRARIES}
    )
  endif ()
endforeach ()

