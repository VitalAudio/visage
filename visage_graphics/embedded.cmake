
function(visage_embed_shaders project include_filename namespace original_shaders)
  file(GLOB_RECURSE SHADER_INCLUDES shaders/*.sh shaders/varying.def.sc)

  set(SHADER_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/shaders)
  set(DX_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/shaders/dx)
  set(METAL_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/shaders/metal)
  set(GLSL_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/shaders/glsl)
  set(SPIRV_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/shaders/spirv)
  set(ESSL_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/shaders/essl)

  if (CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
    set(SHADERC "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bin/win32/shaderc.exe")
  elseif (CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
    set(SHADERC "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bin/macos/shaderc")
  elseif (CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
    set(SHADERC "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bin/linux/shaderc")
  endif()

  file(MAKE_DIRECTORY ${SHADER_FOLDER})
  file(MAKE_DIRECTORY ${DX_FOLDER})
  file(MAKE_DIRECTORY ${METAL_FOLDER})
  file(MAKE_DIRECTORY ${GLSL_FOLDER})
  file(MAKE_DIRECTORY ${SPIRV_FOLDER})
  file(MAKE_DIRECTORY ${ESSL_FOLDER})

  if (EMSCRIPTEN)
    set(OPENGLES TRUE)
    set(SHADER_PLATFORM "asm.js")
  elseif (WIN32)
    set(SHADER_PLATFORM "windows")
    set(DIRECTX TRUE)
  elseif (APPLE)
    set(SHADER_PLATFORM "osx")
    set(METAL TRUE)
  elseif (UNIX)
    set(LINUX TRUE)
    set(VULKAN TRUE)
    set(SHADER_PLATFORM "linux")
  endif()

  foreach(SHADER ${original_shaders})
    get_filename_component(SHADER_NAME ${SHADER} NAME_WE)

    set(SHADER_TYPE "v")
    if (${SHADER_NAME} MATCHES fs_*)
      set(SHADER_TYPE "f")
    endif()

    if (OPENGLES)
      set(ESSL_PATH ${ESSL_FOLDER}/${SHADER_NAME})
      list(APPEND SHADER_BINS ${ESSL_PATH})
      add_custom_command(
        OUTPUT ${ESSL_PATH}
        COMMAND ${SHADERC} -i ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/shaders -O3 -f ${SHADER} -o ${ESSL_PATH} --type ${SHADER_TYPE} --platform ${SHADER_PLATFORM} -p 100_es
        DEPENDS ${SHADER}
      )
    endif()

    if (DIRECTX)
      set(DX_PATH ${DX_FOLDER}/${SHADER_NAME})
      list(APPEND SHADER_BINS ${DX_PATH})
      add_custom_command(
        OUTPUT ${DX_PATH}
        COMMAND ${SHADERC} -i ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/shaders -O3 -f ${SHADER} -o ${DX_PATH} --type ${SHADER_TYPE} --platform ${SHADER_PLATFORM} -p s_4_0
        DEPENDS ${SHADER}
      )
    endif()

    if (METAL)
      set(METAL_PATH ${METAL_FOLDER}/${SHADER_NAME})
      list(APPEND SHADER_BINS ${METAL_PATH})
      add_custom_command(
        OUTPUT ${METAL_PATH}
        COMMAND ${SHADERC} -i ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/shaders -f ${SHADER} -o ${METAL_PATH} --type ${SHADER_TYPE} --platform ${SHADER_PLATFORM} -p metal
        DEPENDS ${SHADER}
      )
    endif()

    if (OPENGL)
      set(GLSL_PATH ${GLSL_FOLDER}/${SHADER_NAME})
      list(APPEND SHADER_BINS ${GLSL_PATH})
      add_custom_command(
        OUTPUT ${GLSL_PATH}
        COMMAND ${SHADERC} -i ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/shaders -f ${SHADER} -o ${GLSL_PATH} --type ${SHADER_TYPE} --platform ${SHADER_PLATFORM} -p 120
        DEPENDS ${SHADER}
      )
    endif()

    if (VULKAN)
      set(SPIRV_PATH ${SPIRV_FOLDER}/${SHADER_NAME})
      list(APPEND SHADER_BINS ${SPIRV_PATH})
      add_custom_command(
        OUTPUT ${SPIRV_PATH}
        COMMAND ${SHADERC} -i ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/shaders -f ${SHADER} -o ${SPIRV_PATH} --type ${SHADER_TYPE} --platform ${SHADER_PLATFORM} -p spirv
        DEPENDS ${SHADER}
      )
    endif()
  endforeach()

  set(SHADER_RESOURCES
    ${SHADER_BINS}
    ${SHADER_INCLUDES}
    ${original_shaders}
  )

  add_embedded_resources(${project} ${include_filename} ${namespace} "${SHADER_RESOURCES}")
  target_sources(${project} PRIVATE ${original_shaders})
  source_group("Shaders" FILES ${original_shaders})
endfunction()

file(GLOB LIBRARY_SHADERS shaders/[vf]s_*.sc)
file(GLOB FONT_TTF_FILES fonts/*.ttf)
file(GLOB ICON_FILES icons/*.svg)

if (UNIX AND NOT APPLE)
  file(GLOB LINUX_TTF_FILES fonts/linux/*.ttf)
  list(APPEND FONT_TTF_FILES ${LINUX_TTF_FILES})
endif ()

visage_embed_shaders(VisageEmbeddedShaders "shaders.h" "visage::shaders" "${LIBRARY_SHADERS}")
add_embedded_resources(VisageEmbeddedFonts "fonts.h" "visage::fonts" "${FONT_TTF_FILES}")
add_embedded_resources(VisageEmbeddedIcons "icons.h" "visage::icons" "${ICON_FILES}")

add_library(VisageGraphicsEmbeds INTERFACE)

target_link_libraries(VisageGraphicsEmbeds
  INTERFACE
  VisageEmbeddedShaders
  VisageEmbeddedFonts
  VisageEmbeddedIcons
)
target_link_libraries(VisageGraphics PRIVATE VisageGraphicsEmbeds)

set_target_properties(VisageEmbeddedShaders PROPERTIES FOLDER "visage")
set_target_properties(VisageEmbeddedFonts PROPERTIES FOLDER "visage")
set_target_properties(VisageEmbeddedIcons PROPERTIES FOLDER "visage")
