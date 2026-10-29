include(ExternalProject)

if (VISAGE_SHADERC)
  set(VISAGE_SHADERC_EXECUTABLE ${VISAGE_SHADERC} CACHE INTERNAL "")
  set(VISAGE_SHADERC_TARGET "" CACHE INTERNAL "")
elseif (NOT TARGET VisageShadercHost)
  set(VISAGE_SHADERC_BINARY_DIR ${CMAKE_BINARY_DIR}/visage_shaderc)
  if (CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
    set(VISAGE_SHADERC_EXECUTABLE ${VISAGE_SHADERC_BINARY_DIR}/bin/visage_shaderc.exe CACHE INTERNAL "")
  else ()
    set(VISAGE_SHADERC_EXECUTABLE ${VISAGE_SHADERC_BINARY_DIR}/bin/visage_shaderc CACHE INTERNAL "")
  endif ()

  set(VISAGE_SHADERC_ARGS -DCMAKE_BUILD_TYPE=Release)
  if (CMAKE_CROSSCOMPILING)
    list(APPEND VISAGE_SHADERC_ARGS -DCMAKE_TOOLCHAIN_FILE=)
  endif ()
  if (CMAKE_CXX_COMPILER_LAUNCHER)
    list(APPEND VISAGE_SHADERC_ARGS -DCMAKE_CXX_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER}
         -DCMAKE_C_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER})
  endif ()

  ExternalProject_Add(VisageShadercHost
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/shaderc
    BINARY_DIR ${VISAGE_SHADERC_BINARY_DIR}
    CMAKE_ARGS ${VISAGE_SHADERC_ARGS}
    BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release --target visage_shaderc
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS ${VISAGE_SHADERC_EXECUTABLE}
    BUILD_ALWAYS TRUE
  )
  set_target_properties(VisageShadercHost PROPERTIES FOLDER "visage/tools")
  set(VISAGE_SHADERC_TARGET VisageShadercHost CACHE INTERNAL "")
endif ()

function(visage_embed_shaders project include_filename namespace original_shaders)
  file(GLOB_RECURSE SHADER_INCLUDES shaders/*.sh shaders/varying.def.sc)
  file(GLOB LIBRARY_SHADER_INCLUDES ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/shaders/*.sh)
  set(SHADER_DEPENDENCIES ${SHADER_INCLUDES} ${LIBRARY_SHADER_INCLUDES} ${VISAGE_SHADERC_EXECUTABLE}
      ${VISAGE_SHADERC_TARGET})
  list(REMOVE_DUPLICATES SHADER_DEPENDENCIES)

  if (EMSCRIPTEN)
    set(SHADER_TARGET essl)
  elseif (WIN32)
    set(SHADER_TARGET dxbc)
  elseif (APPLE)
    set(SHADER_TARGET msl)
  else ()
    set(SHADER_TARGET spirv)
  endif ()

  set(SHADER_FOLDER ${CMAKE_CURRENT_BINARY_DIR}/shaders/${SHADER_TARGET})
  file(MAKE_DIRECTORY ${SHADER_FOLDER})

  foreach(SHADER ${original_shaders})
    get_filename_component(SHADER_NAME ${SHADER} NAME_WE)

    set(SHADER_TYPE "f")
    if (SHADER_NAME MATCHES "^v")
      set(SHADER_TYPE "v")
    elseif (SHADER_NAME MATCHES "^cs_")
      set(SHADER_TYPE "c")
    endif()

    if (SHADER_TYPE STREQUAL "c" AND EMSCRIPTEN)
      continue()
    endif ()

    set(SHADER_PATH ${SHADER_FOLDER}/${SHADER_NAME})
    list(APPEND SHADER_BINS ${SHADER_PATH})
    add_custom_command(
      OUTPUT ${SHADER_PATH}
      COMMAND ${VISAGE_SHADERC_EXECUTABLE} -i ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/shaders -f ${SHADER}
              -o ${SHADER_PATH} --type ${SHADER_TYPE} --target ${SHADER_TARGET}
      DEPENDS ${SHADER} ${SHADER_DEPENDENCIES}
    )
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

file(GLOB LIBRARY_SHADERS shaders/[vfc]s_*.sc)
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
