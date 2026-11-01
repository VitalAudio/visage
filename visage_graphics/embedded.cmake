include(ExternalProject)

set(VISAGE_SHADER_COMPILER_DIR ${CMAKE_CURRENT_LIST_DIR}/shader_compiler)

if (VISAGE_SHADER_COMPILER)
  set(VISAGE_SHADER_COMPILER_EXECUTABLE ${VISAGE_SHADER_COMPILER} CACHE INTERNAL "")
  set(VISAGE_SHADER_COMPILER_TARGET "" CACHE INTERNAL "")
elseif (NOT CMAKE_CROSSCOMPILING)
  if (NOT TARGET shader_compiler)
    add_subdirectory(${VISAGE_SHADER_COMPILER_DIR} ${CMAKE_BINARY_DIR}/shader_compiler)
  endif ()
  set(VISAGE_SHADER_COMPILER_EXECUTABLE "$<TARGET_FILE:shader_compiler>" CACHE INTERNAL "")
  set(VISAGE_SHADER_COMPILER_TARGET shader_compiler CACHE INTERNAL "")
elseif (NOT TARGET VisageShaderCompilerHost)
  set(VISAGE_SHADER_COMPILER_BINARY_DIR ${CMAKE_BINARY_DIR}/shader_compiler_host)
  if (CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
    set(VISAGE_SHADER_COMPILER_EXECUTABLE ${VISAGE_SHADER_COMPILER_BINARY_DIR}/bin/shader_compiler.exe CACHE INTERNAL "")
  else ()
    set(VISAGE_SHADER_COMPILER_EXECUTABLE ${VISAGE_SHADER_COMPILER_BINARY_DIR}/bin/shader_compiler CACHE INTERNAL "")
  endif ()

  set(VISAGE_SHADER_COMPILER_ARGS -DCMAKE_BUILD_TYPE=Release)
  list(APPEND VISAGE_SHADER_COMPILER_ARGS -DCMAKE_TOOLCHAIN_FILE=)
  if (CMAKE_CXX_COMPILER_LAUNCHER)
    list(APPEND VISAGE_SHADER_COMPILER_ARGS -DCMAKE_CXX_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER}
         -DCMAKE_C_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER})
  endif ()

  ExternalProject_Add(VisageShaderCompilerHost
    SOURCE_DIR ${VISAGE_SHADER_COMPILER_DIR}
    BINARY_DIR ${VISAGE_SHADER_COMPILER_BINARY_DIR}
    CMAKE_ARGS ${VISAGE_SHADER_COMPILER_ARGS}
    BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release --target shader_compiler
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS ${VISAGE_SHADER_COMPILER_EXECUTABLE}
    BUILD_ALWAYS TRUE
  )
  set_target_properties(VisageShaderCompilerHost PROPERTIES FOLDER "visage/tools")
  set(VISAGE_SHADER_COMPILER_TARGET VisageShaderCompilerHost CACHE INTERNAL "")
endif ()

set(VISAGE_METAL_SHADER_TARGET msl CACHE INTERNAL "")
set(VISAGE_METAL_SHADER_ARGS "" CACHE INTERNAL "")
if (APPLE AND NOT VISAGE_LIVE_SHADERS)
  set(METAL_VERSION_MIN 10.15)
  if (CMAKE_OSX_DEPLOYMENT_TARGET)
    set(METAL_VERSION_MIN ${CMAKE_OSX_DEPLOYMENT_TARGET})
  endif ()

  set(METAL_CHECK_DIR ${CMAKE_BINARY_DIR}/visage_metal_check)
  file(WRITE ${METAL_CHECK_DIR}/check.metal "kernel void main0() {}\n")
  execute_process(
    COMMAND xcrun -sdk macosx metal -mmacosx-version-min=${METAL_VERSION_MIN} -o ${METAL_CHECK_DIR}/check.metallib
            ${METAL_CHECK_DIR}/check.metal
    RESULT_VARIABLE METAL_CHECK_RESULT
    OUTPUT_QUIET
    ERROR_QUIET
  )

  if (METAL_CHECK_RESULT EQUAL 0)
    set(VISAGE_METAL_SHADER_TARGET metallib CACHE INTERNAL "")
    set(VISAGE_METAL_SHADER_ARGS --macos-version-min ${METAL_VERSION_MIN} CACHE INTERNAL "")
  else ()
    message(WARNING "VISAGE: The Metal Toolchain is missing, so Metal shaders are embedded as source and "
            "compile at runtime, which stalls the first frame that uses each one. Install it with "
            "'xcodebuild -downloadComponent MetalToolchain' and reconfigure.")
  endif ()
endif ()

function(visage_embed_shaders project include_filename namespace original_shaders)
  file(GLOB_RECURSE SHADER_INCLUDES shaders/*.sh shaders/varying.def.sc)
  file(GLOB LIBRARY_SHADER_INCLUDES ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/shaders/*.sh)
  set(SHADER_DEPENDENCIES ${SHADER_INCLUDES} ${LIBRARY_SHADER_INCLUDES} ${VISAGE_SHADER_COMPILER_EXECUTABLE}
      ${VISAGE_SHADER_COMPILER_TARGET})
  list(REMOVE_DUPLICATES SHADER_DEPENDENCIES)

  if (EMSCRIPTEN)
    set(SHADER_TARGET essl)
  elseif (WIN32)
    set(SHADER_TARGET dxbc)
  elseif (APPLE)
    set(SHADER_TARGET ${VISAGE_METAL_SHADER_TARGET})
    set(SHADER_TARGET_ARGS ${VISAGE_METAL_SHADER_ARGS})
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
      COMMAND ${VISAGE_SHADER_COMPILER_EXECUTABLE} -i ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/shaders -f ${SHADER}
              -o ${SHADER_PATH} --type ${SHADER_TYPE} --target ${SHADER_TARGET} ${SHADER_TARGET_ARGS}
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
