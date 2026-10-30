cmake_minimum_required(VERSION 3.20)
project(VisageShaderCompiler LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
foreach (CONFIG Debug Release RelWithDebInfo MinSizeRel)
  string(TOUPPER ${CONFIG} CONFIG_UPPER)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIG_UPPER} ${CMAKE_CURRENT_BINARY_DIR}/bin)
endforeach ()

include(FetchContent)
set(CMAKE_MESSAGE_LOG_LEVEL "ERROR")
set(ENABLE_OPT OFF CACHE INTERNAL "" FORCE)
set(ENABLE_GLSLANG_BINARIES OFF CACHE INTERNAL "" FORCE)
set(ENABLE_HLSL OFF CACHE INTERNAL "" FORCE)
set(ENABLE_SPVREMAPPER OFF CACHE INTERNAL "" FORCE)
set(ENABLE_CTEST OFF CACHE INTERNAL "" FORCE)
set(GLSLANG_TESTS OFF CACHE INTERNAL "" FORCE)
set(GLSLANG_ENABLE_INSTALL OFF CACHE INTERNAL "" FORCE)
set(SPIRV_CROSS_CLI OFF CACHE INTERNAL "" FORCE)
set(SPIRV_CROSS_ENABLE_TESTS OFF CACHE INTERNAL "" FORCE)
set(SPIRV_CROSS_ENABLE_C_API OFF CACHE INTERNAL "" FORCE)
set(SPIRV_CROSS_ENABLE_UTIL OFF CACHE INTERNAL "" FORCE)
set(SPIRV_CROSS_ENABLE_REFLECT OFF CACHE INTERNAL "" FORCE)
set(SPIRV_CROSS_ENABLE_CPP OFF CACHE INTERNAL "" FORCE)
set(SPIRV_CROSS_SKIP_INSTALL ON CACHE INTERNAL "" FORCE)
set(BUILD_SHARED_LIBS OFF)

FetchContent_Declare(glslang GIT_REPOSITORY https://github.com/KhronosGroup/glslang.git
  GIT_TAG vulkan-sdk-1.4.341.0
  GIT_SHALLOW TRUE
  EXCLUDE_FROM_ALL
)
FetchContent_Declare(spirv_cross GIT_REPOSITORY https://github.com/KhronosGroup/SPIRV-Cross.git
  GIT_TAG vulkan-sdk-1.4.341.0
  GIT_SHALLOW TRUE
  EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(glslang spirv_cross)
set(CMAKE_MESSAGE_LOG_LEVEL "NOTICE")

set(SHADER_COMPILER_SOURCES shader_compiler.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../gpu/gpu_shader.cpp)
if (WIN32)
  list(APPEND SHADER_COMPILER_SOURCES visage_shaderc_dxbc.cpp)
endif ()

add_library(VisageShaderCompiler STATIC ${SHADER_COMPILER_SOURCES} shader_compiler.h)
target_include_directories(VisageShaderCompiler PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(VisageShaderCompiler PRIVATE glslang glslang-default-resource-limits SPIRV
                      spirv-cross-core spirv-cross-glsl spirv-cross-hlsl spirv-cross-msl)
target_compile_features(VisageShaderCompiler PUBLIC cxx_std_17)
if (WIN32)
  target_link_libraries(VisageShaderCompiler PRIVATE d3dcompiler)
endif ()

add_executable(shader_compiler visage_shaderc.cpp)
target_link_libraries(shader_compiler PRIVATE VisageShaderCompiler)
set_target_properties(VisageShaderCompiler shader_compiler PROPERTIES FOLDER "visage/tools")
