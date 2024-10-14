add_library(VisageFileEmbedInclude INTERFACE)
target_include_directories(VisageFileEmbedInclude INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

function(add_embedded_resources project include_filename namespace files)
  get_filename_component(current_dir_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)

  set(generated_path ${CMAKE_CURRENT_BINARY_DIR}/${project}_generated)
  set(destination_path ${generated_path}/embedded)
  file(MAKE_DIRECTORY ${generated_path})
  file(MAKE_DIRECTORY ${destination_path})

  set(lookup_file "${destination_path}/embedded_files0.cpp")

  set(source_files)
  list(APPEND source_files "${lookup_file}")

  set(index 1)
  foreach (file IN LISTS files)
    set(source_file "${destination_path}/embedded_files${index}.cpp")
    list(APPEND source_files ${source_file})
    add_custom_command(
      OUTPUT ${source_file}
      COMMAND ${CMAKE_COMMAND} -DDEST_FILE=${source_file} -DORIGINAL_FILE=${file} -DVAR_NAMESPACE=${namespace} -P ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/embed_file.cmake
      DEPENDS ${file} ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/embed_file.cmake
      COMMENT "Generating ${source_file} for ${file}"
    )
    math(EXPR index "${index} + 1")
  endforeach ()

  string(REPLACE ";" "\\;" escaped_files "${files}")
  add_custom_command(
    OUTPUT ${destination_path}/${include_filename} ${lookup_file}
    COMMAND ${CMAKE_COMMAND} -DDEST_FILE=${destination_path}/${include_filename} -DFILE_LIST="${escaped_files}"
    -DVAR_NAMESPACE=${namespace} -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/create_embed_header.cmake"
    COMMAND ${CMAKE_COMMAND} -DDEST_FILE=${lookup_file} -DINCLUDE_FILE=${include_filename} -DFILE_LIST="${escaped_files}"
    -DVAR_NAMESPACE=${namespace} -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/create_file_lookup.cmake"
    DEPENDS ${FILE_EMBEDDER_LIB} ${files} "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/create_embed_header.cmake"
    COMMENT "Generating C++ for embedded files..."
  )

  add_library(${project} STATIC ${destination_path}/${include_filename} ${source_files})
  target_include_directories(${project}
    PUBLIC
    ${generated_path}
  )
  target_link_libraries(${project}
    PUBLIC
    VisageFileEmbedInclude
  )
endfunction()
