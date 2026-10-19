add_library(VisageFileEmbedInclude INTERFACE)
target_include_directories(VisageFileEmbedInclude INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

include(${CMAKE_CURRENT_LIST_DIR}/embedded_name.cmake)

function(add_embedded_resources project include_filename namespace files)
  get_filename_component(current_dir_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)

  set(variable_names)
  foreach (file IN LISTS files)
    embedded_variable_name(${file} variable_name)
    list(FIND variable_names ${variable_name} duplicate_index)
    if (NOT duplicate_index EQUAL -1)
      list(GET files ${duplicate_index} duplicate_file)
      message(FATAL_ERROR "${project} embeds ${duplicate_file} and ${file} with the same name, ${variable_name}. Rename one of them.")
    endif ()
    list(APPEND variable_names ${variable_name})
  endforeach ()

  set(generated_path ${CMAKE_CURRENT_BINARY_DIR}/${project}_generated)
  set(destination_path ${generated_path}/embedded)
  file(MAKE_DIRECTORY ${generated_path})
  file(MAKE_DIRECTORY ${destination_path})

  set(lookup_file "${destination_path}/embedded_files0.cpp")

  set(source_files)
  list(APPEND source_files "${lookup_file}")

  set(index 1)
  foreach (file IN LISTS files)
    set(embedded_file_name "embedded_files${index}.cpp")
    set(source_file "${destination_path}/${embedded_file_name}")
    list(APPEND source_files ${source_file})
    get_filename_component(original_file_name ${file} NAME)
    add_custom_command(
      OUTPUT ${source_file}
      COMMAND ${CMAKE_COMMAND} -DDEST_FILE=${source_file} -DORIGINAL_FILE=${file} -DVAR_NAMESPACE=${namespace} -P ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/embed_file.cmake
      DEPENDS ${file} "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/embedded_name.cmake"
      COMMENT "Generating ${embedded_file_name} for ${original_file_name}"
      VERBATIM
    )
    set_source_files_properties(${source_file} PROPERTIES GENERATED TRUE)
    math(EXPR index "${index} + 1")
  endforeach ()

  add_custom_command(
    OUTPUT ${destination_path}/${include_filename} ${lookup_file}
    COMMAND ${CMAKE_COMMAND} -DDEST_FILE=${destination_path}/${include_filename} "-DFILE_LIST=${files}"
    -DVAR_NAMESPACE=${namespace} -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/create_embed_header.cmake"
    COMMAND ${CMAKE_COMMAND} -DDEST_FILE=${lookup_file} -DINCLUDE_FILE=${include_filename} "-DFILE_LIST=${files}"
    -DVAR_NAMESPACE=${namespace} -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/create_file_lookup.cmake"
    DEPENDS ${files} "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/create_embed_header.cmake"
    "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/embedded_name.cmake"
    COMMENT "Generating C++ for embedded files..."
    VERBATIM
  )

  set_source_files_properties(${destination_path}/${include_filename} PROPERTIES GENERATED TRUE)
  set_source_files_properties(${lookup_file} PROPERTIES GENERATED TRUE)
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
