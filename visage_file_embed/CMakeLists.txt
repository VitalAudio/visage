add_library(VisageEmbeddedFile INTERFACE)

if (UNIX AND NOT APPLE)
  file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/bin/linux/VisageFileEmbed DESTINATION ${CMAKE_BINARY_DIR}/VisageFileEmbed)
else ()
  add_executable(VisageFileEmbed embed.cpp)
  set_target_properties(VisageFileEmbed PROPERTIES FOLDER "visage")
endif ()

add_library(VisageFileEmbedInclude INTERFACE)
target_include_directories(VisageFileEmbedInclude INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

function(add_embedded_resources project include_filename namespace files)
  if (UNIX AND NOT APPLE)
    set(FILE_EMBEDDER_EXECUTABLE ${CMAKE_BINARY_DIR}/VisageFileEmbed/VisageFileEmbed)
  else ()
    set(FILE_EMBEDDER_LIB VisageFileEmbed)
    set(FILE_EMBEDDER_EXECUTABLE VisageFileEmbed)
  endif ()

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
      COMMAND ${FILE_EMBEDDER_EXECUTABLE} ARGS "--embed" ${destination_path} ${include_filename} ${namespace} ${file} ${index}
      DEPENDS ${FILE_EMBEDDER_LIB} ${file}
      COMMENT "Generating ${source_file} for ${file}"
    )
    math(EXPR index "${index} + 1")
  endforeach ()

  add_custom_command(
    OUTPUT ${destination_path}/${include_filename} ${lookup_file}
    COMMAND ${FILE_EMBEDDER_EXECUTABLE} ARGS "--header" ${destination_path} ${include_filename} ${namespace} ${files}
    DEPENDS ${FILE_EMBEDDER_LIB} ${files}
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
    VisageEmbeddedFile
  )
endfunction()
