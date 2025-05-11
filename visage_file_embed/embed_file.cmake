if (NOT DEFINED ORIGINAL_FILE)
  message(FATAL_ERROR "Required ORIGINAL_FILE is not set")
endif ()

if (NOT DEFINED DEST_FILE)
  message(FATAL_ERROR "Required DEST_FILE is not set")
endif ()

if (NOT DEFINED VAR_NAMESPACE)
  message(FATAL_ERROR "Required VAR_NAMESPACE is not set")
endif ()

string(REGEX MATCH "([^/]+)$" VAR_NAME ${ORIGINAL_FILE})
string(REGEX REPLACE "\\.| |-" "_" VAR_NAME ${VAR_NAME})

file(READ ${ORIGINAL_FILE} DATA HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," DATA ${DATA})
set(FILE_CONTENTS_BEGIN "// Generated file, do not edit\n")
set(FILE_CONTENTS_BEGIN "${FILE_CONTENTS_BEGIN}#include \"embedded_file.h\"\n")
set(FILE_CONTENTS_BEGIN "${FILE_CONTENTS_BEGIN}namespace ${VAR_NAMESPACE} {\n")
set(FILE_CONTENTS_BEGIN "${FILE_CONTENTS_BEGIN}static const char ${VAR_NAME}_name[] = \"${VAR_NAME}\";\n")
set(FILE_CONTENTS_BEGIN "${FILE_CONTENTS_BEGIN}static const unsigned char ${VAR_NAME}_tmp[] = {")
set(FILE_CONTENTS_END "};\n")
set(FILE_CONTENTS_END "${FILE_CONTENTS_END}::visage::EmbeddedFile ${VAR_NAME} = { ${VAR_NAME}_name, ")
set(FILE_CONTENTS_END "${FILE_CONTENTS_END}${VAR_NAME}_tmp, sizeof(${VAR_NAME}_tmp) };\n")
set(FILE_CONTENTS_END "${FILE_CONTENTS_END}}\n")

file(CONFIGURE OUTPUT ${DEST_FILE} CONTENT "${FILE_CONTENTS_BEGIN} ${DATA} ${FILE_CONTENTS_END}")
