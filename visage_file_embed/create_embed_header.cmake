if (NOT DEFINED FILE_LIST)
  message(FATAL_ERROR "Required FILE_LIST is not set")
endif ()

if (NOT DEFINED DEST_FILE)
  message(FATAL_ERROR "Required DEST_FILE is not set")
endif ()

if (NOT DEFINED VAR_NAMESPACE)
  message(FATAL_ERROR "Required VAR_NAMESPACE is not set")
endif ()

set(FILE_CONTENTS "// Generated file, do not edit\n")
set(FILE_CONTENTS "${FILE_CONTENTS}#pragma once\n\n")
set(FILE_CONTENTS "${FILE_CONTENTS}#include \"embedded_file.h\"\n")
set(FILE_CONTENTS "${FILE_CONTENTS}#include <string>\n")
set(FILE_CONTENTS "${FILE_CONTENTS}namespace ${VAR_NAMESPACE} {\n")
set(FILE_CONTENTS "${FILE_CONTENTS}  ::visage::EmbeddedFile getFileByName(const std::string& filename);\n")

foreach (FILE ${FILE_LIST})
  string(REGEX MATCH "([^/]+)$" VAR_NAME ${FILE})
  string(REGEX REPLACE "\\.| |-" "_" VAR_NAME ${VAR_NAME})
  set(FILE_CONTENTS "${FILE_CONTENTS}  extern ::visage::EmbeddedFile ${VAR_NAME};\n")
endforeach ()
set(FILE_CONTENTS "${FILE_CONTENTS}}\n")

file(CONFIGURE OUTPUT ${DEST_FILE} CONTENT "${FILE_CONTENTS}")
