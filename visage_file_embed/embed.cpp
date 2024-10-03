/* Copyright Vital Audio, LLC
 *
 * visage is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * visage is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with visage.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>

int charDump(std::ofstream& output, const std::string& source_path) {
  static constexpr int kNumPerLine = 64;
  std::ifstream file(source_path, std::ios::binary | std::ios::ate);
  if (!file) {
    std::cerr << "Cannot open file " << source_path << std::endl;
    return -1;
  }

  int size = static_cast<int>(file.tellg());
  file.seekg(0, std::ios::beg);

  char buffer[kNumPerLine];
  while (file.read(buffer, sizeof(buffer)) || file.gcount()) {
    output << std::endl;
    for (std::streamsize i = 0; i < file.gcount(); ++i)
      output << (static_cast<int>(buffer[i]) & 0xff) << ",";
  }
  output << "0,0" << std::endl;
  return size;
}

std::string createVarName(const std::string& file_name) {
  std::string var_name = file_name;
  std::replace(var_name.begin(), var_name.end(), '.', '_');
  std::replace(var_name.begin(), var_name.end(), '-', '_');
  std::replace(var_name.begin(), var_name.end(), ' ', '_');
  return var_name;
}

int writeEmbedFile(const std::string& output_cpp, const std::string& source_path,
                   const std::string& defined_namespace) {
  std::string file_name = source_path.substr(source_path.find_last_of("/\\") + 1);
  std::string var_name = createVarName(file_name);
  std::ofstream file(output_cpp);
  file << "// Generated file, do not edit" << std::endl << std::endl;
  file << "#include \"embedded_file.h\"" << std::endl << std::endl;
  file << "namespace " << defined_namespace << " {" << std::endl;
  file << "static const char " << var_name << "_name[] = \"" << var_name << "\";" << std::endl;
  file << "static const unsigned char " << var_name << "_tmp[] = {";

  int size = charDump(file, source_path);

  file << "};" << std::endl << std::endl;
  file << "::visage::EmbeddedFile " << var_name << " = { " << var_name << "_name, "
       << "(const char*)" << var_name << "_tmp, " << size << " };" << std::endl;
  file << "}" << std::endl;

  return size;
}

int showUsage(char** argv) {
  std::cerr << "Usage: " << argv[0]
            << " --header <path> <include-filename> <namespace> <filename1> <filename2> ..."
            << std::endl;
  std::cerr << "Usage: " << argv[0]
            << " --embed <path> <include-filename> <namespace> <filename> <index>" << std::endl;
  return 1;
}

int generateHeader(int argc, char** argv) {
  if (argc < 6)
    return showUsage(argv);

  std::string output_path(argv[2]);
  std::string include_filename(argv[3]);
  std::string defined_namespace(argv[4]);
  std::ofstream header(output_path + "/" + include_filename);
  std::ofstream lookup(output_path + "/embedded_files0.cpp");

  if (!header.is_open()) {
    std::cerr << "File Embed Error: Can't create " << output_path + "/" + include_filename << std::endl;
    return 1;
  }

  header << "// Generated file, do not edit" << std::endl << std::endl;
  header << "#pragma once" << std::endl << std::endl;
  header << "#include \"embedded_file.h\"" << std::endl;
  header << "#include <string>" << std::endl << std::endl;
  header << "namespace " << defined_namespace << " {" << std::endl << std::endl;
  header << "  ::visage::EmbeddedFile getFileByName(const std::string& filename);" << std::endl
         << std::endl;

  lookup << "// Generated file, do not edit" << std::endl << std::endl;
  lookup << "#include \"" << include_filename << "\"" << std::endl << std::endl;
  lookup << "namespace " << defined_namespace << " {" << std::endl << std::endl;
  lookup << "  ::visage::EmbeddedFile getFileByName(const std::string& filename) {" << std::endl;

  for (int i = 5; i < argc; ++i) {
    std::string file_path(argv[i]);
    std::string file_name = file_path.substr(file_path.find_last_of("/\\") + 1);
    std::string var_name = createVarName(file_name);

    header << "  extern ::visage::EmbeddedFile " << var_name << ";" << std::endl;

    lookup << "    if (filename == \"" << file_name << "\") " << std::endl;
    lookup << "      return " << var_name << ";" << std::endl;
  }

  header << "}" << std::endl;

  lookup << "    return { nullptr, 0 };" << std::endl;
  lookup << "  }" << std::endl;
  lookup << "}" << std::endl;

  return 0;
}

int embedFile(int argc, char** argv) {
  if (argc < 7)
    return showUsage(argv);

  std::string output_path(argv[2]);
  std::string include_filename(argv[3]);
  std::string defined_namespace(argv[4]);
  std::string file_path(argv[5]);
  char* end = nullptr;
  int index = std::strtol(argv[6], &end, 10);

  std::string cpp_file = output_path + "/embedded_files" + std::to_string(index) + ".cpp";
  writeEmbedFile(cpp_file, file_path, defined_namespace);

  return 0;
}

int main(int argc, char** argv) {
  if (argc <= 1)
    return showUsage(argv);
  std::string mode(argv[1]);

  if (mode == "--header")
    return generateHeader(argc, argv);
  else if (mode == "--embed")
    return embedFile(argc, argv);

  return showUsage(argv);
}
