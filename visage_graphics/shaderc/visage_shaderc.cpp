/* Copyright Vital Audio, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "shader_compiler.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace visage::gpu;

namespace {
  struct Options : visage::shaderc::CompileOptions {
    std::string output;
    std::string varying_path;
    bool stage_set = false;
  };

  bool readFile(const std::filesystem::path& path, std::string& result) {
    std::ifstream file(path, std::ios::binary);
    if (!file)
      return false;
    std::stringstream stream;
    stream << file.rdbuf();
    result = stream.str();
    return true;
  }

  bool parseArguments(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };

      if (arg == "-f")
        options.input = next();
      else if (arg == "-o")
        options.output = next();
      else if (arg == "-i")
        options.include_dirs.push_back(next());
      else if (arg == "--varyingdef")
        options.varying_path = next();
      else if (arg == "--define")
        options.defines.push_back(next());
      else if (arg == "--type") {
        std::string type = next();
        options.stage_set = true;
        if (type == "v" || type == "vertex")
          options.stage = ShaderStage::Vertex;
        else if (type == "c" || type == "compute")
          options.stage = ShaderStage::Compute;
        else
          options.stage = ShaderStage::Fragment;
      }
      else if (arg == "--target" || arg == "-p" || arg == "--profile") {
        std::string target = next();
        if (target == "spirv")
          options.target = ShaderCode::Spirv;
        else if (target == "msl" || target == "metal")
          options.target = ShaderCode::Msl;
        else if (target == "hlsl" || target.rfind("s_", 0) == 0)
          options.target = ShaderCode::Hlsl;
        else if (target == "dxbc")
          options.target = ShaderCode::Dxbc;
        else if (target == "essl" || target == "300_es")
          options.target = ShaderCode::Essl;
        else {
          std::cerr << "Unknown target " << target << std::endl;
          return false;
        }
      }
      else if (arg == "--platform" || arg == "-O")
        next();
      else {
        std::cerr << "Unknown argument " << arg << std::endl;
        return false;
      }
    }

    if (options.input.empty() || options.output.empty()) {
      std::cerr << "Usage: shader_compiler -f input.sc -o output --type v|f|c --target "
                   "spirv|msl|hlsl|dxbc|essl"
                   " [-i include_dir] [--varyingdef varying.def.sc] [--define NAME=VALUE]"
                << std::endl;
      return false;
    }

    if (!options.stage_set) {
      std::string name = std::filesystem::path(options.input).filename().string();
      if (name.rfind("vs_", 0) == 0)
        options.stage = ShaderStage::Vertex;
      else if (name.rfind("cs_", 0) == 0)
        options.stage = ShaderStage::Compute;
    }
    if (options.varying_path.empty())
      options.varying_path = (std::filesystem::path(options.input).parent_path() / "varying.def.sc").string();
    return true;
  }
}

int main(int argc, char** argv) {
  Options options;
  if (!parseArguments(argc, argv, options))
    return 1;

  if (!readFile(options.input, options.source)) {
    std::cerr << "Could not read " << options.input << std::endl;
    return 1;
  }
  readFile(options.varying_path, options.varying_source);

  std::vector<uint8_t> blob;
  std::string error;
  if (!visage::shaderc::compile(options, blob, error)) {
    std::cerr << options.input << ":\n" << error << std::endl;
    return 1;
  }

  std::filesystem::path output(options.output);
  if (output.has_parent_path())
    std::filesystem::create_directories(output.parent_path());
  std::ofstream file(output, std::ios::binary);
  file.write(reinterpret_cast<const char*>(blob.data()), static_cast<std::streamsize>(blob.size()));
  return file ? 0 : 1;
}
