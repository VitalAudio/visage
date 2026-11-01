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

#include <cerrno>
#include <crt_externs.h>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <spawn.h>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

namespace {
  bool readFile(const std::filesystem::path& path, std::string& result) {
    std::ifstream file(path, std::ios::binary);
    if (!file)
      return false;
    std::stringstream stream;
    stream << file.rdbuf();
    result = stream.str();
    return true;
  }

  bool writeFile(const std::filesystem::path& path, const std::string& contents) {
    std::ofstream file(path, std::ios::binary);
    file.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    return static_cast<bool>(file);
  }

  bool runMetalCompiler(const std::filesystem::path& source, const std::filesystem::path& library,
                        const std::filesystem::path& log, const std::string& macos_version_min) {
    std::vector<std::string> arguments = { "xcrun", "-sdk", "macosx", "metal" };
    if (!macos_version_min.empty())
      arguments.push_back("-mmacosx-version-min=" + macos_version_min);
    arguments.push_back("-o");
    arguments.push_back(library.string());
    arguments.push_back(source.string());

    std::vector<char*> argv;
    for (std::string& argument : arguments)
      argv.push_back(argument.data());
    argv.push_back(nullptr);

    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, log.c_str(),
                                     O_WRONLY | O_CREAT | O_TRUNC, 0644);
    posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);

    pid_t pid = 0;
    int result = posix_spawnp(&pid, "xcrun", &actions, nullptr, argv.data(), *_NSGetEnviron());
    posix_spawn_file_actions_destroy(&actions);
    if (result != 0)
      return false;

    int status = 0;
    while (waitpid(pid, &status, 0) < 0) {
      if (errno != EINTR)
        return false;
    }
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
  }
}

bool compileMetallib(const std::string& msl, const std::string& macos_version_min,
                     std::vector<uint8_t>& result, std::string& error) {
  std::string directory_name = (std::filesystem::temp_directory_path() / "visage_metal_XXXXXX").string();
  if (mkdtemp(directory_name.data()) == nullptr) {
    error = "Could not create a directory for the Metal compiler";
    return false;
  }

  std::filesystem::path directory = directory_name;
  std::filesystem::path source = directory / "shader.metal";
  std::filesystem::path library = directory / "shader.metallib";
  std::filesystem::path log = directory / "log.txt";

  std::string code;
  bool compiled = writeFile(source, msl) && runMetalCompiler(source, library, log, macos_version_min) &&
                  readFile(library, code);
  if (!compiled) {
    readFile(log, error);
    error = msl + "\n" + (error.empty() ? "Could not run the Metal compiler through xcrun" : error);
  }

  std::error_code ignored;
  std::filesystem::remove_all(directory, ignored);
  if (!compiled)
    return false;

  result.assign(code.begin(), code.end());
  return true;
}
