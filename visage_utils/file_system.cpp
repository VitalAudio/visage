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

#include "file_system.h"

#include <chrono>
#include <fstream>
#include <regex>
#include <string>

#if VISAGE_WINDOWS
#include <ShlObj.h>
#include <windows.h>
#elif VISAGE_MAC
#include <dlfcn.h>
#include <mach-o/dyld.h>
#else
#include <cstdlib>
#include <dlfcn.h>
#include <unistd.h>

static visage::File xdgFolder(const char* env_var, const char* default_folder) {
  const char* xdg_folder = getenv(env_var);
  if (xdg_folder)
    return xdg_folder;

  return default_folder;
}
#endif

namespace visage {
  bool replaceFileWithData(const File& file, const unsigned char* data, size_t size) {
    std::ofstream stream(file, std::ios::binary);
    if (!stream)
      return false;

    stream.write(reinterpret_cast<const char*>(data), size);

    if (stream.fail())
      return false;

    return true;
  }

  bool replaceFileWithText(const File& file, const std::string& text) {
    std::ofstream stream(file);
    if (!stream)
      return false;

    stream << text;
    if (stream.fail())
      return false;

    return true;
  }

  bool hasWriteAccess(const File& file) {
    std::filesystem::file_status status = std::filesystem::status(file);
    return (status.permissions() & std::filesystem::perms::owner_write) != std::filesystem::perms::none;
  }

  bool fileExists(const File& file) {
    return std::filesystem::exists(file);
  }

  bool appendTextToFile(const File& file, const std::string& text) {
    std::ofstream stream(file, std::ios::app);
    if (!stream)
      return false;

    stream << text;
    if (stream.fail())
      return false;

    return true;
  }

  std::unique_ptr<unsigned char[]> loadFileData(const File& file, int& size) {
    std::ifstream stream(file, std::ios::binary | std::ios::ate);
    if (!stream)
      return {};

    size = stream.tellg();
    stream.seekg(0, std::ios::beg);

    auto data = std::make_unique<unsigned char[]>(size);
    stream.read(reinterpret_cast<char*>(data.get()), size);
    return data;
  }

  std::string loadFileAsString(const File& file) {
    std::ifstream stream(file);
    if (!stream)
      return {};

    return { std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>() };
  }

  File hostExecutable() {
#if VISAGE_WINDOWS
    static File path;
    if (path.empty()) {
      char buffer[MAX_PATH];
      GetModuleFileNameA(nullptr, buffer, MAX_PATH);
      path = buffer;
    }
    return path;
#elif VISAGE_MAC
    static constexpr int kMaxPathLength = 1 << 14;
    char path[kMaxPathLength];
    uint32_t size = kMaxPathLength - 1;
    if (_NSGetExecutablePath(path, &size))
      return {};

    path[size] = '\0';
    return std::filesystem::canonical(path);
#else
    static constexpr int kMaxPathLength = 1 << 14;
    char path[kMaxPathLength];
    ssize_t size = readlink("/proc/self/exe", path, kMaxPathLength - 1);
    if (size < 0)
      return {};

    path[size] = '\0';
    return std::filesystem::canonical(path);
#endif
  }

  File appDataDirectory() {
#if VISAGE_WINDOWS
    static File path;
    if (path.empty()) {
      char buffer[MAX_PATH];
      SHGetFolderPathA(nullptr, CSIDL_APPDATA, nullptr, 0, buffer);
      path = buffer;
    }
    return path;
#elif VISAGE_MAC
    return "~/Library";
#elif VISAGE_LINUX
    return xdgFolder("XDG_DATA_HOME", "~/.config");
#else
    return {};
#endif
  }

  File userDocumentsDirectory() {
#if VISAGE_WINDOWS
    static File path;
    if (path.empty()) {
      char buffer[MAX_PATH];
      SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr, 0, buffer);
      path = buffer;
    }
    return path;
#elif VISAGE_MAC
    return "~/Documents";
#elif VISAGE_LINUX
    return xdgFolder("XDG_DOCUMENTS_DIR", "~/Documents");
#else
    return {};
#endif
  }

  File createTemporaryFile(const std::string& extension) {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    std::string unique_file = std::to_string(ms) + "." + extension;
    return std::filesystem::temp_directory_path() / unique_file;
  }

  std::string fileName(const File& file) {
    return file.filename().string();
  }

  std::string fileStem(const File& file) {
    return file.stem().string();
  }

  std::string hostName() {
    return fileStem(hostExecutable());
  }

  std::vector<File> searchForFiles(const File& directory, const std::string& regex) {
    if (!std::filesystem::is_directory(directory))
      return {};

    std::vector<File> matches;
    std::regex match_pattern(regex);

    for (const auto& entry : std::filesystem::recursive_directory_iterator(directory)) {
      std::string file_name = entry.path().filename().string();
      if (entry.is_regular_file() && std::regex_search(file_name, match_pattern))
        matches.push_back(entry.path());
    }

    return matches;
  }

  std::vector<File> searchForDirectories(const File& directory, const std::string& regex) {
    if (!std::filesystem::is_directory(directory))
      return {};

    std::vector<File> matches;
    std::regex match_pattern(regex);

    for (const auto& entry : std::filesystem::recursive_directory_iterator(directory)) {
      std::string file_name = entry.path().filename().string();
      if (entry.is_directory() && std::regex_search(file_name, match_pattern))
        matches.push_back(entry.path());
    }

    return matches;
  }
}
