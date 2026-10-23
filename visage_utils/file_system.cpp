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

#include "string_utils.h"

#include <fstream>
#include <random>
#include <regex>
#include <string>

#if VISAGE_WINDOWS
#include <ShlObj.h>
#include <windows.h>
#elif VISAGE_MAC
#include <cstdlib>
#include <dlfcn.h>
#include <mach-o/dyld.h>
#include <unistd.h>
#else
#include <cstdlib>
#include <dlfcn.h>
#include <unistd.h>
#endif

#if !VISAGE_WINDOWS
static visage::File homeFolder(const char* folder) {
  const char* home = getenv("HOME");
  return visage::File(home ? home : "") / folder;
}
#endif

#if !VISAGE_WINDOWS && !VISAGE_MAC
static visage::File xdgFolder(const char* env_var, const char* default_folder) {
  const char* xdg_folder = getenv(env_var);
  if (xdg_folder && xdg_folder[0] == '/')
    return xdg_folder;

  return homeFolder(default_folder);
}
#endif

namespace visage {
  static std::string utf8Path(const File& file) {
#if VISAGE_WINDOWS
    return String::convertToUtf8(file.wstring());
#else
    return file.string();
#endif
  }

  bool replaceFileWithData(const File& file, const unsigned char* data, size_t size) {
    std::ofstream stream(file, std::ios::binary);
    if (!stream)
      return false;

    stream.write(reinterpret_cast<const char*>(data), size);
    stream.close();
    return !stream.fail();
  }

  bool replaceFileWithText(const File& file, const std::string& text) {
    std::ofstream stream(file, std::ios::binary);
    if (!stream)
      return false;

    stream << text;
    stream.close();
    return !stream.fail();
  }

  bool hasWriteAccess(const File& file) {
#if VISAGE_WINDOWS
    std::error_code error;
    std::filesystem::file_status status = std::filesystem::status(file, error);
    return std::filesystem::exists(status) &&
           (status.permissions() & std::filesystem::perms::owner_write) != std::filesystem::perms::none;
#else
    return access(file.c_str(), W_OK) == 0;
#endif
  }

  bool fileExists(const File& file) {
    return std::filesystem::exists(file);
  }

  bool isDirectory(const File& file) {
    return std::filesystem::is_directory(file);
  }

  bool appendTextToFile(const File& file, const std::string& text) {
    std::ofstream stream(file, std::ios::binary | std::ios::app);
    if (!stream)
      return false;

    stream << text;
    stream.close();
    return !stream.fail();
  }

  static bool openRegularFile(const File& file, std::ifstream& stream, std::streamoff& length) {
    std::error_code error;
    if (!std::filesystem::is_regular_file(file, error))
      return false;

    stream.open(file, std::ios::binary | std::ios::ate);
    if (!stream)
      return false;

    length = stream.tellg();
    stream.seekg(0, std::ios::beg);
    return length >= 0 && stream;
  }

  std::unique_ptr<unsigned char[]> loadFileData(const File& file, size_t& size) {
    std::ifstream stream;
    std::streamoff length = 0;
    if (!openRegularFile(file, stream, length))
      return {};

    auto data = std::make_unique<unsigned char[]>(length);
    stream.read(reinterpret_cast<char*>(data.get()), length);
    size = stream.gcount();
    return data;
  }

  std::string loadFileAsString(const File& file) {
    std::ifstream stream;
    std::streamoff length = 0;
    if (!openRegularFile(file, stream, length))
      return {};

    std::string out;
    out.resize(length);
    stream.read(out.data(), length);
    out.resize(stream.gcount());
    return out;
  }

  File hostExecutable() {
#if VISAGE_WINDOWS
    static File path;
    if (path.empty()) {
      wchar_t buffer[MAX_PATH];
      if (GetModuleFileNameW(nullptr, buffer, MAX_PATH))
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
      wchar_t buffer[MAX_PATH];
      if (SUCCEEDED(SHGetFolderPathW(nullptr, CSIDL_APPDATA, nullptr, 0, buffer)))
        path = buffer;
    }
    return path;
#elif VISAGE_MAC
    return homeFolder("Library");
#elif VISAGE_LINUX
    return xdgFolder("XDG_DATA_HOME", ".local/share");
#else
    return {};
#endif
  }

  File userDocumentsDirectory() {
#if VISAGE_WINDOWS
    static File path;
    if (path.empty()) {
      wchar_t buffer[MAX_PATH];
      if (SUCCEEDED(SHGetFolderPathW(nullptr, CSIDL_PERSONAL, nullptr, 0, buffer)))
        path = buffer;
    }
    return path;
#elif VISAGE_MAC
    return homeFolder("Documents");
#elif VISAGE_LINUX
    return xdgFolder("XDG_DOCUMENTS_DIR", "Documents");
#else
    return {};
#endif
  }

  File createTemporaryFile(const std::string& extension) {
    static constexpr char kHexDigits[] = "0123456789abcdef";
    static constexpr int kNameLength = 16;

    std::random_device random_device;
    File directory = std::filesystem::temp_directory_path();
    File file;
    std::error_code error;
    do {
      std::string name = "visage_";
      for (int i = 0; i < kNameLength; ++i)
        name.push_back(kHexDigits[random_device() % 16]);
      file = directory / (name + "." + extension);
    } while (std::filesystem::exists(file, error));

    return file;
  }

  void createDirectory(const File& file) {
    std::filesystem::create_directories(file);
  }

  std::string fileName(const File& file) {
    return utf8Path(file.filename());
  }

  std::string fileStem(const File& file) {
    return utf8Path(file.stem());
  }

  std::string hostName() {
    return fileStem(hostExecutable());
  }

  std::vector<File> searchForFiles(const File& directory, const std::string& regex) {
    std::error_code error;
    if (!std::filesystem::is_directory(directory, error))
      return {};

    std::vector<File> matches;
    std::regex match_pattern(regex);

    auto options = std::filesystem::directory_options::skip_permission_denied;
    std::filesystem::recursive_directory_iterator entry(directory, options, error);
    for (; !error && entry != std::filesystem::recursive_directory_iterator(); entry.increment(error)) {
      std::error_code type_error;
      std::string file_name = utf8Path(entry->path().filename());
      if (entry->is_regular_file(type_error) && std::regex_search(file_name, match_pattern))
        matches.push_back(entry->path());
    }

    return matches;
  }

  std::vector<File> searchForDirectories(const File& directory, const std::string& regex) {
    if (!std::filesystem::is_directory(directory))
      return {};

    std::vector<File> matches;
    std::regex match_pattern(regex);

    for (const auto& entry : std::filesystem::recursive_directory_iterator(directory)) {
      std::string file_name = utf8Path(entry.path().filename());
      if (entry.is_directory() && std::regex_search(file_name, match_pattern))
        matches.push_back(entry.path());
    }

    return matches;
  }
}
