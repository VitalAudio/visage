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
  void replaceFileWithData(const File& file, const char* data, size_t size) {
    std::ofstream stream(file, std::ios::binary);
    stream.write(data, size);
  }

  void replaceFileWithText(const File& file, const std::string& text) {
    std::ofstream stream(file);
    stream << text;
  }

  bool hasWriteAccess(const File& file) {
    std::filesystem::file_status status = std::filesystem::status(file);
    return (status.permissions() & std::filesystem::perms::owner_write) != std::filesystem::perms::none;
  }

  bool fileExists(const File& file) {
    return std::filesystem::exists(file);
  }

  void appendTextToFile(const File& file, const std::string& text) {
    std::ofstream stream(file, std::ios::app);
    stream << text;
  }

  std::unique_ptr<char[]> loadFileData(const File& file, int& size) {
    std::ifstream stream(file, std::ios::binary | std::ios::ate);
    if (!stream)
      return {};

    size = stream.tellg();
    stream.seekg(0, std::ios::beg);

    auto data = std::make_unique<char[]>(size);
    stream.read(data.get(), size);
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

  File audioPluginFile() {
#if VISAGE_WINDOWS
    HMODULE module = nullptr;
    auto flags = GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT;
    auto status = GetModuleHandleExA(flags, reinterpret_cast<LPCSTR>(&audioPluginFile), &module);
    if (status == 0 || module == nullptr)
      module = GetModuleHandleA(nullptr);

    WCHAR dest[MAX_PATH] {};
    auto result = GetModuleFileNameW((HINSTANCE)module, dest, MAX_PATH);
    if (result == 0)
      return {};

    return dest;
#else
    Dl_info info;
    visage::File file;
    if (dladdr((void*)getAudioPluginFile, &info))
      file = info.dli_fname;

#if VISAGE_MAC
    if (file.has_parent_path()) {
      visage::File parent = file.parent_path();
      if (parent.filename() == "MacOS" && parent.parent_path().filename() == "Contents")
        return parent.parent_path().parent_path();
    }
#endif
    return file;
#endif
  }

  File audioPluginDataFolder() {
#if VISAGE_WINDOWS
    static File path;
    if (path.empty()) {
      char buffer[MAX_PATH];
      SHGetFolderPathA(nullptr, CSIDL_PERSONAL, nullptr, 0, buffer);
      path = buffer;
      path = path / VISAGE_APPLICATION_NAME;
    }
    return path;
#elif VISAGE_MAC
    return "~/Music/" VISAGE_APPLICATION_NAME;
#elif VISAGE_LINUX
    File path = xdgFolder("XDG_DOCUMENTS_DIR", "~/Documents");
    return path / VISAGE_APPLICATION_NAME;
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
