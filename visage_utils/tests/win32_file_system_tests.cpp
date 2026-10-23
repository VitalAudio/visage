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

#include "visage_utils/defines.h"
#include "visage_utils/file_system.h"

#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <string>
#include <vector>

#if VISAGE_WINDOWS
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <process.h>
#include <windows.h>

using namespace visage;

namespace {
  // A fresh per-process directory under %TEMP%, removed by the destructor.
  struct TempDirectory {
    explicit TempDirectory(const std::wstring& name) {
      path = std::filesystem::temp_directory_path() / (std::to_wstring(_getpid()) + L"_" + name);
      std::filesystem::remove_all(path, error);
      std::filesystem::create_directories(path);
    }
    ~TempDirectory() { std::filesystem::remove_all(path, error); }

    std::filesystem::path path;
    std::error_code error;
  };

  void touch(const std::filesystem::path& path) {
    REQUIRE(replaceFileWithText(path, "contents"));
  }

  // U+4E3B U+673A: outside the Western ANSI code page (1252), and most others.
  const std::wstring kHostStem = L"visage_\u4e3b\u673a";
  const std::string kHostStemUtf8 = "visage_\xe4\xb8\xbb\xe6\x9c\xba";
  const wchar_t* kHostChildFilter = L"[.win32_host_executable_child]";
  const wchar_t* kHostChildVariable = L"VISAGE_WIN32_HOST_EXECUTABLE_CHILD";
}

TEST_CASE("Win32 fileName and fileStem return UTF-8 for names inside the ANSI code page",
          "[utils]") {
  // U+00F1 is in code page 1252, so converting it through the ANSI code page doesn't fail. It just
  // gives the single byte 0xf1 instead of the UTF-8 bytes c3 b1.
  File file = File(L"C:\\visage\\ni\u00f1o.txt");
  REQUIRE(fileName(file) == "ni\xc3\xb1o.txt");
  REQUIRE(fileStem(file) == "ni\xc3\xb1o");
}

TEST_CASE("Win32 searchForDirectories matches directory names outside the ANSI code page",
          "[utils]") {
  TempDirectory directory(L"visage_win32_directories");
  std::filesystem::create_directories(directory.path / L"\u6587\u4ef6\u5939");
  std::filesystem::create_directories(directory.path / L"plain");

  // path::string() throws std::system_error for these names.
  std::vector<File> matches = searchForDirectories(directory.path, "\xe6\x96\x87");
  REQUIRE(matches.size() == 1);
  REQUIRE(matches[0].filename().wstring() == L"\u6587\u4ef6\u5939");
}

TEST_CASE("Win32 file names outside the ANSI code page don't stop searchForFiles", "[utils]") {
  TempDirectory directory(L"visage_win32_files");
  touch(directory.path / L"\u6587\u4ef6.txt");
  touch(directory.path / L"plain.txt");

  // The search is for the ASCII name; the other file only has to not break it.
  std::vector<File> matches = searchForFiles(directory.path, "^plain");
  REQUIRE(matches.size() == 1);
  REQUIRE(fileName(matches[0]) == "plain.txt");
}

TEST_CASE("Win32 hostExecutable and hostName work in a directory outside the ANSI code page",
          "[utils]") {
  std::vector<wchar_t> buffer(32768);
  DWORD length = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
  REQUIRE(length > 0);
  REQUIRE(length < buffer.size());
  std::filesystem::path self(std::wstring(buffer.data(), length));

  TempDirectory directory(L"visage_win32 \u4e3b\u673a");
  std::filesystem::path copy = directory.path / (kHostStem + L".exe");
  std::filesystem::copy_file(self, copy, std::filesystem::copy_options::overwrite_existing);

  // The copy runs only the hidden test below; its exit code is 0 when that test passes.
  std::wstring command_line = L"\"" + copy.wstring() + L"\" \"" + kHostChildFilter + L"\"";
  STARTUPINFOW startup_info = {};
  startup_info.cb = sizeof(startup_info);
  PROCESS_INFORMATION process_info = {};
  SetEnvironmentVariableW(kHostChildVariable, L"1");
  BOOL created = CreateProcessW(copy.c_str(), command_line.data(), nullptr, nullptr, FALSE,
                                CREATE_NO_WINDOW, nullptr, nullptr, &startup_info, &process_info);
  SetEnvironmentVariableW(kHostChildVariable, nullptr);
  REQUIRE(created);

  DWORD wait = WaitForSingleObject(process_info.hProcess, 60000);
  if (wait != WAIT_OBJECT_0) {
    TerminateProcess(process_info.hProcess, 1);
    WaitForSingleObject(process_info.hProcess, INFINITE);
  }
  DWORD exit_code = 1;
  GetExitCodeProcess(process_info.hProcess, &exit_code);
  CloseHandle(process_info.hThread);
  CloseHandle(process_info.hProcess);

  REQUIRE(wait == WAIT_OBJECT_0);
  // GetModuleFileNameA would turn the name into "visage_??", and the child's checks would fail.
  REQUIRE(exit_code == 0);
}

TEST_CASE("Non-ANSI host executable child", "[.win32_host_executable_child]") {
  if (GetEnvironmentVariableW(kHostChildVariable, nullptr, 0) == 0)
    SKIP("Only runs in the copy the Win32 hostExecutable test starts");

  File host = hostExecutable();
  REQUIRE(host.filename().wstring() == kHostStem + L".exe");
  std::error_code error;
  REQUIRE(std::filesystem::exists(host, error));
  REQUIRE(fileExists(host));
  REQUIRE(hostName() == kHostStemUtf8);
}

#endif
