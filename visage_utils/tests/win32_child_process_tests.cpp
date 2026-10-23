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

#include "visage_utils/child_process.h"
#include "visage_utils/defines.h"
#include "visage_utils/string_utils.h"

#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <fstream>
#include <string>

#if VISAGE_WINDOWS
#include <process.h>

using namespace visage;

namespace {
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

  void writeFile(const std::filesystem::path& path, const std::string& contents) {
    std::ofstream stream(path, std::ios::binary);
    stream.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  }

  std::string utf8(const std::filesystem::path& path) {
    return String::convertToUtf8(path.wstring());
  }

  const std::wstring kNonAnsiName = L"\u00e9 \u6587\u4ef6";

  bool isAscii(const std::wstring& text) {
    for (wchar_t c : text) {
      if (c >= 0x80)
        return false;
    }
    return true;
  }

  std::string asciiPathArgument(const std::filesystem::path& path) {
    if (!isAscii(path.wstring()))
      SKIP("The temp directory path isn't ASCII; this check needs an ASCII path");
    return "\"" + path.string() + "\"";
  }
}

TEST_CASE("Win32 child processes get arguments outside the ANSI code page unchanged", "[utils]") {
  TempDirectory directory(L"visage_win32_child_args");
  std::filesystem::path file = directory.path / (kNonAnsiName + L".txt");
  writeFile(file, "contents");
  REQUIRE(std::filesystem::exists(file));

  std::string output;
  bool result = spawnChildProcess("cmd.exe", "/C type \"" + utf8(file) + "\"", output, 10000);

  REQUIRE(result);
  REQUIRE(output == "contents");
}

TEST_CASE("Win32 child process scripts in a non-ANSI directory with spaces get quoted arguments",
          "[utils]") {
  TempDirectory directory(L"visage win32 child " + kNonAnsiName);
  std::filesystem::path script = directory.path / L"print args.cmd";
  writeFile(script, "@echo [%~1] [%~2]\r\n");
  REQUIRE(std::filesystem::exists(script));

  std::string output;
  bool result = spawnChildProcess(utf8(script), "\"two words\" plain", output, 10000);

  REQUIRE(result);
  REQUIRE(String(output).trim().toUtf8() == "[two words] [plain]");
}

TEST_CASE("Win32 child process commands without an extension are still found on the path",
          "[utils]") {
  std::string output;
  REQUIRE(spawnChildProcess("cmd", "/C echo found", output, 10000));
  REQUIRE(String(output).trim().toUtf8() == "found");
}

TEST_CASE("Win32 child process UTF-16 output from cmd /U keeps its NUL bytes", "[utils]") {
  std::string output;
  REQUIRE(spawnChildProcess("cmd.exe", "/U /C echo hi", output, 10000));

  static const std::string kExpected("h\0i\0\r\0\n\0", 8);
  REQUIRE(output.size() == kExpected.size());
  REQUIRE(output == kExpected);
}

TEST_CASE("Win32 child process binary output with NUL bytes arrives intact", "[utils]") {
  TempDirectory directory(L"visage_win32_child_binary");
  std::filesystem::path file = directory.path / "nul_bytes.bin";

  std::string contents;
  contents.reserve(64 * 1024);
  for (int i = 0; i < 64 * 1024; ++i) {
    char c = (i % 7 == 0) ? '\0' : static_cast<char>('a' + (i % 26));
    contents.push_back(c);
  }
  contents[0] = 'x';
  writeFile(file, contents);

  std::string output;
  bool result = spawnChildProcess("cmd.exe", "/C type " + asciiPathArgument(file), output, 10000);

  REQUIRE(result);
  REQUIRE(output.size() == contents.size());
  REQUIRE(output == contents);
}

TEST_CASE("Win32 child process missing commands report a readable error", "[utils]") {
  std::string output;
  REQUIRE_FALSE(spawnChildProcess("visage_win32_no_such_command_zq", "", output, 1000));
  REQUIRE_FALSE(output.empty());
  for (char c : output) {
    unsigned char byte = static_cast<unsigned char>(c);
    bool readable = (byte >= 0x20 && byte < 0x7f) || c == '\r' || c == '\n' || c == '\t';
    REQUIRE(readable);
  }
}

#endif
