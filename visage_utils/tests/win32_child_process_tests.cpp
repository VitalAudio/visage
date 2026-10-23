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

  void writeFile(const std::filesystem::path& path, const std::string& contents) {
    std::ofstream stream(path, std::ios::binary);
    stream.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  }

  // spawnChildProcess takes UTF-8, like the rest of visage.
  std::string utf8(const std::filesystem::path& path) {
    return String::convertToUtf8(path.wstring());
  }

  // U+00E9 is in the Western ANSI code page (1252) and U+6587 U+4EF6 aren't. Read through the
  // ANSI code page, the UTF-8 bytes of either turn into different characters, so the child would
  // be handed a path that doesn't exist.
  const std::wstring kNonAnsiName = L"\u00e9 \u6587\u4ef6";

  bool isAscii(const std::wstring& text) {
    for (wchar_t c : text) {
      if (c >= 0x80)
        return false;
    }
    return true;
  }

  // Quotes a path for a child's command line. The output checks use an ASCII path, so they don't
  // depend on how non-ASCII command lines reach the child.
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

  // cmd.exe is a Unicode program, so `type` opens exactly the path it receives.
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

  // Unquoted, CreateProcess would try "<temp>\<pid>_visage.exe" and so on, which don't exist.
  std::string output;
  bool result = spawnChildProcess(utf8(script), "\"two words\" plain", output, 10000);

  REQUIRE(result);
  REQUIRE(String(output).trim().toUtf8() == "[two words] [plain]");
}

TEST_CASE("Win32 child process commands without an extension are still found on the path",
          "[utils]") {
  // The command is always quoted. CreateProcess must still append ".exe" and search for it.
  std::string output;
  REQUIRE(spawnChildProcess("cmd", "/C echo found", output, 10000));
  REQUIRE(String(output).trim().toUtf8() == "found");
}

TEST_CASE("Win32 child process UTF-16 output from cmd /U keeps its NUL bytes", "[utils]") {
  std::string output;
  REQUIRE(spawnChildProcess("cmd.exe", "/U /C echo hi", output, 10000));

  // Stopping at the first NUL byte would leave just "h".
  static const std::string kExpected("h\0i\0\r\0\n\0", 8);
  REQUIRE(output.size() == kExpected.size());
  REQUIRE(output == kExpected);
}

TEST_CASE("Win32 child process binary output with NUL bytes arrives intact", "[utils]") {
  TempDirectory directory(L"visage_win32_child_binary");
  std::filesystem::path file = directory.path / "nul_bytes.bin";

  // 64 KB, so several 4 KB reads. Every 7th byte is NUL. 0x1a (Ctrl+Z) is avoided and the file
  // doesn't start with a byte order mark, so `type` copies it unchanged.
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
