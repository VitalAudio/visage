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

#include "visage_graphics/emoji.h"

#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#if VISAGE_WINDOWS
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <objbase.h>
#include <process.h>
#include <windows.h>

using namespace visage;

namespace {
  const wchar_t* kResultVariable = L"VISAGE_WIN32_EMOJI_RESULT";
  const wchar_t* kApartmentVariable = L"VISAGE_WIN32_EMOJI_APARTMENT";
  const wchar_t* kChildFilter = L"[.win32_emoji_com_child]";

  wchar_t child_result_path[MAX_PATH * 4] = {};
  HRESULT child_initial_result = E_FAIL;
  int child_drawn_pixels = -1;

  DWORD requestedApartment() {
    wchar_t mode[16] = {};
    if (GetEnvironmentVariableW(kApartmentVariable, mode, 16) && std::wcscmp(mode, L"sta") == 0)
      return COINIT_APARTMENTTHREADED;
    return COINIT_MULTITHREADED;
  }

  void writeChildResult() {
    HRESULT final_result = CoInitializeEx(nullptr, requestedApartment());
    if (SUCCEEDED(final_result))
      CoUninitialize();

    char text[128];
    std::snprintf(text, sizeof(text), "%08lx %d %08lx", static_cast<unsigned long>(child_initial_result),
                  child_drawn_pixels, static_cast<unsigned long>(final_result));
    HANDLE file = CreateFileW(child_result_path, GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
                              FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file != INVALID_HANDLE_VALUE) {
      DWORD written = 0;
      WriteFile(file, text, static_cast<DWORD>(std::strlen(text)), &written, nullptr);
      CloseHandle(file);
    }
  }

  struct ChildResult {
    bool ran = false;
    unsigned long initial_result = 0;
    int drawn_pixels = -1;
    unsigned long final_result = 0;
  };

  ChildResult runChild(const wchar_t* apartment) {
    ChildResult result;
    std::vector<wchar_t> buffer(32768);
    DWORD length = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
    REQUIRE(length > 0);
    REQUIRE(length < buffer.size());
    std::wstring self(buffer.data(), length);

    std::wstring result_name = std::to_wstring(_getpid()) + L"_visage_win32_emoji_result.txt";
    std::filesystem::path result_file = std::filesystem::temp_directory_path() / result_name;
    std::error_code error;
    std::filesystem::remove(result_file, error);

    std::wstring command_line = L"\"" + self + L"\" \"" + kChildFilter + L"\"";
    STARTUPINFOW startup_info = {};
    startup_info.cb = sizeof(startup_info);
    PROCESS_INFORMATION process_info = {};
    SetEnvironmentVariableW(kResultVariable, result_file.c_str());
    SetEnvironmentVariableW(kApartmentVariable, apartment);
    BOOL created = CreateProcessW(self.c_str(), command_line.data(), nullptr, nullptr, FALSE,
                                  CREATE_NO_WINDOW, nullptr, nullptr, &startup_info, &process_info);
    SetEnvironmentVariableW(kResultVariable, nullptr);
    SetEnvironmentVariableW(kApartmentVariable, nullptr);
    REQUIRE(created);

    DWORD wait = WaitForSingleObject(process_info.hProcess, 60000);
    if (wait != WAIT_OBJECT_0) {
      TerminateProcess(process_info.hProcess, 1);
      WaitForSingleObject(process_info.hProcess, INFINITE);
    }
    CloseHandle(process_info.hThread);
    CloseHandle(process_info.hProcess);
    REQUIRE(wait == WAIT_OBJECT_0);

    std::ifstream stream(result_file);
    std::string initial_text, final_text;
    if (stream >> initial_text >> result.drawn_pixels >> final_text) {
      result.ran = true;
      result.initial_result = std::stoul(initial_text, nullptr, 16);
      result.final_result = std::stoul(final_text, nullptr, 16);
    }
    stream.close();
    std::filesystem::remove(result_file, error);
    return result;
  }
}

TEST_CASE("Win32 emoji rasterizer in a multithreaded apartment draws and leaves COM initialized",
          "[graphics]") {
  ChildResult result = runChild(L"mta");
  REQUIRE(result.ran);
  REQUIRE(result.initial_result == static_cast<unsigned long>(S_OK));
  CHECK(result.drawn_pixels > 0);
  CHECK(result.final_result == static_cast<unsigned long>(S_FALSE));
}

TEST_CASE("Win32 emoji rasterizer in a single-threaded apartment draws and balances COM",
          "[graphics]") {
  ChildResult result = runChild(L"sta");
  REQUIRE(result.ran);
  REQUIRE(result.initial_result == static_cast<unsigned long>(S_OK));
  CHECK(result.drawn_pixels > 0);
  CHECK(result.final_result == static_cast<unsigned long>(S_FALSE));
}

TEST_CASE("COM apartment emoji rasterizer child", "[.win32_emoji_com_child]") {
  DWORD length = GetEnvironmentVariableW(kResultVariable, child_result_path, MAX_PATH * 4);
  if (length == 0)
    SKIP("Only runs in the copy the Win32 emoji rasterizer tests start");
  REQUIRE(length < MAX_PATH * 4);

  child_initial_result = CoInitializeEx(nullptr, requestedApartment());
  std::atexit(writeChildResult);

  static constexpr int kSize = 32;
  std::vector<unsigned int> buffer(kSize * kSize, 0);
  EmojiRasterizer::instance().drawIntoBuffer(U'\U0001F600', 24, kSize, buffer.data(), kSize, 0, 0);

  int drawn = 0;
  for (unsigned int pixel : buffer)
    drawn += pixel != 0;
  child_drawn_pixels = drawn;
}

#endif
