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

#include "child_process.h"

#ifndef NOMINMAX
#define NOMINMAX 1
#endif
#include <sstream>
#include <windows.h>

namespace visage {
  bool spawnChildProcess(const std::string& command, const std::string& arguments,
                         std::string& output, int timeout_ms) {
    STARTUPINFO si;
    PROCESS_INFORMATION pi;

    SECURITY_ATTRIBUTES sa;
    HANDLE std_out_read = nullptr;
    HANDLE std_out_write = nullptr;

    ZeroMemory(&sa, sizeof(sa));
    sa.nLength = sizeof(sa);
    sa.bInheritHandle = TRUE;
    sa.lpSecurityDescriptor = nullptr;

    if (!CreatePipe(&std_out_read, &std_out_write, &sa, 0))
      return false;

    if (!SetHandleInformation(std_out_read, HANDLE_FLAG_INHERIT, 0)) {
      CloseHandle(std_out_read);
      CloseHandle(std_out_write);
      return false;
    }

    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);
    si.hStdError = std_out_write;
    si.hStdOutput = std_out_write;
    si.dwFlags |= STARTF_USESTDHANDLES;
    ZeroMemory(&pi, sizeof(pi));

    std::string full_command = "" + command + " " + arguments;
    if (!CreateProcess(nullptr, const_cast<LPSTR>(full_command.c_str()), nullptr, nullptr, TRUE,
                       CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi)) {
      DWORD error_code = GetLastError();
      char message_buffer[256];
      FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, error_code,
                     0, message_buffer, sizeof(message_buffer), NULL);
      output = message_buffer;

      CloseHandle(std_out_read);
      CloseHandle(std_out_write);
      return false;
    }

    CloseHandle(std_out_write);

    std::ostringstream stream;
    CHAR buffer[4096] {};
    DWORD bytes_read = 0;
    DWORD bytes_available = 0;
    size_t total_output_size = 0;

    auto read_available_output = [&]() {
      if (total_output_size >= kMaxOutputSize)
        return false;

      if (PeekNamedPipe(std_out_read, nullptr, 0, nullptr, &bytes_available, nullptr) &&
          bytes_available > 0) {
        DWORD bytes_to_read = min(bytes_available, sizeof(buffer) - 1);
        if (ReadFile(std_out_read, buffer, bytes_to_read, &bytes_read, nullptr) && bytes_read > 0) {
          bytes_read = std::min<DWORD>(bytes_read, kMaxOutputSize - total_output_size);
          buffer[bytes_read] = '\0';
          stream << buffer;
          total_output_size += bytes_read;
          return true;
        }
      }
      return false;
    };

    DWORD wait_result = WAIT_TIMEOUT;
    auto start_time = GetTickCount64();

    while (wait_result == WAIT_TIMEOUT) {
      wait_result = WaitForSingleObject(pi.hProcess, 0);

      if (read_available_output() && total_output_size >= kMaxOutputSize)
        break;

      if (GetTickCount64() - start_time >= (DWORD)timeout_ms) {
        wait_result = WAIT_TIMEOUT;
        break;
      }

      if (wait_result == WAIT_TIMEOUT)
        Sleep(1);
    }

    while (read_available_output())
      ;

    DWORD exit_code = 0;
    bool success = true;

    if (wait_result == WAIT_TIMEOUT) {
      TerminateProcess(pi.hProcess, 1);
      WaitForSingleObject(pi.hProcess, INFINITE);
      success = false;
    }
    else if (wait_result == WAIT_OBJECT_0)
      success = GetExitCodeProcess(pi.hProcess, &exit_code) && exit_code == 0;
    else
      success = false;

    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    CloseHandle(std_out_read);
    output = stream.str();

    return success;
  }
}
