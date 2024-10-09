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

    std::string full_command = "\"" + command + "\" " + arguments;
    if (!CreateProcess(nullptr, const_cast<LPSTR>(full_command.c_str()), nullptr, nullptr, TRUE,
                       CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi)) {
      CloseHandle(std_out_read);
      CloseHandle(std_out_write);
      return false;
    }

    CloseHandle(std_out_write);
    WaitForSingleObject(pi.hProcess, timeout_ms);
    CHAR buffer[256] {};
    DWORD bytes_read = 0;
    std::ostringstream stream;

    while (ReadFile(std_out_read, buffer, sizeof(buffer) - 1, &bytes_read, nullptr) && bytes_read > 0) {
      buffer[bytes_read] = '\0';
      stream << buffer;
    }

    DWORD exit_code;
    bool success = true;
    if (!GetExitCodeProcess(pi.hProcess, &exit_code))
      success = false;
    else {
      if (exit_code)
        success = false;
    }
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
    CloseHandle(std_out_read);
    output = stream.str();

    return success;
  }
}
