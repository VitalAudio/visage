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

#include "defines.h"

#include "string_utils.h"

#ifndef NOMINMAX
#define NOMINMAX 1
#endif
#include <windows.h>

namespace visage {
  void debugLogString(const char* file, unsigned int line, const String& log_message) {
    OutputDebugStringA((std::string(file) + " (" + std::to_string(line) + ") ").c_str());
    OutputDebugStringW(log_message.toWide().c_str());
    if (log_message.isEmpty() || log_message[log_message.size() - 1] != '\n')
      OutputDebugStringW(L"\n");
  }

  void debugLogArgs(const char* file, unsigned int line, const char* format, va_list arg_list) {
    static constexpr int kSize = 500;
    char buffer[kSize];
    std::vsnprintf(buffer, sizeof(buffer), format, arg_list);
    debugLogString(file, line, buffer);
  }

  void debugAssert(bool condition) {
    if (condition)
      return;

    __debugbreak();
  }

  void forceCrash() {
    __debugbreak();
  }
}
