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

#include <csignal>
#include <iostream>

namespace visage {
  inline void debugLogString(const String& log_message) {
    std::cerr << log_message.toUtf8() << "\n";
  }

  void debugLog(const String& log_message) {
    debugLogString(log_message);
  }

  void debugLog(const char* log_message) {
    debugLogString(log_message);
  }

  void debugLog(long long log_message) {
    debugLogString(log_message);
  }

  void debugLog(unsigned long long log_message) {
    debugLogString(log_message);
  }

  void debugLog(int log_message) {
    debugLogString(log_message);
  }

  void debugLog(unsigned int log_message) {
    debugLogString(log_message);
  }

  void debugLog(float log_message) {
    debugLogString(log_message);
  }

  void debugLog(char log_message) {
    debugLogString(log_message);
  }

  void debugAssert(bool condition) {
    if (condition)
      return;

    ::kill(0, SIGTRAP);
  }

  void forceCrash() {
    ::kill(0, SIGTRAP);
  }
}