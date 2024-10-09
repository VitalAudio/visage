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

#pragma once

#include <chrono>
#include <ctime>
#include <string>

namespace visage::time {
  typedef std::chrono::time_point<std::chrono::system_clock> Time;

  inline long long milliseconds() {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  }

  inline long long microseconds() {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
  }

  inline int seconds() {
    auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::seconds>(now).count();
  }

  inline Time now() {
    return std::chrono::system_clock::now();
  }

  inline std::string formatTime(const Time& time, const char* format_string) {
    static constexpr int kMaxLength = 100;

    auto time_t = std::chrono::system_clock::to_time_t(time);
    char buffer[kMaxLength];
    tm time_info {};
#if VISAGE_WINDOWS
    localtime_s(&time_info, &time_t);
#else
    localtime_r(&time_t, &time_info);
#endif
    std::strftime(buffer, kMaxLength, format_string, &time_info);
    return buffer;
  }
}
