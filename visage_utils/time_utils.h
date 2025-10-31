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

#pragma once

#include <chrono>
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

  std::string formatTime(const Time& time, const char* format_string);
}
