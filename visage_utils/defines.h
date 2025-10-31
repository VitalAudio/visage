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

#include <cstdarg>

namespace visage {
  constexpr float kPi = 3.14159265358979323846f;

  class String;
  void debugLogString(const char* file, unsigned int line, const String& log_message);
  void debugLogArgs(const char* file, unsigned int line, const char* format, va_list arg_list);

  template<typename T>
  void debugLog(const char* file, unsigned int line, T message, ...) {
    debugLogString(file, line, message);
  }

  template<>
  inline void debugLog(const char* file, unsigned int line, const char* format, ...) {
    va_list args;
    va_start(args, format);
    debugLogArgs(file, line, format, args);
    va_end(args);
  }

  void debugAssert(bool condition, const char* file, unsigned int line);
  void forceCrash();
}

#define VISAGE_FORCE_CRASH() visage::forceCrash()

#ifndef NDEBUG

#define VISAGE_LOG(log, ...) visage::debugLog(__FILE__, int(__LINE__), log, ##__VA_ARGS__)

#define VISAGE_ASSERT(condition) visage::debugAssert((condition), __FILE__, int(__LINE__))
#define no_except

namespace visage {
  template<typename T>
  class InstanceCounter {
  public:
    static InstanceCounter& instance() {
      static InstanceCounter instance;
      return instance;
    }

    ~InstanceCounter() { VISAGE_ASSERT(count_ == 0); }

    void add() { count_++; }
    void remove() { count_--; }

  private:
    int count_ = 0;
  };

  template<typename T>
  class LeakChecker {
  public:
    LeakChecker() { InstanceCounter<T>::instance().add(); }

    LeakChecker(const LeakChecker& other) { InstanceCounter<T>::instance().add(); }

    ~LeakChecker() { InstanceCounter<T>::instance().remove(); }
  };
}

#define VISAGE_LEAK_CHECKER(className)             \
  friend class visage::InstanceCounter<className>; \
  static const char* vaLeakCheckerName() {         \
    return #className;                             \
  }                                                \
  visage::LeakChecker<className> leakChecker;

#else
#define VISAGE_ASSERT(x) ((void)0)
#define VISAGE_LOG(x, ...) ((void)0)
#define VISAGE_LEAK_CHECKER(className)
#define no_except noexcept
#endif

#if VISAGE_WINDOWS
#define VISAGE_STDCALL __stdcall
#else
#define VISAGE_STDCALL
#endif