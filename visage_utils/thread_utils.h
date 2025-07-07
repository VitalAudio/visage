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

#include "defines.h"
#include "time_utils.h"

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace visage {
  class Thread {
  public:
    static inline std::thread::id main_thread_id_;
    static inline bool main_thread_set_ = false;

    static void setAsMainThread() {
      main_thread_set_ = true;
      main_thread_id_ = std::this_thread::get_id();
    }
    
    static bool isMainThread() {
      return !main_thread_set_ || main_thread_id_ == std::this_thread::get_id();
    }

    explicit Thread(std::string name) : name_(std::move(name)) { }
    explicit Thread() = default;
    virtual ~Thread() {
      // Stop thread before the end of inherited destructor
      VISAGE_ASSERT(!running());
      stop();
    }

    virtual void run() {
      if (task_)
        task_();
    }

    void start() {
      VISAGE_ASSERT(!running());
#if VISAGE_EMSCRIPTEN
      VISAGE_ASSERT(false);
#endif

      if (running())
        return;

      should_run_ = true;
      completed_ = false;
      thread_ = std::make_unique<std::thread>(&Thread::startRun, this);
    }

    void stop() {
      should_run_ = false;
      if (thread_) {
        if (thread_->joinable())
          thread_->join();
        thread_.reset();
      }
    }

    static void sleep(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
    static void sleepUs(int us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }
    static void yield() { std::this_thread::yield(); }

    void setThreadTask(std::function<void()> task) { task_ = std::move(task); }

    bool waitForEnd(int ms_timeout) {
      long long ms_start = time::milliseconds();
      while (!completed()) {
        if (time::milliseconds() - ms_start > ms_timeout)
          return false;

        yield();
      }
      stop();
      return true;
    }

    const std::string& name() const { return name_; }
    bool shouldRun() const { return should_run_.load(); }
    bool running() const { return thread_ && thread_->joinable(); }
    bool completed() const { return completed_.load(); }

  private:
    void startRun() {
      run();
      completed_ = true;
    }

    std::string name_;
    std::atomic<bool> completed_ = true;
    std::atomic<bool> should_run_ = true;
    std::function<void()> task_;
    std::unique_ptr<std::thread> thread_;
  };
}