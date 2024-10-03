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

    static void setAsMainThread() { main_thread_id_ = std::this_thread::get_id(); }
    static bool isMainThread() {
      return main_thread_id_ == std::thread::id() || std::this_thread::get_id() == main_thread_id_;
    }

    explicit Thread(std::string name) : name_(std::move(name)) { }
    explicit Thread() = default;
    virtual ~Thread() { stop(); }

    virtual void run() {
      if (task_)
        task_();
    }

    void start() {
      VISAGE_ASSERT(!running());

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

    void setThreadTask(std::function<void()> task) { task_ = task; }

    bool waitForEnd(int ms_timeout) {
      long long ms_start = time::getMilliseconds();
      while (!completed()) {
        if (time::getMilliseconds() - ms_start > ms_timeout)
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