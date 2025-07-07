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

#include "visage_utils/thread_utils.h"

#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>

using namespace visage;

#if !VISAGE_EMSCRIPTEN

TEST_CASE("Thread basic lifecycle", "[utils]") {
  Thread thread("test_thread");

  REQUIRE(thread.name() == "test_thread");
  REQUIRE_FALSE(thread.running());
  REQUIRE(thread.completed());
  REQUIRE(thread.shouldRun());

  std::atomic<bool> task_executed { false };
  thread.setThreadTask([&task_executed]() { task_executed = true; });

  thread.start();
  REQUIRE(thread.running());
  REQUIRE_FALSE(thread.completed());

  REQUIRE(thread.waitForEnd(1000));
  REQUIRE_FALSE(thread.running());
  REQUIRE(thread.completed());
  REQUIRE(task_executed);
}

TEST_CASE("Thread without task", "[utils]") {
  Thread thread;

  thread.start();
  REQUIRE(thread.running());

  REQUIRE(thread.waitForEnd(100));
  REQUIRE_FALSE(thread.running());
  REQUIRE(thread.completed());
}

TEST_CASE("Thread stop before completion", "[utils]") {
  Thread thread;
  std::atomic<bool> should_continue { true };
  std::atomic<int> counter { 0 };

  thread.setThreadTask([&should_continue, &counter, &thread]() {
    while (thread.shouldRun() && should_continue) {
      counter++;
      Thread::sleep(10);
    }
  });

  thread.start();
  Thread::sleep(50);

  should_continue = false;
  thread.stop();

  REQUIRE_FALSE(thread.running());
  REQUIRE(counter > 0);
}

TEST_CASE("Thread timeout on waitForEnd", "[utils]") {
  Thread thread;

  thread.setThreadTask([]() { Thread::sleep(200); });

  thread.start();

  auto start_time = std::chrono::steady_clock::now();
  bool completed = thread.waitForEnd(50);
  auto elapsed = std::chrono::steady_clock::now() - start_time;

  REQUIRE_FALSE(completed);
  REQUIRE(elapsed >= std::chrono::milliseconds(45));
  REQUIRE(elapsed <= std::chrono::milliseconds(100));

  thread.stop();
}

TEST_CASE("Thread sleep functions", "[utils]") {
  auto start_time = std::chrono::steady_clock::now();
  Thread::sleep(50);

  auto elapsed = std::chrono::steady_clock::now() - start_time;
  REQUIRE(elapsed >= std::chrono::milliseconds(45));

  start_time = std::chrono::steady_clock::now();
  Thread::sleepUs(5000);
  elapsed = std::chrono::steady_clock::now() - start_time;
  REQUIRE(elapsed >= std::chrono::microseconds(4500));
}

TEST_CASE("Thread yield function", "[utils]") {
  Thread::yield();
}

TEST_CASE("Multiple thread instances", "[utils]") {
  Thread thread1("thread1");
  Thread thread2("thread2");

  std::atomic<int> shared_counter { 0 };

  thread1.setThreadTask([&shared_counter]() {
    for (int i = 0; i < 100; ++i) {
      shared_counter++;
      Thread::yield();
    }
  });

  thread2.setThreadTask([&shared_counter]() {
    for (int i = 0; i < 100; ++i) {
      shared_counter++;
      Thread::yield();
    }
  });

  thread1.start();
  thread2.start();

  REQUIRE(thread1.waitForEnd(1000));
  REQUIRE(thread2.waitForEnd(1000));

  REQUIRE(shared_counter == 200);
}

TEST_CASE("Thread restart after completion", "[utils]") {
  Thread thread;
  std::atomic<int> execution_count { 0 };

  thread.setThreadTask([&execution_count]() { execution_count++; });

  thread.start();
  REQUIRE(thread.waitForEnd(1000));
  REQUIRE(execution_count == 1);

  thread.start();
  REQUIRE(thread.waitForEnd(1000));
  REQUIRE(execution_count == 2);
}

TEST_CASE("Thread task modification", "[utils]") {
  Thread thread;
  std::atomic<bool> first_task_executed { false };
  std::atomic<bool> second_task_executed { false };

  thread.setThreadTask([&first_task_executed]() { first_task_executed = true; });

  thread.start();
  REQUIRE(thread.waitForEnd(1000));
  REQUIRE(first_task_executed);
  REQUIRE_FALSE(second_task_executed);

  thread.setThreadTask([&second_task_executed]() { second_task_executed = true; });

  thread.start();
  REQUIRE(thread.waitForEnd(1000));
  REQUIRE(second_task_executed);
}

#endif

TEST_CASE("Main thread detection", "[utils]") {
  REQUIRE_FALSE(Thread::main_thread_set_);

  Thread::setAsMainThread();
  REQUIRE(Thread::main_thread_set_);
  REQUIRE(Thread::isMainThread());

#if !VISAGE_EMSCRIPTEN
  Thread thread;
  std::atomic<bool> is_main_in_thread { true };

  thread.setThreadTask([&is_main_in_thread]() { is_main_in_thread = Thread::isMainThread(); });

  thread.start();
  thread.waitForEnd(1000);

  REQUIRE_FALSE(is_main_in_thread);
#endif
}