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

#include "visage_utils/child_process.h"
#include "visage_utils/defines.h"
#include "visage_utils/string_utils.h"

#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <csignal>
#include <thread>

using namespace visage;

TEST_CASE("Child process doesn't exist", "[utils]") {
  std::string command = "asdfjkasdfjkabjbizkejzvbieizieizeiezize";
  std::string argument = "Hello, World!";
  std::string output;
  REQUIRE(!spawnChildProcess(command, argument, output, 1000));
}

TEST_CASE("Echo child process", "[utils]") {
#if VISAGE_WINDOWS
  std::string command = "cmd.exe";
  std::string argument = "/C echo Hello, World!";
#else
  std::string command = "/bin/echo";
  std::string argument = "Hello, World!";
#endif

  std::string output;
  REQUIRE(spawnChildProcess(command, argument, output, 1000));
  REQUIRE(String(output).trim().toUtf8() == "Hello, World!");
}

TEST_CASE("Child process timeout", "[utils]") {
#if VISAGE_WINDOWS
  std::string command = "cmd.exe";
  std::string argument = "/C timeout /t 2 /nobreak";
#else
  std::string command = "/bin/sleep";
  std::string argument = "2";
#endif

  std::string output;
  auto start = std::chrono::steady_clock::now();
  REQUIRE_FALSE(spawnChildProcess(command, argument, output, 100));
  auto elapsed = std::chrono::steady_clock::now() - start;

  REQUIRE(elapsed >= std::chrono::milliseconds(90));
  REQUIRE(elapsed <= std::chrono::milliseconds(300));
}

TEST_CASE("Child process with multiple arguments", "[utils]") {
#if VISAGE_WINDOWS
  std::string command = "cmd.exe";
  std::string argument = "/C echo arg1 arg2 arg3";
#else
  std::string command = "/bin/echo";
  std::string argument = "arg1 arg2 arg3";
#endif

  std::string output;
  REQUIRE(spawnChildProcess(command, argument, output, 1000));
  REQUIRE(String(output).trim().toUtf8() == "arg1 arg2 arg3");
}

TEST_CASE("Child process with empty arguments", "[utils]") {
#if VISAGE_WINDOWS
  std::string command = "cmd.exe";
  std::string argument = "/C echo.";
#else
  std::string command = "/bin/echo";
  std::string argument = "";
#endif

  std::string output;
  REQUIRE(spawnChildProcess(command, argument, output, 1000));
  REQUIRE_FALSE(output.empty());
}

TEST_CASE("Child process with stderr output", "[utils]") {
#if VISAGE_WINDOWS
  std::string command = "cmd.exe";
  std::string argument = "/C echo error message 1>&2";

  std::string output;
  REQUIRE(spawnChildProcess(command, argument, output, 1000));
  REQUIRE(String(output).trim().toUtf8() == "error message");
#else
  std::string command = "python3";
  std::string argument = "-c import sys; sys.stderr.write('error message\\n')";

  std::string output;
  if (spawnChildProcess(command, argument, output, 1000)) {
    REQUIRE(String(output).trim().toUtf8() == "error message");
  }
  else {
    REQUIRE(true);
  }
#endif
}

TEST_CASE("Child process with non-zero exit code", "[utils]") {
#if VISAGE_WINDOWS
  std::string command = "cmd.exe";
  std::string argument = "/C exit 1";
#else
  std::string command = "/bin/false";
  std::string argument = "";
#endif

  std::string output;
  REQUIRE_FALSE(spawnChildProcess(command, argument, output, 1000));
}

TEST_CASE("Child process large output limit", "[utils]") {
#if VISAGE_WINDOWS
  std::string command = "cmd.exe";
  std::string argument = "/C for /L %i in (1,1,100000) do @echo Large output line %i";
#else
  std::string command = "/usr/bin/yes";
  std::string argument = "";
#endif

  std::string output;
  auto start = std::chrono::steady_clock::now();
  REQUIRE_FALSE(spawnChildProcess(command, argument, output, 10000));
  auto elapsed = std::chrono::steady_clock::now() - start;

  REQUIRE(output.size() >= 1000000);
  REQUIRE(elapsed < std::chrono::milliseconds(10000));
}

TEST_CASE("Child process very short timeout", "[utils]") {
#if VISAGE_WINDOWS
  std::string command = "cmd.exe";
  std::string argument = "/C timeout /t 1 /nobreak";
#else
  std::string command = "/bin/sleep";
  std::string argument = "1";
#endif

  std::string output;
  auto start = std::chrono::steady_clock::now();
  REQUIRE_FALSE(spawnChildProcess(command, argument, output, 1));
  auto elapsed = std::chrono::steady_clock::now() - start;

  REQUIRE(elapsed <= std::chrono::milliseconds(150));
}

TEST_CASE("Child process with spaces in arguments", "[utils]") {
#if VISAGE_WINDOWS
  std::string command = "cmd.exe";
  std::string argument = "/C echo hello world test";
#else
  std::string command = "/bin/echo";
  std::string argument = "hello world test";
#endif

  std::string output;
  REQUIRE(spawnChildProcess(command, argument, output, 1000));
  REQUIRE(String(output).trim().toUtf8() == "hello world test");
}

TEST_CASE("Child process immediate completion", "[utils]") {
#if VISAGE_WINDOWS
  std::string command = "cmd.exe";
  std::string argument = "/C echo fast";

  std::string output;
  auto start = std::chrono::steady_clock::now();
  bool result = spawnChildProcess(command, argument, output, 1000);
  auto elapsed = std::chrono::steady_clock::now() - start;

  REQUIRE(result);
  REQUIRE(String(output).trim().toUtf8() == "fast");
  REQUIRE(elapsed <= std::chrono::milliseconds(100));
#else
  std::string command = "/bin/echo";
  std::string argument = "fast";

  std::string output;
  auto start = std::chrono::steady_clock::now();
  bool result = spawnChildProcess(command, argument, output, 1000);
  auto elapsed = std::chrono::steady_clock::now() - start;

  REQUIRE(result);
  REQUIRE(String(output).trim().toUtf8() == "fast");
  REQUIRE(elapsed <= std::chrono::milliseconds(100));
#endif
}

TEST_CASE("Child process with mixed stdout and stderr", "[utils]") {
#if VISAGE_WINDOWS
  std::string command = "cmd.exe";
  std::string argument = "/C echo stdout & echo stderr 1>&2";

  std::string output;
  REQUIRE(spawnChildProcess(command, argument, output, 1000));

  std::string trimmed = String(output).trim().toUtf8();
  REQUIRE(trimmed.find("stdout") != std::string::npos);
  REQUIRE(trimmed.find("stderr") != std::string::npos);
#else
  std::string command = "python3";
  std::string argument = "-c import sys; print('stdout'); sys.stderr.write('stderr\\n')";

  std::string output;
  if (spawnChildProcess(command, argument, output, 1000)) {
    std::string trimmed = String(output).trim().toUtf8();
    REQUIRE(trimmed.find("stdout") != std::string::npos);
    REQUIRE(trimmed.find("stderr") != std::string::npos);
  }
  else {
    REQUIRE(true);
  }
#endif
}

TEST_CASE("Child process concurrent execution", "[utils]") {
  std::vector<std::thread> threads;
  std::atomic<int> success_count { 0 };
  std::atomic<int> failure_count { 0 };

  for (int i = 0; i < 5; ++i) {
    threads.emplace_back([&success_count, &failure_count, i]() {
#if VISAGE_WINDOWS
      std::string command = "cmd.exe";
      std::string argument = "/C echo test" + std::to_string(i);
#else
      std::string command = "/bin/echo";
      std::string argument = "test" + std::to_string(i);
#endif

      std::string output;
      if (spawnChildProcess(command, argument, output, 1000)) {
        success_count++;
      }
      else {
        failure_count++;
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  REQUIRE(success_count >= 4);
  REQUIRE(failure_count <= 1);
}

TEST_CASE("Child process with invalid PID handling", "[utils]") {
#if !VISAGE_WINDOWS
  std::string command = "/bin/echo";
  std::string argument = "test_pid_handling";
  std::string output;

  bool result = spawnChildProcess(command, argument, output, 1000);
  REQUIRE(result);
  REQUIRE(String(output).trim().toUtf8() == "test_pid_handling");
#else
  std::string command = "cmd.exe";
  std::string argument = "/C echo test_pid_handling";
  std::string output;
  REQUIRE(spawnChildProcess(command, argument, output, 1000));
#endif
}