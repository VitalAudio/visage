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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

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