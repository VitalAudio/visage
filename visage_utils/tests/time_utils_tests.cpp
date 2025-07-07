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

#include "visage_utils/time_utils.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <thread>

using namespace visage::time;

TEST_CASE("Format time with standard formats", "[utils]") {
  auto test_time = std::chrono::system_clock::from_time_t(1609459200);

  std::string date_format = formatTime(test_time, "%Y-%m-%d");
  REQUIRE(date_format.length() == 10);
  REQUIRE(date_format.find("20") != std::string::npos);

  std::string time_format = formatTime(test_time, "%H:%M:%S");
  REQUIRE(time_format.length() == 8);
  REQUIRE(time_format.find(":") != std::string::npos);

  std::string full_format = formatTime(test_time, "%Y-%m-%d %H:%M:%S");
  REQUIRE(full_format.length() == 19);
  REQUIRE(full_format.find(" ") != std::string::npos);
}

TEST_CASE("Format time with custom formats", "[utils]") {
  Time test_time = now();

  std::string year_only = formatTime(test_time, "%Y");
  REQUIRE(year_only.length() == 4);

  std::string month_only = formatTime(test_time, "%m");
  REQUIRE(month_only.length() <= 2);

  std::string weekday = formatTime(test_time, "%A");
  REQUIRE(weekday.length() >= 6);

  std::string empty_format = formatTime(test_time, "");
  REQUIRE(empty_format.empty());

  std::string literal_text = formatTime(test_time, "Current year: %Y");
  REQUIRE(literal_text.find("Current year:") != std::string::npos);
  REQUIRE(literal_text.length() > 13);
}

TEST_CASE("Format time edge cases", "[utils]") {
  auto epoch_time = std::chrono::system_clock::from_time_t(0);
  std::string epoch_formatted = formatTime(epoch_time, "%Y-%m-%d");
  REQUIRE_FALSE(epoch_formatted.empty());

  auto future_time = std::chrono::system_clock::from_time_t(2147483647);
  std::string future_formatted = formatTime(future_time, "%Y");
  REQUIRE_FALSE(future_formatted.empty());

  std::string percent_literal = formatTime(now(), "%%");
  REQUIRE(percent_literal == "%");
}

TEST_CASE("Time consistency", "[utils]") {
  Time time_point = now();
  long long ms_from_function = milliseconds();

  auto time_point_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_point.time_since_epoch())
                           .count();

  long long difference = std::abs(time_point_ms - ms_from_function);
  REQUIRE(difference < 10);
}