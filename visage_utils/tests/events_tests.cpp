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

#include "visage_utils/events.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace visage;

TEST_CASE("KeyCode printable check", "[utils]") {
  REQUIRE(isPrintableKeyCode(KeyCode::A));
  REQUIRE(isPrintableKeyCode(KeyCode::Z));
  REQUIRE(isPrintableKeyCode(KeyCode::Number0));
  REQUIRE(isPrintableKeyCode(KeyCode::Number9));
  REQUIRE(isPrintableKeyCode(KeyCode::Space));
  REQUIRE(isPrintableKeyCode(KeyCode::Return));
  REQUIRE(isPrintableKeyCode(KeyCode::Tab));
  REQUIRE(isPrintableKeyCode(KeyCode::Backspace));
  REQUIRE(isPrintableKeyCode(KeyCode::Comma));
  REQUIRE(isPrintableKeyCode(KeyCode::Period));

  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::Unknown));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::F1));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::F12));
  REQUIRE(isPrintableKeyCode(KeyCode::Escape));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::CapsLock));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::Left));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::Right));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::Up));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::Down));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::Insert));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::Delete));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::Home));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::End));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::LCtrl));
  REQUIRE_FALSE(isPrintableKeyCode(KeyCode::RShift));
}

TEST_CASE("CallbackList basic functionality", "[utils]") {
  CallbackList<void()> callbacks;

  bool callback1_called = false;
  bool callback2_called = false;

  callbacks.add([&callback1_called]() { callback1_called = true; });
  callbacks.add([&callback2_called]() { callback2_called = true; });

  callbacks.callback();

  REQUIRE(callback1_called);
  REQUIRE(callback2_called);
}

TEST_CASE("CallbackList with return values", "[utils]") {
  CallbackList<int()> callbacks;

  callbacks.add([]() { return 1; });
  callbacks.add([]() { return 2; });
  callbacks.add([]() { return 3; });

  int result = callbacks.callback();
  REQUIRE(result == 3);
}

TEST_CASE("CallbackList with parameters", "[utils]") {
  CallbackList<void(int&)> callbacks;

  callbacks.add([](int& value) { value += 10; });
  callbacks.add([](int& value) { value *= 2; });

  int test_value = 5;
  callbacks.callback(test_value);

  REQUIRE(test_value == 30);
}

TEST_CASE("CallbackList operator overloads", "[utils]") {
  CallbackList<void()> callbacks;
  bool called = false;

  callbacks += [&called]() { called = true; };
  callbacks.callback();
  REQUIRE(called);

  called = false;
  auto new_callback = [&called]() { called = true; };
  callbacks = new_callback;
  callbacks.callback();
  REQUIRE(called);
}

TEST_CASE("CallbackList set and clear", "[utils]") {
  CallbackList<int()> callbacks;

  callbacks.add([]() { return 1; });
  callbacks.add([]() { return 2; });

  int result = callbacks.callback();
  REQUIRE(result == 2);

  callbacks.set([]() { return 42; });
  result = callbacks.callback();
  REQUIRE(result == 42);

  callbacks.clear();
  result = callbacks.callback();
  REQUIRE(result == 0);
}

TEST_CASE("CallbackList copy constructor", "[utils]") {
  CallbackList<int()> original;
  original.add([]() { return 123; });

  CallbackList<int()> copy(original);
  int result = copy.callback();
  REQUIRE(result == 123);
}

TEST_CASE("CallbackList assignment operator", "[utils]") {
  CallbackList<int()> original;
  original.add([]() { return 456; });

  CallbackList<int()> assigned;
  assigned = original;

  int result = assigned.callback();
  REQUIRE(result == 456);
}

TEST_CASE("CallbackList reset functionality", "[utils]") {
  auto original_callback = []() { return 100; };
  CallbackList<int()> callbacks(original_callback);

  callbacks.add([]() { return 200; });
  int result = callbacks.callback();
  REQUIRE(result == 200);

  callbacks.reset();
  result = callbacks.callback();
  REQUIRE(result == 100);
}

TEST_CASE("CallbackList empty behavior", "[utils]") {
  CallbackList<int()> empty_callbacks;
  int result = empty_callbacks.callback();
  REQUIRE(result == 0);

  CallbackList<void()> empty_void_callbacks;
  empty_void_callbacks.callback();
}

TEST_CASE("CallbackList with complex return types", "[utils]") {
  CallbackList<std::string()> callbacks;

  callbacks.add([]() { return std::string("first"); });
  callbacks.add([]() { return std::string("second"); });
  callbacks.add([]() { return std::string("third"); });

  std::string result = callbacks.callback();
  REQUIRE(result == "third");
}