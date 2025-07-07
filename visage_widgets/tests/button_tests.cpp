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

#include "visage_widgets/button.h"

#include <catch2/catch_test_macros.hpp>

using namespace visage;

TEST_CASE("Button creation and basic properties", "[widgets]") {
  Button button;
  REQUIRE(button.isActive());
  REQUIRE(button.hoverAmount() == 0.0f);
  REQUIRE_FALSE(button.wasAltClicked());
}

TEST_CASE("Button with name constructor", "[widgets]") {
  Button button("test_button");
  REQUIRE(button.isActive());
  REQUIRE(button.name() == "test_button");
}

TEST_CASE("Button active state", "[widgets]") {
  Button button;
  REQUIRE(button.isActive());

  button.setActive(false);
  REQUIRE_FALSE(button.isActive());

  button.setActive(true);
  REQUIRE(button.isActive());
}

TEST_CASE("Button toggle behavior", "[widgets]") {
  Button button;
  REQUIRE_FALSE(button.toggle());
}

TEST_CASE("Button toggle on mouse down setting", "[widgets]") {
  Button button;
  button.setToggleOnMouseDown(true);
  button.setToggleOnMouseDown(false);
}

TEST_CASE("UiButton creation", "[widgets]") {
  UiButton ui_button("Test Text");
  REQUIRE(ui_button.isActive());
}

TEST_CASE("UiButton with empty text", "[widgets]") {
  UiButton ui_button;
  REQUIRE(ui_button.isActive());
}

TEST_CASE("UiButton text setting", "[widgets]") {
  UiButton ui_button;
  ui_button.setText("New Text");
  ui_button.setActionButton(true);
  ui_button.setActionButton(false);
  ui_button.drawBorderWhenInactive(true);
  ui_button.drawBorderWhenInactive(false);
}

TEST_CASE("ToggleButton creation", "[widgets]") {
  ToggleButton toggle_button;
  REQUIRE(toggle_button.isActive());
  REQUIRE_FALSE(toggle_button.toggled());
}

TEST_CASE("ToggleButton with name", "[widgets]") {
  ToggleButton toggle_button("test_toggle");
  REQUIRE(toggle_button.name() == "test_toggle");
  REQUIRE_FALSE(toggle_button.toggled());
}

TEST_CASE("ToggleButton toggle functionality", "[widgets]") {
  ToggleButton toggle_button;
  REQUIRE_FALSE(toggle_button.toggled());

  bool result = toggle_button.toggle();
  REQUIRE(result);
  REQUIRE(toggle_button.toggled());

  result = toggle_button.toggle();
  REQUIRE_FALSE(result);
  REQUIRE_FALSE(toggle_button.toggled());
}

TEST_CASE("ToggleButton setToggled", "[widgets]") {
  ToggleButton toggle_button;
  REQUIRE_FALSE(toggle_button.toggled());

  toggle_button.setToggled(true);
  REQUIRE(toggle_button.toggled());

  toggle_button.setToggled(false);
  REQUIRE_FALSE(toggle_button.toggled());
}

TEST_CASE("ToggleButton undoable setting", "[widgets]") {
  ToggleButton toggle_button;
  toggle_button.setUndoable(true);
  toggle_button.setUndoable(false);
}

TEST_CASE("ToggleTextButton creation", "[widgets]") {
  ToggleTextButton text_button("Test");
  REQUIRE(text_button.isActive());
  REQUIRE_FALSE(text_button.toggled());
}

TEST_CASE("ToggleTextButton text and settings", "[widgets]") {
  ToggleTextButton text_button("Test");
  text_button.setText("New Text");
  text_button.setDrawBackground(true);
  text_button.setDrawBackground(false);
}

TEST_CASE("Button onToggle callback functionality", "[widgets]") {
  Button button;
  bool callback_called = false;
  bool callback_value = false;

  button.onToggle() += [&](Button* b, bool on) {
    callback_called = true;
    callback_value = on;
  };

  button.notify(true);
  REQUIRE(callback_called);
  REQUIRE(callback_value);

  callback_called = false;
  button.notify(false);
  REQUIRE(callback_called);
  REQUIRE_FALSE(callback_value);
}

TEST_CASE("ToggleButton onToggle callback with setToggledAndNotify", "[widgets]") {
  ToggleButton toggle_button;
  bool callback_called = false;
  bool callback_value = false;

  toggle_button.onToggle() += [&](Button* b, bool on) {
    callback_called = true;
    callback_value = on;
  };

  toggle_button.setToggledAndNotify(true);
  REQUIRE(callback_called);
  REQUIRE(callback_value);
  REQUIRE(toggle_button.toggled());

  callback_called = false;
  toggle_button.setToggledAndNotify(false);
  REQUIRE(callback_called);
  REQUIRE_FALSE(callback_value);
  REQUIRE_FALSE(toggle_button.toggled());
}

TEST_CASE("Button onToggle multiple callbacks", "[widgets]") {
  Button button;
  bool callback1_called = false;
  bool callback2_called = false;

  button.onToggle() += [&](Button* b, bool on) { callback1_called = true; };

  button.onToggle() += [&](Button* b, bool on) { callback2_called = true; };

  button.notify(true);
  REQUIRE(callback1_called);
  REQUIRE(callback2_called);
}

TEST_CASE("Button undo setup function", "[widgets]") {
  Button button;
  bool undo_setup_called = false;

  button.setUndoSetupFunction([&]() { undo_setup_called = true; });

  auto undo_func = button.undoSetupFunction();
  REQUIRE(undo_func != nullptr);

  undo_func();
  REQUIRE(undo_setup_called);
}

TEST_CASE("ButtonChangeAction undo/redo functionality", "[widgets]") {
  ToggleButton toggle_button;
  REQUIRE_FALSE(toggle_button.toggled());

  ButtonChangeAction action(&toggle_button, true);

  action.redo();
  REQUIRE(toggle_button.toggled());

  action.undo();
  REQUIRE_FALSE(toggle_button.toggled());

  action.redo();
  REQUIRE(toggle_button.toggled());
}