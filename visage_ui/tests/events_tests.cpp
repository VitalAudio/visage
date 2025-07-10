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

#include "visage_ui/events.h"
#include "visage_ui/frame.h"

#include <catch2/catch_test_macros.hpp>

using namespace visage;

class TestEventTimer : public EventTimer {
public:
  void timerCallback() override { callback_count++; }

  int callback_count = 0;
};

TEST_CASE("EventTimer basic functionality", "[ui]") {
  TestEventTimer timer;

  SECTION("Initial state") {
    REQUIRE_FALSE(timer.isRunning());
  }

  SECTION("Starting timer") {
    timer.startTimer(100);
    REQUIRE(timer.isRunning());
  }

  SECTION("Stopping timer") {
    timer.startTimer(100);
    timer.stopTimer();
    REQUIRE_FALSE(timer.isRunning());
  }
}

TEST_CASE("EventManager functionality", "[ui]") {
  EventManager& manager = EventManager::instance();
  TestEventTimer timer1, timer2;

  SECTION("Singleton pattern") {
    EventManager& manager2 = EventManager::instance();
    REQUIRE(&manager == &manager2);
  }

  SECTION("Adding and removing timers") {
    manager.addTimer(&timer1);
    manager.addTimer(&timer2);

    manager.removeTimer(&timer1);
    manager.removeTimer(&timer2);
  }

  int callback_count = 0;
  SECTION("Adding callbacks") {
    manager.addCallback([&callback_count]() { callback_count++; });
  }

  SECTION("Checking event timers") {
    manager.addTimer(&timer1);
    timer1.startTimer(1);

    manager.checkEventTimers();

    manager.removeTimer(&timer1);
  }
}

TEST_CASE("MouseEvent basic properties", "[ui]") {
  MouseEvent event;

  SECTION("Default values") {
    REQUIRE(event.button_id == kMouseButtonNone);
    REQUIRE(event.button_state == kMouseButtonNone);
    REQUIRE(event.modifiers == 0);
    REQUIRE_FALSE(event.is_down);
    REQUIRE(event.repeat_click_count == 0);
  }

  SECTION("Position properties") {
    event.position = { 10.0f, 20.0f };
    event.relative_position = { 5.0f, 15.0f };
    event.window_position = { 100.0f, 200.0f };

    REQUIRE(event.relativePosition() == Point { 5.0f, 15.0f });
    REQUIRE(event.windowPosition() == Point { 100.0f, 200.0f });
  }

  SECTION("Button state queries") {
    event.button_state = kMouseButtonLeft | kMouseButtonRight;

    REQUIRE(event.isLeftButtonCurrentlyDown());
    REQUIRE_FALSE(event.isMiddleButtonCurrentlyDown());
    REQUIRE(event.isRightButtonCurrentlyDown());
  }

  SECTION("Button identification") {
    event.button_id = kMouseButtonLeft;
    REQUIRE(event.isLeftButton());
    REQUIRE_FALSE(event.isMiddleButton());
    REQUIRE_FALSE(event.isRightButton());

    event.button_id = kMouseButtonMiddle;
    REQUIRE_FALSE(event.isLeftButton());
    REQUIRE(event.isMiddleButton());
    REQUIRE_FALSE(event.isRightButton());

    event.button_id = kMouseButtonRight;
    REQUIRE_FALSE(event.isLeftButton());
    REQUIRE_FALSE(event.isMiddleButton());
    REQUIRE(event.isRightButton());
  }

  SECTION("Touch detection") {
    event.button_state = kMouseButtonTouch;
    REQUIRE(event.isTouch());
    REQUIRE_FALSE(event.isMouse());

    event.button_state = kMouseButtonLeft;
    REQUIRE_FALSE(event.isTouch());
    REQUIRE(event.isMouse());
  }

  SECTION("Wheel properties") {
    event.wheel_momentum = true;
    REQUIRE(event.hasWheelMomentum());

    event.wheel_delta_x = 5.0f;
    event.wheel_delta_y = -3.0f;
    event.precise_wheel_delta_x = 2.5f;
    event.precise_wheel_delta_y = -1.5f;
    event.wheel_reversed = true;
  }
}

TEST_CASE("MouseEvent modifier detection", "[ui]") {
  MouseEvent event;

  SECTION("Alt modifier") {
    event.modifiers = kModifierAlt;
    REQUIRE(event.isAltDown());
    REQUIRE(event.isOptionDown());
    REQUIRE_FALSE(event.isShiftDown());
  }

  SECTION("Shift modifier") {
    event.modifiers = kModifierShift;
    REQUIRE(event.isShiftDown());
    REQUIRE_FALSE(event.isAltDown());
  }

  SECTION("Control modifiers") {
    event.modifiers = kModifierRegCtrl;
    REQUIRE(event.isRegCtrlDown());
    REQUIRE(event.isCtrlDown());
    REQUIRE_FALSE(event.isMacCtrlDown());

    event.modifiers = kModifierMacCtrl;
    REQUIRE(event.isMacCtrlDown());
    REQUIRE(event.isCtrlDown());
    REQUIRE_FALSE(event.isRegCtrlDown());
  }

  SECTION("Command and Meta modifiers") {
    event.modifiers = kModifierCmd;
    REQUIRE(event.isCmdDown());

    event.modifiers = kModifierMeta;
    REQUIRE(event.isMetaDown());
  }

  SECTION("Main modifier detection") {
    event.modifiers = kModifierRegCtrl;
    REQUIRE(event.isMainModifier());

    event.modifiers = kModifierCmd;
    REQUIRE(event.isMainModifier());

    event.modifiers = kModifierAlt;
    REQUIRE_FALSE(event.isMainModifier());
  }

  SECTION("Multiple modifiers") {
    event.modifiers = kModifierAlt | kModifierShift;
    REQUIRE(event.isAltDown());
    REQUIRE(event.isShiftDown());
    REQUIRE_FALSE(event.isCtrlDown());
  }
}

TEST_CASE("MouseEvent popup trigger detection", "[ui]") {
  MouseEvent event;

  SECTION("Right button triggers popup") {
    event.button_id = kMouseButtonRight;
    REQUIRE(event.shouldTriggerPopup());
  }

  SECTION("Left button with main modifier triggers popup") {
    event.button_id = kMouseButtonLeft;
    event.modifiers = kModifierRegCtrl;
    REQUIRE(event.shouldTriggerPopup());

    event.modifiers = kModifierCmd;
    REQUIRE(event.shouldTriggerPopup());
  }

  SECTION("Left button without modifier doesn't trigger popup") {
    event.button_id = kMouseButtonLeft;
    event.modifiers = 0;
    REQUIRE_FALSE(event.shouldTriggerPopup());

    event.modifiers = kModifierAlt;
    REQUIRE_FALSE(event.shouldTriggerPopup());
  }
}

TEST_CASE("KeyEvent construction and properties", "[ui]") {
  KeyEvent event(KeyCode::A, kModifierShift, true, false);

  SECTION("Basic properties") {
    REQUIRE(event.keyCode() == KeyCode::A);
    REQUIRE(event.key_down);
    REQUIRE_FALSE(event.isRepeat());
    REQUIRE(event.modifierMask() == kModifierShift);
  }

  SECTION("Modifier detection") {
    REQUIRE(event.isShiftDown());
    REQUIRE_FALSE(event.isAltDown());
    REQUIRE_FALSE(event.isCtrlDown());
  }
}

TEST_CASE("KeyEvent modifier methods", "[ui]") {
  KeyEvent base_event(KeyCode::A, 0, true);

  SECTION("Adding main modifier") {
    KeyEvent modified = base_event.withMainModifier();
    REQUIRE(modified.isMainModifier());
    REQUIRE(modified.keyCode() == KeyCode::A);
    REQUIRE(modified.key_down);
  }

  SECTION("Adding meta modifier") {
    KeyEvent modified = base_event.withMeta();
    REQUIRE(modified.isMetaDown());
  }

  SECTION("Adding shift modifier") {
    KeyEvent modified = base_event.withShift();
    REQUIRE(modified.isShiftDown());
  }

  SECTION("Adding alt modifier") {
    KeyEvent modified = base_event.withAlt();
    REQUIRE(modified.isAltDown());

    KeyEvent option_modified = base_event.withOption();
    REQUIRE(option_modified.isOptionDown());
  }

  SECTION("Chaining modifiers") {
    KeyEvent modified = base_event.withShift().withAlt();
    REQUIRE(modified.isShiftDown());
    REQUIRE(modified.isAltDown());
  }
}

TEST_CASE("KeyEvent equality", "[ui]") {
  KeyEvent event1(KeyCode::A, kModifierShift, true);
  KeyEvent event2(KeyCode::A, kModifierShift, true);
  KeyEvent event3(KeyCode::B, kModifierShift, true);
  KeyEvent event4(KeyCode::A, kModifierAlt, true);
  KeyEvent event5(KeyCode::A, kModifierShift, false);

  SECTION("Equality comparison") {
    REQUIRE(event1 == event2);
    REQUIRE_FALSE(event1 == event3);
    REQUIRE_FALSE(event1 == event4);
    REQUIRE_FALSE(event1 == event5);
  }

  SECTION("Inequality comparison") {
    REQUIRE_FALSE(event1 != event2);
    REQUIRE(event1 != event3);
    REQUIRE(event1 != event4);
    REQUIRE(event1 != event5);
  }
}

TEST_CASE("KeyEvent modifier detection", "[ui]") {
  SECTION("Alt modifier") {
    KeyEvent event(KeyCode::A, kModifierAlt, true);
    REQUIRE(event.isAltDown());
    REQUIRE(event.isOptionDown());
  }

  SECTION("Control modifiers") {
    KeyEvent reg_ctrl(KeyCode::A, kModifierRegCtrl, true);
    REQUIRE(reg_ctrl.isRegCtrlDown());
    REQUIRE(reg_ctrl.isCtrlDown());
    REQUIRE(reg_ctrl.isMainModifier());

    KeyEvent mac_ctrl(KeyCode::A, kModifierMacCtrl, true);
    REQUIRE(mac_ctrl.isMacCtrlDown());
    REQUIRE(mac_ctrl.isCtrlDown());
  }

  SECTION("Command modifier") {
    KeyEvent event(KeyCode::A, kModifierCmd, true);
    REQUIRE(event.isCmdDown());
    REQUIRE(event.isMainModifier());
  }

  SECTION("Meta modifier") {
    KeyEvent event(KeyCode::A, kModifierMeta, true);
    REQUIRE(event.isMetaDown());
  }

  SECTION("Multiple modifiers") {
    KeyEvent event(KeyCode::A, kModifierShift | kModifierAlt, true);
    REQUIRE(event.isShiftDown());
    REQUIRE(event.isAltDown());
    REQUIRE_FALSE(event.isCtrlDown());
  }
}

TEST_CASE("KeyEvent repeat handling", "[ui]") {
  KeyEvent repeat_event(KeyCode::A, 0, true, true);
  KeyEvent normal_event(KeyCode::A, 0, true, false);

  REQUIRE(repeat_event.isRepeat());
  REQUIRE_FALSE(normal_event.isRepeat());
}