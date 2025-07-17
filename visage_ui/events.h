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

#include "visage_utils/defines.h"
#include "visage_utils/events.h"
#include "visage_utils/space.h"

#include <functional>
#include <string>

namespace visage {
  class Frame;

  class EventTimer {
  public:
    EventTimer() = default;
    virtual ~EventTimer();

    auto& onTimerCallback() { return on_timer_callback_; }

    void startTimer(int ms);
    void stopTimer();
    bool checkTimer(long long current_time);
    virtual void timerCallback() { }

    bool isRunning() const {
      VISAGE_ASSERT(ms_ >= -1);
      return ms_ > 0;
    }

  private:
    void notifyTimerCallback() { on_timer_callback_.callback(); }

    CallbackList<void()> on_timer_callback_ { [this]() -> void { timerCallback(); } };
    int ms_ = 0;
    long long last_run_time_ = 0;
  };

  class EventManager {
  public:
    static EventManager& instance() {
      static EventManager instance;
      return instance;
    }

    EventManager(const EventManager&) = delete;
    EventManager& operator=(const EventManager&) = delete;

    void addTimer(EventTimer* timer);
    void removeTimer(const EventTimer* timer);
    void addCallback(std::function<void()> function);
    void checkEventTimers();

  private:
    EventManager() = default;
    ~EventManager() = default;

    std::vector<EventTimer*> timers_ {};
    std::vector<std::function<void()>> callbacks_ {};
  };

  static void runOnEventThread(std::function<void()> function) {
    EventManager::instance().addCallback(std::move(function));
  }

  struct MouseEvent {
    Point relativePosition() const { return relative_position; }
    Point windowPosition() const { return window_position; }
    bool isAltDown() const { return modifiers & kModifierAlt; }
    bool isShiftDown() const { return modifiers & kModifierShift; }
    bool isRegCtrlDown() const { return modifiers & kModifierRegCtrl; }
    bool isMacCtrlDown() const { return modifiers & kModifierMacCtrl; }
    bool isCtrlDown() const { return isRegCtrlDown() || isMacCtrlDown(); }
    bool isCmdDown() const { return modifiers & kModifierCmd; }
    bool isMetaDown() const { return modifiers & kModifierMeta; }
    bool isOptionDown() const { return modifiers & kModifierAlt; }
    bool isMainModifier() const { return isRegCtrlDown() || isCmdDown(); }

    bool isDown() const { return is_down; }
    bool isMouse() const { return !isTouch(); }
    bool isTouch() const { return button_state & kMouseButtonTouch; }
    bool hasWheelMomentum() const { return wheel_momentum; }
    int repeatClickCount() const { return repeat_click_count; }

    bool isLeftButtonCurrentlyDown() const { return button_state & kMouseButtonLeft; }
    bool isMiddleButtonCurrentlyDown() const { return button_state & kMouseButtonMiddle; }
    bool isRightButtonCurrentlyDown() const { return button_state & kMouseButtonRight; }

    bool isLeftButton() const { return button_id == kMouseButtonLeft; }
    bool isMiddleButton() const { return button_id == kMouseButtonMiddle; }
    bool isRightButton() const { return button_id == kMouseButtonRight; }

    MouseEvent relativeTo(const Frame* new_frame) const;

    bool shouldTriggerPopup() const {
      return isRightButton() || (isLeftButton() && isMainModifier());
    }

    const Frame* frame = nullptr;
    Point position;
    Point relative_position;
    Point window_position;
    MouseButton button_id = kMouseButtonNone;
    int button_state = kMouseButtonNone;

    int modifiers = 0;
    bool is_down = false;
    float wheel_delta_x = 0.0f;
    float wheel_delta_y = 0.0f;
    float precise_wheel_delta_x = 0.0f;
    float precise_wheel_delta_y = 0.0f;
    bool wheel_reversed = false;
    bool wheel_momentum = false;
    int repeat_click_count = 0;
  };

  class KeyEvent {
  public:
    KeyEvent(KeyCode key, int mods, bool is_down, bool repeat = false) :
        key_code(key), modifiers(mods), key_down(is_down), is_repeat(repeat) { }

    KeyCode keyCode() const { return key_code; }
    bool isAltDown() const { return modifiers & kModifierAlt; }
    bool isShiftDown() const { return modifiers & kModifierShift; }
    bool isRegCtrlDown() const { return modifiers & kModifierRegCtrl; }
    bool isMacCtrlDown() const { return modifiers & kModifierMacCtrl; }
    bool isCtrlDown() const { return isRegCtrlDown() || isMacCtrlDown(); }
    bool isCmdDown() const { return modifiers & kModifierCmd; }
    bool isMetaDown() const { return modifiers & kModifierMeta; }
    bool isOptionDown() const { return modifiers & kModifierAlt; }
    int modifierMask() const { return modifiers; }
    bool isMainModifier() const { return isRegCtrlDown() || isCmdDown(); }
    bool isRepeat() const { return is_repeat; }

    KeyEvent withMainModifier() const {
      KeyEvent copy = *this;
      copy.modifiers = modifiers | kModifierRegCtrl;
      return copy;
    }

    KeyEvent withMeta() const {
      KeyEvent copy = *this;
      copy.modifiers = modifiers | kModifierMeta;
      return copy;
    }

    KeyEvent withShift() const {
      KeyEvent copy = *this;
      copy.modifiers = modifiers | kModifierShift;
      return copy;
    }

    KeyEvent withAlt() const {
      KeyEvent copy = *this;
      copy.modifiers = modifiers | kModifierAlt;
      return copy;
    }

    KeyEvent withOption() const { return withAlt(); }

    bool operator==(const KeyEvent& other) const {
      return key_code == other.key_code && key_down == other.key_down && modifiers == other.modifiers;
    }

    bool operator!=(const KeyEvent& other) const { return !(*this == other); }

    KeyCode key_code = KeyCode::Unknown;
    int modifiers = 0;
    bool key_down = false;
    bool is_repeat = false;
  };
}
