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

#include "visage_utils/defines.h"
#include "visage_utils/keycodes.h"
#include "visage_utils/space.h"

#include <functional>
#include <string>

namespace visage {
  class UiFrame;

  class EventTimer {
  public:
    EventTimer() = default;
    virtual ~EventTimer();

    void startTimer(int ms);
    void stopTimer();
    bool checkTimer(long long current_time);
    virtual void timerCallback() = 0;
    bool isRunning() const {
      VISAGE_ASSERT(ms_ >= -1);
      return ms_ > 0;
    }

  private:
    int ms_ = 0;
    long long last_run_time_ = 0;
  };

  class EventManager {
  public:
    static EventManager& getInstance() {
      static EventManager instance;
      return instance;
    }

    void addTimer(EventTimer* timer);
    void removeTimer(const EventTimer* timer);
    void addCallback(std::function<void()> function);
    void checkEventTimers();

  private:
    EventManager() = default;
    ~EventManager() = default;

    std::vector<EventTimer*> timers_;
    std::vector<std::function<void()>> callbacks_;
  };

  static void runOnEventThread(std::function<void()> function) {
    EventManager::getInstance().addCallback(function);
  }

  struct MouseEvent {
    Point getPosition() const { return position; }
    Point getRelativePosition() const { return relative_position; }
    Point getWindowPosition() const { return window_position; }
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
    int getRepeatClickCount() const { return repeat_click_count; }

    bool isLeftButtonCurrentlyDown() const { return button_state & kMouseButtonLeft; }
    bool isMiddleButtonCurrentlyDown() const { return button_state & kMouseButtonMiddle; }
    bool isRightButtonCurrentlyDown() const { return button_state & kMouseButtonRight; }

    bool isLeftButton() const { return button_id == kMouseButtonLeft; }
    bool isMiddleButton() const { return button_id == kMouseButtonMiddle; }
    bool isRightButton() const { return button_id == kMouseButtonRight; }

    MouseEvent relativeTo(const UiFrame* new_frame) const;

    bool shouldTriggerPopup() const {
      return isRightButton() || (isLeftButton() && isMainModifier());
    }

    const UiFrame* frame = nullptr;
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

    KeyCode getKeyCode() const { return key_code; }
    bool isAltDown() const { return modifiers & kModifierAlt; }
    bool isShiftDown() const { return modifiers & kModifierShift; }
    bool isRegCtrlDown() const { return modifiers & kModifierRegCtrl; }
    bool isMacCtrlDown() const { return modifiers & kModifierMacCtrl; }
    bool isCtrlDown() const { return isRegCtrlDown() || isMacCtrlDown(); }
    bool isCmdDown() const { return modifiers & kModifierCmd; }
    bool isMetaDown() const { return modifiers & kModifierMeta; }
    bool isOptionDown() const { return modifiers & kModifierAlt; }
    int getModifierMask() const { return modifiers; }
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
