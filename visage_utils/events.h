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

#include <algorithm>
#include <functional>
#include <memory>

namespace visage {
  static constexpr int kUnprintableKeycodeMask = 1 << 30;

  enum MouseButton {
    kMouseButtonNone = 0,
    kMouseButtonLeft = 1 << 0,
    kMouseButtonMiddle = 1 << 1,
    kMouseButtonRight = 1 << 2,
    kMouseButtonTouch = 1 << 3
  };

  enum class MouseCursor {
    Invisible,
    Arrow,
    IBeam,
    Crosshair,
    Pointing,
    Dragging,
    HorizontalResize,
    VerticalResize,
    TopLeftResize,
    TopRightResize,
    BottomLeftResize,
    BottomRightResize,
    MultiDirectionalResize,
  };

  enum Modifiers {
    kModifierNone = 0,
    kModifierShift = 1 << 0,
    kModifierRegCtrl = 1 << 1,
    kModifierMacCtrl = 1 << 2,
    kModifierAlt = 1 << 3,
    kModifierOption = 1 << 3,
    kModifierCmd = 1 << 4,
    kModifierMeta = 1 << 5,
  };

  enum class HitTestResult {
    Client,
    TitleBar,
    CloseButton,
    MinimizeButton,
    MaximizeButton,
  };

  enum class KeyCode {
    Unknown = 0,
    A = 'a',
    B = 'b',
    C = 'c',
    D = 'd',
    E = 'e',
    F = 'f',
    G = 'g',
    H = 'h',
    I = 'i',
    J = 'j',
    K = 'k',
    L = 'l',
    M = 'm',
    N = 'n',
    O = 'o',
    P = 'p',
    Q = 'q',
    R = 'r',
    S = 's',
    T = 't',
    U = 'u',
    V = 'v',
    W = 'w',
    X = 'x',
    Y = 'y',
    Z = 'z',
    Number1 = '1',
    Number2 = '2',
    Number3 = '3',
    Number4 = '4',
    Number5 = '5',
    Number6 = '6',
    Number7 = '7',
    Number8 = '8',
    Number9 = '9',
    Number0 = '0',
    Return = '\n',
    Escape = '\x1B',
    Backspace = '\b',
    Tab = '\t',
    Space = ' ',
    Minus = '-',
    Equals = '=',
    LeftBracket = '[',
    RightBracket = ']',
    Backslash = '\\',
    NonUSHash = 0,
    Semicolon = ';',
    Apostrophe = '\'',
    Grave = '`',
    Comma = ',',
    Period = '.',
    Slash = '/',
    CapsLock = kUnprintableKeycodeMask | 1,
    F1 = kUnprintableKeycodeMask | 2,
    F2 = kUnprintableKeycodeMask | 3,
    F3 = kUnprintableKeycodeMask | 4,
    F4 = kUnprintableKeycodeMask | 5,
    F5 = kUnprintableKeycodeMask | 6,
    F6 = kUnprintableKeycodeMask | 7,
    F7 = kUnprintableKeycodeMask | 8,
    F8 = kUnprintableKeycodeMask | 9,
    F9 = kUnprintableKeycodeMask | 10,
    F10 = kUnprintableKeycodeMask | 11,
    F11 = kUnprintableKeycodeMask | 12,
    F12 = kUnprintableKeycodeMask | 13,
    PrintScreen = kUnprintableKeycodeMask | 14,
    ScrollLock = kUnprintableKeycodeMask | 15,
    Pause = kUnprintableKeycodeMask | 16,
    Insert = kUnprintableKeycodeMask | 17,
    Home = kUnprintableKeycodeMask | 18,
    PageUp = kUnprintableKeycodeMask | 19,
    Delete = kUnprintableKeycodeMask | 20,
    End = kUnprintableKeycodeMask | 21,
    PageDown = kUnprintableKeycodeMask | 22,
    Right = kUnprintableKeycodeMask | 23,
    Left = kUnprintableKeycodeMask | 24,
    Down = kUnprintableKeycodeMask | 25,
    Up = kUnprintableKeycodeMask | 26,
    NumLock = kUnprintableKeycodeMask | 27,
    KPDivide = kUnprintableKeycodeMask | 28,
    KPMultiply = kUnprintableKeycodeMask | 29,
    KPMinus = kUnprintableKeycodeMask | 30,
    KPPlus = kUnprintableKeycodeMask | 31,
    KPEnter = kUnprintableKeycodeMask | 32,
    KP1 = kUnprintableKeycodeMask | 33,
    KP2 = kUnprintableKeycodeMask | 34,
    KP3 = kUnprintableKeycodeMask | 35,
    KP4 = kUnprintableKeycodeMask | 36,
    KP5 = kUnprintableKeycodeMask | 37,
    KP6 = kUnprintableKeycodeMask | 38,
    KP7 = kUnprintableKeycodeMask | 39,
    KP8 = kUnprintableKeycodeMask | 40,
    KP9 = kUnprintableKeycodeMask | 41,
    KP0 = kUnprintableKeycodeMask | 42,
    KPPeriod = kUnprintableKeycodeMask | 43,
    NonUSBackslash = kUnprintableKeycodeMask | 44,
    Application = kUnprintableKeycodeMask | 45,
    Power = kUnprintableKeycodeMask | 46,
    KPEquals = kUnprintableKeycodeMask | 47,
    F13 = kUnprintableKeycodeMask | 48,
    F14 = kUnprintableKeycodeMask | 49,
    F15 = kUnprintableKeycodeMask | 50,
    F16 = kUnprintableKeycodeMask | 51,
    F17 = kUnprintableKeycodeMask | 52,
    F18 = kUnprintableKeycodeMask | 53,
    F19 = kUnprintableKeycodeMask | 54,
    F20 = kUnprintableKeycodeMask | 55,
    F21 = kUnprintableKeycodeMask | 56,
    F22 = kUnprintableKeycodeMask | 57,
    F23 = kUnprintableKeycodeMask | 58,
    F24 = kUnprintableKeycodeMask | 59,
    Execute = kUnprintableKeycodeMask | 60,
    Help = kUnprintableKeycodeMask | 61,
    Menu = kUnprintableKeycodeMask | 62,
    Select = kUnprintableKeycodeMask | 63,
    Stop = kUnprintableKeycodeMask | 64,
    Again = kUnprintableKeycodeMask | 65,
    Undo = kUnprintableKeycodeMask | 66,
    Cut = kUnprintableKeycodeMask | 67,
    Copy = kUnprintableKeycodeMask | 68,
    Paste = kUnprintableKeycodeMask | 69,
    Find = kUnprintableKeycodeMask | 70,
    Mute = kUnprintableKeycodeMask | 71,
    VolumeUp = kUnprintableKeycodeMask | 72,
    VolumeDown = kUnprintableKeycodeMask | 73,
    LockingCapsLock = kUnprintableKeycodeMask | 74,
    LockingNumLock = kUnprintableKeycodeMask | 75,
    LockingScrollLock = kUnprintableKeycodeMask | 76,
    KPComma = kUnprintableKeycodeMask | 77,
    KPEqualsAS400 = kUnprintableKeycodeMask | 78,
    International1 = kUnprintableKeycodeMask | 79,
    International2 = kUnprintableKeycodeMask | 80,
    International3 = kUnprintableKeycodeMask | 81,
    International4 = kUnprintableKeycodeMask | 82,
    International5 = kUnprintableKeycodeMask | 83,
    International6 = kUnprintableKeycodeMask | 84,
    International7 = kUnprintableKeycodeMask | 85,
    International8 = kUnprintableKeycodeMask | 86,
    International9 = kUnprintableKeycodeMask | 87,
    Lang1 = kUnprintableKeycodeMask | 88,
    Lang2 = kUnprintableKeycodeMask | 89,
    Lang3 = kUnprintableKeycodeMask | 90,
    Lang4 = kUnprintableKeycodeMask | 91,
    Lang5 = kUnprintableKeycodeMask | 92,
    Lang6 = kUnprintableKeycodeMask | 93,
    Lang7 = kUnprintableKeycodeMask | 94,
    Lang8 = kUnprintableKeycodeMask | 95,
    Lang9 = kUnprintableKeycodeMask | 96,
    AltErase = kUnprintableKeycodeMask | 97,
    SysReq = kUnprintableKeycodeMask | 98,
    Cancel = kUnprintableKeycodeMask | 99,
    Clear = kUnprintableKeycodeMask | 100,
    Prior = kUnprintableKeycodeMask | 101,
    Return2 = kUnprintableKeycodeMask | 102,
    Separator = kUnprintableKeycodeMask | 103,
    Out = kUnprintableKeycodeMask | 104,
    Oper = kUnprintableKeycodeMask | 105,
    ClearAgain = kUnprintableKeycodeMask | 106,
    CrSel = kUnprintableKeycodeMask | 107,
    ExSel = kUnprintableKeycodeMask | 108,
    KP00 = kUnprintableKeycodeMask | 109,
    KP000 = kUnprintableKeycodeMask | 110,
    ThousandsSeparator = kUnprintableKeycodeMask | 111,
    DecimalSeparator = kUnprintableKeycodeMask | 112,
    CurrencyUnit = kUnprintableKeycodeMask | 113,
    CurrencySubunit = kUnprintableKeycodeMask | 114,
    KPLeftParen = kUnprintableKeycodeMask | 115,
    KPRightParen = kUnprintableKeycodeMask | 116,
    KPLeftBrace = kUnprintableKeycodeMask | 117,
    KPRightBrace = kUnprintableKeycodeMask | 118,
    KPTab = kUnprintableKeycodeMask | 119,
    KPBackspace = kUnprintableKeycodeMask | 120,
    KPA = kUnprintableKeycodeMask | 121,
    KPB = kUnprintableKeycodeMask | 122,
    KPC = kUnprintableKeycodeMask | 123,
    KPD = kUnprintableKeycodeMask | 124,
    KPE = kUnprintableKeycodeMask | 125,
    KPF = kUnprintableKeycodeMask | 126,
    KPXOR = kUnprintableKeycodeMask | 127,
    KPPower = kUnprintableKeycodeMask | 128,
    KPPercent = kUnprintableKeycodeMask | 129,
    KPLess = kUnprintableKeycodeMask | 130,
    KPGreater = kUnprintableKeycodeMask | 131,
    KPAmpersand = kUnprintableKeycodeMask | 132,
    KPDblAmpersand = kUnprintableKeycodeMask | 133,
    KPVerticalBar = kUnprintableKeycodeMask | 134,
    KPDblVerticalBar = kUnprintableKeycodeMask | 135,
    KPColon = kUnprintableKeycodeMask | 136,
    KPHash = kUnprintableKeycodeMask | 137,
    KPSpace = kUnprintableKeycodeMask | 138,
    KPAt = kUnprintableKeycodeMask | 139,
    KPExclam = kUnprintableKeycodeMask | 140,
    KPMemStore = kUnprintableKeycodeMask | 141,
    KPMemRecall = kUnprintableKeycodeMask | 142,
    KPMemClear = kUnprintableKeycodeMask | 143,
    KPMemAdd = kUnprintableKeycodeMask | 144,
    KPMemSubtract = kUnprintableKeycodeMask | 145,
    KPMemMultiply = kUnprintableKeycodeMask | 146,
    KPMemDivide = kUnprintableKeycodeMask | 147,
    KPPlusMinus = kUnprintableKeycodeMask | 148,
    KPClear = kUnprintableKeycodeMask | 149,
    KPClearEntry = kUnprintableKeycodeMask | 150,
    KPBinary = kUnprintableKeycodeMask | 151,
    KPOctal = kUnprintableKeycodeMask | 152,
    KPDecimal = kUnprintableKeycodeMask | 153,
    KPHexadecimal = kUnprintableKeycodeMask | 154,
    LCtrl = kUnprintableKeycodeMask | 155,
    LShift = kUnprintableKeycodeMask | 156,
    LAlt = kUnprintableKeycodeMask | 157,
    LGui = kUnprintableKeycodeMask | 158,
    RCtrl = kUnprintableKeycodeMask | 159,
    RShift = kUnprintableKeycodeMask | 160,
    RAlt = kUnprintableKeycodeMask | 161,
    RGui = kUnprintableKeycodeMask | 162,
    Mode = kUnprintableKeycodeMask | 163,
    AudioNext = kUnprintableKeycodeMask | 164,
    AudioPrev = kUnprintableKeycodeMask | 165,
    AudioStop = kUnprintableKeycodeMask | 166,
    AudioPlay = kUnprintableKeycodeMask | 167,
    AudioMute = kUnprintableKeycodeMask | 168,
    MediaSelect = kUnprintableKeycodeMask | 169,
    WWW = kUnprintableKeycodeMask | 170,
    Mail = kUnprintableKeycodeMask | 171,
    Calculator = kUnprintableKeycodeMask | 172,
    Computer = kUnprintableKeycodeMask | 173,
    ACSearch = kUnprintableKeycodeMask | 174,
    ACHome = kUnprintableKeycodeMask | 175,
    ACBack = kUnprintableKeycodeMask | 176,
    ACForward = kUnprintableKeycodeMask | 177,
    ACStop = kUnprintableKeycodeMask | 178,
    ACRefresh = kUnprintableKeycodeMask | 179,
    ACBookmarks = kUnprintableKeycodeMask | 180,
    BrightnessDown = kUnprintableKeycodeMask | 181,
    BrightnessUp = kUnprintableKeycodeMask | 182,
    DisplaySwitch = kUnprintableKeycodeMask | 183,
    KBDIllumToggle = kUnprintableKeycodeMask | 184,
    KBDIllumDown = kUnprintableKeycodeMask | 185,
    KBDIllumUp = kUnprintableKeycodeMask | 186,
    Eject = kUnprintableKeycodeMask | 187,
    Sleep = kUnprintableKeycodeMask | 188,
    App1 = kUnprintableKeycodeMask | 189,
    App2 = kUnprintableKeycodeMask | 190,
    AudioRewind = kUnprintableKeycodeMask | 191,
    AudioFastForward = kUnprintableKeycodeMask | 192,
  };

  static constexpr bool isPrintableKeyCode(KeyCode key_code) {
    return key_code != KeyCode::Unknown && (static_cast<int>(key_code) & kUnprintableKeycodeMask) == 0;
  }

  template<typename T>
  class CallbackList {
  public:
    template<typename R>
    static R defaultResult() {
      if constexpr (std::is_default_constructible_v<R>)
        return {};
      else
        static_assert(std::is_void_v<R>, "Callback return value must be default constructable");
    }

    CallbackList() = default;

    explicit CallbackList(std::function<T> callback) :
        original_(std::make_unique<std::function<T>>(callback)) {
      add(callback);
    }

    CallbackList(const CallbackList& other) {
      if (other.original_)
        original_ = std::make_unique<std::function<T>>(*other.original_);
      callbacks_ = other.callbacks_;
    }

    CallbackList& operator=(const CallbackList& other) {
      if (this != &other) {
        if (other.original_)
          original_ = std::make_unique<std::function<T>>(*other.original_);
        else
          original_.reset();

        callbacks_ = other.callbacks_;
      }
      return *this;
    }

    void add(std::function<T> callback) { callbacks_.push_back(std::move(callback)); }

    CallbackList& operator+=(std::function<T> callback) {
      add(callback);
      return *this;
    }

    void set(std::function<T> callback) {
      callbacks_.clear();
      callbacks_.push_back(std::move(callback));
    }

    CallbackList& operator=(const std::function<T>& callback) {
      set(callback);
      return *this;
    }

    void remove(const std::function<T>& callback) {
      auto compare = [&](const std::function<T>& other) {
        return other.target_type() == callback.target_type();
      };

      auto it = std::remove_if(callbacks_.begin(), callbacks_.end(), compare);
      callbacks_.erase(it);
    }

    CallbackList& operator-=(const std::function<T>& callback) {
      remove(callback);
      return *this;
    }

    void reset() {
      callbacks_.clear();
      if (original_)
        callbacks_.push_back(*original_);
    }

    void clear() { callbacks_.clear(); }

    template<typename... Args>
    auto callback(Args&&... args) const {
      if (callbacks_.empty())
        return defaultResult<decltype(std::declval<std::function<T>>()(args...))>();

      for (size_t i = 0; i + 1 < callbacks_.size(); ++i)
        callbacks_[i](std::forward<Args>(args)...);

      return callbacks_.back()(std::forward<Args>(args)...);
    }

  private:
    std::unique_ptr<std::function<T>> original_;
    std::vector<std::function<T>> callbacks_;
  };
}
