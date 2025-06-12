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

#if VISAGE_EMSCRIPTEN
#include "windowing_emscripten.h"

#include "visage_utils/time_utils.h"

#include <emscripten/emscripten.h>
#include <emscripten/html5.h>
#include <map>

namespace visage {
  std::string clipboard_text_ = "";
}

extern "C" void pasteCallback(const char* text) {
  visage::clipboard_text_ = text;
  visage::WindowEmscripten::runningInstance()->handleKeyDown(visage::KeyCode::V,
                                                             visage::Modifiers::kModifierRegCtrl, 0);
}

namespace visage {
  std::string readClipboardText() {
    return clipboard_text_;
  }

  void setupPasteCallback() {
    EM_ASM({
      document.addEventListener(
          'paste', function(event) {
            navigator.clipboard.readText()
                .then(function(text) { ccall('pasteCallback', null, ['string'], [text]); })
                .catch(function(err) { console.error("Failed to access clipboard:", err); });
          });
    });
  }

  void setClipboardText(const std::string& text) {
    EM_ASM(
        {
          var text = UTF8ToString($0);
          navigator.clipboard.writeText(text).then(function() {}).catch(function(err) {
            console.error("Failed to copy text: ", err);
          });
        },
        text.c_str());
  }

  std::string cursorString(MouseCursor cursor) {
    switch (cursor) {
    case MouseCursor::Arrow: return "default";
    case MouseCursor::IBeam: return "text";
    case MouseCursor::Crosshair: return "crosshair";
    case MouseCursor::Pointing: return "pointer";
    case MouseCursor::HorizontalResize: return "ew-resize";
    case MouseCursor::VerticalResize: return "ns-resize";
    case MouseCursor::Dragging:
    case MouseCursor::MultiDirectionalResize: return "move";
    default: return "";
    }
  }

  void setCursorStyle(MouseCursor style) {
    std::string cursor_string = cursorString(style);
    if (cursor_string.empty())
      return;

    EM_ASM({ document.body.style.cursor = UTF8ToString($0); }, cursor_string.c_str());
  }

  void setCursorVisible(bool visible) { }

  float defaultDpiScale() {
    return EM_ASM_DOUBLE({ return window.devicePixelRatio; });
  }

  Point cursorPosition() {
    EmscriptenMouseEvent event;
    emscripten_get_mouse_status(&event);
    float x = event.targetX - EM_ASM_DOUBLE({
                var canvas = document.getElementById('canvas');
                var rect = canvas.getBoundingClientRect();
                return rect.left;
              });

    float y = event.targetY - EM_ASM_DOUBLE({
                var canvas = document.getElementById('canvas');
                var rect = canvas.getBoundingClientRect();
                return rect.top;
              });
    return { x, y };
  }

  void setCursorPosition(Point window_position) { }

  void setCursorScreenPosition(Point screen_position) { }

  bool isMobileDevice() {
    return EM_ASM_INT({
      if (navigator.userAgentData && navigator.userAgentData.mobile)
        return 1;
      return new RegExp("Mobi|Android|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini", "i")
          .test(navigator.userAgent);
    });
  }

  void WindowEmscripten::showMaximized() {
    maximized_ = true;
    int width = EM_ASM_INT({ return window.innerWidth; });
    int height = EM_ASM_INT({ return window.innerHeight; });
    initial_width_ = width * dpiScale();
    initial_height_ = height * dpiScale();
    handleWindowResize(width, height);
  }

  std::unique_ptr<Window> createWindow(const Dimension& x, const Dimension& y, const Dimension& width,
                                       const Dimension& height, Window::Decoration decoration) {
    float scale = defaultDpiScale();
    int display_width = scale * EM_ASM_INT({ return window.innerWidth; });
    int display_height = scale * EM_ASM_INT({ return window.innerHeight; });

    return std::make_unique<WindowEmscripten>(width.compute(scale, display_width, display_height),
                                              height.compute(scale, display_width, display_height));
  }

  void* headlessWindowHandle() {
    return nullptr;
  }

  void closeApplication() { }

  std::unique_ptr<Window> createPluginWindow(const Dimension& width, const Dimension& height,
                                             void* parent_handle) {
    VISAGE_ASSERT(false);
    return nullptr;
  }

  void showMessageBox(std::string title, std::string message) {
    std::string escaped_message;
    for (char c : message) {
      if (c == '"')
        escaped_message += "\\\"";
      else if (c == '\\')
        escaped_message += "\\\\";
      else
        escaped_message += c;
    }

    std::string script = "alert(\"" + escaped_message + "\");";
    emscripten_run_script(script.c_str());
  }

  void runLoop() {
    WindowEmscripten::runningInstance()->runLoopCallback();
  }

  void WindowEmscripten::runLoopCallback() {
    long long delta = time::microseconds() - start_microseconds_;
    drawCallback(delta / 1000000.0);
  }

  WindowEmscripten* WindowEmscripten::running_instance_ = nullptr;

  WindowEmscripten::WindowEmscripten(int width, int height) :
      Window(width, height), initial_width_(width), initial_height_(height) {
    WindowEmscripten::running_instance_ = this;
    setDpiScale(defaultDpiScale());
    start_microseconds_ = time::microseconds();
  }

  static MouseButton mouseButton(const EmscriptenMouseEvent* event) {
    if (event->button == 0)
      return kMouseButtonLeft;
    if (event->button == 1)
      return kMouseButtonMiddle;
    if (event->button == 2)
      return kMouseButtonRight;
    return kMouseButtonNone;
  }

  static int mouseButtonState(const EmscriptenMouseEvent* event) {
    int state = 0;
    if (event->buttons & 1)
      state |= kMouseButtonLeft;
    if (event->buttons & 2)
      state |= kMouseButtonMiddle;
    if (event->buttons & 4)
      state |= kMouseButtonMiddle;
    return state;
  }

  static int keyboardModifiers(const EmscriptenMouseEvent* event) {
    int state = 0;
    if (event->ctrlKey)
      state |= Modifiers::kModifierRegCtrl;
    if (event->altKey)
      state |= Modifiers::kModifierAlt;
    if (event->shiftKey)
      state |= Modifiers::kModifierShift;
    if (event->metaKey)
      state |= kModifierMeta;
    return state;
  }

  static int keyboardModifiers(const EmscriptenKeyboardEvent* event) {
    int state = 0;
    if (event->ctrlKey)
      state |= Modifiers::kModifierRegCtrl;
    if (event->altKey)
      state |= Modifiers::kModifierAlt;
    if (event->shiftKey)
      state |= Modifiers::kModifierShift;
    if (event->metaKey)
      state |= kModifierMeta;
    return state;
  }

  static EM_BOOL mouseCallback(int event_type, const EmscriptenMouseEvent* event, void* user_data) {
    WindowEmscripten* window = (WindowEmscripten*)user_data;

    if (event == nullptr || window == nullptr)
      return false;

    float x = event->targetX - EM_ASM_DOUBLE({
                var canvas = document.getElementById('canvas');
                var rect = canvas.getBoundingClientRect();
                return rect.left;
              });
    x *= window->dpiScale();

    float y = event->targetY - EM_ASM_DOUBLE({
                var canvas = document.getElementById('canvas');
                var rect = canvas.getBoundingClientRect();
                return rect.top;
              });
    y *= window->dpiScale();

    MouseButton button = mouseButton(event);
    int button_state = mouseButtonState(event);
    int modifier_state = keyboardModifiers(event);
    switch (event_type) {
    case EMSCRIPTEN_EVENT_MOUSEDOWN:
      window->handleMouseDown(button, x, y, button_state, modifier_state);
      return true;
    case EMSCRIPTEN_EVENT_MOUSEUP:
      window->handleMouseUp(button, x, y, button_state, modifier_state);
      return true;
    case EMSCRIPTEN_EVENT_MOUSEMOVE:
      window->handleMouseMove(x, y, button_state, modifier_state);
      return true;
    default: return true;
    }
  }

  static EM_BOOL wheelCallback(int event_type, const EmscriptenWheelEvent* event, void* user_data) {
    static constexpr float kPreciseScrollingScale = 0.01f;

    WindowEmscripten* window = (WindowEmscripten*)user_data;
    if (event_type != EMSCRIPTEN_EVENT_WHEEL || window == nullptr)
      return true;

    float delta_x = event->deltaX;
    float delta_y = -event->deltaY;
    if (event->deltaMode == DOM_DELTA_PIXEL) {
      delta_x *= kPreciseScrollingScale;
      delta_y *= kPreciseScrollingScale;
    }
    float x = event->mouse.targetX - EM_ASM_DOUBLE({
                var canvas = document.getElementById('canvas');
                var rect = canvas.getBoundingClientRect();
                return rect.left;
              });
    x *= window->dpiScale();

    float y = event->mouse.targetY - EM_ASM_DOUBLE({
                var canvas = document.getElementById('canvas');
                var rect = canvas.getBoundingClientRect();
                return rect.top;
              });
    y *= window->dpiScale();

    window->handleMouseWheel(delta_x, delta_y, delta_x, delta_y, x, y,
                             mouseButtonState(&event->mouse), keyboardModifiers(&event->mouse));
    return true;
  }

  KeyCode translateKeyCode(const EmscriptenKeyboardEvent* event) {
    const static std::map<std::string, KeyCode> key_map = {
      { "KeyA", KeyCode::A },
      { "KeyB", KeyCode::B },
      { "KeyC", KeyCode::C },
      { "KeyD", KeyCode::D },
      { "KeyE", KeyCode::E },
      { "KeyF", KeyCode::F },
      { "KeyG", KeyCode::G },
      { "KeyH", KeyCode::H },
      { "KeyI", KeyCode::I },
      { "KeyJ", KeyCode::J },
      { "KeyK", KeyCode::K },
      { "KeyL", KeyCode::L },
      { "KeyM", KeyCode::M },
      { "KeyN", KeyCode::N },
      { "KeyO", KeyCode::O },
      { "KeyP", KeyCode::P },
      { "KeyQ", KeyCode::Q },
      { "KeyR", KeyCode::R },
      { "KeyS", KeyCode::S },
      { "KeyT", KeyCode::T },
      { "KeyU", KeyCode::U },
      { "KeyV", KeyCode::V },
      { "KeyW", KeyCode::W },
      { "KeyX", KeyCode::X },
      { "KeyY", KeyCode::Y },
      { "KeyZ", KeyCode::Z },
      { "Digit1", KeyCode::Number1 },
      { "Digit2", KeyCode::Number2 },
      { "Digit3", KeyCode::Number3 },
      { "Digit4", KeyCode::Number4 },
      { "Digit5", KeyCode::Number5 },
      { "Digit6", KeyCode::Number6 },
      { "Digit7", KeyCode::Number7 },
      { "Digit8", KeyCode::Number8 },
      { "Digit9", KeyCode::Number9 },
      { "Digit0", KeyCode::Number0 },
      { "Enter", KeyCode::Return },
      { "Escape", KeyCode::Escape },
      { "Backspace", KeyCode::Backspace },
      { "Tab", KeyCode::Tab },
      { "Space", KeyCode::Space },
      { "Minus", KeyCode::Minus },
      { "Equal", KeyCode::Equals },
      { "BracketLeft", KeyCode::LeftBracket },
      { "BracketRight", KeyCode::RightBracket },
      { "Backslash", KeyCode::Backslash },
      { "Semicolon", KeyCode::Semicolon },
      { "Quote", KeyCode::Apostrophe },
      { "Backquote", KeyCode::Grave },
      { "Comma", KeyCode::Comma },
      { "Period", KeyCode::Period },
      { "Slash", KeyCode::Slash },
      { "CapsLock", KeyCode::CapsLock },
      { "F1", KeyCode::F1 },
      { "F2", KeyCode::F2 },
      { "F3", KeyCode::F3 },
      { "F4", KeyCode::F4 },
      { "F5", KeyCode::F5 },
      { "F6", KeyCode::F6 },
      { "F7", KeyCode::F7 },
      { "F8", KeyCode::F8 },
      { "F9", KeyCode::F9 },
      { "F10", KeyCode::F10 },
      { "F11", KeyCode::F11 },
      { "F12", KeyCode::F12 },
      { "PrintScreen", KeyCode::PrintScreen },
      { "ScrollLock", KeyCode::ScrollLock },
      { "Pause", KeyCode::Pause },
      { "Insert", KeyCode::Insert },
      { "Home", KeyCode::Home },
      { "PageUp", KeyCode::PageUp },
      { "Delete", KeyCode::Delete },
      { "End", KeyCode::End },
      { "PageDown", KeyCode::PageDown },
      { "ArrowRight", KeyCode::Right },
      { "ArrowLeft", KeyCode::Left },
      { "ArrowDown", KeyCode::Down },
      { "ArrowUp", KeyCode::Up },
      { "NumLock", KeyCode::NumLock },
      { "NumpadDivide", KeyCode::KPDivide },
      { "NumpadMultiply", KeyCode::KPMultiply },
      { "NumpadSubtract", KeyCode::KPMinus },
      { "NumpadAdd", KeyCode::KPPlus },
      { "NumpadEnter", KeyCode::KPEnter },
      { "Numpad1", KeyCode::KP1 },
      { "Numpad2", KeyCode::KP2 },
      { "Numpad3", KeyCode::KP3 },
      { "Numpad4", KeyCode::KP4 },
      { "Numpad5", KeyCode::KP5 },
      { "Numpad6", KeyCode::KP6 },
      { "Numpad7", KeyCode::KP7 },
      { "Numpad8", KeyCode::KP8 },
      { "Numpad9", KeyCode::KP9 },
      { "Numpad0", KeyCode::KP0 },
      { "NumpadDecimal", KeyCode::KPPeriod },
      { "IntlBackslash", KeyCode::NonUSBackslash },
      { "ContextMenu", KeyCode::Application },
      { "Power", KeyCode::Power },
      { "NumpadEqual", KeyCode::KPEquals },
      { "F13", KeyCode::F13 },
      { "F14", KeyCode::F14 },
      { "F15", KeyCode::F15 },
      { "F16", KeyCode::F16 },
      { "F17", KeyCode::F17 },
      { "F18", KeyCode::F18 },
      { "F19", KeyCode::F19 },
      { "F20", KeyCode::F20 },
      { "F21", KeyCode::F21 },
      { "F22", KeyCode::F22 },
      { "F23", KeyCode::F23 },
      { "F24", KeyCode::F24 },
      { "Execute", KeyCode::Execute },
      { "Help", KeyCode::Help },
      { "Menu", KeyCode::Menu },
      { "Select", KeyCode::Select },
      { "Stop", KeyCode::Stop },
      { "Again", KeyCode::Again },
      { "Undo", KeyCode::Undo },
      { "Cut", KeyCode::Cut },
      { "Copy", KeyCode::Copy },
      { "Paste", KeyCode::Paste },
      { "Find", KeyCode::Find },
      { "VolumeMute", KeyCode::Mute },
      { "VolumeUp", KeyCode::VolumeUp },
      { "VolumeDown", KeyCode::VolumeDown },
      { "LockingCapsLock", KeyCode::LockingCapsLock },
      { "LockingNumLock", KeyCode::LockingNumLock },
      { "LockingScrollLock", KeyCode::LockingScrollLock },
      { "NumpadComma", KeyCode::KPComma },
      { "NumpadEqual", KeyCode::KPEqualsAS400 },
      { "Intl1", KeyCode::International1 },
      { "Intl2", KeyCode::International2 },
      { "Intl3", KeyCode::International3 },
      { "Intl4", KeyCode::International4 },
      { "Intl5", KeyCode::International5 },
      { "Intl6", KeyCode::International6 },
      { "Intl7", KeyCode::International7 },
      { "Intl8", KeyCode::International8 },
      { "Intl9", KeyCode::International9 },
      { "Lang1", KeyCode::Lang1 },
      { "Lang2", KeyCode::Lang2 },
      { "Lang3", KeyCode::Lang3 },
      { "Lang4", KeyCode::Lang4 },
      { "Lang5", KeyCode::Lang5 },
      { "Lang6", KeyCode::Lang6 },
      { "Lang7", KeyCode::Lang7 },
      { "Lang8", KeyCode::Lang8 },
      { "Lang9", KeyCode::Lang9 },
      { "AltErase", KeyCode::AltErase },
      { "SysReq", KeyCode::SysReq },
      { "Cancel", KeyCode::Cancel },
      { "Clear", KeyCode::Clear },
      { "Prior", KeyCode::Prior },
      { "Return", KeyCode::Return2 },
      { "Separator", KeyCode::Separator },
      { "Out", KeyCode::Out },
      { "Oper", KeyCode::Oper },
      { "ClearAgain", KeyCode::ClearAgain },
      { "CrSel", KeyCode::CrSel },
      { "ExSel", KeyCode::ExSel },
      { "Numpad00", KeyCode::KP00 },
      { "Numpad000", KeyCode::KP000 },
      { "ThousandsSeparator", KeyCode::ThousandsSeparator },
      { "DecimalSeparator", KeyCode::DecimalSeparator },
      { "CurrencyUnit", KeyCode::CurrencyUnit },
      { "CurrencySubunit", KeyCode::CurrencySubunit },
      { "NumpadParenLeft", KeyCode::KPLeftParen },
      { "NumpadParenRight", KeyCode::KPRightParen },
      { "NumpadBraceLeft", KeyCode::KPLeftBrace },
      { "NumpadBraceRight", KeyCode::KPRightBrace },
      { "NumpadTab", KeyCode::KPTab },
      { "NumpadBackspace", KeyCode::KPBackspace },
      { "NumpadA", KeyCode::KPA },
      { "NumpadB", KeyCode::KPB },
      { "NumpadC", KeyCode::KPC },
      { "NumpadD", KeyCode::KPD },
      { "NumpadE", KeyCode::KPE },
      { "NumpadF", KeyCode::KPF },
      { "NumpadXor", KeyCode::KPXOR },
      { "NumpadPower", KeyCode::KPPower },
      { "NumpadPercent", KeyCode::KPPercent },
      { "NumpadLess", KeyCode::KPLess },
      { "NumpadGreater", KeyCode::KPGreater },
      { "NumpadAmpersand", KeyCode::KPAmpersand },
      { "NumpadDblAmpersand", KeyCode::KPDblAmpersand },
      { "NumpadVerticalBar", KeyCode::KPVerticalBar },
      { "NumpadDblVerticalBar", KeyCode::KPDblVerticalBar },
      { "NumpadColon", KeyCode::KPColon },
      { "NumpadHash", KeyCode::KPHash },
      { "NumpadSpace", KeyCode::KPSpace },
      { "NumpadAt", KeyCode::KPAt },
      { "NumpadExclam", KeyCode::KPExclam },
      { "NumpadMemStore", KeyCode::KPMemStore },
      { "NumpadMemRecall", KeyCode::KPMemRecall },
      { "NumpadMemClear", KeyCode::KPMemClear },
      { "NumpadMemAdd", KeyCode::KPMemAdd },
      { "NumpadMemSubtract", KeyCode::KPMemSubtract },
      { "NumpadMemMultiply", KeyCode::KPMemMultiply },
      { "NumpadMemDivide", KeyCode::KPMemDivide },
      { "NumpadPlusMinus", KeyCode::KPPlusMinus },
      { "NumpadClear", KeyCode::KPClear },
      { "NumpadClearEntry", KeyCode::KPClearEntry },
      { "NumpadBinary", KeyCode::KPBinary },
      { "NumpadOctal", KeyCode::KPOctal },
      { "NumpadDecimal", KeyCode::KPDecimal },
      { "NumpadHexadecimal", KeyCode::KPHexadecimal },
      { "ControlLeft", KeyCode::LCtrl },
      { "ShiftLeft", KeyCode::LShift },
      { "AltLeft", KeyCode::LAlt },
      { "MetaLeft", KeyCode::LGui },
      { "ControlRight", KeyCode::RCtrl },
      { "ShiftRight", KeyCode::RShift },
      { "AltRight", KeyCode::RAlt },
      { "MetaRight", KeyCode::RGui },
      { "ModeChange", KeyCode::Mode },
      { "AudioNext", KeyCode::AudioNext },
      { "AudioPrev", KeyCode::AudioPrev },
      { "AudioStop", KeyCode::AudioStop },
      { "AudioPlay", KeyCode::AudioPlay },
      { "AudioMute", KeyCode::AudioMute },
      { "MediaSelect", KeyCode::MediaSelect },
      { "LaunchMail", KeyCode::Mail },
      { "LaunchApp2", KeyCode::App2 },
      { "LaunchApp1", KeyCode::App1 },
      { "LaunchControlPanel", KeyCode::Computer },
      { "LaunchCalendar", KeyCode::Calculator },
      { "SelectMedia", KeyCode::WWW },
      { "LaunchMediaPlayer", KeyCode::ACSearch },
      { "LaunchMail", KeyCode::ACHome },
      { "BrowserBack", KeyCode::ACBack },
      { "BrowserForward", KeyCode::ACForward },
      { "BrowserStop", KeyCode::ACStop },
      { "BrowserRefresh", KeyCode::ACRefresh },
      { "BrowserFavorites", KeyCode::ACBookmarks },
      { "BrightnessDown", KeyCode::BrightnessDown },
      { "BrightnessUp", KeyCode::BrightnessUp },
      { "DisplaySwap", KeyCode::DisplaySwitch },
      { "KeyboardIlluminationToggle", KeyCode::KBDIllumToggle },
      { "KeyboardIlluminationDown", KeyCode::KBDIllumDown },
      { "KeyboardIlluminationUp", KeyCode::KBDIllumUp },
      { "Eject", KeyCode::Eject },
      { "Sleep", KeyCode::Sleep },
      { "AudioRewind", KeyCode::AudioRewind },
      { "AudioFastForward", KeyCode::AudioFastForward }
    };

    std::string code(event->code);
    if (key_map.count(code))
      return key_map.at(code);
    return KeyCode::Unknown;
  }

  static EM_BOOL keyCallback(int event_type, const EmscriptenKeyboardEvent* event, void* user_data) {
    WindowEmscripten* window = (WindowEmscripten*)user_data;
    if (event == nullptr || window == nullptr)
      return false;

    int modifier_state = keyboardModifiers(event);
    KeyCode code = translateKeyCode(event);

    switch (event_type) {
    case EMSCRIPTEN_EVENT_KEYPRESS:
      if (code == KeyCode::Return || strlen(event->key) == 0)
        return true;
      return window->handleTextInput(event->key);
    case EMSCRIPTEN_EVENT_KEYDOWN: {
      if (modifier_state != 0 && code == KeyCode::V)
        return false;

      bool down_used = window->handleKeyDown(code, modifier_state, event->repeat);
      bool text_input = modifier_state == 0 || modifier_state == kModifierShift;
      return (code == KeyCode::Tab || !text_input) && down_used;
    }
    case EMSCRIPTEN_EVENT_KEYUP: {
      bool up_used = window->handleKeyUp(code, modifier_state);
      bool text_input = modifier_state == 0 || modifier_state == kModifierShift;
      return (code == KeyCode::Tab || !text_input) && up_used;
    }
    default: return false;
    }
  }

  static EM_BOOL resizeCallback(int event_type, const EmscriptenUiEvent* event, void* user_data) {
    WindowEmscripten* window = (WindowEmscripten*)user_data;
    if (event == nullptr || window == nullptr)
      return false;

    int new_width = event->windowInnerWidth;
    int new_height = event->windowInnerHeight;
    if (!window->maximized()) {
      new_width = std::min<int>(window->initialWidth() / window->dpiScale(), new_width);
      new_height = std::min<int>(window->initialHeight() / window->dpiScale(), new_height);
    }
    window->handleWindowResize(new_width, new_height);
    return true;
  }

  void WindowEmscripten::runEventLoop() {
    static const char* canvas = "#canvas";
    emscripten_set_mousedown_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, this, true, mouseCallback);
    emscripten_set_mouseup_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, this, true, mouseCallback);
    emscripten_set_mousemove_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, this, true, mouseCallback);

    emscripten_set_wheel_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, this, true, wheelCallback);

    emscripten_set_keydown_callback(EMSCRIPTEN_EVENT_TARGET_DOCUMENT, this, true, keyCallback);
    emscripten_set_keyup_callback(EMSCRIPTEN_EVENT_TARGET_DOCUMENT, this, true, keyCallback);
    emscripten_set_keypress_callback(EMSCRIPTEN_EVENT_TARGET_DOCUMENT, this, true, keyCallback);
    setupPasteCallback();

    emscripten_set_resize_callback(EMSCRIPTEN_EVENT_TARGET_WINDOW, this, true, resizeCallback);

    setDpiScale(defaultDpiScale());
    emscripten_set_element_css_size("canvas", clientWidth() / dpiScale(), clientHeight() / dpiScale());
    emscripten_set_canvas_element_size("canvas", clientWidth(), clientHeight());
    emscripten_set_main_loop(runLoop, 0, 1);
  }

  void WindowEmscripten::windowContentsResized(int width, int height) {
    emscripten_set_element_css_size("canvas", width / dpiScale(), height / dpiScale());
    emscripten_set_canvas_element_size("canvas", width, height);
  }

  void WindowEmscripten::setWindowTitle(const std::string& title) {
    emscripten_run_script(("document.title = '" + title + "';").c_str());
  }

  IPoint WindowEmscripten::maxWindowDimensions() const {
    int display_width = EM_ASM_INT({ return screen.width; });
    int display_height = EM_ASM_INT({ return screen.height; });

    float aspect_ratio = aspectRatio();
    return { std::min<int>(display_width, display_height * aspect_ratio),
             std::min<int>(display_height, display_width / aspect_ratio) };
  }

  IPoint WindowEmscripten::minWindowDimensions() const {
    return { 0, 0 };
  }

  void WindowEmscripten::handleWindowResize(int window_width, int window_height) {
    int width = window_width;
    int height = window_height;
    if (!maximized_) {
      float aspect_ratio = aspectRatio();
      int width = std::min<int>(window_width, window_height * aspect_ratio);
      int height = std::min<int>(window_height, window_width / aspect_ratio);
    }
    handleResized(width * dpiScale(), height * dpiScale());
    emscripten_set_element_css_size("canvas", width, height);
    emscripten_set_canvas_element_size("canvas", clientWidth(), clientHeight());
  }
}
#endif
