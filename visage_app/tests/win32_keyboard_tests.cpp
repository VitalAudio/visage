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

#include "visage/app.h"

#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#if VISAGE_WINDOWS
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

using namespace visage;

namespace {
  class KeyRecorder : public Window::EventHandler {
  public:
    HitTestResult handleHitTest(int, int) override { return HitTestResult::Client; }
    HitTestResult currentHitTest() const override { return HitTestResult::Client; }
    void handleMouseMove(int, int, int, int) override { }
    void handleMouseDown(MouseButton, int, int, int, int, int) override { }
    void handleMouseUp(MouseButton, int, int, int, int, int) override { }
    void handleMouseEnter(int, int) override { }
    void handleMouseLeave(int, int, int, int) override { }
    void handleMouseWheel(float, float, float, float, int, int, int, int, bool) override { }

    bool handleKeyDown(KeyCode key_code, int, bool) override {
      key_downs.push_back(key_code);
      return handle_keys;
    }
    bool handleKeyUp(KeyCode, int) override {
      key_ups++;
      return handle_keys;
    }

    bool handleTextInput(const std::string& input) override {
      text += input;
      return true;
    }
    bool hasActiveTextEntry() override { return active_text_entry; }

    void handleFocusLost() override { }
    void handleFocusGained() override { }
    void handleResized(int, int) override { }
    void handleWindowShown() override { }
    void handleWindowHidden() override { }
    bool handleCloseRequested() override { return true; }

    bool handleFileDrag(int, int, const std::vector<std::string>&) override { return false; }
    void handleFileDragLeave() override { }
    bool handleFileDrop(int, int, const std::vector<std::string>&) override { return false; }

    bool isDragDropSource() override { return false; }
    std::string startDragDropSource() override { return {}; }
    void cleanupDragDropSource() override { }

    bool handle_keys = false;
    bool active_text_entry = false;
    std::vector<KeyCode> key_downs;
    int key_ups = 0;
    std::string text;
  };

  void runMessages() {
    MSG message;
    while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
      TranslateMessage(&message);
      DispatchMessageW(&message);
    }
  }

  struct RecordedWindow {
    RecordedWindow() {
      window = createWindow(Dimension::nativePixels(100), Dimension::nativePixels(100));
      if (window) {
        window->setEventHandler(&recorder);
        hwnd = static_cast<HWND>(window->nativeHandle());
      }
    }

    ~RecordedWindow() {
      if (window)
        window->clearEventHandler();
      window.reset();
      runMessages();
    }

    KeyRecorder recorder;
    std::unique_ptr<Window> window;
    HWND hwnd = nullptr;
  };

  LPARAM keyLParam(int scan_code, bool extended = false) {
    LPARAM l_param = 1 | (static_cast<LPARAM>(scan_code & 0xFF) << 16);
    if (extended)
      l_param |= static_cast<LPARAM>(1) << 24;
    return l_param;
  }

  LPARAM altKeyLParam(int scan_code) {
    return keyLParam(scan_code) | (static_cast<LPARAM>(1) << 29);
  }

  constexpr int kScanA = 0x1E;
  constexpr int kScanD = 0x20;
  constexpr int kScanF = 0x21;
  constexpr int kScanSpace = 0x39;
  constexpr int kScanF5 = 0x3F;

  class ScopedNumLock {
  public:
    explicit ScopedNumLock(bool on) {
      GetKeyboardState(saved_);
      BYTE state[256];
      GetKeyboardState(state);
      state[VK_NUMLOCK] = on ? (state[VK_NUMLOCK] | 0x01) : (state[VK_NUMLOCK] & ~0x01);
      SetKeyboardState(state);
    }

    ~ScopedNumLock() { SetKeyboardState(saved_); }

  private:
    BYTE saved_[256] {};
  };

  struct KeyCase {
    const char* name;
    WPARAM virtual_key;
    int scan_code;
    bool extended;
    KeyCode expected;
  };

  void checkKeys(const std::vector<KeyCase>& cases, bool num_lock) {
    RecordedWindow setup;
    REQUIRE(setup.window);
    REQUIRE(setup.hwnd);
    setup.recorder.handle_keys = true;

    ScopedNumLock num_lock_state(num_lock);
    REQUIRE(((GetKeyState(VK_NUMLOCK) & 0x01) != 0) == num_lock);

    for (const KeyCase& key : cases) {
      INFO(key.name << (num_lock ? " (NumLock on)" : " (NumLock off)"));
      setup.recorder.key_downs.clear();
      SendMessageW(setup.hwnd, WM_KEYDOWN, key.virtual_key, keyLParam(key.scan_code, key.extended));
      REQUIRE(setup.recorder.key_downs.size() == 1);
      CHECK(static_cast<int>(setup.recorder.key_downs[0]) == static_cast<int>(key.expected));
    }
  }

  constexpr wchar_t kSpyProperty[] = L"VisageWin32SysCommandSpy";

  class SysCommandSpy {
  public:
    explicit SysCommandSpy(HWND hwnd) : hwnd_(hwnd) {
      SetPropW(hwnd_, kSpyProperty, this);
      original_ = reinterpret_cast<WNDPROC>(SetWindowLongPtrW(hwnd_, GWLP_WNDPROC,
                                                              reinterpret_cast<LONG_PTR>(&SysCommandSpy::proc)));
    }

    ~SysCommandSpy() {
      SetWindowLongPtrW(hwnd_, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(original_));
      RemovePropW(hwnd_, kSpyProperty);
    }

    std::vector<LPARAM> keyMenuCharacters() const {
      std::vector<LPARAM> result;
      for (const auto& command : commands_) {
        if (command.first == SC_KEYMENU)
          result.push_back(command.second);
      }
      return result;
    }

  private:
    static LRESULT CALLBACK proc(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
      SysCommandSpy* spy = static_cast<SysCommandSpy*>(GetPropW(hwnd, kSpyProperty));
      if (msg == WM_SYSCOMMAND) {
        spy->commands_.push_back({ w_param & 0xFFF0, l_param });
        return 0;
      }
      return CallWindowProcW(spy->original_, hwnd, msg, w_param, l_param);
    }

    HWND hwnd_ = nullptr;
    WNDPROC original_ = nullptr;
    std::vector<std::pair<WPARAM, LPARAM>> commands_;
  };

  void altKey(HWND hwnd, WPARAM virtual_key, wchar_t character, int scan_code) {
    PostMessageW(hwnd, WM_SYSCHAR, character, altKeyLParam(scan_code));
    SendMessageW(hwnd, WM_SYSKEYDOWN, virtual_key, altKeyLParam(scan_code));
    runMessages();
  }

  constexpr wchar_t kHostClass[] = L"VisageWin32KeyboardTestHost";
  constexpr wchar_t kHostProperty[] = L"VisageWin32KeyboardTestHostRecord";

  class HostWindow {
  public:
    HostWindow() {
      static bool registered = false;
      if (!registered) {
        WNDCLASSW window_class {};
        window_class.lpfnWndProc = &HostWindow::proc;
        window_class.hInstance = GetModuleHandleW(nullptr);
        window_class.lpszClassName = kHostClass;
        registered = RegisterClassW(&window_class) != 0;
      }
      hwnd = CreateWindowExW(0, kHostClass, L"host", WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN, 0, 0,
                             400, 300, nullptr, nullptr, GetModuleHandleW(nullptr), nullptr);
      SetPropW(hwnd, kHostProperty, this);
    }

    ~HostWindow() {
      RemovePropW(hwnd, kHostProperty);
      DestroyWindow(hwnd);
    }

    int count(UINT message) const {
      int result = 0;
      for (UINT m : messages)
        result += m == message;
      return result;
    }

    HWND hwnd = nullptr;
    std::vector<UINT> messages;

  private:
    static LRESULT CALLBACK proc(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
      HostWindow* host = static_cast<HostWindow*>(GetPropW(hwnd, kHostProperty));
      bool key_message = msg >= WM_KEYFIRST && msg <= WM_KEYLAST;
      if (host && (key_message || msg == WM_SYSCOMMAND)) {
        host->messages.push_back(msg);
        return 0;
      }
      return DefWindowProcW(hwnd, msg, w_param, l_param);
    }
  };

  struct PluginInHost {
    HostWindow host;
    KeyRecorder recorder;
    std::unique_ptr<Window> window;
    HWND plugin = nullptr;
    int loop_saw_plugin_key_messages = 0;

    PluginInHost() {
      window = createPluginWindow(Dimension::nativePixels(100), Dimension::nativePixels(100), host.hwnd);
      if (window) {
        window->setEventHandler(&recorder);
        plugin = static_cast<HWND>(window->nativeHandle());
      }
      runHostLoop();
      loop_saw_plugin_key_messages = 0;
    }

    ~PluginInHost() {
      if (window)
        window->clearEventHandler();
      window.reset();
      runHostLoop();
    }

    void runHostLoop() {
      MSG message;
      while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
        if (plugin && message.hwnd == plugin && message.message >= WM_KEYFIRST && message.message <= WM_KEYLAST)
          loop_saw_plugin_key_messages++;
        TranslateMessage(&message);
        DispatchMessageW(&message);
      }
    }

    void postKey(UINT down_message, UINT up_message, WPARAM virtual_key, int scan_code, bool alt) {
      LPARAM l_param = alt ? altKeyLParam(scan_code) : keyLParam(scan_code);
      LPARAM up_l_param = l_param | static_cast<LPARAM>(0xC0000000u);
      PostMessageW(plugin, down_message, virtual_key, l_param);
      PostMessageW(plugin, up_message, virtual_key, up_l_param);
    }
  };
}

TEST_CASE("Win32 navigation keys aren't keypad keys with NumLock on", "[integration]") {
  checkKeys({ { "Delete (E0 53)", VK_DELETE, 0x53, true, KeyCode::Delete },
              { "Home (E0 47)", VK_HOME, 0x47, true, KeyCode::Home },
              { "End (E0 4F)", VK_END, 0x4F, true, KeyCode::End },
              { "PageUp (E0 49)", VK_PRIOR, 0x49, true, KeyCode::PageUp },
              { "PageDown (E0 51)", VK_NEXT, 0x51, true, KeyCode::PageDown },
              { "Insert (E0 52)", VK_INSERT, 0x52, true, KeyCode::Insert } },
            true);
}

TEST_CASE("Win32 keypad digits map to keypad keys with NumLock on", "[integration]") {
  checkKeys({ { "Keypad 7 (47)", VK_NUMPAD7, 0x47, false, KeyCode::KP7 },
              { "Keypad 1 (4F)", VK_NUMPAD1, 0x4F, false, KeyCode::KP1 },
              { "Keypad 0 (52)", VK_NUMPAD0, 0x52, false, KeyCode::KP0 },
              { "Keypad . (53)", VK_DECIMAL, 0x53, false, KeyCode::KPPeriod },
              { "Keypad 8 (48)", VK_NUMPAD8, 0x48, false, KeyCode::KP8 } },
            true);
}

TEST_CASE("Win32 keypad keys map to navigation keys with NumLock off", "[integration]") {
  checkKeys({ { "Keypad 7 as Home (47)", VK_HOME, 0x47, false, KeyCode::Home },
              { "Keypad . as Delete (53)", VK_DELETE, 0x53, false, KeyCode::Delete },
              { "Delete (E0 53)", VK_DELETE, 0x53, true, KeyCode::Delete },
              { "End (E0 4F)", VK_END, 0x4F, true, KeyCode::End } },
            false);
}

TEST_CASE("Win32 keypad and right-hand keys use the extended-key flag", "[integration]") {
  checkKeys({ { "Keypad * (37)", VK_MULTIPLY, 0x37, false, KeyCode::KPMultiply },
              { "PrintScreen (E0 37)", VK_SNAPSHOT, 0x37, true, KeyCode::PrintScreen },
              { "Keypad / (E0 35)", VK_DIVIDE, 0x35, true, KeyCode::KPDivide },
              { "Slash (35)", VK_OEM_2, 0x35, false, KeyCode::Slash },
              { "Keypad Enter (E0 1C)", VK_RETURN, 0x1C, true, KeyCode::KPEnter },
              { "Return (1C)", VK_RETURN, 0x1C, false, KeyCode::Return },
              { "Right Ctrl (E0 1D)", VK_CONTROL, 0x1D, true, KeyCode::RCtrl },
              { "Left Ctrl (1D)", VK_CONTROL, 0x1D, false, KeyCode::LCtrl },
              { "Right Alt (E0 38)", VK_MENU, 0x38, true, KeyCode::RAlt },
              { "Left Alt (38)", VK_MENU, 0x38, false, KeyCode::LAlt } },
            true);
}

TEST_CASE("Win32 Pause and Break report Pause while the lock keys keep their own codes",
          "[integration]") {
  checkKeys({ { "Pause (45)", VK_PAUSE, 0x45, false, KeyCode::Pause },
              { "Ctrl+Pause / Break (E0 46)", VK_CANCEL, 0x46, true, KeyCode::Pause },
              { "NumLock (E0 45)", VK_NUMLOCK, 0x45, true, KeyCode::NumLock },
              { "ScrollLock (46)", VK_SCROLL, 0x46, false, KeyCode::ScrollLock } },
            true);
}

TEST_CASE("Win32 Alt+Space in a standalone window opens the system menu", "[integration]") {
  RecordedWindow setup;
  REQUIRE(setup.window);
  REQUIRE(GetParent(setup.hwnd) == nullptr);

  std::vector<LPARAM> key_menu;
  {
    SysCommandSpy spy(setup.hwnd);
    altKey(setup.hwnd, VK_SPACE, L' ', kScanSpace);
    key_menu = spy.keyMenuCharacters();
  }

  CHECK(setup.recorder.key_downs == std::vector<KeyCode> { KeyCode::Space });
  CHECK(key_menu == std::vector<LPARAM> { L' ' });
}

TEST_CASE("Win32 Alt+key shortcuts the app handles don't reach DefWindowProc", "[integration]") {
  RecordedWindow setup;
  setup.recorder.handle_keys = true;
  REQUIRE(setup.window);

  std::vector<LPARAM> key_menu;
  {
    SysCommandSpy spy(setup.hwnd);
    altKey(setup.hwnd, 'D', L'd', kScanD);
    altKey(setup.hwnd, VK_SPACE, L' ', kScanSpace);
    key_menu = spy.keyMenuCharacters();
  }

  CHECK(setup.recorder.key_downs == std::vector<KeyCode> { KeyCode::D, KeyCode::Space });
  CHECK(key_menu.empty());
}

TEST_CASE("Win32 Alt+letter doesn't type into an active text field", "[integration]") {
  RecordedWindow setup;
  setup.recorder.active_text_entry = true;
  REQUIRE(setup.window);

  {
    SysCommandSpy spy(setup.hwnd);
    altKey(setup.hwnd, 'D', L'd', kScanD);
    CHECK(setup.recorder.text.empty());

    SendMessageW(setup.hwnd, WM_CHAR, L'd', keyLParam(kScanD));
    CHECK(setup.recorder.text == "d");
  }
}

TEST_CASE("Win32 plugin windows don't pass Alt+key characters on", "[integration]") {
  HostWindow host;
  REQUIRE(host.hwnd);
  KeyRecorder recorder;
  std::unique_ptr<Window> window = createPluginWindow(Dimension::nativePixels(100),
                                                      Dimension::nativePixels(100), host.hwnd);
  REQUIRE(window);
  window->setEventHandler(&recorder);
  HWND plugin = static_cast<HWND>(window->nativeHandle());
  REQUIRE(GetParent(plugin) == host.hwnd);

  std::vector<LPARAM> key_menu;
  {
    SysCommandSpy spy(plugin);
    SendMessageW(plugin, WM_SYSCHAR, L'f', altKeyLParam(kScanF));
    SendMessageW(plugin, WM_SYSCHAR, L' ', altKeyLParam(kScanSpace));
    runMessages();
    key_menu = spy.keyMenuCharacters();
  }

  CHECK(key_menu.empty());
  CHECK(host.count(WM_SYSCHAR) == 0);
  CHECK(host.count(WM_SYSCOMMAND) == 0);
  CHECK(recorder.text.empty());

  window->clearEventHandler();
  window.reset();
  runMessages();
}

TEST_CASE("Win32 plugin keys the app doesn't handle reach the key handlers once", "[integration]") {
  PluginInHost setup;
  REQUIRE(setup.plugin);
  REQUIRE(GetParent(setup.plugin) == setup.host.hwnd);

  setup.postKey(WM_KEYDOWN, WM_KEYUP, VK_F5, kScanF5, false);
  setup.runHostLoop();

  CHECK(setup.recorder.key_downs.size() == 1);
  CHECK(setup.recorder.key_ups == 1);
  CHECK(setup.host.count(WM_KEYDOWN) == 1);
  CHECK(setup.host.count(WM_KEYUP) == 1);
  CHECK(setup.loop_saw_plugin_key_messages == 0);
}

TEST_CASE("Win32 plugin Alt+keys the app doesn't handle reach the key handlers once",
          "[integration]") {
  PluginInHost setup;
  REQUIRE(setup.plugin);

  setup.postKey(WM_SYSKEYDOWN, WM_SYSKEYUP, 'F', kScanF, true);
  setup.runHostLoop();

  CHECK(setup.recorder.key_downs.size() == 1);
  CHECK(setup.recorder.key_ups == 1);
  CHECK(setup.host.count(WM_SYSKEYDOWN) == 1);
  CHECK(setup.host.count(WM_SYSKEYUP) == 1);
  CHECK(setup.loop_saw_plugin_key_messages == 0);
}

TEST_CASE("Win32 plugin keys the app handles aren't passed to the host", "[integration]") {
  PluginInHost setup;
  REQUIRE(setup.plugin);
  setup.recorder.handle_keys = true;

  setup.postKey(WM_KEYDOWN, WM_KEYUP, VK_F5, kScanF5, false);
  setup.runHostLoop();

  CHECK(setup.recorder.key_downs.size() == 1);
  CHECK(setup.recorder.key_ups == 1);
  CHECK(setup.host.messages.empty());
  CHECK(setup.loop_saw_plugin_key_messages == 0);
}

TEST_CASE("Win32 typing in a plugin text field isn't passed to the host", "[integration]") {
  PluginInHost setup;
  REQUIRE(setup.plugin);
  setup.recorder.active_text_entry = true;

  PostMessageW(setup.plugin, WM_KEYDOWN, 'A', keyLParam(kScanA));
  setup.runHostLoop();

  CHECK(setup.recorder.key_downs.size() == 1);
  CHECK(setup.recorder.text.size() == 1);
  CHECK(setup.host.count(WM_KEYDOWN) == 0);
  CHECK(setup.host.count(WM_CHAR) == 0);
  CHECK(setup.loop_saw_plugin_key_messages == 0);
}

#endif
