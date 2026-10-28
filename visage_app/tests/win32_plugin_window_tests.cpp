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
#include <chrono>
#include <cmath>
#include <memory>

#if VISAGE_WINDOWS
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

using namespace visage;

namespace {
  struct HostLog {
    int pings = 0;
    int key_downs = 0;
    int key_ups = 0;
    int sys_key_downs = 0;
    int sizing = 0;
    RECT sizing_rect = {};
    int dpi_changes = 0;
  };
  HostLog host_log;

  constexpr UINT kPing = WM_APP + 1;
  constexpr LRESULT kPong = 0x1234;

  LRESULT CALLBACK hostProc(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
    switch (msg) {
    case kPing: host_log.pings++; return kPong;
    case WM_KEYDOWN: host_log.key_downs++; return 0;
    case WM_KEYUP: host_log.key_ups++; return 0;
    case WM_SYSKEYDOWN: host_log.sys_key_downs++; return 0;
    case WM_SYSKEYUP:
    case WM_SYSCHAR:
    case WM_CHAR: return 0;
    default: return DefWindowProcW(hwnd, msg, w_param, l_param);
    }
  }

  LRESULT CALLBACK perMonitorHostProc(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
    if (msg == WM_SIZING) {
      host_log.sizing++;
      host_log.sizing_rect = *reinterpret_cast<RECT*>(l_param);
      return TRUE;
    }
    if (msg == WM_DPICHANGED) {
      host_log.dpi_changes++;
      const RECT* suggested = reinterpret_cast<const RECT*>(l_param);
      SetWindowPos(hwnd, nullptr, suggested->left, suggested->top, suggested->right - suggested->left,
                   suggested->bottom - suggested->top, SWP_NOZORDER | SWP_NOACTIVATE);
      return 0;
    }
    if (msg == WM_SIZE) {
      HWND child = GetWindow(hwnd, GW_CHILD);
      if (child) {
        SetWindowPos(child, nullptr, 0, 0, LOWORD(l_param), HIWORD(l_param),
                     SWP_NOZORDER | SWP_NOACTIVATE | SWP_NOMOVE);
      }
      return 0;
    }
    return DefWindowProcW(hwnd, msg, w_param, l_param);
  }

  struct HostClass {
    const wchar_t* name;
    WNDPROC proc;
  };

  const HostClass kHost = { L"VisageWin32PluginTestHost", hostProc };
  const HostClass kPerMonitorHost = { L"VisageWin32PluginTestPerMonitorHost", perMonitorHostProc };
  const HostClass kPlainHost = { L"VisageWin32PluginTestPlainHost", DefWindowProcW };

  struct HostWindow {
    HostWindow(const HostClass& host_class, int width, int height, DPI_AWARENESS_CONTEXT awareness = nullptr) {
      WNDCLASSEXW window_class = {};
      window_class.cbSize = sizeof(window_class);
      window_class.lpfnWndProc = host_class.proc;
      window_class.hInstance = GetModuleHandleW(nullptr);
      window_class.lpszClassName = host_class.name;
      RegisterClassExW(&window_class);

      DPI_AWARENESS_CONTEXT previous = nullptr;
      if (awareness)
        previous = SetThreadDpiAwarenessContext(awareness);
      hwnd = CreateWindowExW(WS_EX_TOOLWINDOW | WS_EX_NOACTIVATE, host_class.name, L"host", WS_POPUP, 40,
                             40, width, height, nullptr, nullptr, GetModuleHandleW(nullptr), nullptr);
      if (previous)
        SetThreadDpiAwarenessContext(previous);
      host_log = {};
    }

    ~HostWindow() {
      if (IsWindow(hwnd))
        DestroyWindow(hwnd);
    }

    HWND hwnd = nullptr;
  };

  class ScopedDpiAwareness {
  public:
    explicit ScopedDpiAwareness(DPI_AWARENESS_CONTEXT awareness) :
        previous_(SetThreadDpiAwarenessContext(awareness)) { }

    ~ScopedDpiAwareness() {
      if (previous_)
        SetThreadDpiAwarenessContext(previous_);
    }

  private:
    DPI_AWARENESS_CONTEXT previous_ = nullptr;
  };

  void pump(int milliseconds) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    while (std::chrono::steady_clock::now() < end) {
      MSG message;
      if (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
        if (message.message != WM_QUIT) {
          TranslateMessage(&message);
          DispatchMessageW(&message);
        }
      }
      else
        MsgWaitForMultipleObjects(0, nullptr, FALSE, 2, QS_ALLINPUT);
    }
  }

  void takePendingQuit() {
    MSG message;
    for (int i = 0; i < 10000 && PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE); ++i) {
      if (message.message != WM_QUIT) {
        TranslateMessage(&message);
        DispatchMessageW(&message);
      }
    }
  }

  WNDPROC windowProc(HWND hwnd) {
    return reinterpret_cast<WNDPROC>(GetWindowLongPtrW(hwnd, GWLP_WNDPROC));
  }

  std::unique_ptr<Window> showPluginWindow(HWND parent) {
    auto window = createPluginWindow(Dimension::nativePixels(200), Dimension::nativePixels(100), parent);
    if (window)
      window->show();
    return window;
  }

  void sendUnhandledKeys(Window* window) {
    HWND hwnd = static_cast<HWND>(window->nativeHandle());
    LPARAM f5_down = 1 | (MapVirtualKeyW(VK_F5, MAPVK_VK_TO_VSC) << 16);
    LPARAM f5_up = f5_down | (1u << 30) | (1u << 31);
    LPARAM alt_f = 1 | (MapVirtualKeyW('F', MAPVK_VK_TO_VSC) << 16) | (1u << 29);
    SendMessageW(hwnd, WM_KEYDOWN, VK_F5, f5_down);
    SendMessageW(hwnd, WM_KEYUP, VK_F5, static_cast<LPARAM>(f5_up));
    SendMessageW(hwnd, WM_SYSKEYDOWN, 'F', alt_f);
  }

  UINT systemDpi() {
    ScopedDpiAwareness per_monitor(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    return GetDpiForSystem();
  }

  const char* awarenessName(HWND hwnd) {
    switch (GetAwarenessFromDpiAwarenessContext(GetWindowDpiAwarenessContext(hwnd))) {
    case DPI_AWARENESS_UNAWARE: return "unaware";
    case DPI_AWARENESS_SYSTEM_AWARE: return "system aware";
    case DPI_AWARENESS_PER_MONITOR_AWARE: return "per monitor aware";
    default: return "invalid";
    }
  }

  void checkPluginStartsAtItsOwnDpi(HWND parent) {
    std::unique_ptr<Window> window = createPluginWindow(Dimension::logicalPixels(200),
                                                        Dimension::logicalPixels(100), parent);
    REQUIRE(window);
    HWND plugin = static_cast<HWND>(window->nativeHandle());
    REQUIRE(IsWindow(plugin));

    float scale = GetDpiForWindow(plugin) / 96.0f;
    INFO("System DPI " << systemDpi() << ", host window DPI " << GetDpiForWindow(parent) << " ("
                       << awarenessName(parent) << "), plugin window DPI "
                       << GetDpiForWindow(plugin) << " (" << awarenessName(plugin) << ")");

    CHECK(window->dpiScale() == scale);
    CHECK(window->clientWidth() == std::lround(200 * scale));
    CHECK(window->clientHeight() == std::lround(100 * scale));
  }

  float otherScale(float scale) {
    return scale == 1.0f ? 2.0f : 1.0f;
  }
}

TEST_CASE("Win32 re-showing a plugin window leaves the host window's procedure intact",
          "[integration]") {
  HostWindow parent(kHost, 200, 100);
  REQUIRE(parent.hwnd);
  WNDPROC original_proc = windowProc(parent.hwnd);
  REQUIRE(original_proc == hostProc);

  {
    ApplicationWindow app;
    app.setWindowDimensions(Dimension::nativePixels(200), Dimension::nativePixels(100));
    app.setFixedAspectRatio(true);

    app.show(Dimension::nativePixels(200), Dimension::nativePixels(100), parent.hwnd);
    REQUIRE(app.window());
    HWND first = static_cast<HWND>(app.window()->nativeHandle());
    app.show(Dimension::nativePixels(200), Dimension::nativePixels(100), parent.hwnd);
    REQUIRE(app.window());
    HWND second = static_cast<HWND>(app.window()->nativeHandle());
    CHECK(first != second);
    CHECK_FALSE(IsWindow(first));
    pump(30);

    RECT sizing = { 0, 0, 300, 100 };
    SendMessageW(parent.hwnd, WM_SIZING, WMSZ_RIGHT, reinterpret_cast<LPARAM>(&sizing));
    CHECK(sizing.right - sizing.left == 300);
    CHECK(sizing.bottom - sizing.top == 150);

    host_log.pings = 0;
    CHECK(SendMessageW(parent.hwnd, kPing, 0, 0) == kPong);
    CHECK(host_log.pings == 1);
  }

  CHECK(windowProc(parent.hwnd) == original_proc);
  host_log.pings = 0;
  CHECK(SendMessageW(parent.hwnd, kPing, 0, 0) == kPong);
  CHECK(host_log.pings == 1);
}

TEST_CASE("Win32 plugin windows on one parent can be destroyed in either order", "[integration]") {
  HostWindow parent(kHost, 200, 100);
  REQUIRE(parent.hwnd);
  WNDPROC original_proc = windowProc(parent.hwnd);

  SECTION("First created is destroyed first") {
    auto first = createPluginWindow(Dimension::nativePixels(100), Dimension::nativePixels(50),
                                    parent.hwnd);
    auto second = createPluginWindow(Dimension::nativePixels(100), Dimension::nativePixels(50),
                                     parent.hwnd);
    REQUIRE(first);
    REQUIRE(second);
    first.reset();
    second.reset();
  }

  SECTION("Last created is destroyed first") {
    auto first = createPluginWindow(Dimension::nativePixels(100), Dimension::nativePixels(50),
                                    parent.hwnd);
    auto second = createPluginWindow(Dimension::nativePixels(100), Dimension::nativePixels(50),
                                     parent.hwnd);
    REQUIRE(first);
    REQUIRE(second);
    second.reset();
    first.reset();
  }

  CHECK(windowProc(parent.hwnd) == original_proc);
  host_log.pings = 0;
  CHECK(SendMessageW(parent.hwnd, kPing, 0, 0) == kPong);
  CHECK(host_log.pings == 1);
}

TEST_CASE("Win32 plugin windows survive the host destroying their parent first", "[integration]") {
  HostWindow parent(kHost, 200, 100);
  REQUIRE(parent.hwnd);

  auto app = std::make_unique<ApplicationWindow>();
  app->show(Dimension::nativePixels(200), Dimension::nativePixels(100), parent.hwnd);
  REQUIRE(app->window());
  HWND plugin = static_cast<HWND>(app->window()->nativeHandle());
  pump(30);

  DestroyWindow(parent.hwnd);
  CHECK_FALSE(IsWindow(parent.hwnd));
  CHECK_FALSE(IsWindow(plugin));
  pump(30);

  app.reset();
  SUCCEED("The plugin window was destroyed after its parent without crashing");
}

TEST_CASE("Win32 plugin windows pass unhandled keys up to the host", "[integration]") {
  HostWindow host(kHost, 300, 200);
  REQUIRE(host.hwnd);
  auto window = showPluginWindow(host.hwnd);
  REQUIRE(window);

  sendUnhandledKeys(window.get());
  pump(100);
  CHECK(host_log.key_downs == 1);
  CHECK(host_log.key_ups == 1);
  CHECK(host_log.sys_key_downs == 1);
  CHECK(SendMessageW(host.hwnd, kPing, 0, 0) == kPong);
}

TEST_CASE("Win32 re-shown plugin windows pass unhandled keys up once", "[integration]") {
  HostWindow host(kHost, 300, 200);
  REQUIRE(host.hwnd);
  auto first = showPluginWindow(host.hwnd);
  REQUIRE(first);
  auto second = showPluginWindow(host.hwnd);
  REQUIRE(second);
  first.reset();
  pump(50);

  sendUnhandledKeys(second.get());
  pump(100);
  CHECK(host_log.key_downs == 1);
  CHECK(host_log.key_ups == 1);
  CHECK(host_log.sys_key_downs == 1);
}

TEST_CASE("Win32 hosts still get their own messages after the plugin windows close",
          "[integration]") {
  HostWindow host(kHost, 300, 200);
  REQUIRE(host.hwnd);
  auto first = showPluginWindow(host.hwnd);
  auto second = showPluginWindow(host.hwnd);
  REQUIRE(first);
  REQUIRE(second);
  first.reset();
  second.reset();
  pump(50);

  PostMessageW(host.hwnd, WM_KEYDOWN, VK_F5, 1);
  pump(100);
  CHECK(host_log.key_downs == 1);
  CHECK(SendMessageW(host.hwnd, kPing, 0, 0) == kPong);
  CHECK(windowProc(host.hwnd) == hostProc);
}

TEST_CASE("Win32 two live plugin windows on one parent each pass keys up once", "[integration]") {
  HostWindow host(kHost, 300, 200);
  REQUIRE(host.hwnd);
  auto first = showPluginWindow(host.hwnd);
  auto second = showPluginWindow(host.hwnd);
  REQUIRE(first);
  REQUIRE(second);

  sendUnhandledKeys(first.get());
  sendUnhandledKeys(second.get());
  pump(100);
  CHECK(host_log.key_downs == 2);
  CHECK(host_log.key_ups == 2);
  CHECK(host_log.sys_key_downs == 2);
  CHECK(SendMessageW(host.hwnd, kPing, 0, 0) == kPong);
}

TEST_CASE("Win32 plugin windows in a DPI-unaware host start at the DPI Windows gives them",
          "[integration]") {
  if (systemDpi() == 96)
    WARN("System DPI is 96, so this can't tell the system DPI from the window's DPI");

  HostWindow parent(kPlainHost, 400, 300, DPI_AWARENESS_CONTEXT_UNAWARE);
  REQUIRE(parent.hwnd);
  REQUIRE(GetDpiForWindow(parent.hwnd) == 96);
  checkPluginStartsAtItsOwnDpi(parent.hwnd);
}

TEST_CASE("Win32 plugin windows in a per-monitor-aware host start at their monitor's DPI",
          "[integration]") {
  HostWindow parent(kPlainHost, 400, 300, DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
  REQUIRE(parent.hwnd);
  checkPluginStartsAtItsOwnDpi(parent.hwnd);
}

TEST_CASE("Win32 plugin windows pass the host window's WM_SIZING on after adjusting it",
          "[integration]") {
  ScopedDpiAwareness per_monitor(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
  HostWindow parent(kPerMonitorHost, 200, 100, DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
  REQUIRE(parent.hwnd);

  ApplicationWindow app;
  app.setWindowDimensions(Dimension::nativePixels(200), Dimension::nativePixels(100));
  app.setFixedAspectRatio(true);
  app.show(Dimension::nativePixels(200), Dimension::nativePixels(100), parent.hwnd);
  REQUIRE(app.window());
  pump(30);

  host_log = {};
  RECT sizing = { 0, 0, 300, 100 };
  SendMessageW(parent.hwnd, WM_SIZING, WMSZ_RIGHT, reinterpret_cast<LPARAM>(&sizing));

  CHECK(sizing.right - sizing.left == 300);
  CHECK(sizing.bottom - sizing.top == 150);
  CHECK(host_log.sizing == 1);
  CHECK(host_log.sizing_rect.bottom - host_log.sizing_rect.top == 150);
}

TEST_CASE("Win32 plugin windows let the host handle its own WM_DPICHANGED", "[integration]") {
  ScopedDpiAwareness per_monitor(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
  HostWindow parent(kPerMonitorHost, 200, 100, DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
  REQUIRE(parent.hwnd);

  ApplicationWindow app;
  app.show(Dimension::nativePixels(200), Dimension::nativePixels(100), parent.hwnd);
  REQUIRE(app.window());
  HWND plugin = static_cast<HWND>(app.window()->nativeHandle());
  pump(30);

  UINT dpi = GetDpiForWindow(parent.hwnd);
  float scale = GetDpiForWindow(plugin) / 96.0f;
  app.window()->setDpiScale(otherScale(scale));

  RECT before;
  GetWindowRect(parent.hwnd, &before);
  RECT suggested = { before.left + 5, before.top + 5, before.left + 5 + 260, before.top + 5 + 130 };
  host_log = {};
  SendMessageW(parent.hwnd, WM_DPICHANGED, MAKEWPARAM(dpi, dpi), reinterpret_cast<LPARAM>(&suggested));

  CHECK(host_log.dpi_changes == 1);

  RECT after;
  GetWindowRect(parent.hwnd, &after);
  CHECK(after.left == suggested.left);
  CHECK(after.top == suggested.top);
  CHECK(after.right == suggested.right);
  CHECK(after.bottom == suggested.bottom);

  CHECK(app.window()->dpiScale() == scale);
  CHECK(app.window()->clientWidth() == 260);
  CHECK(app.window()->clientHeight() == 130);
}

TEST_CASE("Win32 standalone windows take WM_DPICHANGED's suggested rectangle", "[integration]") {
  ScopedDpiAwareness per_monitor(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
  std::unique_ptr<Window> window = createWindow(Dimension::nativePixels(300),
                                                Dimension::nativePixels(200));
  REQUIRE(window);
  HWND hwnd = static_cast<HWND>(window->nativeHandle());
  window->show();
  pump(30);

  UINT dpi = GetDpiForWindow(hwnd);
  RECT before;
  GetWindowRect(hwnd, &before);
  RECT suggested = { before.left + 10, before.top + 10, before.right + 50, before.bottom + 40 };
  SendMessageW(hwnd, WM_DPICHANGED, MAKEWPARAM(dpi, dpi), reinterpret_cast<LPARAM>(&suggested));
  pump(20);

  RECT after;
  GetWindowRect(hwnd, &after);
  CHECK(after.left == suggested.left);
  CHECK(after.top == suggested.top);
  CHECK(after.right == suggested.right);
  CHECK(after.bottom == suggested.bottom);

  RECT client;
  GetClientRect(hwnd, &client);
  CHECK(window->clientWidth() == client.right - client.left);
  CHECK(window->clientHeight() == client.bottom - client.top);
  CHECK(window->dpiScale() == dpi / 96.0f);

  window.reset();
  takePendingQuit();
}

TEST_CASE("Win32 plugin windows report their new size when their parent's DPI changes",
          "[integration]") {
  static constexpr int kLogicalWidth = 192;
  static constexpr int kLogicalHeight = 96;

  ScopedDpiAwareness per_monitor(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
  HostWindow parent(kPerMonitorHost, kLogicalWidth, kLogicalHeight,
                    DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
  REQUIRE(parent.hwnd);
  UINT dpi = GetDpiForWindow(parent.hwnd);
  float scale = dpi / 96.0f;
  float stale_scale = scale == 1.0f ? 2.0f : 1.0f;
  int old_width = static_cast<int>(kLogicalWidth * stale_scale);
  int old_height = static_cast<int>(kLogicalHeight * stale_scale);
  int new_width = 2 * static_cast<int>(dpi);
  int new_height = static_cast<int>(dpi);
  SetWindowPos(parent.hwnd, nullptr, 0, 0, old_width, old_height, SWP_NOZORDER | SWP_NOACTIVATE | SWP_NOMOVE);

  std::unique_ptr<Window> window = createPluginWindow(Dimension::nativePixels(old_width),
                                                      Dimension::nativePixels(old_height), parent.hwnd);
  REQUIRE(window);
  window->setDpiScale(stale_scale);

  ApplicationEditor editor;
  editor.addToWindow(window.get());
  REQUIRE(editor.nativeWidth() == old_width);
  REQUIRE(editor.width() == kLogicalWidth);
  REQUIRE(editor.height() == kLogicalHeight);

  int reports = 0;
  editor.onWindowContentsResized() = [&reports] { reports++; };

  RECT window_rect;
  GetWindowRect(parent.hwnd, &window_rect);
  RECT suggested = { window_rect.left, window_rect.top, window_rect.left + new_width,
                     window_rect.top + new_height };
  host_log.dpi_changes = 0;
  SendMessageW(parent.hwnd, WM_DPICHANGED, MAKEWPARAM(dpi, dpi), reinterpret_cast<LPARAM>(&suggested));
  INFO("Host saw " << host_log.dpi_changes << " WM_DPICHANGED");

  CHECK(window->dpiScale() == scale);
  CHECK(editor.dpiScale() == scale);
  CHECK(editor.nativeWidth() == new_width);
  CHECK(editor.nativeHeight() == new_height);
  CHECK(window->clientWidth() == new_width);
  CHECK(reports == 1);

  editor.removeFromWindow();
}

#endif
