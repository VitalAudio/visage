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
#include <functional>
#include <memory>

#if VISAGE_WINDOWS
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

using namespace visage;

namespace {
  bool pumpUntil(const std::function<bool()>& done, int timeout_ms) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
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

      if (done && done())
        return true;
    }
    return false;
  }

  void pump(int milliseconds) {
    pumpUntil(nullptr, milliseconds);
  }

  bool takePendingQuit() {
    bool quit = false;
    MSG message;
    for (int i = 0; i < 10000 && PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE); ++i) {
      if (message.message == WM_QUIT)
        quit = true;
      else {
        TranslateMessage(&message);
        DispatchMessageW(&message);
      }
    }
    return quit;
  }

  std::function<void()> timer_tick;

  void CALLBACK timerTickProc(HWND, UINT, UINT_PTR, DWORD) {
    if (timer_tick)
      timer_tick();
  }

  struct ThreadTimer {
    explicit ThreadTimer(UINT ms) : id(SetTimer(nullptr, 0, ms, timerTickProc)) { }
    ~ThreadTimer() {
      KillTimer(nullptr, id);
      timer_tick = nullptr;
    }

    UINT_PTR id = 0;
  };

  struct FrameCounter {
    explicit FrameCounter(Window* window) : window(window) {
      window->setDrawCallback([this](double) { count++; });
    }

    ~FrameCounter() { detach(); }

    void detach() {
      if (window)
        window->setDrawCallback(nullptr);
      window = nullptr;
    }

    bool advance(int frames = 2) {
      int target = count + frames;
      return pumpUntil([&] { return count >= target; }, 2000);
    }

    Window* window = nullptr;
    int count = 0;
  };

  struct HostWindow {
    HostWindow() {
      static const wchar_t* kClassName = L"VisageWin32LifecycleTestHost";
      static bool registered = false;
      if (!registered) {
        WNDCLASSEXW window_class = {};
        window_class.cbSize = sizeof(window_class);
        window_class.lpfnWndProc = DefWindowProcW;
        window_class.hInstance = GetModuleHandleW(nullptr);
        window_class.hbrBackground = static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));
        window_class.lpszClassName = kClassName;
        registered = RegisterClassExW(&window_class) != 0;
      }
      hwnd = CreateWindowExW(WS_EX_TOOLWINDOW | WS_EX_NOACTIVATE, kClassName, L"host", WS_POPUP | WS_VISIBLE,
                             40, 40, 200, 150, nullptr, nullptr, GetModuleHandleW(nullptr), nullptr);
    }

    ~HostWindow() {
      if (IsWindow(hwnd))
        DestroyWindow(hwnd);
    }

    HWND hwnd = nullptr;
  };
}

TEST_CASE("Win32 destroying an open window from code doesn't post WM_QUIT", "[integration]") {
  takePendingQuit();

  std::unique_ptr<Window> window = createWindow(Dimension::nativePixels(160),
                                                Dimension::nativePixels(120));
  REQUIRE(window);
  HWND hwnd = static_cast<HWND>(window->nativeHandle());
  REQUIRE(IsWindow(hwnd));
  window->show();
  pump(50);
  REQUIRE(IsWindowVisible(hwnd));

  window.reset();
  CHECK_FALSE(IsWindow(hwnd));

  CHECK_FALSE(takePendingQuit());
}

TEST_CASE("Win32 destroying a window from a timer doesn't end the running event loop",
          "[integration]") {
  takePendingQuit();

  std::unique_ptr<Window> loop_window = createWindow(Dimension::nativePixels(100),
                                                     Dimension::nativePixels(80));
  std::unique_ptr<Window> other = createWindow(Dimension::nativePixels(160), Dimension::nativePixels(120));
  REQUIRE(loop_window);
  REQUIRE(other);
  other->show();
  pump(30);

  static constexpr int kTicksAfterDestroy = 5;
  bool destroyed = false;
  int ticks_after_destroy = 0;
  int total_ticks = 0;
  timer_tick = [&] {
    if (++total_ticks > 200) {
      PostQuitMessage(0);
      return;
    }
    if (!destroyed) {
      other.reset();
      destroyed = true;
      return;
    }
    if (++ticks_after_destroy == kTicksAfterDestroy)
      PostQuitMessage(0);
  };

  {
    ThreadTimer timer(15);
    loop_window->runEventLoop();
  }

  CHECK(destroyed);
  CHECK(ticks_after_destroy == kTicksAfterDestroy);

  loop_window.reset();
  takePendingQuit();
}

TEST_CASE("Win32 closing the last window from its title bar ends the event loop", "[integration]") {
  takePendingQuit();

  std::unique_ptr<Window> window = createWindow(Dimension::nativePixels(160),
                                                Dimension::nativePixels(120));
  REQUIRE(window);
  HWND hwnd = static_cast<HWND>(window->nativeHandle());
  window->show();
  pump(30);

  bool timed_out = false;
  timer_tick = [&] {
    timed_out = true;
    PostQuitMessage(0);
  };

  {
    ThreadTimer timeout(2000);
    PostMessageW(hwnd, WM_NCLBUTTONUP, HTCLOSE, 0);
    window->runEventLoop();
  }

  CHECK_FALSE(timed_out);
  CHECK_FALSE(IsWindow(hwnd));

  window.reset();
  CHECK_FALSE(takePendingQuit());
}

TEST_CASE("Win32 hiding and showing a window inside the event loop keeps the loop running",
          "[integration]") {
  takePendingQuit();

  ApplicationWindow app;
  app.show(Dimension::nativePixels(100), Dimension::nativePixels(80));
  REQUIRE(app.window());
  void* handle = app.window()->nativeHandle();

  int step = 0;
  int shown_ticks = 0;
  int total_ticks = 0;
  bool same_handle = false;
  bool timed_out = false;
  timer_tick = [&] {
    if (++total_ticks > 150) {
      timed_out = true;
      PostQuitMessage(0);
      return;
    }

    if (step == 0) {
      app.hide();
      step++;
    }
    else if (step == 1) {
      app.show(Dimension::nativePixels(120), Dimension::nativePixels(90));
      same_handle = app.window() && app.window()->nativeHandle() == handle;
      step++;
    }
    else if (step == 2 && app.isShowing() && ++shown_ticks == 5) {
      step++;
      app.close();
    }
  };

  {
    ThreadTimer timer(15);
    app.runEventLoop();
  }

  CHECK_FALSE(timed_out);
  CHECK(step == 3);
  CHECK(shown_ticks == 5);
  CHECK(same_handle);
  CHECK(app.nativeWidth() == 120);
  CHECK(app.nativeHeight() == 90);
  takePendingQuit();
}

TEST_CASE("Win32 windows aren't visible while hidden or minimized", "[integration]") {
  std::unique_ptr<Window> window = createWindow(Dimension::nativePixels(160),
                                                Dimension::nativePixels(120));
  REQUIRE(window);
  HWND hwnd = static_cast<HWND>(window->nativeHandle());
  FrameCounter frames(window.get());

  window->show();
  REQUIRE(frames.advance());
  CHECK(window->isVisible());

  window->hide();
  REQUIRE(frames.advance());
  CHECK_FALSE(window->isVisible());

  window->show();
  REQUIRE(frames.advance());
  CHECK(window->isVisible());

  ShowWindow(hwnd, SW_SHOWMINNOACTIVE);
  REQUIRE(IsIconic(hwnd));
  REQUIRE(frames.advance());
  CHECK_FALSE(window->isVisible());

  ShowWindow(hwnd, SW_RESTORE);
  REQUIRE_FALSE(IsIconic(hwnd));
  REQUIRE(frames.advance());
  CHECK(window->isVisible());

  frames.detach();
  window.reset();
  takePendingQuit();
}

TEST_CASE("Win32 plugin windows aren't visible while the host window is hidden or minimized",
          "[integration]") {
  HostWindow parent;
  REQUIRE(parent.hwnd);
  std::unique_ptr<Window> window = createPluginWindow(Dimension::nativePixels(100),
                                                      Dimension::nativePixels(80), parent.hwnd);
  REQUIRE(window);
  FrameCounter frames(window.get());

  window->show();
  REQUIRE(frames.advance());
  CHECK(window->isVisible());

  ShowWindow(parent.hwnd, SW_HIDE);
  REQUIRE(frames.advance());
  CHECK_FALSE(window->isVisible());

  ShowWindow(parent.hwnd, SW_SHOWNA);
  REQUIRE(frames.advance());
  CHECK(window->isVisible());

  ShowWindow(parent.hwnd, SW_SHOWMINNOACTIVE);
  REQUIRE(IsIconic(parent.hwnd));
  REQUIRE(frames.advance());
  CHECK_FALSE(window->isVisible());

  ShowWindow(parent.hwnd, SW_RESTORE);
  REQUIRE_FALSE(IsIconic(parent.hwnd));
  REQUIRE(frames.advance());
  CHECK(window->isVisible());

  frames.detach();
  window.reset();
}

TEST_CASE("Win32 hidden application windows stop drawing but keep running their timers",
          "[integration]") {
  struct Ticker : EventTimer {
    void timerCallback() override { ticks++; }
    int ticks = 0;
  };

  ApplicationWindow app;
  Frame frame;
  int draws = 0;
  frame.onDraw() = [&](Canvas&) {
    draws++;
    frame.redraw();
  };
  app.addChild(frame);
  app.show(Dimension::nativePixels(160), Dimension::nativePixels(120));
  REQUIRE(app.window());
  frame.setBounds(0, 0, 40, 40);
  frame.redraw();

  Ticker ticker;
  ticker.startTimer(5);
  REQUIRE(pumpUntil([&] { return draws >= 3 && ticker.ticks >= 3; }, 2000));

  int draws_at_hide = draws;
  int ticks_at_hide = ticker.ticks;
  app.hide();
  pump(200);
  CHECK(draws == draws_at_hide);
  CHECK(ticker.ticks > ticks_at_hide);

  app.show();
  CHECK(pumpUntil([&] { return draws > draws_at_hide; }, 2000));

  ticker.stopTimer();
  app.close();
  pump(30);
  takePendingQuit();
}

#endif
