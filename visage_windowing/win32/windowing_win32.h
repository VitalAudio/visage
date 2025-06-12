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

#if VISAGE_WINDOWS
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "windowing.h"

#include <atomic>
#include <windows.h>

namespace visage {
  class DragDropTarget;
  class VBlankThread;

  class EventHooks {
  public:
    static LRESULT CALLBACK eventHook(int code, WPARAM w_param, LPARAM l_param);

    EventHooks();
    ~EventHooks();

  private:
    static int instance_count_;
    static HHOOK event_hook_;
  };

  class WindowWin32 : public Window {
  public:
    static constexpr int kTimerId = 1;
    static HCURSOR cursor_;

    static void setCursor(HCURSOR cursor);
    static HCURSOR cursor() { return cursor_; }

    WindowWin32(int x, int y, int width, int height, Decoration decoration);
    WindowWin32(int width, int height, void* parent_handle);

    ~WindowWin32() override;

    void finishWindowSetup();

    void runEventLoop() override;
    LRESULT handleWindowProc(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param);

    HWND windowHandle() const { return window_handle_; }
    void* nativeHandle() const override { return window_handle_; }
    HWND parentHandle() const { return parent_handle_; }

    void windowContentsResized(int width, int height) override;
    void show() override;
    void showMaximized() override;
    void hide() override;
    void close() override;
    bool isShowing() const override;
    void setWindowTitle(const std::string& title) override;
    IPoint maxWindowDimensions() const override;
    IPoint minWindowDimensions() const override;

    bool isMouseTracked() const { return mouse_tracked_; }

    void setMouseTracked(bool tracked) { mouse_tracked_ = tracked; }

    bool handleCharacterEntry(wchar_t character);
    bool handleHookedMessage(const MSG* message);
    void handleResizing(HWND hwnd, LPARAM l_param, WPARAM w_param);
    void handleResizeEnd(HWND hwnd);
    void handleDpiChange(HWND hwnd, LPARAM l_param, WPARAM w_param);
    void updateMonitor();
    HMONITOR monitor() const { return monitor_.load(); }
    WNDPROC parentWindowProc() const { return parent_window_proc_; }
    Window::Decoration decoration() const { return decoration_; }

  private:
    void show(int show_flag);
    void registerWindowClass();

    std::wstring unique_window_class_name_;
    WNDCLASSEX window_class_ = {};
    HWND window_handle_ = nullptr;
    HWND parent_handle_ = nullptr;
    HMODULE module_handle_ = nullptr;
    std::atomic<HMONITOR> monitor_ = nullptr;
    WNDPROC parent_window_proc_ = nullptr;
    std::unique_ptr<EventHooks> event_hooks_;
    DragDropTarget* drag_drop_target_ = nullptr;
    std::unique_ptr<VBlankThread> v_blank_thread_;

    Window::Decoration decoration_ = Window::Decoration::Native;
    std::wstring utf16_string_entry_;
    bool initialized_ = false;
    bool mouse_tracked_ = false;
    int mouse_down_flags_ = 0;
  };
}

#endif