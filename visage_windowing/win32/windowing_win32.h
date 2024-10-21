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

#if VISAGE_WINDOWS
#define NOMINMAX

#include "windowing.h"

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

    WindowWin32(int x, int y, int width, int height, bool popup);
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
    void hide() override;
    void setWindowTitle(const std::string& title) override;
    Point maxWindowDimensions() const override;
    Point minWindowDimensions() const override;

    bool isMouseTracked() const { return mouse_tracked_; }

    void setMouseTracked(bool tracked) { mouse_tracked_ = tracked; }

    bool handleCharacterEntry(wchar_t character);
    bool handleHookedMessage(const MSG* message);
    void handleResizing(HWND hwnd, LPARAM l_param, WPARAM w_param);
    void handleResizeEnd(HWND hwnd);
    void handleDpiChange(HWND hwnd, LPARAM l_param, WPARAM w_param);
    void updateMonitor();
    HMONITOR monitor() const { return monitor_; }
    WNDPROC parentWindowProc() const { return parent_window_proc_; }

  private:
    void registerWindowClass();

    std::string unique_window_class_name_;
    WNDCLASSEX window_class_ = {};
    HWND window_handle_ = nullptr;
    HWND parent_handle_ = nullptr;
    HMODULE module_handle_ = nullptr;
    HMONITOR monitor_ = nullptr;
    WNDPROC parent_window_proc_ = nullptr;
    std::unique_ptr<EventHooks> event_hooks_;
    DragDropTarget* drag_drop_target_ = nullptr;
    std::unique_ptr<VBlankThread> v_blank_thread_;

    std::wstring utf16_string_entry_;
    bool initialized_ = false;
    bool mouse_tracked_ = false;
    int mouse_down_flags_ = 0;
  };
}

#endif