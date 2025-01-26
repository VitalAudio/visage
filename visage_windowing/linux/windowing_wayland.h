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

#if VISAGE_LINUX && VISAGE_WAYLAND
#include "visage_utils/string_utils.h"
#include "windowing.h"
#include "xdg-shell-client-protocol.h"

#include <atomic>
#include <map>
#include <thread>
#include <wayland-client.h>

namespace visage {
  class WindowWayland : public Window {
  public:
    static WindowWayland* lastActiveWindow() { return last_active_window_; }

    WindowWayland(int x, int y, int width, int height, bool popup);
    WindowWayland(int width, int height, void* parent_handle);

    ~WindowWayland() override;

    void runEventLoop() override;
    void processPluginFdEvents() override;

    void* nativeHandle() const override { return (void*)surface_; }
    void* initWindow() const override { return (void*)surface_; }
    void* globalDisplay() const override;
    int posixFd() const override;

    void setFixedAspectRatio(bool fixed) override;

    void windowContentsResized(int width, int height) override;
    void show() override;
    void showMaximized() override;
    void hide() override;
    void setWindowTitle(const std::string& title) override;
    Point maxWindowDimensions() const override;
    Point minWindowDimensions() const override;

    xdg_toplevel* topLevel() const { return xdg_toplevel_; }
    wl_surface* surface() const { return surface_; }

  private:
    static WindowWayland* last_active_window_;

    Point retrieveWindowDimensions();
    int mouseButtonState() const;
    int modifierState() const;

    std::vector<std::string> drag_drop_files_;

    long long start_draw_microseconds_ = 0;
    Point mouse_down_position_;
    wl_surface* surface_ = nullptr;
    xdg_surface* xdg_surface_ = nullptr;
    xdg_toplevel* xdg_toplevel_ = nullptr;
  };
}

#endif
