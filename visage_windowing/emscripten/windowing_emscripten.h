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

#if VISAGE_EMSCRIPTEN
#include "windowing.h"

namespace visage {
  class WindowEmscripten : public Window {
  public:
    static WindowEmscripten* running_instance_;
    static WindowEmscripten* runningInstance() { return running_instance_; }

    WindowEmscripten(int width, int height);

    void* initWindow() const override { return (void*)"#canvas"; }
    void* nativeHandle() const override { return (void*)"#canvas"; }

    void runEventLoop() override;
    void windowContentsResized(int width, int height) override;
    void show() override { }
    void showMaximized() override;
    bool maximized() { return false; }
    void hide() override { }
    void close() override { }
    bool isShowing() const override { return true; }
    void setWindowTitle(const std::string& title) override;
    IPoint maxWindowDimensions() const override;
    IPoint minWindowDimensions() const override;
    int initialWidth() const { return initial_width_; }
    int initialHeight() const { return initial_height_; }
    bool mouseRelativeMode() const override { return false; }

    void handleWindowResize(int window_width, int window_height);
    void runLoopCallback();

  private:
    int initial_width_ = 0;
    int initial_height_ = 0;
    float display_scale_ = 1.0f;
    bool maximized_ = false;
    int mouse_x_ = 0;
    int mouse_y_ = 0;
    long long start_microseconds_ = 0;
  };
}

#endif
