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

#if __has_include("../windowing.h")
#include "../windowing.h"
#elif __has_include(<visage/windowing.h>)
#include <visage/windowing.h>
#else
#include "windowing.h"
#endif

#include <android/native_window.h>
#include <chrono>
#include <string>

struct android_app;

namespace visage {

  class WindowAndroid : public Window {
  public:
    static void SetGlobalAndroidApp(struct android_app* app);
    static struct android_app* GlobalAndroidApp();

    static void SetGlobalNativeWindow(ANativeWindow* window);
    static ANativeWindow* GlobalNativeWindow();

    static void SetGlobalDpiScale(float scale);
    static float GlobalDpiScale();

    WindowAndroid(int width, int height);
    ~WindowAndroid() override;

    void* initWindow() const override { return native_window_; }

    void* nativeHandle() const override { return native_window_; }

    void* globalDisplay() const override { return nullptr; }

    void setNativeWindow(ANativeWindow* window);

    ANativeWindow* nativeWindow() const { return native_window_; }

    void runEventLoop() override;
    void windowContentsResized(int width, int height) override;

    void show() override { is_showing_ = true; }

    void showMaximized() override { is_showing_ = true; }

    void hide() override { is_showing_ = false; }

    void close() override { is_showing_ = false; }

    bool isShowing() const override { return is_showing_; }

    void setWindowTitle(const std::string& title) override { }

    IPoint maxWindowDimensions() const override { return { 4096, 4096 }; }

    void handleTouch(int action, float x, float y);

  private:
    static struct android_app* global_android_app_;
    static ANativeWindow* global_native_window_;
    static float global_dpi_scale_;

    ANativeWindow* native_window_ = nullptr;
    bool is_showing_ = true;
    std::chrono::steady_clock::time_point start_time_;
  };

  std::string readClipboardText();
  void setClipboardText(const std::string& text);
  void setCursorStyle(MouseCursor style);
  void setCursorVisible(bool visible);
  float defaultDpiScale();
  Point cursorPosition();
  void setCursorPosition(Point window_position);
  void setCursorScreenPosition(Point screen_position);
  bool isMobileDevice();
  void showMessageBox(std::string title, std::string message);
  void closeApplication();
  std::unique_ptr<Window> createPluginWindow(const Dimension& width, const Dimension& height,
                                             void* parent_handle);
  std::unique_ptr<Window> createWindow(const Dimension& x, const Dimension& y, const Dimension& width,
                                       const Dimension& height, Window::Decoration decoration);

}  // namespace visage
