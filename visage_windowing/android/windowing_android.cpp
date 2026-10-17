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

#include "windowing_android.h"

#include <android/input.h>
#include <android/log.h>
#include <android_native_app_glue.h>
#include <cmath>
#include <utility>

namespace visage {

  struct android_app* WindowAndroid::global_android_app_ = nullptr;
  ANativeWindow* WindowAndroid::global_native_window_ = nullptr;
  float WindowAndroid::global_dpi_scale_ = 2.75f;

  void WindowAndroid::SetGlobalAndroidApp(struct android_app* app) {
    global_android_app_ = app;
  }

  struct android_app* WindowAndroid::GlobalAndroidApp() {
    return global_android_app_;
  }

  void WindowAndroid::SetGlobalNativeWindow(ANativeWindow* window) {
    global_native_window_ = window;
  }

  ANativeWindow* WindowAndroid::GlobalNativeWindow() {
    return global_native_window_;
  }

  void WindowAndroid::SetGlobalDpiScale(float scale) {
    if (scale > 0.0f) {
      global_dpi_scale_ = scale;
    }
  }

  float WindowAndroid::GlobalDpiScale() {
    return global_dpi_scale_;
  }

  WindowAndroid::WindowAndroid(int width, int height) :
      Window(width, height), start_time_(std::chrono::steady_clock::now()) {
    native_window_ = global_native_window_;
    if (native_window_ != nullptr) {
      ANativeWindow_acquire(native_window_);
    }
    setDpiScale(defaultDpiScale());
  }

  WindowAndroid::~WindowAndroid() {
    if (native_window_ != nullptr) {
      ANativeWindow_release(native_window_);
      native_window_ = nullptr;
    }
  }

  void WindowAndroid::setNativeWindow(ANativeWindow* window) {
    if (native_window_ == window) {
      return;
    }

    if (native_window_ != nullptr) {
      ANativeWindow_release(native_window_);
    }

    native_window_ = window;

    if (native_window_ != nullptr) {
      ANativeWindow_acquire(native_window_);
      int width = ANativeWindow_getWidth(native_window_);
      int height = ANativeWindow_getHeight(native_window_);
      if (width > 0 && height > 0) {
        windowContentsResized(width, height);
      }
    }
  }

  void WindowAndroid::runEventLoop() {
    struct android_app* app = global_android_app_;
    if (app == nullptr) {
      return;
    }

    app->userData = this;
    app->onAppCmd = [](struct android_app* app, int32_t cmd) {
      auto* window = static_cast<WindowAndroid*>(app->userData);
      if (window == nullptr) {
        return;
      }

      switch (cmd) {
      case APP_CMD_INIT_WINDOW:
        if (app->window != nullptr) {
          window->setNativeWindow(app->window);
        }
        break;
      case APP_CMD_TERM_WINDOW: window->setNativeWindow(nullptr); break;
      case APP_CMD_WINDOW_RESIZED:
        if (app->window != nullptr) {
          int width = ANativeWindow_getWidth(app->window);
          int height = ANativeWindow_getHeight(app->window);
          if (width > 0 && height > 0) {
            window->windowContentsResized(width, height);
          }
        }
        break;
      default: break;
      }
    };

    app->onInputEvent = [](struct android_app* app, AInputEvent* event) -> int32_t {
      auto* window = static_cast<WindowAndroid*>(app->userData);
      if (window == nullptr || event == nullptr) {
        return 0;
      }

      if (AInputEvent_getType(event) == AINPUT_EVENT_TYPE_MOTION) {
        const int32_t action = AMotionEvent_getAction(event);
        const float x = AMotionEvent_getX(event, 0);
        const float y = AMotionEvent_getY(event, 0);
        window->handleTouch(action, x, y);
        return 1;
      }
      return 0;
    };

    while (!app->destroyRequested) {
      int events = 0;
      struct android_poll_source* source = nullptr;
      int timeout = (is_showing_ && native_window_ != nullptr) ? 0 : -1;
      while (ALooper_pollOnce(timeout, nullptr, &events, reinterpret_cast<void**>(&source)) >= 0) {
        if (source != nullptr) {
          source->process(app, source);
        }
        if (app->destroyRequested) {
          return;
        }
      }
      if (native_window_ != nullptr && is_showing_) {
        const auto now = std::chrono::steady_clock::now();
        const double time_sec = std::chrono::duration<double>(now - start_time_).count();
        drawCallback(time_sec);
      }
    }
  }

  void WindowAndroid::windowContentsResized(int width, int height) {
    setInternalWindowSize(width, height);
    handleResized(width, height);
  }

  void WindowAndroid::handleTouch(int action, float x, float y) {
    const int px = static_cast<int>(std::round(x));
    const int py = static_cast<int>(std::round(y));
    const int masked_action = action & AMOTION_EVENT_ACTION_MASK;

    switch (masked_action) {
    case AMOTION_EVENT_ACTION_DOWN:
    case AMOTION_EVENT_ACTION_POINTER_DOWN:
      handleMouseDown(kMouseButtonLeft, px, py, kMouseButtonLeft, kModifierNone);
      break;
    case AMOTION_EVENT_ACTION_UP:
    case AMOTION_EVENT_ACTION_POINTER_UP:
      handleMouseUp(kMouseButtonLeft, px, py, kMouseButtonNone, kModifierNone);
      break;
    case AMOTION_EVENT_ACTION_MOVE: handleMouseMove(px, py, kMouseButtonLeft, kModifierNone); break;
    case AMOTION_EVENT_ACTION_CANCEL:
      handleMouseUp(kMouseButtonLeft, px, py, kMouseButtonNone, kModifierNone);
      break;
    default: break;
    }
  }

  std::string readClipboardText() {
    return {};
  }

  void setClipboardText(const std::string& /*text*/) { }

  void setCursorStyle(MouseCursor /*style*/) { }

  void setCursorVisible(bool /*visible*/) { }

  float defaultDpiScale() {
    return WindowAndroid::GlobalDpiScale();
  }

  Point cursorPosition() {
    return { 0.0f, 0.0f };
  }

  void setCursorPosition(Point /*window_position*/) { }

  void setCursorScreenPosition(Point /*screen_position*/) { }

  bool isMobileDevice() {
    return true;
  }

  void showMessageBox(std::string title, std::string message) {
    VISAGE_LOG("%s: %s", title.c_str(), message.c_str());
  }

  void closeApplication() { }

  std::unique_ptr<Window> createPluginWindow(const Dimension& /*width*/,
                                             const Dimension& /*height*/, void* /*parent_handle*/) {
    return nullptr;
  }

  static std::pair<int, int> getDisplayDimensions() {
    int display_width = 1080;
    int display_height = 2400;

    ANativeWindow* global_win = WindowAndroid::GlobalNativeWindow();
    if (global_win != nullptr) {
      int w = ANativeWindow_getWidth(global_win);
      int h = ANativeWindow_getHeight(global_win);
      if (w > 0 && h > 0) {
        display_width = w;
        display_height = h;
      }
    }
    return { display_width, display_height };
  }

  IBounds computeWindowBounds(const Dimension& x, const Dimension& y, const Dimension& width,
                              const Dimension& height) {
    const float scale = defaultDpiScale();
    const auto [display_width, display_height] = getDisplayDimensions();

    const int w = width.compute(scale, display_width, display_height);
    const int h = height.compute(scale, display_width, display_height);
    const int x_pos = x.compute(scale, display_width, display_height);
    const int y_pos = y.compute(scale, display_width, display_height);

    return IBounds(x_pos, y_pos, w > 0 ? w : display_width, h > 0 ? h : display_height);
  }

  std::unique_ptr<Window> createWindow(const Dimension& /*x*/, const Dimension& /*y*/,
                                       const Dimension& width, const Dimension& height,
                                       Window::Decoration /*decoration*/) {
    const float scale = defaultDpiScale();
    const auto [display_width, display_height] = getDisplayDimensions();

    const int w = width.compute(scale, display_width, display_height);
    const int h = height.compute(scale, display_width, display_height);

    return std::make_unique<WindowAndroid>(w, h);
  }

}  // namespace visage
