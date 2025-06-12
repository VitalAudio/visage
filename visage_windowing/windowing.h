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

#include "visage_utils/defines.h"
#include "visage_utils/dimension.h"
#include "visage_utils/events.h"
#include "visage_utils/space.h"

#include <climits>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace visage {
  class Window {
  public:
    static constexpr float kDefaultDpi = 96.0f;
    static constexpr float kDefaultMinWindowScale = 0.1f;

    enum class Decoration {
      Native,
      Client,
      Popup
    };

    static void setDoubleClickSpeed(int ms) { double_click_speed_ = ms; }
    static int doubleClickSpeed() { return double_click_speed_; }

    class EventHandler {
    public:
      virtual ~EventHandler() = default;

      virtual HitTestResult handleHitTest(int x, int y) = 0;
      virtual HitTestResult currentHitTest() const = 0;
      virtual void handleMouseMove(int x, int y, int button_state, int modifiers) = 0;
      virtual void handleMouseDown(MouseButton button_id, int x, int y, int button_state,
                                   int modifiers, int repeat_clicks) = 0;
      virtual void handleMouseUp(MouseButton button_id, int x, int y, int button_state,
                                 int modifiers, int repeat_clicks) = 0;
      virtual void handleMouseEnter(int x, int y) = 0;
      virtual void handleMouseLeave(int last_x, int last_y, int button_state, int modifiers) = 0;
      virtual void handleMouseWheel(float delta_x, float delta_y, float precise_x, float precise_y,
                                    int mouse_x, int mouse_y, int button_state, int modifiers,
                                    bool momentum) = 0;

      virtual bool handleKeyDown(KeyCode key_code, int modifiers, bool repeat) = 0;
      virtual bool handleKeyUp(KeyCode key_code, int modifiers) = 0;

      virtual bool handleTextInput(const std::string& text) = 0;
      virtual bool hasActiveTextEntry() = 0;

      virtual void handleFocusLost() = 0;
      virtual void handleFocusGained() = 0;
      virtual void handleResized(int width, int height) = 0;

      virtual bool handleFileDrag(int x, int y, const std::vector<std::string>& files) = 0;
      virtual void handleFileDragLeave() = 0;
      virtual bool handleFileDrop(int x, int y, const std::vector<std::string>& files) = 0;

      virtual bool isDragDropSource() = 0;
      virtual std::string startDragDropSource() = 0;
      virtual void cleanupDragDropSource() = 0;
    };

    Window() = delete;
    Window(const Window&) = delete;

    explicit Window(float aspect_ratio);
    Window(int width, int height);
    virtual ~Window() = default;

    auto& onShow() { return on_show_; }
    auto& onHide() { return on_hide_; }
    auto& onWindowContentsResized() { return on_contents_resized_; }

    virtual void runEventLoop() = 0;
    virtual void* nativeHandle() const = 0;
    virtual void windowContentsResized(int width, int height) = 0;
    virtual bool closeRequested() { return true; }

    virtual void* initWindow() const { return nullptr; }
    virtual void* globalDisplay() const { return nullptr; }
    virtual void processPluginFdEvents() { }
    virtual int posixFd() const { return 0; }

    virtual void show() = 0;
    virtual void showMaximized() = 0;
    virtual void hide() = 0;
    virtual void close() = 0;
    virtual bool isShowing() const = 0;
    virtual void setWindowTitle(const std::string& title) = 0;
    virtual IPoint maxWindowDimensions() const = 0;
    virtual IPoint minWindowDimensions() const = 0;

    void notifyShow() { on_show_.callback(); }
    void notifyHide() { on_hide_.callback(); }

    void setDrawCallback(std::function<void(double)> callback) {
      draw_callback_ = std::move(callback);
    }

    void drawCallback(double time) const {
      if (draw_callback_)
        draw_callback_(time);
    }

    void setMinimumWindowScale(float scale) { min_window_scale_ = scale; }
    float minimumWindowScale() const { return min_window_scale_; }
    virtual void setFixedAspectRatio(bool fixed) { fixed_aspect_ratio_ = fixed; }
    bool isFixedAspectRatio() const { return fixed_aspect_ratio_; }
    float aspectRatio() const { return aspect_ratio_; }
    bool isVisible() const { return visible_; }

    IPoint lastWindowMousePosition() const { return last_window_mouse_position_; }

    void setWindowSize(int width, int height);
    void setNativeWindowSize(int width, int height);
    void setInternalWindowSize(int width, int height);
    void setDpiScale(float scale) { dpi_scale_ = scale; }
    float dpiScale() const { return dpi_scale_; }

    IPoint convertToNative(const Point& logical_point) const {
      return { static_cast<int>(std::round(logical_point.x * dpi_scale_)),
               static_cast<int>(std::round(logical_point.y * dpi_scale_)) };
    }
    Point convertToLogical(const IPoint& point) const {
      return { point.x / dpi_scale_, point.y / dpi_scale_ };
    }

    void setMouseRelativeMode(bool relative) { mouse_relative_mode_ = relative; }
    virtual bool mouseRelativeMode() const { return mouse_relative_mode_; }

    int clientWidth() const { return client_width_; }
    int clientHeight() const { return client_height_; }
    void setEventHandler(EventHandler* event_handler) { event_handler_ = event_handler; }
    void clearEventHandler() { event_handler_ = nullptr; }

    bool hasActiveTextEntry() const;

    HitTestResult handleHitTest(int x, int y);
    HitTestResult currentHitTest() const;
    void handleMouseMove(int x, int y, int button_state, int modifiers);
    void handleMouseDown(MouseButton button_id, int x, int y, int button_state, int modifiers);
    void handleMouseUp(MouseButton button_id, int x, int y, int button_state, int modifiers);
    void handleMouseEnter(int x, int y);
    void handleMouseLeave(int button_state, int modifiers);

    void handleMouseWheel(float delta_x, float delta_y, float precise_x, float precise_y, int x,
                          int y, int button_state, int modifiers, bool momentum = false);
    void handleMouseWheel(float delta_x, float delta_y, int x, int y, int button_state,
                          int modifiers, bool momentum = false);

    void handleFocusLost();
    void handleFocusGained();
    void handleResized(int width, int height);

    bool handleKeyDown(KeyCode key_code, int modifiers, bool repeat);
    bool handleKeyUp(KeyCode key_code, int modifiers);
    bool handleTextInput(const std::string& text);

    bool handleFileDrag(int x, int y, const std::vector<std::string>& files);
    void handleFileDragLeave();
    bool handleFileDrop(int x, int y, const std::vector<std::string>& files);

    bool isDragDropSource() const;
    std::string startDragDropSource();
    void cleanupDragDropSource();
    void setVisible(bool visible) { visible_ = visible; }

  private:
    static int double_click_speed_;

    struct RepeatClick {
      int click_count = 0;
      long long last_click_ms = 0;
    };

    EventHandler* event_handler_ = nullptr;
    IPoint last_window_mouse_position_ = { 0, 0 };
    RepeatClick mouse_repeat_clicks_;

    std::function<void(double)> draw_callback_ = nullptr;
    CallbackList<void()> on_show_;
    CallbackList<void()> on_hide_;
    CallbackList<void()> on_contents_resized_;
    float dpi_scale_ = 1.0f;
    float min_window_scale_ = kDefaultMinWindowScale;
    bool visible_ = true;
    bool fixed_aspect_ratio_ = false;
    bool mouse_relative_mode_ = false;
    float aspect_ratio_ = 1.0f;
    int client_width_ = 0;
    int client_height_ = 0;

    VISAGE_LEAK_CHECKER(Window)
  };

  void setCursorStyle(MouseCursor style);
  void setCursorVisible(bool visible);
  Point cursorPosition();
  void setCursorPosition(Point window_position);
  void setCursorScreenPosition(Point screen_position);
  bool isMobileDevice();
  void showMessageBox(std::string title, std::string message);
  std::string readClipboardText();
  void setClipboardText(const std::string& text);

  int doubleClickSpeed();
  void setDoubleClickSpeed(int ms);

  float defaultDpiScale();
  IBounds computeWindowBounds(const Dimension& x, const Dimension& y, const Dimension& width,
                              const Dimension& height);

  std::unique_ptr<Window> createWindow(const Dimension& x, const Dimension& y,
                                       const Dimension& width, const Dimension& height,
                                       Window::Decoration decoration_style = Window::Decoration::Native);
  std::unique_ptr<Window> createPluginWindow(const Dimension& width, const Dimension& height,
                                             void* parent_handle);
  void* headlessWindowHandle();

  inline std::unique_ptr<Window> createWindow(const Dimension& width, const Dimension& height,
                                              Window::Decoration decoration_style = Window::Decoration::Native) {
    return createWindow({}, {}, width, height, decoration_style);
  }

  inline IBounds computeWindowBounds(const Dimension& width, const Dimension& height) {
    return computeWindowBounds({}, {}, width, height);
  }

  void closeApplication();
}
