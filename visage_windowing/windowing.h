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
#include "visage_utils/keycodes.h"
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

    static void setDoubleClickSpeed(int ms) { double_click_speed_ = ms; }
    static int doubleClickSpeed() { return double_click_speed_; }

    class EventHandler {
    public:
      virtual ~EventHandler() = default;

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
      virtual visage::Bounds dragDropSourceBounds() = 0;
      virtual void cleanupDragDropSource() = 0;
    };

    Window() = delete;
    Window(const Window&) = delete;

    explicit Window(float aspect_ratio);
    Window(int width, int height);
    virtual ~Window() = default;

    virtual void runEventLoop() = 0;
    virtual void* nativeHandle() const = 0;
    virtual void windowContentsResized(int width, int height) = 0;

    virtual void* initWindow() const { return nullptr; }
    virtual void* globalDisplay() const { return nullptr; }
    virtual void processPluginFdEvents() { }
    virtual int posixFd() const { return 0; }

    virtual void show() = 0;
    virtual void showMaximized() = 0;
    virtual void hide() = 0;
    virtual void setWindowTitle(const std::string& title) = 0;
    virtual Point maxWindowDimensions() const = 0;
    virtual Point minWindowDimensions() const = 0;

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

    Point lastWindowMousePosition() const { return last_window_mouse_position_; }
    Point convertPointToWindowPosition(const Point& point) const {
      return { static_cast<int>(std::round(point.x / pixel_scale_)),
               static_cast<int>(std::round(point.y / pixel_scale_)) };
    }
    Point convertPointToFramePosition(const Point& point) const {
      return { static_cast<int>(std::round(point.x * pixel_scale_)),
               static_cast<int>(std::round(point.y * pixel_scale_)) };
    }

    void setWindowSize(int width, int height);
    void setInternalWindowSize(int width, int height);
    void setDpiScale(float scale) { dpi_scale_ = scale; }
    float dpiScale() const { return dpi_scale_; }
    void setPixelScale(float scale) { pixel_scale_ = scale; }
    float pixelScale() const { return pixel_scale_; }
    void setMouseRelativeMode(bool relative) { mouse_relative_mode_ = relative; }
    virtual bool mouseRelativeMode() const { return mouse_relative_mode_; }

    int clientWidth() const { return client_width_; }
    int clientHeight() const { return client_height_; }
    void setEventHandler(EventHandler* event_handler) { event_handler_ = event_handler; }
    void clearEventHandler() { event_handler_ = nullptr; }

    bool hasActiveTextEntry() const;

    void handleMouseMove(int x, int y, int button_state, int modifiers);
    void handleMouseDown(MouseButton button_id, int x, int y, int button_state, int modifiers);
    void handleMouseUp(MouseButton button_id, int x, int y, int button_state, int modifiers) const;
    void handleMouseEnter(int x, int y);
    void handleMouseLeave(int button_state, int modifiers) const;

    void handleMouseWheel(float delta_x, float delta_y, float precise_x, float precise_y, int x,
                          int y, int button_state, int modifiers, bool momentum = false) const;
    void handleMouseWheel(float delta_x, float delta_y, int x, int y, int button_state,
                          int modifiers, bool momentum = false) const;

    void handleFocusLost();
    void handleFocusGained() const;
    void handleResized(int width, int height);

    bool handleKeyDown(KeyCode key_code, int modifiers, bool repeat) const;
    bool handleKeyUp(KeyCode key_code, int modifiers) const;
    bool handleTextInput(const std::string& text) const;

    bool handleFileDrag(int x, int y, const std::vector<std::string>& files) const;
    void handleFileDragLeave() const;
    bool handleFileDrop(int x, int y, const std::vector<std::string>& files) const;

    bool isDragDropSource() const;
    std::string startDragDropSource() const;
    Bounds dragDropSourceBounds();
    void cleanupDragDropSource() const;
    void setVisible(bool visible) { visible_ = visible; }

  private:
    static int double_click_speed_;

    struct RepeatClick {
      int click_count = 0;
      long long last_click_ms = 0;
    };

    EventHandler* event_handler_ = nullptr;
    Point last_window_mouse_position_ = { 0, 0 };
    RepeatClick mouse_repeat_clicks_;

    std::function<void(double)> draw_callback_ = nullptr;
    float dpi_scale_ = 1.0f;
    float pixel_scale_ = 1.0f;
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
  float windowPixelScale();
  void showMessageBox(std::string title, std::string message);
  std::string readClipboardText();
  void setClipboardText(const std::string& text);

  int doubleClickSpeed();
  void setDoubleClickSpeed(int ms);

  std::unique_ptr<Window> createWindow(Dimension x, Dimension y, Dimension width, Dimension height,
                                       bool popup = false);
  std::unique_ptr<Window> createPluginWindow(Dimension width, Dimension height, void* parent_handle);

  inline std::unique_ptr<Window> createWindow(Dimension width, Dimension height, bool popup = false) {
    return createWindow({}, {}, width, height, popup);
  }
}
