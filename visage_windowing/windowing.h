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

#include "visage_utils/defines.h"
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
    static constexpr int kNotSet = INT_MAX;
    static constexpr float kDefaultMinWindowScale = 0.1f;

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
      virtual visage::Bounds getDragDropSourceBounds() = 0;
      virtual void cleanupDragDropSource() = 0;
    };

    Window() = delete;
    Window(const Window&) = delete;

    explicit Window(float aspect_ratio);
    Window(int width, int height);
    virtual ~Window() = default;

    virtual void runEventThread() = 0;
    virtual void* getNativeHandle() const = 0;
    virtual void windowContentsResized(int width, int height) = 0;

    virtual void* getGlobalDisplay() { return nullptr; }
    virtual void* getGlobalRootWindowHandle() { return nullptr; }
    virtual void processPluginFdEvents() { }
    virtual int getPosixFd() const { return 0; }

    virtual void show() = 0;
    virtual void hide() = 0;
    virtual void setWindowTitle(const std::string& title) = 0;
    virtual Point getMaxWindowDimensions() const = 0;
    virtual Point getMinWindowDimensions() const = 0;

    void setDrawCallback(std::function<void(double)> callback) { draw_callback_ = callback; }

    void drawCallback(double time) const {
      if (draw_callback_)
        draw_callback_(time);
    }

    void setMinimumWindowScale(float scale) { min_window_scale_ = scale; }
    float getMinimumWindowScale() const { return min_window_scale_; }
    virtual void setFixedAspectRatio(bool fixed) { fixed_aspect_ratio_ = fixed; }
    bool isFixedAspectRatio() const { return fixed_aspect_ratio_; }
    float getAspectRatio() const { return aspect_ratio_; }
    bool isVisible() const { return visible_; }

    Point getLastWindowMousePosition() const { return last_window_mouse_position_; }
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
    void setPixelScale(float scale) { pixel_scale_ = scale; }
    float getPixelScale() const { return pixel_scale_; }
    void setMouseRelativeMode(bool relative) { mouse_relative_mode_ = relative; }
    virtual bool getMouseRelativeMode() const { return mouse_relative_mode_; }

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
    Bounds getDragDropSourceBounds();
    void cleanupDragDropSource() const;
    void setVisible(bool visible) { visible_ = visible; }

  private:
    struct RepeatClick {
      int click_count = 0;
      long long last_click_ms = 0;
    };

    EventHandler* event_handler_ = nullptr;
    Point last_window_mouse_position_ = { 0, 0 };
    RepeatClick mouse_repeat_clicks_;

    std::function<void(double)> draw_callback_ = nullptr;
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
  Point getCursorPosition();
  void setCursorPosition(Point window_position);
  void setCursorScreenPosition(Point screen_position);
  int getDisplayFps();
  float getWindowPixelScale();
  void showMessageBox(std::string title, std::string message);
  std::string getClipboardText();
  void setClipboardText(const std::string& text);

  int getDoubleClickSpeed();
  void setDoubleClickSpeed(int ms);

  Bounds getScaledWindowBounds(float aspect_ratio, float display_scale, int x = Window::kNotSet,
                               int y = Window::kNotSet);
  std::unique_ptr<Window> createWindow(int x, int y, int width, int height);
  std::unique_ptr<Window> createPluginWindow(int width, int height, void* parent_handle);

  inline std::unique_ptr<Window> createWindow(int width, int height) {
    return createWindow(Window::kNotSet, Window::kNotSet, width, height);
  }

  inline std::unique_ptr<Window> createScaledWindow(float aspect_ratio, float display_scale = 0.7f,
                                                    int x = Window::kNotSet, int y = Window::kNotSet) {
    Bounds bounds = getScaledWindowBounds(aspect_ratio, display_scale, x, y);
    return createWindow(bounds.x(), bounds.y(), bounds.width(), bounds.height());
  }
}
