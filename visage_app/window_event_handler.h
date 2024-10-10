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

#include "visage_ui/events.h"
#include "visage_windowing/windowing.h"

namespace visage {
  class UiFrame;

  class WindowEventHandler : public Window::EventHandler {
  public:
    WindowEventHandler() = delete;
    WindowEventHandler(const WindowEventHandler&) = delete;

    WindowEventHandler(Window* window, UiFrame* frame);
    ~WindowEventHandler() override;

    void onFrameResize(const UiFrame* frame) const;

    UiFrame* getContentFrame() const { return content_frame_; }
    void setKeyboardFocus(UiFrame* frame);

    Point getLastMousePosition() const { return last_mouse_position_; }
    Point convertPointToWindowPosition(const Point& point) const {
      return { static_cast<int>(std::round(point.x / window_->pixelScale())),
               static_cast<int>(std::round(point.y / window_->pixelScale())) };
    }
    Point convertPointToFramePosition(const Point& point) const {
      return { static_cast<int>(std::round(point.x * window_->pixelScale())),
               static_cast<int>(std::round(point.y * window_->pixelScale())) };
    }

    MouseEvent getMouseEvent(int x, int y, int button_state, int modifiers);
    MouseEvent getButtonMouseEvent(MouseButton button_id, int x, int y, int button_state, int modifiers);

    void handleMouseMove(int x, int y, int button_state, int modifiers) override;
    void handleMouseDown(MouseButton button_id, int x, int y, int button_state, int modifiers,
                         int repeat) override;
    void handleMouseUp(MouseButton button_id, int x, int y, int button_state, int modifiers,
                       int repeat) override;
    void handleMouseEnter(int x, int y) override;
    void handleMouseLeave(int x, int y, int button_state, int modifiers) override;
    void handleMouseWheel(float delta_x, float delta_y, float precise_x, float precise_y, int x,
                          int y, int button_state, int modifiers, bool momentum) override;

    bool handleKeyDown(const KeyEvent& e) const;
    bool handleKeyUp(const KeyEvent& e) const;
    bool handleKeyDown(KeyCode key_code, int modifiers, bool repeat) override;
    bool handleKeyUp(KeyCode key_code, int modifiers) override;

    bool handleTextInput(const std::string& text) override;
    bool hasActiveTextEntry() override;

    void handleFocusLost() override;
    void handleFocusGained() override;
    void handleResized(int width, int height) override;

    bool handleFileDrag(int x, int y, const std::vector<std::string>& files) override;
    void handleFileDragLeave() override;
    bool handleFileDrop(int x, int y, const std::vector<std::string>& files) override;

    bool isDragDropSource() override;
    std::string startDragDropSource() override;
    visage::Bounds dragDropSourceBounds() override;
    void cleanupDragDropSource() override;

  private:
    UiFrame* getDragDropFrame(Point point, const std::vector<std::string>& files) const;

    std::function<void(UiFrame*)> resize_callback_ = [this](const UiFrame* frame) {
      onFrameResize(frame);
    };
    Window* window_ = nullptr;
    UiFrame* content_frame_ = nullptr;
    UiFrame* mouse_hovered_frame_ = nullptr;
    UiFrame* mouse_down_frame_ = nullptr;
    UiFrame* keyboard_focused_frame_ = nullptr;
    UiFrame* drag_drop_target_frame_ = nullptr;

    Point last_mouse_position_ = { 0, 0 };

    VISAGE_LEAK_CHECKER(WindowEventHandler)
  };
}
