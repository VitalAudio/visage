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

#include "window_event_handler.h"

#include "visage_ui/ui_frame.h"
#include "visage_utils/time_utils.h"

#include <regex>

namespace visage {
  WindowEventHandler::WindowEventHandler(Window* window, UiFrame* frame) :
      window_(window), content_frame_(frame) {
    window->setEventHandler(this);
    content_frame_->addResizeCallback(resize_callback_);
  }

  WindowEventHandler::~WindowEventHandler() {
    window_->clearEventHandler();
    if (content_frame_)
      content_frame_->removeResizeCallback(resize_callback_);
  }

  void WindowEventHandler::onFrameResize(UiFrame* frame) {
    window_->setInternalWindowSize(frame->width(), frame->height());
  }

  void WindowEventHandler::setKeyboardFocus(UiFrame* frame) {
    if (keyboard_focused_frame_)
      keyboard_focused_frame_->focusChanged(false, false);

    keyboard_focused_frame_ = frame;
    keyboard_focused_frame_->focusChanged(true, false);
  }

  void WindowEventHandler::handleFocusLost() {
    if (keyboard_focused_frame_)
      keyboard_focused_frame_->focusChanged(false, false);
    if (mouse_down_frame_) {
      mouse_down_frame_->mouseUp(getMouseEvent(last_mouse_position_.x, last_mouse_position_.y, 0, 0));
      mouse_down_frame_ = nullptr;
    }
    if (mouse_hovered_frame_) {
      mouse_hovered_frame_->mouseExit(getMouseEvent(last_mouse_position_.x, last_mouse_position_.y, 0, 0));
      mouse_hovered_frame_ = nullptr;
    }
  }

  void WindowEventHandler::handleFocusGained() {
    if (keyboard_focused_frame_)
      keyboard_focused_frame_->focusChanged(true, false);
  }

  void WindowEventHandler::handleResized(int width, int height) {
    VISAGE_ASSERT(width >= 0 && height >= 0);
    content_frame_->setBounds(0, 0, width, height);
  }

  bool WindowEventHandler::handleKeyDown(const KeyEvent& e) const {
    if (keyboard_focused_frame_ == nullptr)
      return false;

    UiFrame* keyboard_frame = keyboard_focused_frame_;
    bool used = false;
    while (!used && keyboard_frame) {
      used = keyboard_frame->keyPress(e);

      keyboard_frame = keyboard_frame->parent();
      while (keyboard_frame && !keyboard_frame->acceptsKeystrokes())
        keyboard_frame = keyboard_frame->parent();
    }
    return used;
  }

  bool WindowEventHandler::handleKeyDown(KeyCode key_code, int modifiers, bool repeat) {
    return handleKeyDown(KeyEvent(key_code, modifiers, true, repeat));
  }

  bool WindowEventHandler::handleKeyUp(const KeyEvent& e) const {
    if (keyboard_focused_frame_ == nullptr)
      return false;

    UiFrame* keyboard_frame = keyboard_focused_frame_;
    bool used = false;
    while (!used && keyboard_frame) {
      used = keyboard_frame->keyRelease(e);

      keyboard_frame = keyboard_frame->parent();
      while (keyboard_frame && !keyboard_frame->acceptsKeystrokes())
        keyboard_frame = keyboard_frame->parent();
    }
    return used;
  }

  bool WindowEventHandler::handleKeyUp(KeyCode key_code, int modifiers) {
    return handleKeyUp(KeyEvent(key_code, modifiers, false));
  }

  bool WindowEventHandler::handleTextInput(const std::string& text) {
    bool text_entry = hasActiveTextEntry();
    if (text_entry)
      keyboard_focused_frame_->textInput(text);

    return text_entry;
  }

  bool WindowEventHandler::hasActiveTextEntry() {
    return keyboard_focused_frame_ && keyboard_focused_frame_->receivesTextInput();
  }

  bool WindowEventHandler::handleFileDrag(int x, int y, const std::vector<std::string>& files) {
    if (files.empty())
      return false;

    UiFrame* new_drag_drop_frame = getDragDropFrame(convertPointToFramePosition(Point(x, y)), files);
    if (mouse_down_frame_ == new_drag_drop_frame && new_drag_drop_frame)
      return true;

    if (new_drag_drop_frame != drag_drop_target_frame_) {
      if (drag_drop_target_frame_)
        drag_drop_target_frame_->dragFilesExit();

      if (new_drag_drop_frame)
        new_drag_drop_frame->dragFilesEnter(files);
      drag_drop_target_frame_ = new_drag_drop_frame;
    }

    return drag_drop_target_frame_;
  }

  void WindowEventHandler::handleFileDragLeave() {
    if (drag_drop_target_frame_)
      drag_drop_target_frame_->dragFilesExit();
    drag_drop_target_frame_ = nullptr;
  }

  bool WindowEventHandler::handleFileDrop(int x, int y, const std::vector<std::string>& files) {
    if (files.empty())
      return false;

    UiFrame* drag_drop_frame = getDragDropFrame(convertPointToFramePosition(Point(x, y)), files);
    if (mouse_down_frame_ == drag_drop_frame && drag_drop_frame)
      return false;

    if (drag_drop_target_frame_) {
      if (drag_drop_target_frame_ != drag_drop_frame)
        drag_drop_target_frame_->dragFilesExit();
      drag_drop_target_frame_ = nullptr;
    }

    if (drag_drop_frame)
      drag_drop_frame->dropFiles(files);
    return drag_drop_frame != nullptr;
  }

  MouseEvent WindowEventHandler::getMouseEvent(int x, int y, int button_state, int modifiers) {
    MouseEvent mouse_event;
    Point original_window_position = { x, y };
    mouse_event.window_position = convertPointToFramePosition(original_window_position);
    mouse_event.relative_position = original_window_position - window_->lastWindowMousePosition();
    float pixel_scale = window_->pixelScale();
    mouse_event.relative_position.x = std::round(mouse_event.relative_position.x * pixel_scale);
    mouse_event.relative_position.y = std::round(mouse_event.relative_position.y * pixel_scale);
    if (!window_->mouseRelativeMode())
      last_mouse_position_ = mouse_event.window_position;

    mouse_event.button_state = button_state;
    mouse_event.modifiers = modifiers;

    return mouse_event;
  }

  MouseEvent WindowEventHandler::getButtonMouseEvent(MouseButton button_id, int x, int y,
                                                     int button_state, int modifiers) {
    MouseEvent mouse_event = getMouseEvent(x, y, button_state, modifiers);
    mouse_event.button_id = button_id;
    return mouse_event;
  }

  void WindowEventHandler::handleMouseMove(int x, int y, int button_state, int modifiers) {
    MouseEvent mouse_event = getMouseEvent(x, y, button_state, modifiers);
    if (window_->mouseRelativeMode() && mouse_event.relative_position == Point(0, 0))
      return;

    if (mouse_down_frame_) {
      mouse_event.position = mouse_event.window_position - mouse_down_frame_->positionInWindow();
      mouse_event.frame = mouse_down_frame_;
      mouse_down_frame_->mouseDrag(mouse_event);
      return;
    }

    UiFrame* new_hovered_frame = content_frame_->frameAtPoint(mouse_event.window_position);
    if (new_hovered_frame != mouse_hovered_frame_) {
      if (mouse_hovered_frame_) {
        mouse_event.position = mouse_event.window_position - mouse_hovered_frame_->positionInWindow();
        mouse_event.frame = mouse_hovered_frame_;
        mouse_hovered_frame_->mouseExit(mouse_event);
      }

      if (new_hovered_frame) {
        mouse_event.position = mouse_event.window_position - new_hovered_frame->positionInWindow();
        mouse_event.frame = new_hovered_frame;
        new_hovered_frame->mouseEnter(mouse_event);
      }
      mouse_hovered_frame_ = new_hovered_frame;
    }
    else if (mouse_hovered_frame_) {
      mouse_event.position = mouse_event.window_position - mouse_hovered_frame_->positionInWindow();
      mouse_event.frame = mouse_hovered_frame_;
      mouse_hovered_frame_->mouseMove(mouse_event);
    }
  }

  void WindowEventHandler::handleMouseDown(MouseButton button_id, int x, int y, int button_state,
                                           int modifiers, int repeat) {
    MouseEvent mouse_event = getButtonMouseEvent(button_id, x, y, button_state, modifiers);
    mouse_event.repeat_click_count = repeat;

    mouse_down_frame_ = content_frame_->frameAtPoint(mouse_event.window_position);
    UiFrame* new_keyboard_frame = mouse_down_frame_;
    while (new_keyboard_frame && !new_keyboard_frame->acceptsKeystrokes())
      new_keyboard_frame = new_keyboard_frame->parent();

    if (keyboard_focused_frame_ && new_keyboard_frame != keyboard_focused_frame_)
      keyboard_focused_frame_->focusChanged(false, true);

    keyboard_focused_frame_ = new_keyboard_frame;
    if (keyboard_focused_frame_)
      keyboard_focused_frame_->focusChanged(true, true);

    if (mouse_down_frame_) {
      mouse_event.position = mouse_event.window_position - mouse_down_frame_->positionInWindow();
      mouse_event.frame = mouse_down_frame_;
      mouse_down_frame_->mouseDown(mouse_event);
    }
  }

  void WindowEventHandler::handleMouseUp(MouseButton button_id, int x, int y, int button_state,
                                         int modifiers, int repeat) {
    MouseEvent mouse_event = getButtonMouseEvent(button_id, x, y, button_state, modifiers);
    mouse_event.repeat_click_count = repeat;

    mouse_hovered_frame_ = content_frame_->frameAtPoint(mouse_event.window_position);
    bool exited = mouse_hovered_frame_ != mouse_down_frame_;

    if (mouse_down_frame_) {
      mouse_event.position = mouse_event.window_position - mouse_down_frame_->positionInWindow();
      mouse_event.frame = mouse_down_frame_;
      mouse_down_frame_->mouseUp(mouse_event);
      if (exited)
        mouse_down_frame_->mouseExit(mouse_event);
      mouse_down_frame_ = nullptr;
    }

    mouse_event.frame = mouse_hovered_frame_;
    if (exited && mouse_hovered_frame_)
      mouse_hovered_frame_->mouseEnter(mouse_event);
  }

  void WindowEventHandler::handleMouseEnter(int x, int y) {
    last_mouse_position_ = convertPointToFramePosition({ x, y });
  }

  void WindowEventHandler::handleMouseLeave(int x, int y, int button_state, int modifiers) {
    if (mouse_hovered_frame_) {
      MouseEvent mouse_event = getMouseEvent(last_mouse_position_.x, last_mouse_position_.y,
                                             button_state, modifiers);
      mouse_event.position = mouse_event.window_position - mouse_hovered_frame_->positionInWindow();
      mouse_event.frame = mouse_hovered_frame_;
      mouse_hovered_frame_->mouseExit(mouse_event);
      mouse_hovered_frame_ = nullptr;
    }
  }

  void WindowEventHandler::handleMouseWheel(float delta_x, float delta_y, float precise_x,
                                            float precise_y, int x, int y, int button_state,
                                            int modifiers, bool momentum) {
    MouseEvent mouse_event = getMouseEvent(x, y, button_state, modifiers);
    mouse_event.wheel_delta_x = delta_x;
    mouse_event.wheel_delta_y = delta_y;
    mouse_event.precise_wheel_delta_x = precise_x;
    mouse_event.precise_wheel_delta_y = precise_y;
    mouse_event.wheel_momentum = momentum;

    mouse_hovered_frame_ = content_frame_->frameAtPoint(mouse_event.window_position);
    if (mouse_hovered_frame_) {
      mouse_event.position = mouse_event.window_position - mouse_hovered_frame_->positionInWindow();
      mouse_hovered_frame_->mouseWheel(mouse_event);
    }
  }

  bool WindowEventHandler::isDragDropSource() {
    return mouse_down_frame_ != nullptr && mouse_down_frame_->isDragDropSource();
  }

  std::string WindowEventHandler::startDragDropSource() {
    if (mouse_down_frame_ == nullptr)
      return {};
    return mouse_down_frame_->startDragDropSource();
  }

  visage::Bounds WindowEventHandler::dragDropSourceBounds() {
    if (mouse_down_frame_ == nullptr)
      return content_frame_->localBounds();
    return mouse_down_frame_->relativeBounds(content_frame_);
  }

  void WindowEventHandler::cleanupDragDropSource() {
    if (mouse_down_frame_)
      mouse_down_frame_->cleanupDragDropSource();
  }

  UiFrame* WindowEventHandler::getDragDropFrame(Point point, const std::vector<std::string>& files) const {
    auto receives_files = [](UiFrame* frame, const std::vector<std::string>& files) {
      int num_files = files.size();
      if (!frame->receivesDragDropFiles() || (num_files > 1 && !frame->receivesMultipleDragDropFiles()))
        return false;

      std::regex regex(frame->dragDropFileExtensionRegex());
      for (auto& path : files) {
        size_t extension_position = path.find_last_of('.');
        std::string extension;
        if (extension_position != std::string::npos)
          extension = path.substr(extension_position);

        if (!std::regex_search(extension, regex))
          return false;
      }
      return true;
    };

    UiFrame* drag_drop_frame = content_frame_->frameAtPoint(point);
    while (drag_drop_frame && !receives_files(drag_drop_frame, files))
      drag_drop_frame = drag_drop_frame->parent();

    return drag_drop_frame;
  }
}