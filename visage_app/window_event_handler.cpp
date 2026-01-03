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

#include "window_event_handler.h"

#include "visage_ui/frame.h"

#include <regex>

namespace visage {
  WindowEventHandler::WindowEventHandler(ApplicationEditor* editor, Frame* frame) :
      editor_(editor), window_(editor->window()), content_frame_(frame) {
    window_->setEventHandler(this);
  }

  WindowEventHandler::~WindowEventHandler() {
    window_->clearEventHandler();
  }

  void WindowEventHandler::setKeyboardFocus(Frame* frame) {
    if (keyboard_focused_frame_)
      keyboard_focused_frame_->processFocusChanged(false, false);

    keyboard_focused_frame_ = frame;
    if (keyboard_focused_frame_)
      keyboard_focused_frame_->processFocusChanged(true, false);
  }

  void WindowEventHandler::giveUpFocus(Frame* frame) {
    if (frame == nullptr)
      return;

    if (mouse_hovered_frame_ == frame)
      mouse_hovered_frame_ = nullptr;
    if (temporary_frame_ == frame)
      temporary_frame_ = nullptr;
    if (mouse_down_frame_ == frame)
      mouse_down_frame_ = nullptr;
    if (keyboard_focused_frame_ == frame)
      keyboard_focused_frame_ = nullptr;
    if (drag_drop_target_frame_ == frame)
      drag_drop_target_frame_ = nullptr;
  }

  void WindowEventHandler::handleFocusLost() {
    if (keyboard_focused_frame_)
      keyboard_focused_frame_->processFocusChanged(false, false);
    if (mouse_down_frame_) {
      mouse_down_frame_->processMouseUp(mouseEvent(last_mouse_position_.x, last_mouse_position_.y, 0, 0));
      mouse_down_frame_ = nullptr;
    }
    if (mouse_hovered_frame_) {
      mouse_hovered_frame_->processMouseExit(mouseEvent(last_mouse_position_.x,
                                                        last_mouse_position_.y, 0, 0));
      mouse_hovered_frame_ = nullptr;
    }
  }

  void WindowEventHandler::handleFocusGained() {
    if (keyboard_focused_frame_)
      keyboard_focused_frame_->processFocusChanged(true, false);
  }

  void WindowEventHandler::handleResized(int width, int height) {
    VISAGE_ASSERT(width >= 0 && height >= 0);
    content_frame_->setNativeBounds(0, 0, width, height);
    content_frame_->redraw();
  }

  void WindowEventHandler::handleAdjustResize(int* width, int* height, bool horizontal_resize,
                                              bool vertical_resize) {
    editor_->adjustWindowDimensions(width, height, horizontal_resize, vertical_resize);
  }

  void WindowEventHandler::handleWindowShown() {
    editor_->onShow().callback();
  }

  void WindowEventHandler::handleWindowHidden() {
    editor_->onHide().callback();
  }

  bool WindowEventHandler::handleCloseRequested() {
    if (editor_->onCloseRequested().isEmpty())
      return true;
    return editor_->onCloseRequested().callback();
  }

  bool WindowEventHandler::handleKeyDown(const KeyEvent& e) {
    if (keyboard_focused_frame_ == nullptr)
      return false;

    temporary_frame_ = keyboard_focused_frame_;
    bool used = false;
    while (!used && temporary_frame_) {
      used = temporary_frame_->processKeyPress(e);

      if (temporary_frame_)
        temporary_frame_ = temporary_frame_->parent();

      while (temporary_frame_ && !temporary_frame_->acceptsKeystrokes())
        temporary_frame_ = temporary_frame_->parent();
    }
    temporary_frame_ = nullptr;
    return used;
  }

  bool WindowEventHandler::handleKeyDown(KeyCode key_code, int modifiers, bool repeat) {
    return handleKeyDown(KeyEvent(key_code, modifiers, true, repeat));
  }

  bool WindowEventHandler::handleKeyUp(const KeyEvent& e) {
    if (keyboard_focused_frame_ == nullptr)
      return false;

    temporary_frame_ = keyboard_focused_frame_;
    bool used = false;
    while (!used && temporary_frame_) {
      used = temporary_frame_->processKeyRelease(e);

      if (temporary_frame_)
        temporary_frame_ = temporary_frame_->parent();

      while (temporary_frame_ && !temporary_frame_->acceptsKeystrokes())
        temporary_frame_ = temporary_frame_->parent();
    }
    temporary_frame_ = nullptr;
    return used;
  }

  bool WindowEventHandler::handleKeyUp(KeyCode key_code, int modifiers) {
    return handleKeyUp(KeyEvent(key_code, modifiers, false));
  }

  bool WindowEventHandler::handleTextInput(const std::string& text) {
    bool text_entry = hasActiveTextEntry();
    if (text_entry)
      keyboard_focused_frame_->processTextInput(text);

    return text_entry;
  }

  bool WindowEventHandler::hasActiveTextEntry() {
    return keyboard_focused_frame_ && keyboard_focused_frame_->receivesTextInput();
  }

  bool WindowEventHandler::handleFileDrag(int x, int y, const std::vector<std::string>& files) {
    if (files.empty())
      return false;

    temporary_frame_ = dragDropFrame(convertToLogical(IPoint(x, y)), files);
    if (mouse_down_frame_ == temporary_frame_ && temporary_frame_) {
      temporary_frame_ = nullptr;
      return true;
    }

    if (temporary_frame_ != drag_drop_target_frame_) {
      if (drag_drop_target_frame_)
        drag_drop_target_frame_->dragFilesExit();

      if (temporary_frame_)
        temporary_frame_->dragFilesEnter(files);
      drag_drop_target_frame_ = temporary_frame_;
    }

    temporary_frame_ = nullptr;
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

    temporary_frame_ = dragDropFrame(convertToLogical(IPoint(x, y)), files);
    if (mouse_down_frame_ == temporary_frame_ && temporary_frame_)
      return false;

    if (drag_drop_target_frame_) {
      if (drag_drop_target_frame_ != temporary_frame_)
        drag_drop_target_frame_->dragFilesExit();
      drag_drop_target_frame_ = nullptr;
    }

    if (temporary_frame_)
      temporary_frame_->dropFiles(files);
    bool result = temporary_frame_ != nullptr;
    temporary_frame_ = nullptr;
    return result;
  }

  MouseEvent WindowEventHandler::mouseEvent(int x, int y, int button_state, int modifiers) {
    MouseEvent mouse_event;
    IPoint original_window_position = { x, y };
    mouse_event.window_position = convertToLogical(original_window_position);
    mouse_event.relative_position = mouse_event.window_position -
                                    convertToLogical(window_->lastWindowMousePosition());
    mouse_event.relative_position.x = std::round(mouse_event.relative_position.x);
    mouse_event.relative_position.y = std::round(mouse_event.relative_position.y);
    if (!window_->mouseRelativeMode())
      last_mouse_position_ = mouse_event.window_position;

    mouse_event.button_state = button_state;
    mouse_event.modifiers = modifiers;

    return mouse_event;
  }

  MouseEvent WindowEventHandler::buttonMouseEvent(MouseButton button_id, int x, int y,
                                                  int button_state, int modifiers) {
    MouseEvent mouse_event = mouseEvent(x, y, button_state, modifiers);
    mouse_event.button_id = button_id;
    return mouse_event;
  }

  HitTestResult WindowEventHandler::handleHitTest(int x, int y) {
    Point window_position = convertToLogical({ x, y });
    Frame* hovered_frame = content_frame_->frameAtPoint(window_position);
    if (hovered_frame == nullptr)
      current_hit_test_ = HitTestResult::Client;
    else {
      Point position = window_position - hovered_frame->positionInWindow();
      current_hit_test_ = hovered_frame->hitTest(position);
    }
    return current_hit_test_;
  }

  void WindowEventHandler::handleMouseMove(int x, int y, int button_state, int modifiers) {
    MouseEvent mouse_event = mouseEvent(x, y, button_state, modifiers);
    if (window_->mouseRelativeMode() && mouse_event.relative_position == Point(0, 0))
      return;

    if (mouse_down_frame_) {
      mouse_event.position = mouse_event.window_position - mouse_down_frame_->positionInWindow();
      mouse_event.event_frame = mouse_down_frame_;
      mouse_down_frame_->processMouseDrag(mouse_event);
      return;
    }

    temporary_frame_ = content_frame_->frameAtPoint(mouse_event.window_position);
    if (temporary_frame_ != mouse_hovered_frame_) {
      if (mouse_hovered_frame_) {
        mouse_event.position = mouse_event.window_position - mouse_hovered_frame_->positionInWindow();
        mouse_event.event_frame = mouse_hovered_frame_;
        mouse_hovered_frame_->processMouseExit(mouse_event);
      }

      if (temporary_frame_) {
        mouse_event.position = mouse_event.window_position - temporary_frame_->positionInWindow();
        mouse_event.event_frame = temporary_frame_;
        temporary_frame_->processMouseEnter(mouse_event);
      }
      mouse_hovered_frame_ = temporary_frame_;
      temporary_frame_ = nullptr;
    }
    else if (mouse_hovered_frame_) {
      mouse_event.position = mouse_event.window_position - mouse_hovered_frame_->positionInWindow();
      mouse_event.event_frame = mouse_hovered_frame_;
      mouse_hovered_frame_->processMouseMove(mouse_event);
    }
  }

  void WindowEventHandler::handleMouseDown(MouseButton button_id, int x, int y, int button_state,
                                           int modifiers, int repeat) {
    MouseEvent mouse_event = buttonMouseEvent(button_id, x, y, button_state, modifiers);
    mouse_event.repeat_click_count = repeat;

    mouse_down_frame_ = content_frame_->frameAtPoint(mouse_event.window_position);
    temporary_frame_ = mouse_down_frame_;
    while (temporary_frame_ && !temporary_frame_->acceptsKeystrokes())
      temporary_frame_ = temporary_frame_->parent();

    if (keyboard_focused_frame_ && temporary_frame_ != keyboard_focused_frame_)
      keyboard_focused_frame_->processFocusChanged(false, true);

    keyboard_focused_frame_ = temporary_frame_;
    temporary_frame_ = nullptr;
    if (keyboard_focused_frame_)
      keyboard_focused_frame_->processFocusChanged(true, true);

    if (mouse_down_frame_) {
      mouse_event.position = mouse_event.window_position - mouse_down_frame_->positionInWindow();
      mouse_event.event_frame = mouse_down_frame_;
      mouse_down_frame_->processMouseDown(mouse_event);
    }
  }

  void WindowEventHandler::handleMouseUp(MouseButton button_id, int x, int y, int button_state,
                                         int modifiers, int repeat) {
    MouseEvent mouse_event = buttonMouseEvent(button_id, x, y, button_state, modifiers);
    mouse_event.repeat_click_count = repeat;

    mouse_hovered_frame_ = content_frame_->frameAtPoint(mouse_event.window_position);
    bool exited = mouse_hovered_frame_ != mouse_down_frame_;

    if (mouse_down_frame_) {
      mouse_event.position = mouse_event.window_position - mouse_down_frame_->positionInWindow();
      mouse_event.event_frame = mouse_down_frame_;
      auto frame = mouse_down_frame_;
      mouse_down_frame_ = nullptr;
      frame->processMouseUp(mouse_event);
      if (exited && frame)
        frame->processMouseExit(mouse_event);
    }

    mouse_event.event_frame = mouse_hovered_frame_;
    if (exited && mouse_hovered_frame_)
      mouse_hovered_frame_->processMouseEnter(mouse_event);
  }

  void WindowEventHandler::handleMouseEnter(int x, int y) {
    last_mouse_position_ = convertToLogical({ x, y });
  }

  void WindowEventHandler::handleMouseLeave(int x, int y, int button_state, int modifiers) {
    if (mouse_hovered_frame_) {
      MouseEvent mouse_event = mouseEvent(last_mouse_position_.x, last_mouse_position_.y,
                                          button_state, modifiers);
      mouse_event.position = mouse_event.window_position - mouse_hovered_frame_->positionInWindow();
      mouse_event.event_frame = mouse_hovered_frame_;
      mouse_hovered_frame_->processMouseExit(mouse_event);
      mouse_hovered_frame_ = nullptr;
    }
  }

  void WindowEventHandler::handleMouseWheel(float delta_x, float delta_y, float precise_x,
                                            float precise_y, int x, int y, int button_state,
                                            int modifiers, bool momentum) {
    MouseEvent mouse_event = mouseEvent(x, y, button_state, modifiers);
    mouse_event.wheel_delta_x = delta_x;
    mouse_event.wheel_delta_y = delta_y;
    mouse_event.precise_wheel_delta_x = precise_x;
    mouse_event.precise_wheel_delta_y = precise_y;
    mouse_event.wheel_momentum = momentum;

    mouse_hovered_frame_ = content_frame_->frameAtPoint(mouse_event.window_position);
    if (mouse_hovered_frame_) {
      temporary_frame_ = mouse_hovered_frame_;
      bool used = false;

      while (!used && temporary_frame_) {
        while (temporary_frame_ && temporary_frame_->ignoresMouseEvents())
          temporary_frame_ = temporary_frame_->parent();

        if (temporary_frame_) {
          mouse_event.position = mouse_event.window_position - temporary_frame_->positionInWindow();
          used = temporary_frame_->processMouseWheel(mouse_event);
          if (temporary_frame_)
            temporary_frame_ = temporary_frame_->parent();
        }
      }
    }
    temporary_frame_ = nullptr;
  }

  bool WindowEventHandler::isDragDropSource() {
    return mouse_down_frame_ != nullptr && mouse_down_frame_->isDragDropSource();
  }

  std::string WindowEventHandler::startDragDropSource() {
    if (mouse_down_frame_ == nullptr)
      return {};
    return mouse_down_frame_->startDragDropSource();
  }

  void WindowEventHandler::cleanupDragDropSource() {
    if (mouse_down_frame_)
      mouse_down_frame_->cleanupDragDropSource();
  }

  Frame* WindowEventHandler::dragDropFrame(Point point, const std::vector<std::string>& files) const {
    auto receives_files = [](Frame* frame, const std::vector<std::string>& files) {
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

    Frame* drag_drop_frame = content_frame_->frameAtPoint(point);
    while (drag_drop_frame && !receives_files(drag_drop_frame, files))
      drag_drop_frame = drag_drop_frame->parent();

    return drag_drop_frame;
  }
}