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

#include "windowing.h"

#include "visage_utils/thread_utils.h"
#include "visage_utils/time_utils.h"

namespace visage {
  Window::Window() {
    Thread::setAsMainThread();
  }

  Window::Window(int width, int height) : client_width_(width), client_height_(height) {
    Thread::setAsMainThread();
  }

  void Window::handleFocusLost() {
    setMouseRelativeMode(false);
    if (event_handler_)
      event_handler_->handleFocusLost();
  }

  void Window::handleFocusGained() {
    if (event_handler_)
      event_handler_->handleFocusGained();
  }

  void Window::handleResized(int width, int height) {
    VISAGE_ASSERT(width >= 0 && height >= 0);
    client_width_ = width;
    client_height_ = height;
    if (event_handler_)
      event_handler_->handleResized(width, height);
  }

  void Window::handleAdjustResize(int* width, int* height, bool horizontal_resize, bool vertical_resize) {
    if (event_handler_)
      event_handler_->handleAdjustResize(width, height, horizontal_resize, vertical_resize);
  }

  void Window::handleWindowShown() {
    if (event_handler_)
      return event_handler_->handleWindowShown();
  }

  void Window::handleWindowHidden() {
    if (event_handler_)
      return event_handler_->handleWindowHidden();
  }

  bool Window::handleCloseRequested() {
    if (event_handler_)
      return event_handler_->handleCloseRequested();
    return true;
  }

  bool Window::handleKeyDown(KeyCode key_code, int modifiers, bool repeat) {
    if (event_handler_ == nullptr)
      return false;

    return event_handler_->handleKeyDown(key_code, modifiers, repeat);
  }

  bool Window::handleKeyUp(KeyCode key_code, int modifiers) {
    if (event_handler_ == nullptr)
      return false;

    return event_handler_->handleKeyUp(key_code, modifiers);
  }

  bool Window::handleTextInput(const std::string& text) {
    if (event_handler_ == nullptr)
      return false;

    return event_handler_->handleTextInput(text);
  }

  void Window::setWindowSize(int width, int height) {
    setNativeWindowSize(std::round(width * dpi_scale_), std::round(height * dpi_scale_));
  }

  void Window::setNativeWindowSize(int width, int height) {
    handleResized(width, height);
    windowContentsResized(width, height);
  }

  void Window::setInternalWindowSize(int width, int height) {
    if (width == client_width_ && height == client_height_)
      return;

    client_width_ = width;
    client_height_ = height;
    windowContentsResized(client_width_, client_height_);
  }

  bool Window::hasActiveTextEntry() const {
    if (event_handler_ == nullptr)
      return false;

    return event_handler_->hasActiveTextEntry();
  }

  bool Window::handleFileDrag(int x, int y, const std::vector<std::string>& files) {
    if (files.empty() || event_handler_ == nullptr)
      return false;

    return event_handler_->handleFileDrag(x, y, files);
  }

  void Window::handleFileDragLeave() {
    if (event_handler_)
      event_handler_->handleFileDragLeave();
  }

  bool Window::handleFileDrop(int x, int y, const std::vector<std::string>& files) {
    if (files.empty() || event_handler_ == nullptr)
      return false;

    return event_handler_->handleFileDrop(x, y, files);
  }

  HitTestResult Window::handleHitTest(int x, int y) {
    if (event_handler_ == nullptr)
      return HitTestResult::Client;

    return event_handler_->handleHitTest(x, y);
  }

  HitTestResult Window::currentHitTest() const {
    if (event_handler_ == nullptr)
      return HitTestResult::Client;

    return event_handler_->currentHitTest();
  }

  void Window::handleMouseMove(int x, int y, int button_state, int modifiers) {
    if (event_handler_ == nullptr)
      return;

    if (last_window_mouse_position_.x != x || last_window_mouse_position_.y != y)
      mouse_repeat_clicks_.click_count = 0;

    event_handler_->handleMouseMove(x, y, button_state, modifiers);

    if (!mouseRelativeMode())
      last_window_mouse_position_ = { x, y };
  }

  void Window::handleMouseDown(MouseButton button_id, int x, int y, int button_state, int modifiers) {
    if (event_handler_ == nullptr)
      return;

    setMouseRelativeMode(false);
    long long current_ms = time::milliseconds();
    long long delta_ms = current_ms - mouse_repeat_clicks_.last_click_ms;
    if (delta_ms > 0 && delta_ms < doubleClickSpeed())
      mouse_repeat_clicks_.click_count++;
    else
      mouse_repeat_clicks_.click_count = 1;
    mouse_repeat_clicks_.last_click_ms = current_ms;
    event_handler_->handleMouseDown(button_id, x, y, button_state, modifiers,
                                    mouse_repeat_clicks_.click_count);
  }

  void Window::handleMouseUp(MouseButton button_id, int x, int y, int button_state, int modifiers) {
    if (event_handler_ == nullptr)
      return;

    event_handler_->handleMouseUp(button_id, x, y, button_state, modifiers, mouse_repeat_clicks_.click_count);
  }

  void Window::handleMouseEnter(int x, int y) {
    last_window_mouse_position_ = { x, y };
    if (event_handler_)
      event_handler_->handleMouseEnter(x, y);
  }

  void Window::handleMouseLeave(int button_state, int modifiers) {
    if (event_handler_) {
      event_handler_->handleMouseLeave(last_window_mouse_position_.x, last_window_mouse_position_.y,
                                       button_state, modifiers);
    }
  }

  void Window::handleMouseWheel(float delta_x, float delta_y, float precise_x, float precise_y,
                                int x, int y, int button_state, int modifiers, bool momentum) {
    if (event_handler_)
      event_handler_->handleMouseWheel(delta_x, delta_y, precise_x, precise_y, x, y, button_state,
                                       modifiers, momentum);
  }

  void Window::handleMouseWheel(float delta_x, float delta_y, int x, int y, int button_state,
                                int modifiers, bool momentum) {
    handleMouseWheel(delta_x, delta_y, delta_x, delta_y, x, y, button_state, modifiers, momentum);
  }

  bool Window::isDragDropSource() const {
    if (event_handler_ == nullptr)
      return false;

    return event_handler_->isDragDropSource();
  }

  std::string Window::startDragDropSource() {
    if (event_handler_ == nullptr)
      return {};

    return event_handler_->startDragDropSource();
  }

  void Window::cleanupDragDropSource() {
    if (event_handler_)
      event_handler_->cleanupDragDropSource();
  }

  int Window::double_click_speed_ = 500;

  int doubleClickSpeed() {
    return Window::doubleClickSpeed();
  }

  void setDoubleClickSpeed(int ms) {
    Window::setDoubleClickSpeed(ms);
  }
}