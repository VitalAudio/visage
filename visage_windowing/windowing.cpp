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

#include "windowing.h"

#include "visage_utils/thread_utils.h"
#include "visage_utils/time_utils.h"

namespace visage {
  Window::Window(float aspect_ratio) : aspect_ratio_(aspect_ratio) {
    Thread::setAsMainThread();
  }
  Window::Window(int width, int height) :
      aspect_ratio_(width * 1.0f / height), client_width_(width), client_height_(height) {
    Thread::setAsMainThread();
  }

  void Window::handleFocusLost() {
    setMouseRelativeMode(false);
    if (event_handler_)
      event_handler_->handleFocusLost();
  }

  void Window::handleFocusGained() const {
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

  bool Window::handleKeyDown(KeyCode key_code, int modifiers, bool repeat) const {
    if (event_handler_ == nullptr)
      return false;

    return event_handler_->handleKeyDown(key_code, modifiers, repeat);
  }

  bool Window::handleKeyUp(KeyCode key_code, int modifiers) const {
    if (event_handler_ == nullptr)
      return false;

    return event_handler_->handleKeyUp(key_code, modifiers);
  }

  bool Window::handleTextInput(const std::string& text) const {
    if (event_handler_ == nullptr)
      return false;

    return event_handler_->handleTextInput(text);
  }

  void Window::setWindowSize(int width, int height) {
    handleResized(std::round(width * pixel_scale_), std::round(height * pixel_scale_));
    windowContentsResized(width, height);
  }

  void Window::setInternalWindowSize(int width, int height) {
    if (width == client_width_ && height == client_height_)
      return;

    client_width_ = width;
    client_height_ = height;
    if (fixed_aspect_ratio_)
      aspect_ratio_ = width * 1.0f / height;
    windowContentsResized(std::round(width / pixel_scale_), std::round(height / pixel_scale_));
  }

  bool Window::hasActiveTextEntry() const {
    if (event_handler_ == nullptr)
      return false;

    return event_handler_->hasActiveTextEntry();
  }

  bool Window::handleFileDrag(int x, int y, const std::vector<std::string>& files) const {
    if (files.empty() || event_handler_ == nullptr)
      return false;

    return event_handler_->handleFileDrag(x, y, files);
  }

  void Window::handleFileDragLeave() const {
    if (event_handler_)
      event_handler_->handleFileDragLeave();
  }

  bool Window::handleFileDrop(int x, int y, const std::vector<std::string>& files) const {
    if (files.empty() || event_handler_ == nullptr)
      return false;

    return event_handler_->handleFileDrop(x, y, files);
  }

  void Window::handleMouseMove(int x, int y, int button_state, int modifiers) {
    if (event_handler_ == nullptr)
      return;

    if (last_window_mouse_position_.x != x || last_window_mouse_position_.y != y)
      mouse_repeat_clicks_.click_count = 0;

    event_handler_->handleMouseMove(x, y, button_state, modifiers);

    if (!getMouseRelativeMode())
      last_window_mouse_position_ = { x, y };
  }

  void Window::handleMouseDown(MouseButton button_id, int x, int y, int button_state, int modifiers) {
    if (event_handler_ == nullptr)
      return;

    setMouseRelativeMode(false);
    long long current_ms = time::getMilliseconds();
    long long delta_ms = current_ms - mouse_repeat_clicks_.last_click_ms;
    if (delta_ms > 0 && delta_ms < getDoubleClickSpeed())
      mouse_repeat_clicks_.click_count++;
    else
      mouse_repeat_clicks_.click_count = 1;
    mouse_repeat_clicks_.last_click_ms = current_ms;
    event_handler_->handleMouseDown(button_id, x, y, button_state, modifiers,
                                    mouse_repeat_clicks_.click_count);
  }

  void Window::handleMouseUp(MouseButton button_id, int x, int y, int button_state, int modifiers) const {
    if (event_handler_ == nullptr)
      return;

    event_handler_->handleMouseUp(button_id, x, y, button_state, modifiers, mouse_repeat_clicks_.click_count);
  }

  void Window::handleMouseEnter(int x, int y) {
    last_window_mouse_position_ = { x, y };
    if (event_handler_)
      event_handler_->handleMouseEnter(x, y);
  }

  void Window::handleMouseLeave(int button_state, int modifiers) const {
    if (event_handler_) {
      event_handler_->handleMouseLeave(last_window_mouse_position_.x, last_window_mouse_position_.y,
                                       button_state, modifiers);
    }
  }

  void Window::handleMouseWheel(float delta_x, float delta_y, float precise_x, float precise_y,
                                int x, int y, int button_state, int modifiers, bool momentum) const {
    if (event_handler_)
      event_handler_->handleMouseWheel(delta_x, delta_y, precise_x, precise_y, x, y, button_state,
                                       modifiers, momentum);
  }

  void Window::handleMouseWheel(float delta_x, float delta_y, int x, int y, int button_state,
                                int modifiers, bool momentum) const {
    handleMouseWheel(delta_x, delta_y, delta_x, delta_y, x, y, button_state, modifiers, momentum);
  }

  bool Window::isDragDropSource() const {
    if (event_handler_ == nullptr)
      return false;

    return event_handler_->isDragDropSource();
  }

  std::string Window::startDragDropSource() const {
    if (event_handler_ == nullptr)
      return {};

    return event_handler_->startDragDropSource();
  }

  visage::Bounds Window::getDragDropSourceBounds() {
    if (event_handler_ == nullptr)
      return { 0, 0, client_width_, client_height_ };

    return event_handler_->getDragDropSourceBounds();
  }

  void Window::cleanupDragDropSource() const {
    if (event_handler_)
      event_handler_->cleanupDragDropSource();
  }

  int& doubleClickSpeed() {
    static int double_click_speed = 500;
    return double_click_speed;
  }

  int getDoubleClickSpeed() {
    return doubleClickSpeed();
  }

  void setDoubleClickSpeed(int ms) {
    doubleClickSpeed() = ms;
  }
}