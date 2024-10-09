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

#include "ui_frame.h"

namespace visage {
  void UiFrame::mouseEnter(const MouseEvent& e) {
    if (on_mouse_enter_)
      on_mouse_enter_(e);
    else
      onMouseEnter(e);
  }

  void UiFrame::mouseExit(const MouseEvent& e) {
    if (on_mouse_exit_)
      on_mouse_exit_(e);
    else
      onMouseExit(e);
  }

  void UiFrame::mouseDown(const MouseEvent& e) {
    if (on_mouse_down_)
      on_mouse_down_(e);
    else
      onMouseDown(e);
  }

  void UiFrame::mouseUp(const MouseEvent& e) {
    if (on_mouse_up_)
      on_mouse_up_(e);
    else
      onMouseUp(e);
  }

  void UiFrame::mouseMove(const MouseEvent& e) {
    if (on_mouse_move_)
      on_mouse_move_(e);
    else
      onMouseMove(e);
  }

  void UiFrame::mouseDrag(const MouseEvent& e) {
    if (on_mouse_drag_)
      on_mouse_drag_(e);
    else
      onMouseDrag(e);
  }

  void UiFrame::mouseWheel(const MouseEvent& e) {
    if (on_mouse_wheel_)
      on_mouse_wheel_(e);
    else
      onMouseWheel(e);
  }

  bool UiFrame::tryFocusTextReceiver() {
    if (!isVisible())
      return false;

    if (receivesTextInput()) {
      requestKeyboardFocus();
      return true;
    }

    for (auto& child : children_) {
      if (child->tryFocusTextReceiver())
        return true;
    }
    return false;
  }

  bool UiFrame::focusNextTextReceiver(const UiFrame* starting_child) const {
    int index = std::max(0, indexOfChild(starting_child));
    for (int i = index + 1; i < children_.size(); ++i) {
      if (children_[i]->tryFocusTextReceiver())
        return true;
    }

    if (parent_ && parent_->focusNextTextReceiver(this))
      return true;

    for (int i = 0; i < index; ++i) {
      if (children_[i]->tryFocusTextReceiver())
        return true;
    }
    return false;
  }

  bool UiFrame::focusPreviousTextReceiver(const UiFrame* starting_child) const {
    int index = std::max(0, indexOfChild(starting_child));
    for (int i = index - 1; i >= 0; --i) {
      if (children_[i]->tryFocusTextReceiver())
        return true;
    }

    if (parent_ && parent_->focusPreviousTextReceiver(this))
      return true;

    for (int i = children_.size() - 1; i > index; --i) {
      if (children_[i]->tryFocusTextReceiver())
        return true;
    }
    return false;
  }

  UiFrame* UiFrame::frameAtPoint(Point point) {
    if (pass_mouse_events_to_children_) {
      for (auto it = children_.rbegin(); it != children_.rend(); ++it) {
        auto& child = *it;
        if (child->isOnTop() && child->isVisible() && child->containsPoint(point)) {
          UiFrame* result = child->frameAtPoint(point - child->position());
          if (result)
            return result;
        }
      }
      for (auto it = children_.rbegin(); it != children_.rend(); ++it) {
        auto& child = *it;
        if (!child->isOnTop() && child->isVisible() && child->containsPoint(point)) {
          UiFrame* result = child->frameAtPoint(point - child->position());
          if (result)
            return result;
        }
      }
    }

    if (!ignores_mouse_events_)
      return this;
    return nullptr;
  }
}