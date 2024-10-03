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

#include "events.h"
#include "visage_utils/space.h"

#include <string>
#include <vector>

namespace visage {

  class UiFrame {
  public:
    UiFrame() = default;
    explicit UiFrame(std::string name) : name_(std::move(name)) { }

    virtual ~UiFrame() = default;

    const std::string& getName() const { return name_; }
    void setName(std::string name) { name_ = std::move(name); }

    bool isVisible() const { return visible_; }

    virtual void setVisible(bool visible) {
      if (visible_ != visible) {
        visible_ = visible;
        onVisibilityChange();
      }
    }

    virtual void onVisibilityChange() { }

    void addChild(UiFrame* child) {
      child->parent_ = this;
      children_.push_back(child);
    }

    void removeChild(UiFrame* child) {
      child->parent_ = nullptr;
      children_.erase(std::find(children_.begin(), children_.end(), child));
    }

    int indexOfChild(const UiFrame* child) const {
      for (int i = 0; i < children_.size(); ++i) {
        if (children_[i] == child)
          return i;
      }
      return -1;
    }

    virtual void hierarchyChanged() { }

    template<class T>
    T* findParent() const {
      UiFrame* frame = parent_;
      while (frame) {
        T* casted = dynamic_cast<T*>(frame);
        if (casted)
          return casted;

        frame = frame->parent_;
      }

      return nullptr;
    }

    UiFrame* getParent() const { return parent_; }

    Point getPosition() const { return { bounds_.x(), bounds_.y() }; }
    const Bounds& getBounds() const { return bounds_; }
    virtual void setBounds(Bounds bounds) {
      if (bounds_ != bounds) {
        bounds_ = bounds;
        resized();
        for (auto& callback : resize_callbacks_)
          callback(this);
      }
    }

    virtual void setBounds(int x, int y, int width, int height) {
      setBounds({ x, y, width, height });
    }

    void setTopLeft(int x, int y) { setBounds(x, y, getWidth(), getHeight()); }

    bool isOnTop() const { return on_top_; }
    void setOnTop(bool on_top) { on_top_ = on_top; }

    virtual void resized() { }

    int getX() const { return bounds_.x(); }
    int getY() const { return bounds_.y(); }
    int getWidth() const { return bounds_.width(); }
    int getHeight() const { return bounds_.height(); }
    int getRight() const { return bounds_.right(); }
    int getBottom() const { return bounds_.bottom(); }
    float getAspectRatio() const { return bounds_.width() * 1.0f / bounds_.height(); }

    Bounds getLocalBounds() const { return { 0, 0, getWidth(), getHeight() }; }

    Point getPositionInWindow() const {
      Point global_position = getPosition();
      UiFrame* frame = parent_;
      while (frame) {
        global_position = global_position + frame->getPosition();
        frame = frame->parent_;
      }

      return global_position;
    }

    Bounds getRelativeBounds(const UiFrame* other) const {
      Point position = getPositionInWindow();
      Point other_position = other->getPositionInWindow();
      int width = other->getBounds().width();
      int height = other->getBounds().height();
      return { other_position.x - position.x, other_position.y - position.y, width, height };
    }

    bool acceptsKeystrokes() const { return accepts_keystrokes_; }
    void setAcceptsKeystrokes(bool accepts_keystrokes) { accepts_keystrokes_ = accepts_keystrokes; }

    bool ignoresMouseEvents() const { return ignores_mouse_events_; }
    void setIgnoresMouseEvents(bool ignore, bool pass_to_children) {
      ignores_mouse_events_ = ignore;
      pass_mouse_events_to_children_ = pass_to_children;
    }

    bool hasKeyboardFocus() const { return keyboard_focus_; }
    bool tryFocusTextReceiver();
    bool focusNextTextReceiver(const UiFrame* starting_child = nullptr) const;
    bool focusPreviousTextReceiver(const UiFrame* starting_child = nullptr) const;

    virtual void onMouseEnter(const MouseEvent& e) { }
    virtual void onMouseExit(const MouseEvent& e) { }
    virtual void onMouseDown(const MouseEvent& e) { }
    virtual void onMouseUp(const MouseEvent& e) { }
    virtual void onMouseMove(const MouseEvent& e) { }
    virtual void onMouseDrag(const MouseEvent& e) { }
    virtual void onMouseWheel(const MouseEvent& e) { }
    virtual bool onKeyPress(const KeyEvent& e) { return false; }
    virtual bool onKeyRelease(const KeyEvent& e) { return false; }
    virtual void onFocusChange(bool is_focused, bool was_clicked) { }

    void setOnMouseEnter(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_enter_ = callback;
    }

    void setOnMouseExit(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_exit_ = callback;
    }

    void setOnMouseDown(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_down_ = callback;
    }

    void setOnMouseUp(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_up_ = callback;
    }

    void setOnMouseMove(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_move_ = callback;
    }

    void setOnMouseDrag(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_drag_ = callback;
    }

    void setOnMouseWheel(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_wheel_ = callback;
    }

    void setOnKeyPress(std::function<bool(const KeyEvent& e)> callback) {
      on_key_press_ = callback;
    }

    void setOnKeyRelease(std::function<bool(const KeyEvent& e)> callback) {
      on_key_release_ = callback;
    }

    virtual bool receivesTextInput() { return false; }
    virtual void onTextInput(const std::string& text) { }

    virtual bool receivesDragDropFiles() { return false; }
    virtual std::string getDragDropFileExtensionRegex() { return ".*"; }
    virtual bool receivesMultipleDragDropFiles() { return false; }
    virtual void dragFilesEnter(const std::vector<std::string>& paths) { }
    virtual void dragFilesExit() { }
    virtual void dropFiles(const std::vector<std::string>& paths) { }

    virtual bool isDragDropSource() { return false; }
    virtual std::string startDragDropSource() { return ""; }
    virtual void cleanupDragDropSource() { }

    void mouseEnter(const MouseEvent& e);
    void mouseExit(const MouseEvent& e);
    void mouseDown(const MouseEvent& e);
    void mouseUp(const MouseEvent& e);
    void mouseMove(const MouseEvent& e);
    void mouseDrag(const MouseEvent& e);
    void mouseWheel(const MouseEvent& e);

    void focusChanged(bool is_focused, bool was_clicked) {
      keyboard_focus_ = is_focused && accepts_keystrokes_;
      onFocusChange(is_focused, was_clicked);
    }

    bool keyPress(const KeyEvent& e) {
      if (on_key_press_)
        return on_key_press_(e);
      return onKeyPress(e);
    }

    bool keyRelease(const KeyEvent& e) {
      if (on_key_release_)
        return on_key_press_(e);
      return onKeyRelease(e);
    }

    void textInput(const std::string& text) { return onTextInput(text); }

    bool containsPoint(Point point) const { return bounds_.contains(point); }

    UiFrame* getFrameAtPoint(Point point);
    UiFrame* getTopParentFrame() {
      UiFrame* frame = this;
      while (frame->parent_)
        frame = frame->parent_;
      return frame;
    }

    virtual void requestKeyboardFocus(UiFrame* frame) {
      if (parent_)
        getTopParentFrame()->requestKeyboardFocus(frame);
    }

    void requestKeyboardFocus() { requestKeyboardFocus(this); }

    void addResizeCallback(std::function<void(UiFrame*)> callback) {
      resize_callbacks_.push_back(callback);
    }

    void removeResizeCallback(std::function<void(UiFrame*)> callback) {
      auto compare = [&](const std::function<void(UiFrame*)>& other) {
        return other.target_type() == callback.target_type();
      };

      auto it = std::remove_if(resize_callbacks_.begin(), resize_callbacks_.end(), compare);
      resize_callbacks_.erase(it);
    }

  private:
    void notifyHierarchyChanged() {
      hierarchyChanged();
      for (UiFrame* child : children_)
        child->notifyHierarchyChanged();
    }

    std::string name_;
    Bounds bounds_;
    std::vector<std::function<void(UiFrame*)>> resize_callbacks_;

    std::function<void(const MouseEvent& e)> on_mouse_enter_ = nullptr;
    std::function<void(const MouseEvent& e)> on_mouse_exit_ = nullptr;
    std::function<void(const MouseEvent& e)> on_mouse_down_ = nullptr;
    std::function<void(const MouseEvent& e)> on_mouse_up_ = nullptr;
    std::function<void(const MouseEvent& e)> on_mouse_move_ = nullptr;
    std::function<void(const MouseEvent& e)> on_mouse_drag_ = nullptr;
    std::function<void(const MouseEvent& e)> on_mouse_wheel_ = nullptr;
    std::function<bool(const KeyEvent& e)> on_key_press_ = nullptr;
    std::function<bool(const KeyEvent& e)> on_key_release_ = nullptr;

    bool on_top_ = false;
    bool visible_ = true;
    bool keyboard_focus_ = false;
    bool accepts_keystrokes_ = false;
    bool accepts_dropped_files_ = false;
    bool ignores_mouse_events_ = false;
    bool pass_mouse_events_to_children_ = true;

    std::vector<UiFrame*> children_;
    UiFrame* parent_ = nullptr;
  };
}