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

#include "events.h"
#include "layout.h"
#include "undo_history.h"
#include "visage_graphics/canvas.h"
#include "visage_graphics/image.h"
#include "visage_graphics/palette.h"
#include "visage_graphics/theme.h"
#include "visage_utils/space.h"
#include "visage_utils/string_utils.h"

#include <string>
#include <vector>

namespace visage {
  class Frame;

  template<typename T>
  class CallbackList {
  public:
    template<typename R>
    static R getDefaultResult() {
      if constexpr (std::is_default_constructible_v<R>)
        return {};
      else
        static_assert(std::is_void_v<R>, "Callback return value must be default constructable");
    }

    CallbackList() = default;
    explicit CallbackList(std::function<T> callback) :
        original_(std::make_unique<std::function<T>>(callback)) {
      add(callback);
    }
    CallbackList(const CallbackList& other) {
      if (other.original_)
        original_ = std::make_unique<std::function<T>>(*other.original_);
      callbacks_ = other.callbacks_;
    }

    void add(std::function<T> callback) { callbacks_.push_back(std::move(callback)); }

    CallbackList& operator+=(std::function<T> callback) {
      add(callback);
      return *this;
    }

    void set(std::function<T> callback) {
      callbacks_.clear();
      callbacks_.push_back(std::move(callback));
    }

    CallbackList& operator=(const std::function<T>& callback) {
      set(callback);
      return *this;
    }

    void remove(const std::function<T>& callback) {
      auto compare = [&](const std::function<T>& other) {
        return other.target_type() == callback.target_type();
      };

      auto it = std::remove_if(callbacks_.begin(), callbacks_.end(), compare);
      callbacks_.erase(it);
    }

    CallbackList& operator-=(const std::function<T>& callback) {
      remove(callback);
      return *this;
    }

    void reset() {
      callbacks_.clear();
      if (original_)
        callbacks_.push_back(*original_);
    }

    void clear() { callbacks_.clear(); }

    template<typename... Args>
    auto callback(Args&&... args) {
      if (callbacks_.empty())
        return getDefaultResult<decltype(std::declval<std::function<T>>()(args...))>();

      for (size_t i = 0; i < callbacks_.size() - 1; ++i)
        callbacks_[i](std::forward<Args>(args)...);

      return callbacks_.back()(std::forward<Args>(args)...);
    }

  private:
    std::unique_ptr<std::function<T>> original_;
    std::vector<std::function<T>> callbacks_;
  };

  struct FrameEventHandler {
    std::function<void(Frame*)> request_redraw = nullptr;
    std::function<void(Frame*)> request_keyboard_focus = nullptr;
    std::function<void(Frame*)> remove_from_hierarchy = nullptr;
    std::function<void(bool)> set_mouse_relative_mode = nullptr;
    std::function<void(MouseCursor)> set_cursor_style = nullptr;
    std::function<void(bool)> set_cursor_visible = nullptr;
    std::function<std::string()> read_clipboard_text = nullptr;
    std::function<void(std::string)> set_clipboard_text = nullptr;
  };

  class Frame {
  public:
    Frame() = default;
    explicit Frame(std::string name) : name_(std::move(name)) { }
    virtual ~Frame() {
      notifyRemoveFromHierarchy();
      if (parent_)
        parent_->removeChild(this);

      removeAllChildren();
    }

    auto& onDraw() { return on_draw_; }
    auto& onResize() { return on_resize_; }
    auto& onVisibilityChange() { return on_visibility_change_; }
    auto& onHierarchyChange() { return on_hierarchy_change_; }
    auto& onFocusChange() { return on_focus_change_; }
    auto& onMouseEnter() { return on_mouse_enter_; }
    auto& onMouseExit() { return on_mouse_exit_; }
    auto& onMouseDown() { return on_mouse_down_; }
    auto& onMouseUp() { return on_mouse_up_; }
    auto& onMouseMove() { return on_mouse_move_; }
    auto& onMouseDrag() { return on_mouse_drag_; }
    auto& onMouseWheel() { return on_mouse_wheel_; }
    auto& onKeyPress() { return on_key_press_; }
    auto& onKeyRelease() { return on_key_release_; }
    auto& onTextInput() { return on_text_input_; }

    virtual void init() { initChildren(); }
    virtual void draw(Canvas& canvas) { }
    virtual void destroy() { destroyChildren(); }

    virtual void resized() { }
    virtual void visibilityChanged() { }
    virtual void hierarchyChanged() { }
    virtual void focusChanged(bool is_focused, bool was_clicked) { }

    virtual void mouseEnter(const MouseEvent& e) { }
    virtual void mouseExit(const MouseEvent& e) { }
    virtual void mouseDown(const MouseEvent& e) { }
    virtual void mouseUp(const MouseEvent& e) { }
    virtual void mouseMove(const MouseEvent& e) { }
    virtual void mouseDrag(const MouseEvent& e) { }
    virtual void mouseWheel(const MouseEvent& e) { }
    virtual bool keyPress(const KeyEvent& e) { return false; }
    virtual bool keyRelease(const KeyEvent& e) { return false; }

    virtual bool receivesTextInput() { return false; }
    virtual void textInput(const std::string& text) { }

    virtual bool receivesDragDropFiles() { return false; }
    virtual std::string dragDropFileExtensionRegex() { return ".*"; }
    virtual bool receivesMultipleDragDropFiles() { return false; }
    virtual void dragFilesEnter(const std::vector<std::string>& paths) { }
    virtual void dragFilesExit() { }
    virtual void dropFiles(const std::vector<std::string>& paths) { }
    virtual bool isDragDropSource() { return false; }
    virtual std::string startDragDropSource() { return ""; }
    virtual void cleanupDragDropSource() { }

    void setPalette(Palette* palette) {
      palette_ = palette;
      for (Frame* child : children_)
        child->setPalette(palette);
    }

    Palette* palette() const { return palette_; }

    void setPaletteOverride(int override_id, bool recursive = true) {
      palette_override_ = override_id;
      if (recursive) {
        for (Frame* child : children_)
          child->setPaletteOverride(override_id, true);
      }
    }
    int paletteOverride() const { return palette_override_; }

    bool initialized() const { return initialized_; }
    void redraw() {
      if (isVisible() && isDrawing() && !redrawing_)
        redrawing_ = requestRedraw();
    }

    void redrawAll() {
      redraw();
      for (Frame* child : children_)
        child->redrawAll();
    }

    Region* region() { return &region_; }

    void setPostEffect(PostEffect* post_effect);
    PostEffect* postEffect() const { return post_effect_; }
    void removePostEffect();

    void setAlphaTransparency(float alpha) {
      alpha_transparency_ = alpha;
      redraw();
    }

    void removeAlphaTransparency() {
      alpha_transparency_ = 1.0f;
      redraw();
    }

    void setCached(bool cached) {
      cached_ = cached;
      redraw();
    }

    void setStenciled(bool stenciled) {
      stenciled_ = stenciled;
      redraw();
    }

    const std::string& name() const { return name_; }
    void setName(std::string name) { name_ = std::move(name); }

    void setVisible(bool visible);
    bool isVisible() const { return visible_; }
    void setDrawing(bool drawing);
    bool isDrawing() const { return drawing_; }

    void addChild(Frame* child, bool make_visible = true);
    void removeChild(Frame* child);
    void removeAllChildren();
    int indexOfChild(const Frame* child) const;
    void setParent(Frame* parent) {
      VISAGE_ASSERT(parent != this);

      parent_ = parent;
      if (parent && parent->palette())
        setPalette(parent->palette());
    }
    Frame* parent() const { return parent_; }

    void setEventHandler(FrameEventHandler* handler) {
      event_handler_ = handler;
      for (Frame* child : children_)
        child->setEventHandler(handler);
    }
    FrameEventHandler* eventHandler() const { return event_handler_; }

    template<typename T>
    T* findParent() const {
      Frame* frame = parent_;
      while (frame) {
        T* casted = dynamic_cast<T*>(frame);
        if (casted)
          return casted;

        frame = frame->parent_;
      }

      return nullptr;
    }

    bool containsPoint(Point point) const { return bounds_.contains(point); }
    Frame* frameAtPoint(Point point);
    Frame* topParentFrame();

    void setBounds(Bounds bounds);
    void setBounds(int x, int y, int width, int height) { setBounds({ x, y, width, height }); }
    void computeLayout();
    const Bounds& bounds() const { return bounds_; }
    void setTopLeft(int x, int y) { setBounds(x, y, width(), height()); }
    Point topLeft() const { return { bounds_.x(), bounds_.y() }; }
    void setOnTop(bool on_top) { on_top_ = on_top; }
    bool isOnTop() const { return on_top_; }

    Layout& layout() {
      if (layout_ == nullptr)
        layout_ = std::make_unique<Layout>();
      return *layout_;
    }
    void clearLayout() { layout_ = nullptr; }
    void setFlexLayout(bool flex) { layout().setFlex(flex); }

    int x() const { return bounds_.x(); }
    int y() const { return bounds_.y(); }
    int width() const { return bounds_.width(); }
    int height() const { return bounds_.height(); }
    int right() const { return bounds_.right(); }
    int bottom() const { return bounds_.bottom(); }
    float aspectRatio() const { return bounds_.width() * 1.0f / bounds_.height(); }
    Bounds localBounds() const { return { 0, 0, width(), height() }; }
    Point positionInWindow() const;
    Bounds relativeBounds(const Frame* other) const;

    bool acceptsKeystrokes() const { return accepts_keystrokes_; }
    void setAcceptsKeystrokes(bool accepts_keystrokes) { accepts_keystrokes_ = accepts_keystrokes; }

    bool ignoresMouseEvents() const { return ignores_mouse_events_; }
    void setIgnoresMouseEvents(bool ignore, bool pass_to_children) {
      ignores_mouse_events_ = ignore;
      pass_mouse_events_to_children_ = pass_to_children;
    }

    bool hasKeyboardFocus() const { return keyboard_focus_; }
    bool tryFocusTextReceiver();
    bool focusNextTextReceiver(const Frame* starting_child = nullptr) const;
    bool focusPreviousTextReceiver(const Frame* starting_child = nullptr) const;

    void drawToRegion(Canvas& canvas);

    void setDimensionScaling(float dpi_scale, float width_scale, float height_scale) {
      dpi_scale_ = dpi_scale;
      width_scale_ = width_scale;
      height_scale_ = height_scale;

      for (Frame* child : children_)
        child->setDimensionScaling(dpi_scale, width_scale, height_scale);
    }

    float dpiScale() const { return dpi_scale_; }
    float widthScale() const { return width_scale_; }
    float heightScale() const { return height_scale_; }

    bool requestRedraw() {
      if (event_handler_ && event_handler_->request_redraw) {
        event_handler_->request_redraw(this);
        return true;
      }
      return false;
    }

    void requestKeyboardFocus() {
      if (event_handler_ && event_handler_->request_keyboard_focus)
        event_handler_->request_keyboard_focus(this);
    }

    void notifyRemoveFromHierarchy() {
      if (event_handler_ && event_handler_->remove_from_hierarchy)
        event_handler_->remove_from_hierarchy(this);
    }

    void setMouseRelativeMode(bool visible) {
      if (event_handler_ && event_handler_->set_mouse_relative_mode)
        event_handler_->set_mouse_relative_mode(visible);
    }

    void setCursorStyle(MouseCursor style) {
      if (event_handler_ && event_handler_->set_cursor_style)
        event_handler_->set_cursor_style(style);
    }

    void setCursorVisible(bool visible) {
      if (event_handler_ && event_handler_->set_cursor_visible)
        event_handler_->set_cursor_visible(visible);
    }

    std::string readClipboardText() {
      if (event_handler_ && event_handler_->read_clipboard_text)
        return event_handler_->read_clipboard_text();
      return "";
    }

    void setClipboardText(const std::string& text) {
      if (event_handler_ && event_handler_->set_clipboard_text)
        event_handler_->set_clipboard_text(text);
    }

    void processMouseEnter(const MouseEvent& e) { on_mouse_enter_.callback(e); }
    void processMouseExit(const MouseEvent& e) { on_mouse_exit_.callback(e); }
    void processMouseDown(const MouseEvent& e) { on_mouse_down_.callback(e); }
    void processMouseUp(const MouseEvent& e) { on_mouse_up_.callback(e); }
    void processMouseMove(const MouseEvent& e) { on_mouse_move_.callback(e); }
    void processMouseDrag(const MouseEvent& e) { on_mouse_drag_.callback(e); }
    void processMouseWheel(const MouseEvent& e) { on_mouse_wheel_.callback(e); }
    void processFocusChanged(bool is_focused, bool was_clicked) {
      keyboard_focus_ = is_focused && accepts_keystrokes_;
      focusChanged(is_focused, was_clicked);
    }

    bool processKeyPress(const KeyEvent& e) { return on_key_press_.callback(e); }
    bool processKeyRelease(const KeyEvent& e) { return on_key_release_.callback(e); }
    void processTextInput(const std::string& text) { textInput(text); }

    float paletteValue(unsigned int value_id) const;
    QuadColor paletteColor(unsigned int color_id) const;

    void addUndoableAction(std::unique_ptr<UndoableAction> action) const;
    void triggerUndo() const;
    void triggerRedo() const;
    bool canUndo() const;
    bool canRedo() const;

  private:
    void notifyHierarchyChanged() {
      on_hierarchy_change_.callback();
      for (Frame* child : children_)
        child->notifyHierarchyChanged();
    }

    void initChildren();
    void destroyChildren();

    bool requiresLayer() const {
      return post_effect_ || cached_ || stenciled_ || alpha_transparency_ != 1.0f;
    }

    std::string name_;
    Bounds bounds_;

    CallbackList<void(Canvas&)> on_draw_ { [this](Canvas& e) -> void { draw(e); } };
    CallbackList<void()> on_resize_ { [this] { resized(); } };
    CallbackList<void()> on_visibility_change_ { [this] { visibilityChanged(); } };
    CallbackList<void()> on_hierarchy_change_ { [this] { hierarchyChanged(); } };
    CallbackList<void(bool, bool)> on_focus_change_ { [this](bool is_focused, bool was_clicked) {
      focusChanged(is_focused, was_clicked);
    } };
    CallbackList<void(const MouseEvent&)> on_mouse_enter_ { [this](auto& e) { mouseEnter(e); } };
    CallbackList<void(const MouseEvent&)> on_mouse_exit_ { [this](auto& e) { mouseExit(e); } };
    CallbackList<void(const MouseEvent&)> on_mouse_down_ { [this](auto& e) { mouseDown(e); } };
    CallbackList<void(const MouseEvent&)> on_mouse_up_ { [this](auto& e) { mouseUp(e); } };
    CallbackList<void(const MouseEvent&)> on_mouse_move_ { [this](auto& e) { mouseMove(e); } };
    CallbackList<void(const MouseEvent&)> on_mouse_drag_ { [this](auto& e) { mouseDrag(e); } };
    CallbackList<void(const MouseEvent&)> on_mouse_wheel_ { [this](auto& e) { mouseWheel(e); } };
    CallbackList<bool(const KeyEvent&)> on_key_press_ { [this](auto& e) { return keyPress(e); } };
    CallbackList<bool(const KeyEvent&)> on_key_release_ { [this](auto& e) { return keyRelease(e); } };
    CallbackList<void(const std::string&)> on_text_input_ { [this](auto& text) { textInput(text); } };

    bool on_top_ = false;
    bool visible_ = true;
    bool keyboard_focus_ = false;
    bool accepts_keystrokes_ = false;
    bool ignores_mouse_events_ = false;
    bool pass_mouse_events_to_children_ = true;

    std::vector<Frame*> children_;
    Frame* parent_ = nullptr;
    FrameEventHandler* event_handler_ = nullptr;

    float dpi_scale_ = 1.0f;
    float width_scale_ = 1.0f;
    float height_scale_ = 1.0f;
    Palette* palette_ = nullptr;
    int palette_override_ = 0;
    bool initialized_ = false;

    PostEffect* post_effect_ = nullptr;
    bool cached_ = false;
    bool stenciled_ = false;
    float alpha_transparency_ = 1.0f;
    Region region_;
    std::unique_ptr<Layout> layout_;
    bool drawing_ = true;
    bool redrawing_ = false;
  };
}