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
#include "undo_history.h"
#include "visage_graphics/canvas.h"
#include "visage_graphics/icon.h"
#include "visage_graphics/palette.h"
#include "visage_graphics/theme.h"
#include "visage_utils/space.h"
#include "visage_utils/string_utils.h"

#include <string>
#include <vector>

namespace visage {
  class UiFrame;

  struct PopupOptions {
    String name;
    int id = -1;
    Icon icon;
    bool is_break = false;
    bool selected = false;
    bool auto_select = true;
    std::vector<PopupOptions> sub_options;

    PopupOptions* subOption(int search_id) {
      for (PopupOptions& option : sub_options) {
        if (option.id == search_id)
          return &option;

        PopupOptions* sub_option = option.subOption(search_id);
        if (sub_option)
          return sub_option;
      }
      return nullptr;
    }

    void addOption(int option_id, const String& option_name, bool option_selected = false) {
      sub_options.push_back({ option_name, option_id });
      sub_options.back().selected = option_selected;
    }

    void addOption(PopupOptions options) { sub_options.push_back(std::move(options)); }
    void addBreak() { sub_options.push_back({ "", -1, {}, true }); }
    int size() const { return sub_options.size(); }
  };

  struct FrameEventHandler {
    std::function<void(UiFrame*)> request_redraw = nullptr;
    std::function<void(UiFrame*)> request_keyboard_focus = nullptr;
    std::function<void(bool)> set_mouse_relative_mode = nullptr;
    std::function<void(MouseCursor)> set_cursor_style = nullptr;
    std::function<void(bool)> set_cursor_visible = nullptr;
    std::function<std::string()> read_clipboard_text = nullptr;
    std::function<void(std::string)> set_clipboard_text = nullptr;
  };

  class UiFrame {
  public:
    UiFrame() = default;
    explicit UiFrame(std::string name) : name_(std::move(name)) { }
    virtual ~UiFrame() = default;

    virtual void onVisibilityChange() { }
    virtual void onHierarchyChanged() { }
    virtual void resized() { }

    virtual void init() { initChildren(); }
    virtual void draw(Canvas& canvas) { }
    virtual void destroy() { destroyChildren(); }

    void setPalette(Palette* palette) {
      palette_ = palette;
      for (UiFrame* child : children_)
        child->setPalette(palette);
    }

    Palette* palette() const { return palette_; }
    Canvas* postEffectCanvas() const { return post_effect_canvas_.get(); }
    Canvas* canvas() const { return canvas_; }

    void setPaletteOverride(int override_id) { palette_override_ = override_id; }
    int paletteOverride() const { return palette_override_; }

    bool initialized() const { return initialized_; }
    void redraw() {
      if (isVisible() && isDrawing() && !redrawing_) {
        redrawing_ = requestRedraw();
        region_.invalidate();
      }
    }

    void setCanvas(Canvas* canvas) {
      if (post_effect_canvas_ && post_effect_canvas_.get() != canvas)
        return;

      canvas_ = canvas;
      for (UiFrame* child : children_)
        child->setCanvas(canvas);
    }

    Canvas::Region* region() { return &region_; }
    
    void setPostEffectCanvasSettings();
    void setPostEffect(PostEffect* post_effect);
    void removePostEffect();

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
    virtual void onColorsChanged() { }

    virtual bool receivesTextInput() { return false; }
    virtual void onTextInput(const std::string& text) { }

    virtual bool receivesDragDropFiles() { return false; }
    virtual std::string dragDropFileExtensionRegex() { return ".*"; }
    virtual bool receivesMultipleDragDropFiles() { return false; }
    virtual void dragFilesEnter(const std::vector<std::string>& paths) { }
    virtual void dragFilesExit() { }
    virtual void dropFiles(const std::vector<std::string>& paths) { }
    virtual bool isDragDropSource() { return false; }
    virtual std::string startDragDropSource() { return ""; }
    virtual void cleanupDragDropSource() { }

    const std::string& name() const { return name_; }
    void setName(std::string name) { name_ = std::move(name); }

    void setVisible(bool visible);
    bool isVisible() const { return visible_; }
    void setDrawing(bool drawing);
    bool isDrawing() const { return drawing_; }

    void addChild(UiFrame* child, bool make_visible = true);
    void removeChild(UiFrame* child);
    int indexOfChild(const UiFrame* child) const;
    void setParent(UiFrame* parent) {
      VISAGE_ASSERT(parent != this);

      parent_ = parent;
      if (parent && parent->palette())
        setPalette(parent->palette());
    }
    UiFrame* parent() const { return parent_; }

    void setEventHandler(FrameEventHandler* handler) {
      event_handler_ = handler;
      for (UiFrame* child : children_)
        child->setEventHandler(handler);
    }
    FrameEventHandler* eventHandler() const { return event_handler_; }

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

    bool containsPoint(Point point) const { return bounds_.contains(point); }
    UiFrame* frameAtPoint(Point point);
    UiFrame* topParentFrame();

    void setBounds(Bounds bounds);
    void setBounds(int x, int y, int width, int height) { setBounds({ x, y, width, height }); }
    const Bounds& bounds() const { return bounds_; }
    void setTopLeft(int x, int y) { setBounds(x, y, width(), height()); }
    Point topLeft() const { return { bounds_.x(), bounds_.y() }; }
    void setOnTop(bool on_top) { on_top_ = on_top; }
    bool isOnTop() const { return on_top_; }

    int x() const { return bounds_.x(); }
    int y() const { return bounds_.y(); }
    int width() const { return bounds_.width(); }
    int height() const { return bounds_.height(); }
    int right() const { return bounds_.right(); }
    int bottom() const { return bounds_.bottom(); }
    float aspectRatio() const { return bounds_.width() * 1.0f / bounds_.height(); }
    Bounds localBounds() const { return { 0, 0, width(), height() }; }
    Point positionInWindow() const;
    Bounds relativeBounds(const UiFrame* other) const;

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

    void drawToRegion();

    void setDrawFunction(std::function<void(Canvas&)> draw_function) {
      draw_function_ = std::move(draw_function);
    }

    void setOnMouseEnter(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_enter_ = std::move(callback);
    }

    void setOnMouseExit(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_exit_ = std::move(callback);
    }

    void setOnMouseDown(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_down_ = std::move(callback);
    }

    void setOnMouseUp(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_up_ = std::move(callback);
    }

    void setOnMouseMove(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_move_ = std::move(callback);
    }

    void setOnMouseDrag(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_drag_ = std::move(callback);
    }

    void setOnMouseWheel(std::function<void(const MouseEvent& e)> callback) {
      on_mouse_wheel_ = std::move(callback);
    }

    void setOnKeyPress(std::function<bool(const KeyEvent& e)> callback) {
      on_key_press_ = std::move(callback);
    }

    void setOnKeyRelease(std::function<bool(const KeyEvent& e)> callback) {
      on_key_release_ = std::move(callback);
    }

    void setDimensionScaling(float dpi_scale, float width_scale, float height_scale) {
      dpi_scale_ = dpi_scale;
      width_scale_ = width_scale;
      height_scale_ = height_scale;

      for (UiFrame* child : children_)
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

    void addResizeCallback(std::function<void(UiFrame*)> callback) {
      resize_callbacks_.push_back(std::move(callback));
    }

    void removeResizeCallback(const std::function<void(UiFrame*)>& callback) {
      auto compare = [&](const std::function<void(UiFrame*)>& other) {
        return other.target_type() == callback.target_type();
      };

      auto it = std::remove_if(resize_callbacks_.begin(), resize_callbacks_.end(), compare);
      resize_callbacks_.erase(it);
    }

    float paletteValue(unsigned int value_id);
    QuadColor paletteColor(unsigned int color_id);

    bool isPopupVisible() const;
    void showPopupMenu(const PopupOptions& options, Bounds bounds,
                       std::function<void(int)> callback, std::function<void()> cancel = {});
    void showPopupMenu(const PopupOptions& options, Point position,
                       std::function<void(int)> callback, std::function<void()> cancel = {});
    void showValueDisplay(const std::string& text, Bounds bounds, Font::Justification justification,
                          bool primary);
    void hideValueDisplay(bool primary) const;
    void addUndoableAction(std::unique_ptr<UndoableAction> action) const;
    void triggerUndo() const;
    void triggerRedo() const;
    bool canUndo() const;
    bool canRedo() const;

  private:
    void notifyHierarchyChanged() {
      onHierarchyChanged();
      for (UiFrame* child : children_)
        child->notifyHierarchyChanged();
    }

    void initChildren();
    static void drawChildSubcanvas(const UiFrame* child, Canvas& canvas);
    void drawChildrenSubcanvases(Canvas& canvas);
    void destroyChildren();

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
    std::function<void(Canvas&)> draw_function_;

    bool on_top_ = false;
    bool visible_ = true;
    bool keyboard_focus_ = false;
    bool accepts_keystrokes_ = false;
    bool accepts_dropped_files_ = false;
    bool ignores_mouse_events_ = false;
    bool pass_mouse_events_to_children_ = true;

    std::vector<UiFrame*> children_;
    UiFrame* parent_ = nullptr;
    FrameEventHandler* event_handler_ = nullptr;

    float dpi_scale_ = 1.0f;
    float width_scale_ = 1.0f;
    float height_scale_ = 1.0f;
    Palette* palette_ = nullptr;
    int palette_override_ = 0;
    bool initialized_ = false;

    PostEffect* post_effect_ = nullptr;
    std::unique_ptr<Canvas> post_effect_canvas_;
    Canvas* canvas_ = nullptr;
    Canvas::Region region_;
    bool drawing_ = true;
    bool redrawing_ = false;
  };

  class CachedUiFrame : public UiFrame {
  public:
    class CachedImage : public Image {
    public:
      explicit CachedImage(CachedUiFrame* component) : component_(component) { }

      void draw(visage::Canvas& canvas) override {
        need_redraw_ = false;
        component_->drawToCache(canvas);
      }

      void redraw() { need_redraw_ = true; }
      bool needsRedraw() const override { return need_redraw_; }

      int width() const override { return component_->width(); }
      int height() const override { return component_->height(); }

    private:
      CachedUiFrame* component_ = nullptr;
      bool need_redraw_ = false;
    };

    void redrawCache() { cached_image_.redraw(); }

    virtual void drawToCache(Canvas& canvas) = 0;
    virtual void drawCachedImage(Canvas& canvas) {
      canvas.setColor(0xffffffff);
      canvas.image(&cached_image_, 0, 0);
    }

    CachedUiFrame() : cached_image_(this) { }

    void draw(Canvas& canvas) final { drawCachedImage(canvas); }

    CachedImage* cachedImage() { return &cached_image_; }

  private:
    CachedImage cached_image_;
    bool initialized_ = false;
  };
}