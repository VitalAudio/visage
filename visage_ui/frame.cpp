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

#include "frame.h"

#include "visage_graphics/theme.h"

namespace visage {
  void Frame::setVisible(bool visible) {
    if (visible_ != visible) {
      visible_ = visible;
      on_visibility_change_.callback();
    }

    region_.setVisible(visible);
    if (visible)
      redraw();
    else
      redrawing_ = false;

    setDrawing(visible && (parent_ == nullptr || parent_->isDrawing()));
  }

  void Frame::setDrawing(bool drawing) {
    if (drawing == drawing_)
      return;

    drawing_ = drawing;
    if (drawing_)
      redraw();
    else
      redrawing_ = false;

    for (Frame* child : children_) {
      if (child->isVisible() && child->isDrawing() != drawing_)
        child->setDrawing(drawing_);
    }
  }

  void Frame::addChild(Frame* child, bool make_visible) {
    VISAGE_ASSERT(child && child != this);
    if (child == nullptr)
      return;

    children_.push_back(child);
    child->parent_ = this;
    child->setEventHandler(event_handler_);
    if (palette_)
      child->setPalette(palette_);

    if (!make_visible)
      child->setVisible(false);

    region_.addRegion(child->region());

    child->setDpiScale(dpi_scale_);
    if (initialized_)
      child->init();

    computeLayout();
    computeLayout(child);
    child->redrawAll();
  }

  void Frame::addChild(std::unique_ptr<Frame> child, bool make_visible) {
    addChild(child.get(), make_visible);
    owned_children_[child.get()] = std::move(child);
  }

  void Frame::removeChild(Frame* child) {
    VISAGE_ASSERT(child && child != this);
    if (child == nullptr)
      return;

    child->region()->invalidate();
    child->notifyRemoveFromHierarchy();
    eraseChild(child);
    child->notifyHierarchyChanged();
    if (owned_children_.count(child))
      owned_children_.erase(child);

    computeLayout();
  }

  void Frame::removeAllChildren() {
    while (!children_.empty())
      eraseChild(children_.back());

    owned_children_.clear();
    computeLayout();
  }

  int Frame::indexOfChild(const Frame* child) const {
    for (int i = 0; i < children_.size(); ++i) {
      if (children_[i] == child)
        return i;
    }
    return -1;
  }

  Frame* Frame::frameAtPoint(Point point) {
    if (pass_mouse_events_to_children_) {
      for (auto it = children_.rbegin(); it != children_.rend(); ++it) {
        auto& child = *it;
        if (child->isOnTop() && child->isVisible() && child->containsPoint(point)) {
          Frame* result = child->frameAtPoint(point - child->topLeft());
          if (result)
            return result;
        }
      }
      for (auto it = children_.rbegin(); it != children_.rend(); ++it) {
        auto& child = *it;
        if (!child->isOnTop() && child->isVisible() && child->containsPoint(point)) {
          Frame* result = child->frameAtPoint(point - child->topLeft());
          if (result)
            return result;
        }
      }
    }

    if (!ignores_mouse_events_)
      return this;
    return nullptr;
  }

  Frame* Frame::topParentFrame() {
    Frame* frame = this;
    while (frame->parent_)
      frame = frame->parent_;
    return frame;
  }

  void Frame::setBounds(Bounds bounds) {
    IBounds new_native_bounds = (bounds * dpi_scale_).round();
    if (bounds_ == bounds && native_bounds_ == new_native_bounds)
      return;

    bounds_ = bounds;
    native_bounds_ = new_native_bounds;
    region_.setBounds(native_bounds_.x(), native_bounds_.y(), native_bounds_.width(),
                      native_bounds_.height());
    computeLayout();
    if (layout_ == nullptr || !layout_->flex()) {
      for (Frame* child : children_)
        computeLayout(child);
    }

    on_resize_.callback();
    redraw();
  }

  void Frame::setNativeBounds(visage::IBounds native_bounds) {
    setBounds(Bounds(native_bounds) * (1.0f / dpi_scale_));
  }

  void Frame::computeLayout() {
    if (nativeWidth() && nativeHeight() && layout_.get() && layout().flex()) {
      std::vector<const Layout*> children_layouts;
      for (Frame* child : children_) {
        if (child->layout_)
          children_layouts.push_back(child->layout_.get());
      }

      std::vector<IBounds> children_bounds = layout().flexPositions(children_layouts,
                                                                    nativeLocalBounds(), dpi_scale_);
      for (int i = 0; i < children_.size(); ++i) {
        if (children_[i]->layout_)
          children_[i]->setNativeBounds(children_bounds[i]);
      }
    }
  }

  void Frame::computeLayout(Frame* child) {
    if (child->layout_ == nullptr || (layout_ && layout_->flex()))
      return;

    int width = this->nativeWidth();
    int height = this->nativeHeight();
    float dpi = dpi_scale_;

    int pad_left = 0;
    int pad_top = 0;
    int pad_right = 0;
    int pad_bottom = 0;

    if (layout_) {
      pad_left = layout_->paddingLeft().computeInt(dpi, width, height, 0);
      pad_top = layout_->paddingTop().computeInt(dpi, width, height, 0);
      pad_right = layout_->paddingRight().computeInt(dpi, width, height, 0);
      pad_bottom = layout_->paddingBottom().computeInt(dpi, width, height, 0);
    }

    int x = child->x();
    int y = child->y();
    int dist_right = width - child->nativeRight();
    int dist_bottom = height - child->nativeBottom();

    x = pad_left + child->layout_->marginLeft().computeInt(dpi, width, height, x - pad_left);
    y = pad_top + child->layout_->marginTop().computeInt(dpi, width, height, y - pad_top);
    dist_right = pad_right +
                 child->layout_->marginRight().computeInt(dpi, width, height, dist_right - pad_right);
    dist_bottom = pad_bottom + child->layout_->marginBottom().computeInt(dpi, width, height,
                                                                         dist_bottom - pad_bottom);

    int right = width - dist_right;
    int bottom = height - dist_bottom;
    int w = child->layout_->width().computeInt(dpi, width, height, right - x);
    int h = child->layout_->height().computeInt(dpi, width, height, bottom - y);
    child->setNativeBounds(x, y, w, h);
  }

  Point Frame::positionInWindow() const {
    Point global_position = topLeft();
    Frame* frame = parent_;
    while (frame) {
      global_position = global_position + frame->topLeft();
      frame = frame->parent_;
    }

    return global_position;
  }

  Bounds Frame::relativeBounds(const Frame* other) const {
    Point position = positionInWindow();
    Point other_position = other->positionInWindow();
    float width = other->bounds().width();
    float height = other->bounds().height();
    return { other_position.x - position.x, other_position.y - position.y, width, height };
  }

  bool Frame::tryFocusTextReceiver() {
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

  bool Frame::focusNextTextReceiver(const Frame* starting_child) const {
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

  bool Frame::focusPreviousTextReceiver(const Frame* starting_child) const {
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

  void Frame::initChildren() {
    if (initialized_)
      return;

    initialized_ = true;
    for (Frame* child : children_)
      child->init();
  }

  void Frame::drawToRegion(Canvas& canvas) {
    if (!redrawing_)
      return;

    redrawing_ = false;
    region_.invalidate();
    region_.setNeedsLayer(requiresLayer());
    if (width() <= 0 || height() <= 0) {
      region_.clear();
      return;
    }

    canvas.beginRegion(&region_);

    if (!palette_override_.isDefault())
      canvas.setPaletteOverride(palette_override_);
    if (palette_)
      canvas.setPalette(palette_);

    on_draw_.callback(canvas);
    if (alpha_transparency_ != 1.0f) {
      canvas.setBlendMode(BlendMode::Mult);
      canvas.setColor(Color(0xffffffff).withAlpha(alpha_transparency_));
      canvas.fill(0, 0, width(), height());
    }
    canvas.endRegion();
  }

  void Frame::destroyChildren() {
    initialized_ = false;
    for (Frame* child : children_)
      child->destroy();
  }

  void Frame::eraseChild(Frame* child) {
    child->parent_ = nullptr;
    child->event_handler_ = nullptr;
    region_.removeRegion(child->region());
    children_.erase(std::find(children_.begin(), children_.end(), child));
  }

  void Frame::setPostEffect(PostEffect* post_effect) {
    post_effect_ = post_effect;
    region_.setPostEffect(post_effect);
    if (parent_)
      parent_->redraw();
  }

  void Frame::removePostEffect() {
    VISAGE_ASSERT(post_effect_);
    post_effect_ = nullptr;
  }

  float Frame::paletteValue(theme::ValueId value_id) const {
    if (palette_) {
      const Frame* frame = this;
      float result = 0.0f;
      while (frame) {
        theme::OverrideId override_id = frame->palette_override_;
        if (!override_id.isDefault() && palette_->value(override_id, value_id, result))
          return result;
        frame = frame->parent_;
      }
      if (palette_->value({}, value_id, result))
        return result;
    }

    return theme::ValueId::defaultValue(value_id);
  }

  Brush Frame::paletteColor(theme::ColorId color_id) const {
    if (palette_) {
      Brush result;
      const Frame* frame = this;
      while (frame) {
        theme::OverrideId override_id = frame->palette_override_;
        if (!override_id.isDefault() && palette_->color(override_id, color_id, result))
          return result;
        frame = frame->parent_;
      }
      if (palette_->color({}, color_id, result))
        return result;
    }

    return Brush::solid(theme::ColorId::defaultColor(color_id));
  }

  void Frame::addUndoableAction(std::unique_ptr<UndoableAction> action) const {
    UndoHistory* history = findParent<UndoHistory>();
    if (history)
      history->push(std::move(action));
  }

  void Frame::triggerUndo() const {
    UndoHistory* history = findParent<UndoHistory>();
    if (history)
      history->undo();
  }

  void Frame::triggerRedo() const {
    UndoHistory* history = findParent<UndoHistory>();
    if (history)
      history->redo();
  }

  bool Frame::canUndo() const {
    UndoHistory* history = findParent<UndoHistory>();
    if (history)
      return history->canUndo();
    return false;
  }

  bool Frame::canRedo() const {
    UndoHistory* history = findParent<UndoHistory>();
    if (history)
      return history->canRedo();
    return false;
  }
}
