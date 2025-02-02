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

#include "popup_menu.h"
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

    child->setDimensionScaling(dpi_scale_, width_scale_, height_scale_);
    if (initialized_)
      child->init();

    computeLayout();
    computeLayout(child);
    child->redraw();
  }

  void Frame::removeChild(Frame* child) {
    VISAGE_ASSERT(child && child != this);
    if (child == nullptr)
      return;

    child->region()->invalidate();
    child->notifyRemoveFromHierarchy();
    eraseChild(child);
    child->notifyHierarchyChanged();

    computeLayout();
  }

  void Frame::removeAllChildren() {
    while (!children_.empty())
      eraseChild(children_.back());

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
    if (bounds_ == bounds)
      return;

    bounds_ = bounds;
    region_.setBounds(bounds.x(), bounds.y(), bounds.width(), bounds.height());
    computeLayout();
    if (layout_ == nullptr || !layout_->flex()) {
      for (Frame* child : children_)
        computeLayout(child);
    }

    on_resize_.callback();
    redraw();
  }

  void Frame::computeLayout() {
    if (layout_.get() && layout().flex()) {
      std::vector<const Layout*> children_layouts;
      for (Frame* child : children_) {
        if (child->layout_.get())
          children_layouts.push_back(child->layout_.get());
      }

      std::vector<Bounds> children_bounds = layout().flexPositions(children_layouts, localBounds(),
                                                                   dpi_scale_);
      for (int i = 0; i < children_.size(); ++i) {
        if (children_[i]->layout_.get())
          children_[i]->setBounds(children_bounds[i]);
      }
    }
  }

  void Frame::computeLayout(Frame* child) {
    if (child->layout_ == nullptr || (layout_ && layout_->flex()))
      return;

    int width = this->width();
    int height = this->height();
    float dpi = dpi_scale_;

    int pad_left = 0;
    int pad_top = 0;
    int pad_right = 0;
    int pad_bottom = 0;

    if (layout_) {
      pad_left = layout_->paddingLeft().computeWithDefault(dpi, width, height, 0);
      pad_top = layout_->paddingTop().computeWithDefault(dpi, width, height, 0);
      pad_right = layout_->paddingRight().computeWithDefault(dpi, width, height, 0);
      pad_bottom = layout_->paddingBottom().computeWithDefault(dpi, width, height, 0);
    }

    int x = child->x();
    int y = child->y();
    int dist_right = width - child->right();
    int dist_bottom = height - child->bottom();

    x = pad_left + child->layout_->marginLeft().computeWithDefault(dpi, width, height, x - pad_left);
    y = pad_top + child->layout_->marginTop().computeWithDefault(dpi, width, height, y - pad_top);
    dist_right = pad_right + child->layout_->marginRight().computeWithDefault(dpi, width, height,
                                                                              dist_right - pad_right);
    dist_bottom = pad_bottom + child->layout_->marginBottom().computeWithDefault(dpi, width, height,
                                                                                 dist_bottom - pad_bottom);

    int right = width - dist_right;
    int bottom = height - dist_bottom;
    int w = child->layout_->width().computeWithDefault(dpi, width, height, right - x);
    int h = child->layout_->height().computeWithDefault(dpi, width, height, bottom - y);
    child->setBounds(x, y, w, h);
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
    int width = other->bounds().width();
    int height = other->bounds().height();
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

  inline QuadColor colorForSampledFrame(const Frame* parent, const Frame* child, const QuadColor& background) {
    Bounds bounds = parent->relativeBounds(child);
    float width = parent->width();
    float height = parent->height();
    return { background.sampleColor(bounds.x() / width, bounds.y() / height),
             background.sampleColor(bounds.right() / width, bounds.y() / height),
             background.sampleColor(bounds.x() / width, bounds.bottom() / height),
             background.sampleColor(bounds.right() / width, bounds.bottom() / height),
             background.sampleHdr(bounds.x() / width, bounds.y() / height),
             background.sampleHdr(bounds.right() / width, bounds.y() / height),
             background.sampleHdr(bounds.x() / width, bounds.bottom() / height),
             background.sampleHdr(bounds.right() / width, bounds.bottom() / height) };
  }

  void Frame::initChildren() {
    VISAGE_ASSERT(!initialized_);

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

    if (palette_override_)
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

  float Frame::paletteValue(unsigned int value_id) const {
    float scale = 1.0f;
    theme::ValueId::ValueIdInfo info = theme::ValueId::info(value_id);
    if (info.scale_type == theme::ValueId::kScaledWidth)
      scale = widthScale();
    else if (info.scale_type == theme::ValueId::kScaledHeight)
      scale = heightScale();
    else if (info.scale_type == theme::ValueId::kScaledDpi)
      scale = dpiScale();

    if (palette_) {
      const Frame* frame = this;
      float result = 0.0f;
      while (frame) {
        int override_id = frame->palette_override_;
        if (override_id && palette_->value(override_id, value_id, result))
          return scale * result;
        frame = frame->parent_;
      }
      if (palette_->value(0, value_id, result))
        return scale * result;
    }

    return scale * theme::ValueId::defaultValue(value_id);
  }

  QuadColor Frame::paletteColor(unsigned int color_id) const {
    if (palette_) {
      QuadColor result;
      const Frame* frame = this;
      while (frame) {
        int override_id = frame->palette_override_;
        if (override_id && palette_->color(override_id, color_id, result))
          return result;
        frame = frame->parent_;
      }
      if (palette_->color(0, color_id, result))
        return result;
    }

    return theme::ColorId::defaultColor(color_id);
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