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

#include "drawable_component.h"

#include "popup_menu.h"
#include "visage_graphics/canvas.h"
#include "visage_graphics/palette.h"

#include <utility>

namespace visage {
  THEME_IMPLEMENT_COLOR(DrawableComponent, TextColor, 0xffeeeeee);
  THEME_IMPLEMENT_COLOR(DrawableComponent, ShadowColor, 0x44000000);
  THEME_IMPLEMENT_COLOR(DrawableComponent, WidgetBackgroundColor, 0xff1d2125);

  THEME_IMPLEMENT_VALUE(DrawableComponent, WidgetRoundedCorner, 9.0f, ScaledHeight, true);
  THEME_IMPLEMENT_VALUE(DrawableComponent, WidgetOverlayAlpha, 0.7f, Constant, false);

  namespace {
    QuadColor colorForSampledComponent(const DrawableComponent* parent,
                                       const DrawableComponent* child, const QuadColor& background) {
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
  }

  void DrawableComponent::initChildren() {
    VISAGE_ASSERT(!initialized_);

    initialized_ = true;
    for (DrawableComponent* child : children_)
      child->init();
  }

  void DrawableComponent::drawToRegion() {
    if (redrawing_) {
      redrawing_ = false;
      canvas_->beginRegion(&region_, x(), y(), width(), height());

      if (palette_override_)
        canvas_->setPaletteOverride(palette_override_);
      if (palette_)
        canvas_->setPalette(palette_);

      canvas_->saveState();
      if (draw_function_)
        draw_function_(*canvas_);
      else
        draw(*canvas_);
      canvas_->restoreState();
      drawChildrenSubcanvases(*canvas_);
      canvas_->endRegion();
    }
  }

  void DrawableComponent::drawChildSubcanvas(const DrawableComponent* child, Canvas& canvas) {
    if (child->isVisible() && child->post_effect_) {
      Canvas* child_canvas = child->post_effect_canvas_.get();
      canvas.subcanvas(child_canvas, child->x(), child->y(), child->width(), child->height(),
                       child->post_effect_);
    }
  }

  void DrawableComponent::drawChildrenSubcanvases(Canvas& canvas) {
    for (DrawableComponent* child : children_) {
      if (!child->isOnTop())
        drawChildSubcanvas(child, canvas);
    }
    for (DrawableComponent* child : children_) {
      if (child->isOnTop())
        drawChildSubcanvas(child, canvas);
    }
  }

  void DrawableComponent::destroyChildren() {
    initialized_ = false;
    for (DrawableComponent* child : children_)
      child->destroy();
  }

  void DrawableComponent::notifyChildrenColorsChanged() {
    for (DrawableComponent* child : children_) {
      child->onColorsChanged();
      child->notifyChildrenColorsChanged();
    }
  }

  float DrawableComponent::paletteValue(unsigned int value_id) {
    float scale = 1.0f;
    theme::ValueId::ValueIdInfo info = theme::ValueId::info(value_id);
    if (info.scale_type == theme::ValueId::kScaledWidth)
      scale = widthScale();
    else if (info.scale_type == theme::ValueId::kScaledHeight)
      scale = heightScale();
    else if (info.scale_type == theme::ValueId::kScaledDpi)
      scale = dpiScale();

    if (palette_) {
      DrawableComponent* component = this;
      float result = 0.0f;
      while (component) {
        int override_id = component->palette_override_;
        if (override_id && palette_->value(override_id, value_id, result))
          return scale * result;
        component = component->parent_;
      }
      if (palette_->value(0, value_id, result))
        return scale * result;
    }

    return scale * theme::ValueId::defaultValue(value_id);
  }

  QuadColor DrawableComponent::paletteColor(unsigned int color_id) {
    if (palette_) {
      QuadColor result;
      DrawableComponent* component = this;
      while (component) {
        int override_id = component->palette_override_;
        if (override_id && palette_->color(override_id, color_id, result))
          return result;
        component = component->parent_;
      }
      if (palette_->color(0, color_id, result))
        return result;
    }

    return theme::ColorId::defaultColor(color_id);
  }

  bool DrawableComponent::isPopupVisible() const {
    PopupDisplayer* displayer = findParent<PopupDisplayer>();
    if (displayer)
      return displayer->isPopupVisible();
    return false;
  }

  void DrawableComponent::showPopupMenu(const PopupOptions& options, Bounds bounds,
                                        std::function<void(int)> callback, std::function<void()> cancel) {
    PopupDisplayer* displayer = findParent<PopupDisplayer>();
    if (displayer)
      displayer->showPopup(options, this, bounds, std::move(callback), std::move(cancel));
  }

  void DrawableComponent::showPopupMenu(const PopupOptions& options, Point position,
                                        std::function<void(int)> callback, std::function<void()> cancel) {
    showPopupMenu(options, Bounds(position.x, position.y, 0, 0), std::move(callback), std::move(cancel));
  }

  void DrawableComponent::showValueDisplay(const std::string& text, Bounds bounds,
                                           Font::Justification justification, bool primary) {
    PopupDisplayer* displayer = findParent<PopupDisplayer>();
    if (displayer)
      displayer->showValueDisplay(text, this, bounds, justification, primary);
  }

  void DrawableComponent::hideValueDisplay(bool primary) const {
    PopupDisplayer* displayer = findParent<PopupDisplayer>();
    if (displayer)
      displayer->hideValueDisplay(primary);
  }

  void DrawableComponent::addUndoableAction(std::unique_ptr<UndoableAction> action) const {
    UndoHistory* history = findParent<UndoHistory>();
    if (history)
      history->push(std::move(action));
  }

  void DrawableComponent::triggerUndo() const {
    UndoHistory* history = findParent<UndoHistory>();
    if (history)
      history->undo();
  }

  void DrawableComponent::triggerRedo() const {
    UndoHistory* history = findParent<UndoHistory>();
    if (history)
      history->redo();
  }

  bool DrawableComponent::canUndo() const {
    UndoHistory* history = findParent<UndoHistory>();
    if (history)
      return history->canUndo();
    return false;
  }

  bool DrawableComponent::canRedo() const {
    UndoHistory* history = findParent<UndoHistory>();
    if (history)
      return history->canRedo();
    return false;
  }
}