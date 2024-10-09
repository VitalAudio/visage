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

#include "ui_frame.h"
#include "undo_history.h"
#include "visage_graphics/canvas.h"
#include "visage_graphics/color.h"
#include "visage_graphics/theme.h"

namespace visage {
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

  class DrawableComponent : public UiFrame {
  public:
    THEME_DEFINE_COLOR(TextColor);
    THEME_DEFINE_COLOR(ShadowColor);
    THEME_DEFINE_COLOR(WidgetBackgroundColor);

    THEME_DEFINE_VALUE(WidgetRoundedCorner);
    THEME_DEFINE_VALUE(WidgetOverlayAlpha);

    struct ViewBounds {
      float scale_x = 1.0f;
      float scale_y = 1.0f;
      float center_x = 0.0f;
      float center_y = 0.0f;
    };

    DrawableComponent() = default;
    explicit DrawableComponent(const std::string& name) : UiFrame(name) { }

    bool isDrawing() const { return drawing_; }

    virtual void setDrawing(bool drawing) {
      if (drawing == drawing_)
        return;

      drawing_ = drawing;
      if (drawing_)
        redraw();

      for (DrawableComponent* child : children_) {
        if (child->isVisible() && child->isDrawing() != drawing_)
          child->setDrawing(drawing_);
      }
    }

    void setVisible(bool visible) override {
      UiFrame::setVisible(visible);
      region_.setVisible(visible);
      if (visible)
        redraw();

      setDrawing(visible && (parent_ == nullptr || parent_->isDrawing()));
    }

    void setBounds(Bounds bounds) override {
      UiFrame::setBounds(bounds);
      region_.setBounds(bounds.x(), bounds.y(), bounds.width(), bounds.height());
      setPostEffectCanvasSettings();
      redraw();
    }

    void setBounds(int x, int y, int width, int height) override {
      setBounds({ x, y, width, height });
    }

    virtual void init() { initChildren(); }
    virtual void draw(Canvas& canvas) { }
    void setDrawFunction(std::function<void(Canvas&)> draw_function) {
      draw_function_ = draw_function;
    }
    void drawToRegion();
    virtual void destroy() { destroyChildren(); }
    virtual void onColorsChanged() { }

    virtual bool requestRedraw(DrawableComponent* component) { return false; }

    virtual float widthScale() {
      if (parent_)
        return topParentComponent()->widthScale();
      return 1.0f;
    }

    virtual float heightScale() {
      if (parent_)
        return topParentComponent()->heightScale();
      return 1.0f;
    }

    virtual float dpiScale() {
      if (parent_)
        return topParentComponent()->dpiScale();
      return 1.0f;
    }

    virtual void setCursorStyle(MouseCursor style) {
      if (parent_)
        topParentComponent()->setCursorStyle(style);
    }

    virtual void setCursorVisible(bool visible) {
      if (parent_)
        topParentComponent()->setCursorVisible(visible);
    }

    virtual std::string readClipboardText() {
      if (parent_)
        return topParentComponent()->readClipboardText();
      return "";
    }

    virtual void setClipboardText(const std::string& text) {
      if (parent_)
        topParentComponent()->setClipboardText(text);
    }

    virtual void setMouseRelativeMode(bool relative) {
      if (parent_)
        topParentComponent()->setMouseRelativeMode(relative);
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
    bool visibleInParents() const {
      return isVisible() && (parent_ == nullptr || parent_->visibleInParents());
    }

    DrawableComponent* topParentComponent() {
      DrawableComponent* parent = this;
      while (parent->parent_)
        parent = parent->parent_;
      return parent;
    }

    void addDrawableComponent(DrawableComponent* component, bool make_visible = true) {
      VISAGE_ASSERT(component && component != this);
      if (component == nullptr)
        return;

      children_.push_back(component);
      component->parent_ = this;
      if (palette_)
        component->setPalette(palette_);

      if (!make_visible)
        component->setVisible(false);

      if (component->post_effect_ == nullptr) {
        region_.addRegion(component->region());
        component->setCanvas(canvas_);
      }

      addChild(component);
      if (initialized_)
        component->init();
    }

    void setPostEffectCanvasSettings() {
      if (post_effect_canvas_ == nullptr)
        return;

      post_effect_canvas_->setDimensions(width(), height());
      post_effect_canvas_->setWidthScale(widthScale());
      post_effect_canvas_->setHeightScale(heightScale());
      post_effect_canvas_->setDpiScale(dpiScale());
    }

    void addDrawableComponent(DrawableComponent* component, PostEffect* post_effect,
                              bool make_visible = true) {
      component->post_effect_ = post_effect;
      component->post_effect_canvas_ = std::make_unique<Canvas>();
      component->post_effect_canvas_->addRegion(component->region());
      component->post_effect_canvas_->setHdr(post_effect->hdr());
      component->setCanvas(component->post_effect_canvas_.get());
      addDrawableComponent(component, make_visible);
      setPostEffectCanvasSettings();
    }

    void removeDrawableComponent(DrawableComponent* component) {
      VISAGE_ASSERT(component && component != this);

      region_.removeRegion(component->region());
      children_.erase(std::find(children_.begin(), children_.end(), component));
      removeChild(component);
    }

    void setParent(DrawableComponent* parent) {
      VISAGE_ASSERT(parent != this);

      parent_ = parent;
      if (parent && parent->palette())
        setPalette(parent->palette());
    }

    void setPalette(Palette* palette) {
      palette_ = palette;
      for (DrawableComponent* child : children_)
        child->setPalette(palette);
    }

    Palette* palette() const { return palette_; }

    void setPaletteOverride(int override_id) { palette_override_ = override_id; }
    int paletteOverride() const { return palette_override_; }

    bool initialized() const { return initialized_; }
    void redraw() {
      if (isVisible() && isDrawing() && !redrawing_) {
        redrawing_ = topParentComponent()->requestRedraw(this);
        region_.invalidate();
      }
    }

  protected:
    void setCanvas(Canvas* canvas) {
      if (post_effect_canvas_ && post_effect_canvas_.get() != canvas)
        return;

      canvas_ = canvas;
      for (DrawableComponent* child : children_)
        child->setCanvas(canvas);
    }

    Canvas::Region* region() { return &region_; }

  private:
    void initChildren();
    void drawChildSubcanvas(DrawableComponent* child, Canvas& canvas);
    void drawChildrenSubcanvases(Canvas& canvas);
    void destroyChildren();
    void notifyChildrenColorsChanged();
    bool needsRedraw() const { return redrawing_; }

    ViewBounds view_bounds_;
    std::vector<DrawableComponent*> children_;
    DrawableComponent* parent_ = nullptr;
    Palette* palette_ = nullptr;
    int palette_override_ = 0;
    bool initialized_ = false;

    PostEffect* post_effect_ = nullptr;
    std::unique_ptr<Canvas> post_effect_canvas_;
    Canvas* canvas_ = nullptr;
    Canvas::Region region_;
    bool drawing_ = true;

    std::function<void(Canvas&)> draw_function_;
    bool redrawing_ = false;

    VISAGE_LEAK_CHECKER(DrawableComponent)
  };

  class CachedDrawableComponent : public DrawableComponent {
  public:
    class CachedImage : public Image {
    public:
      explicit CachedImage(CachedDrawableComponent* component) : component_(component) { }

      void draw(visage::Canvas& canvas) override {
        need_redraw_ = false;
        component_->drawToCache(canvas);
      }

      void redraw() { need_redraw_ = true; }
      bool needsRedraw() const override { return need_redraw_; }

      int width() const override { return component_->width(); }
      int height() const override { return component_->height(); }

    private:
      CachedDrawableComponent* component_ = nullptr;
      bool need_redraw_ = false;
    };

    void redraw() { cached_image_.redraw(); }

    virtual void drawToCache(Canvas& canvas) = 0;
    virtual void drawCachedImage(Canvas& canvas) {
      canvas.setColor(0xffffffff);
      canvas.image(&cached_image_, 0, 0);
    }

    CachedDrawableComponent() : cached_image_(this) { }

    void draw(Canvas& canvas) final { drawCachedImage(canvas); }

    CachedImage* cachedImage() { return &cached_image_; }

  private:
    CachedImage cached_image_;
    bool initialized_ = false;
  };
}