/* Copyright Vital Audio, LLC
 *
 * va is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * va is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with va.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "visage_file_embed/embedded_file.h"
#include "visage_graphics/animation.h"
#include "visage_graphics/icon.h"
#include "visage_graphics/text.h"
#include "visage_ui/drawable_component.h"

#include <functional>

namespace visage {
  class Button : public DrawableComponent {
  public:
    class Listener {
    public:
      virtual ~Listener() = default;
      virtual void buttonToggled(Button* button, bool on) = 0;
    };

    Button() { hover_amount_.setTargetValue(1.0f); }

    explicit Button(const std::string& name) : DrawableComponent(name) {
      hover_amount_.setTargetValue(1.0f);
    }

    void draw(Canvas& canvas) final;
    virtual void draw(Canvas& canvas, float hover_amount) { }

    void onMouseEnter(const MouseEvent& e) override;
    void onMouseExit(const MouseEvent& e) override;
    void onMouseDown(const MouseEvent& e) override;
    void onMouseUp(const MouseEvent& e) override;

    void notify(bool on) {
      for (auto& callback : toggle_callbacks_)
        callback(this, on);
    }
    virtual bool toggle() { return false; }
    virtual void setToggled(bool toggled) { }
    virtual void setToggledAndNotify(bool toggled) {
      if (toggled)
        notify(false);
    }

    void addToggleCallback(std::function<void(Button*, bool)> callback) {
      toggle_callbacks_.push_back(std::move(callback));
    }

    void setToggleOnMouseDown(bool mouse_down) { toggle_on_mouse_down_ = mouse_down; }
    float hoverAmount() const { return hover_amount_.value(); }
    void setActive(bool active) { active_ = active; }
    bool isActive() const { return active_; }
    void setUndoSetupFunction(std::function<void()> undo_setup_function) {
      undo_setup_function_ = std::move(undo_setup_function);
    }
    std::function<void()> undoSetupFunction() { return undo_setup_function_; }
    bool wasAltClicked() const { return alt_clicked_; }

  private:
    std::vector<std::function<void(Button*, bool)>> toggle_callbacks_;
    Animation<float> hover_amount_;
    std::function<void()> undo_setup_function_ = nullptr;

    bool active_ = true;
    bool toggle_on_mouse_down_ = false;
    bool set_pointer_cursor_ = true;
    bool alt_clicked_ = false;

    VISAGE_LEAK_CHECKER(Button)
  };

  class UiButton : public Button {
  public:
    explicit UiButton(const std::string& text);
    explicit UiButton(const std::string& text, const Font& font);

    virtual void drawBackground(Canvas& canvas, float hover_amount);
    void draw(Canvas& canvas, float hover_amount) override;
    void setFont(const Font& font) { text_.setFont(font); }
    void setActionButton(bool action = true) { action_ = action; }
    void setText(const std::string& text) { text_.setText(text); }
    void drawBorderWhenInactive(bool border) { border_when_inactive_ = border; }

  private:
    Text text_;
    bool action_ = false;
    bool border_when_inactive_ = false;
  };

  class IconButton : public Button {
  public:
    static constexpr float kDefaultShadowRatio = 0.1f;

    IconButton() = default;
    explicit IconButton(const Icon& icon, bool shadow = false) { setIcon(icon, shadow); }

    explicit IconButton(const EmbeddedFile& icon_file, bool shadow = false) {
      setIcon(icon_file, shadow);
    }

    IconButton(const char* svg, int svg_size, bool shadow = false) {
      setIcon(svg, svg_size, shadow);
    }

    void setIcon(const EmbeddedFile& icon_file, bool shadow = false) {
      setIcon({ icon_file.data, icon_file.size, 0, 0 }, shadow);
    }

    void setIcon(const char* svg, int svg_size, bool shadow = false) {
      setIcon({ svg, svg_size, 0, 0 }, shadow);
    }

    void setIcon(Icon icon, bool shadow = false) {
      icon_ = icon;
      if (shadow) {
        shadow_ratio_ = kDefaultShadowRatio;
        shadow_ = icon_;
      }
    }

    void draw(Canvas& canvas, float hover_amount) override;

    void resized() override {
      Button::resized();
      setIconSizes();
    }

    int getMargin() const { return std::min(width(), height()) * margin_ratio_; }
    int getIconX() const { return getMargin() + std::max(0, width() - height()) / 2; }
    int getIconY() const { return getMargin() + std::max(0, height() - width()) / 2; }

    void setIconSizes() {
      int margin = std::min(width(), height()) * margin_ratio_;
      icon_.width = width() - 2 * margin;
      icon_.height = height() - 2 * margin;
      shadow_.width = icon_.width;
      shadow_.height = icon_.height;
      shadow_.blur_radius = shadow_ratio_ * std::min(icon_.width, icon_.height);
    }

    void setShadowRatio(float ratio) {
      shadow_ratio_ = ratio;
      shadow_.blur_radius = shadow_ratio_ * shadow_.width;
    }

    void setMarginRatio(float ratio) {
      margin_ratio_ = ratio;
      shadow_.blur_radius = shadow_ratio_ * shadow_.width;
      setIconSizes();
    }

  private:
    Icon icon_;
    Icon shadow_;

    float shadow_ratio_ = 0.0f;
    float margin_ratio_ = 0.0f;
  };

  class ButtonChangeAction;

  class ToggleButton : public Button {
  public:
    THEME_DEFINE_COLOR(ToggleButtonDisabled);
    THEME_DEFINE_COLOR(ToggleButtonOff);
    THEME_DEFINE_COLOR(ToggleButtonOffHover);
    THEME_DEFINE_COLOR(ToggleButtonOn);
    THEME_DEFINE_COLOR(ToggleButtonOnHover);

    THEME_DEFINE_VALUE(ButtonTextHeight);

    ToggleButton() = default;
    explicit ToggleButton(const std::string& name) : Button(name) { }

    bool toggle() override;
    void setToggled(bool toggled) override {
      toggled_ = toggled;
      toggleValueChanged();
      redraw();
    }
    virtual void toggleValueChanged() { }
    void setToggledAndNotify(bool toggled) override {
      toggled_ = toggled;
      notify(toggled);
      redraw();
    }

    bool toggled() const { return toggled_; }
    void setUndoable(bool undoable) { undoable_ = undoable; }

  private:
    bool toggled_ = false;
    bool undoable_ = true;

    VISAGE_LEAK_CHECKER(ToggleButton)
  };

  class ButtonChangeAction : public UndoableAction {
  public:
    ButtonChangeAction(ToggleButton* button, bool toggled_on) :
        button_(button), toggled_on_(toggled_on) { }

    void undo() override { button_->setToggledAndNotify(!toggled_on_); }
    void redo() override { button_->setToggledAndNotify(toggled_on_); }

  private:
    ToggleButton* button_ = nullptr;
    bool toggled_on_ = false;
  };

  class ToggleIconButton : public ToggleButton {
  public:
    static constexpr float kDefaultShadowRatio = 0.1f;

    explicit ToggleIconButton(const Icon& icon, bool shadow = false) : ToggleButton(), icon_(icon) {
      initSettings(shadow);
    }

    ToggleIconButton(const std::string& name, const Icon& icon, bool shadow = false) :
        ToggleButton(name), icon_(icon) {
      initSettings(shadow);
    }

    ToggleIconButton(const char* svg, int svg_size, bool shadow = false) :
        ToggleButton(), icon_({ svg, svg_size, 0, 0 }) {
      initSettings(shadow);
    }

    ToggleIconButton(const std::string& name, const char* svg, int svg_size, bool shadow = false) :
        ToggleButton(name), icon_({ svg, svg_size, 0, 0 }) {
      initSettings(shadow);
    }

    void initSettings(bool shadow) {
      if (shadow) {
        shadow_ratio_ = kDefaultShadowRatio;
        shadow_ = icon_;
      }
    }

    void draw(Canvas& canvas, float hover_amount) override;

    void resized() override {
      ToggleButton::resized();
      setIconSizes();
    }

    int getMargin() const { return std::min(width(), height()) * margin_ratio_; }
    int getIconX() const { return getMargin() + std::max(0, width() - height()) / 2; }
    int getIconY() const { return getMargin() + std::max(0, height() - width()) / 2; }

    void setIconSizes() {
      int margin = std::min(width(), height()) * margin_ratio_;
      icon_.width = width() - 2 * margin;
      icon_.height = height() - 2 * margin;
      shadow_.width = icon_.width;
      shadow_.height = icon_.height;
      shadow_.blur_radius = shadow_ratio_ * std::min(icon_.width, icon_.height);
    }

    void setShadowRatio(float ratio) {
      shadow_ratio_ = ratio;
      shadow_.blur_radius = shadow_ratio_ * shadow_.width;
    }

    void setMarginRatio(float ratio) {
      margin_ratio_ = ratio;
      shadow_.blur_radius = shadow_ratio_ * shadow_.width;
      setIconSizes();
    }

  private:
    Icon icon_;
    Icon shadow_;

    float shadow_ratio_ = 0.0f;
    float margin_ratio_ = 0.0f;
  };

  class ToggleTextButton : public ToggleButton {
  public:
    explicit ToggleTextButton(const std::string& name);
    explicit ToggleTextButton(const std::string& name, const Font& font);

    virtual void drawBackground(Canvas& canvas, float hover_amount);
    void draw(Canvas& canvas, float hover_amount) override;
    void resized() override;
    void setFont(const Font& font) { text_.setFont(font); }
    void setText(const std::string& text) { text_.setText(text); }
    void setDrawBackground(bool draw_background) { draw_background_ = draw_background; }

  private:
    bool draw_background_ = true;
    Text text_;
  };
}