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

#include "visage_file_embed/embedded_file.h"
#include "visage_graphics/animation.h"
#include "visage_graphics/image.h"
#include "visage_graphics/text.h"
#include "visage_graphics/theme.h"
#include "visage_ui/frame.h"

#include <functional>

namespace visage {
  class Button : public Frame {
  public:
    Button() { hover_amount_.setTargetValue(1.0f); }

    explicit Button(const std::string& name) : Frame(name) { hover_amount_.setTargetValue(1.0f); }

    auto& onToggle() { return on_toggle_; }

    virtual bool toggle() { return false; }
    virtual void setToggled(bool toggled) { }
    virtual void setToggledAndNotify(bool toggled) {
      if (toggled)
        notify(false);
    }
    void notify(bool on) { on_toggle_.callback(this, on); }

    void draw(Canvas& canvas) final;
    virtual void draw(Canvas& canvas, float hover_amount) { }

    void mouseEnter(const MouseEvent& e) override;
    void mouseExit(const MouseEvent& e) override;
    void mouseDown(const MouseEvent& e) override;
    void mouseUp(const MouseEvent& e) override;

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
    CallbackList<void(Button*, bool)> on_toggle_;
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
    UiButton() : UiButton("") { }
    explicit UiButton(const std::string& text, const Font& font);

    virtual void drawBackground(Canvas& canvas, float hover_amount);
    void draw(Canvas& canvas, float hover_amount) override;
    void setFont(const Font& font) {
      text_.setFont(font);
      redraw();
    }
    void setActionButton(bool action = true) {
      action_ = action;
      redraw();
    }
    void setText(const std::string& text) {
      text_.setText(text);
      redraw();
    }
    void drawBorderWhenInactive(bool border) { border_when_inactive_ = border; }

  private:
    Text text_;
    bool action_ = false;
    bool border_when_inactive_ = false;
  };

  class IconButton : public Button {
  public:
    static constexpr float kDefaultShadowProportion = 0.1f;

    IconButton() = default;
    explicit IconButton(const Svg& icon, bool shadow = false) { setIcon(icon, shadow); }

    explicit IconButton(const EmbeddedFile& icon_file, bool shadow = false) {
      setIcon(icon_file, shadow);
    }

    IconButton(const unsigned char* svg, int svg_size, bool shadow = false) {
      setIcon(svg, svg_size, shadow);
    }

    void setIcon(const EmbeddedFile& icon_file, bool shadow = false) {
      setIcon({ icon_file.data, icon_file.size, 0, 0 }, shadow);
    }

    void setIcon(const unsigned char* svg, int svg_size, bool shadow = false) {
      setIcon({ svg, svg_size, 0, 0 }, shadow);
    }

    void setIcon(const Svg& icon, bool shadow = false) {
      icon_ = icon;
      if (shadow) {
        shadow_proportion_ = kDefaultShadowProportion;
        shadow_ = icon_;
      }
    }

    void draw(Canvas& canvas, float hover_amount) override;

    void resized() override {
      Button::resized();
      setIconSizes();
    }

    float margin() const { return std::min(width(), height()) * margin_ratio_; }
    float iconX() const { return margin() + std::max(0.0f, width() - height()) / 2.0f; }
    float iconY() const { return margin() + std::max(0.0f, height() - width()) / 2.0f; }

    void setIconSizes() {
      int margin = std::min(width(), height()) * margin_ratio_;
      icon_.width = std::min(width(), height()) - 2 * margin;
      icon_.height = icon_.width;
      shadow_.width = icon_.width;
      shadow_.height = icon_.height;
      shadow_.blur_radius = shadow_proportion_ * icon_.width;
    }

    void setShadowProportion(float proportion) {
      shadow_proportion_ = proportion;
      shadow_.blur_radius = shadow_proportion_ * shadow_.width;
    }

    void setMarginRatio(float ratio) {
      margin_ratio_ = ratio;
      shadow_.blur_radius = shadow_proportion_ * shadow_.width;
      setIconSizes();
    }

  private:
    Svg icon_;
    Svg shadow_;

    float shadow_proportion_ = 0.0f;
    float margin_ratio_ = 0.0f;
  };

  class ButtonChangeAction;

  class ToggleButton : public Button {
  public:
    VISAGE_THEME_DEFINE_COLOR(ToggleButtonDisabled);
    VISAGE_THEME_DEFINE_COLOR(ToggleButtonOff);
    VISAGE_THEME_DEFINE_COLOR(ToggleButtonOffHover);
    VISAGE_THEME_DEFINE_COLOR(ToggleButtonOn);
    VISAGE_THEME_DEFINE_COLOR(ToggleButtonOnHover);

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
      redraw();
      notify(toggled);
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

    explicit ToggleIconButton(const Svg& icon, bool shadow = false) : ToggleButton(), icon_(icon) {
      initSettings(shadow);
    }

    ToggleIconButton(const std::string& name, const Svg& icon, bool shadow = false) :
        ToggleButton(name), icon_(icon) {
      initSettings(shadow);
    }

    ToggleIconButton(const unsigned char* svg, int svg_size, bool shadow = false) :
        ToggleButton(), icon_({ svg, svg_size, 0, 0 }) {
      initSettings(shadow);
    }

    ToggleIconButton(const std::string& name, const unsigned char* svg, int svg_size,
                     bool shadow = false) : ToggleButton(name), icon_({ svg, svg_size, 0, 0 }) {
      initSettings(shadow);
    }

    void initSettings(bool shadow) {
      if (shadow) {
        shadow_proportion_ = kDefaultShadowRatio;
        shadow_ = icon_;
      }
    }

    void draw(Canvas& canvas, float hover_amount) override;

    void resized() override {
      ToggleButton::resized();
      setIconSizes();
    }

    float margin() const { return std::min(width(), height()) * margin_proportion_; }
    float iconX() const { return margin() + std::max(0.0f, width() - height()) / 2.0f; }
    float iconY() const { return margin() + std::max(0.0f, height() - width()) / 2.0f; }

    void setIconSizes() {
      int margin = std::min(width(), height()) * margin_proportion_;
      icon_.width = std::min(width(), height()) - 2 * margin;
      icon_.height = icon_.width;
      shadow_.width = icon_.width;
      shadow_.height = icon_.height;
      shadow_.blur_radius = shadow_proportion_ * icon_.width;
    }

    void setShadowProportion(float proportion) {
      shadow_proportion_ = proportion;
      shadow_.blur_radius = shadow_proportion_ * shadow_.width;
    }

    void setMarginProportion(float proportion) {
      margin_proportion_ = proportion;
      shadow_.blur_radius = shadow_proportion_ * shadow_.width;
      setIconSizes();
    }

  private:
    Svg icon_;
    Svg shadow_;

    float shadow_proportion_ = 0.0f;
    float margin_proportion_ = 0.0f;
  };

  class ToggleTextButton : public ToggleButton {
  public:
    explicit ToggleTextButton(const std::string& name);
    explicit ToggleTextButton(const std::string& name, const Font& font);

    virtual void drawBackground(Canvas& canvas, float hover_amount);
    void draw(Canvas& canvas, float hover_amount) override;
    void setFont(const Font& font) {
      text_.setFont(font);
      redraw();
    }
    void setText(const std::string& text) { text_.setText(text); }
    void setDrawBackground(bool draw_background) { draw_background_ = draw_background; }

  private:
    bool draw_background_ = true;
    Text text_;
  };
}