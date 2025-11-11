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
#include "visage_graphics/svg.h"
#include "visage_graphics/text.h"
#include "visage_graphics/theme.h"
#include "visage_ui/frame.h"
#include "visage_ui/svg_frame.h"

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
    VISAGE_THEME_DEFINE_COLOR(UiButtonBackground);
    VISAGE_THEME_DEFINE_COLOR(UiButtonBackgroundHover);
    VISAGE_THEME_DEFINE_COLOR(UiButtonText);
    VISAGE_THEME_DEFINE_COLOR(UiButtonTextHover);

    VISAGE_THEME_DEFINE_COLOR(UiActionButtonBackground);
    VISAGE_THEME_DEFINE_COLOR(UiActionButtonBackgroundHover);
    VISAGE_THEME_DEFINE_COLOR(UiActionButtonText);
    VISAGE_THEME_DEFINE_COLOR(UiActionButtonTextHover);

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
    static constexpr float kDefaultShadowRadius = 3.0f;

    explicit IconButton(bool shadow = false) { initSettings(shadow); }

    explicit IconButton(const Svg& icon, bool shadow = false) {
      setIcon(icon);
      initSettings(shadow);
    }

    explicit IconButton(const EmbeddedFile& icon_file, bool shadow = false) {
      setIcon(icon_file);
      initSettings(shadow);
    }

    IconButton(const unsigned char* svg, int svg_size, bool shadow = false) {
      setIcon(svg, svg_size);
      initSettings(shadow);
    }

    void setIcon(const EmbeddedFile& icon_file) { setIcon({ icon_file.data, icon_file.size }); }
    void setIcon(const unsigned char* svg, int svg_size) { setIcon({ svg, svg_size }); }
    void setIcon(const Svg& icon) {
      icon_.load(icon);
      shadow_.load(icon);
    }

    void draw(Canvas& canvas, float hover_amount) override;

    void resized() override {
      Button::resized();
      icon_.setBounds(localBounds());
      shadow_.setBounds(localBounds());
    }

    void setShadowRadius(const Dimension& radius) {
      shadow_radius_ = radius;
      computeShadowRadius();
    }

    void setMargin(const Dimension& margin) {
      icon_.setMargin(margin);
      shadow_.setMargin(margin);
    }

  private:
    void initSettings(bool shadow) {
      addChild(shadow_, shadow);
      shadow_.setIgnoresMouseEvents(true, false);

      addChild(icon_);
      icon_.setIgnoresMouseEvents(true, false);
      if (shadow)
        setShadowRadius(kDefaultShadowRadius);
    }

    void computeShadowRadius() {
      float r = shadow_radius_.compute(dpiScale(), nativeWidth(), nativeHeight(), 0.0f) / dpiScale();
      shadow_.setVisible(r > 0.0f);
      shadow_.setBlurRadius(r);
    }

    SvgFrame icon_;
    SvgFrame shadow_;

    Dimension shadow_radius_;
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
    static constexpr float kDefaultShadowRadius = 3.0f;

    explicit ToggleIconButton(const Svg& icon, bool shadow = false) : ToggleButton() {
      setIcon(icon);
      initSettings(shadow);
    }

    ToggleIconButton(const std::string& name, const Svg& icon, bool shadow = false) :
        ToggleButton(name) {
      setIcon(icon);
      initSettings(shadow);
    }

    ToggleIconButton(const EmbeddedFile& icon_file, bool shadow = false) : ToggleButton() {
      setIcon(icon_file);
      initSettings(shadow);
    }

    ToggleIconButton(const unsigned char* svg, int svg_size, bool shadow = false) : ToggleButton() {
      setIcon({ svg, svg_size });
      initSettings(shadow);
    }

    ToggleIconButton(const std::string& name, const unsigned char* svg, int svg_size,
                     bool shadow = false) : ToggleButton(name) {
      setIcon({ svg, svg_size });
      initSettings(shadow);
    }

    void setIcon(const EmbeddedFile& icon_file) { setIcon({ icon_file.data, icon_file.size }); }

    void setIcon(const Svg& icon) {
      shadow_.load(icon);
      icon_.load(icon);
    }

    void draw(Canvas& canvas, float hover_amount) override;
    void resized() override;

    void setShadowRadius(const Dimension& radius) {
      shadow_radius_ = radius;
      computeShadowRadius();
    }

    void setMargin(const Dimension& margin) {
      icon_.setMargin(margin);
      shadow_.setMargin(margin);
    }

  private:
    void initSettings(bool shadow) {
      addChild(shadow_, shadow);
      shadow_.setIgnoresMouseEvents(true, false);

      addChild(icon_);
      icon_.setIgnoresMouseEvents(true, false);
      if (shadow)
        setShadowRadius(kDefaultShadowRadius);
    }

    void computeShadowRadius() {
      float r = shadow_radius_.compute(dpiScale(), nativeWidth(), nativeHeight(), 0.0f) / dpiScale();
      shadow_.setVisible(r > 0.0f);
      shadow_.setBlurRadius(r);
    }

    SvgFrame icon_;
    SvgFrame shadow_;

    Dimension shadow_radius_;
    Dimension margin_;
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
