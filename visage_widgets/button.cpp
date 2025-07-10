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

#include "button.h"

#include "embedded/fonts.h"
#include "visage_graphics/theme.h"

namespace visage {
  VISAGE_THEME_COLOR(ButtonShadow, 0x88000000);

  VISAGE_THEME_COLOR(TextButtonBackgroundOff, 0xff2c3033);
  VISAGE_THEME_COLOR(TextButtonBackgroundOffHover, 0xff3e4245);
  VISAGE_THEME_COLOR(TextButtonBackgroundOn, 0xff2c3033);
  VISAGE_THEME_COLOR(TextButtonBackgroundOnHover, 0xff3e4245);

  VISAGE_THEME_COLOR(TextButtonTextOff, 0xff848789);
  VISAGE_THEME_COLOR(TextButtonTextOffHover, 0xffaaacad);
  VISAGE_THEME_COLOR(TextButtonTextOn, 0xffaa88ff);
  VISAGE_THEME_COLOR(TextButtonTextOnHover, 0xffbb99ff);

  VISAGE_THEME_IMPLEMENT_COLOR(ToggleButton, ToggleButtonDisabled, 0xff4c4f52);
  VISAGE_THEME_IMPLEMENT_COLOR(ToggleButton, ToggleButtonOff, 0xff848789);
  VISAGE_THEME_IMPLEMENT_COLOR(ToggleButton, ToggleButtonOffHover, 0xffaaacad);
  VISAGE_THEME_IMPLEMENT_COLOR(ToggleButton, ToggleButtonOn, 0xffaa88ff);
  VISAGE_THEME_IMPLEMENT_COLOR(ToggleButton, ToggleButtonOnHover, 0xffbb99ff);

  VISAGE_THEME_COLOR(UiButtonBackground, 0xff4c4f52);
  VISAGE_THEME_COLOR(UiButtonBackgroundHover, 0xff606265);
  VISAGE_THEME_COLOR(UiButtonText, 0xffdddddd);
  VISAGE_THEME_COLOR(UiButtonTextHover, 0xffffffff);

  VISAGE_THEME_COLOR(UiActionButtonBackground, 0xff9977ee);
  VISAGE_THEME_COLOR(UiActionButtonBackgroundHover, 0xffaa88ff);
  VISAGE_THEME_COLOR(UiActionButtonText, 0xffdddddd);
  VISAGE_THEME_COLOR(UiActionButtonTextHover, 0xffffffff);

  VISAGE_THEME_VALUE(TextButtonRounding, 9.0f);
  VISAGE_THEME_VALUE(UiButtonRounding, 9.0f);
  VISAGE_THEME_VALUE(UiButtonHoverRoundingMult, 0.7f);

  void Button::draw(Canvas& canvas) {
    draw(canvas, active_ ? hover_amount_.update() : 0.0f);

    if (hover_amount_.isAnimating())
      redraw();
  }

  void Button::mouseEnter(const MouseEvent& e) {
    hover_amount_.target(true);
    if (set_pointer_cursor_ && active_)
      setCursorStyle(MouseCursor::Pointing);

    redraw();
  }

  void Button::mouseExit(const MouseEvent& e) {
    hover_amount_.target(false);
    if (set_pointer_cursor_)
      setCursorStyle(MouseCursor::Arrow);

    redraw();
  }

  void Button::mouseDown(const MouseEvent& e) {
    if (!active_)
      return;

    alt_clicked_ = e.isAltDown();
    hover_amount_.target(false);
    redraw();

    if (toggle_on_mouse_down_)
      notify(toggle());
  }

  void Button::mouseUp(const MouseEvent& e) {
    if (!active_)
      return;

    redraw();
    if (localBounds().contains(e.position)) {
      hover_amount_.target(true, true);
      if (!toggle_on_mouse_down_)
        notify(toggle());
    }
  }

  UiButton::UiButton(const std::string& text) : text_(text, Font(10, fonts::Lato_Regular_ttf)) { }

  UiButton::UiButton(const std::string& text, const Font& font) : text_(text, font) { }

  void UiButton::drawBackground(Canvas& canvas, float hover_amount) {
    if (action_)
      canvas.setBlendedColor(UiActionButtonBackground, UiActionButtonBackgroundHover, hover_amount);
    else
      canvas.setBlendedColor(UiButtonBackground, UiButtonBackgroundHover, hover_amount);

    int w = width();
    int h = height();
    float rounding = canvas.value(UiButtonRounding);

    if (isActive() || !border_when_inactive_) {
      float mult = hover_amount * canvas.value(UiButtonHoverRoundingMult) + (1.0f - hover_amount);
      canvas.roundedRectangle(0, 0, w, h, rounding * mult);
    }
    else
      canvas.roundedRectangleBorder(0, 0, w, h, rounding, 2.0f);
  }

  void UiButton::draw(Canvas& canvas, float hover_amount) {
    drawBackground(canvas, hover_amount);

    if (border_when_inactive_ && !isActive()) {
      if (action_)
        canvas.setColor(UiActionButtonBackground);
      else
        canvas.setColor(UiButtonBackground);
    }
    else if (action_)
      canvas.setBlendedColor(UiActionButtonText, UiActionButtonTextHover, hover_amount);
    else
      canvas.setBlendedColor(UiButtonText, UiButtonTextHover, hover_amount);
    canvas.text(&text_, 0, 0, width(), height());
  }

  void IconButton::draw(Canvas& canvas, float hover_amount) {
    int x = iconX();
    int y = iconY();

    if (shadow_.blur_radius) {
      canvas.setColor(ButtonShadow);
      canvas.svg(shadow_, x, y);
    }

    if (isActive())
      canvas.setBlendedColor(ToggleButton::ToggleButtonOff, ToggleButton::ToggleButtonOffHover, hover_amount);
    else
      canvas.setColor(ToggleButton::ToggleButtonDisabled);

    canvas.svg(icon_, x, y);
  }

  bool ToggleButton::toggle() {
    toggled_ = !toggled_;

    if (undoable_) {
      auto change_action = std::make_unique<ButtonChangeAction>(this, toggled_);
      if (undoSetupFunction())
        change_action->setSetupFunction(undoSetupFunction());
      addUndoableAction(std::move(change_action));
    }
    toggleValueChanged();
    return toggled_;
  }

  void ToggleIconButton::draw(Canvas& canvas, float hover_amount) {
    int x = iconX();
    int y = iconY();
    if (shadow_.blur_radius) {
      canvas.setColor(ButtonShadow);
      canvas.svg(shadow_, x, y);
    }

    if (toggled())
      canvas.setBlendedColor(ToggleButtonOn, ToggleButtonOnHover, hover_amount);
    else
      canvas.setBlendedColor(ToggleButtonOff, ToggleButtonOffHover, hover_amount);
    canvas.svg(icon_, x, y);
  }

  ToggleTextButton::ToggleTextButton(const std::string& name) :
      ToggleButton(name), text_(name, Font(10, fonts::Lato_Regular_ttf)) { }

  ToggleTextButton::ToggleTextButton(const std::string& name, const Font& font) :
      ToggleButton(name), text_(name, font) { }

  void ToggleTextButton::drawBackground(Canvas& canvas, float hover_amount) {
    if (toggled())
      canvas.setBlendedColor(TextButtonBackgroundOn, TextButtonBackgroundOnHover, hover_amount);
    else
      canvas.setBlendedColor(TextButtonBackgroundOff, TextButtonBackgroundOffHover, hover_amount);

    float rounding = canvas.value(TextButtonRounding);
    canvas.roundedRectangle(0, 0, width(), height(), rounding);
  }

  void ToggleTextButton::draw(Canvas& canvas, float hover_amount) {
    if (draw_background_)
      drawBackground(canvas, hover_amount);

    if (toggled())
      canvas.setBlendedColor(TextButtonTextOn, TextButtonTextOnHover, hover_amount);
    else
      canvas.setBlendedColor(TextButtonTextOff, TextButtonTextOffHover, hover_amount);
    canvas.text(&text_, 0, 0, width(), height());
  }
}