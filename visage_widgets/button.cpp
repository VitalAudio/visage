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

#include "button.h"

#include "embedded/fonts.h"
#include "visage_graphics/theme.h"

namespace visage {
  THEME_COLOR(ButtonShadow, 0x88000000);

  THEME_COLOR(TextButtonBackgroundOff, 0xff2c3033);
  THEME_COLOR(TextButtonBackgroundOffHover, 0xff3e4245);
  THEME_COLOR(TextButtonBackgroundOn, 0xff2c3033);
  THEME_COLOR(TextButtonBackgroundOnHover, 0xff3e4245);

  THEME_COLOR(TextButtonTextOff, 0xff848789);
  THEME_COLOR(TextButtonTextOffHover, 0xffaaacad);
  THEME_COLOR(TextButtonTextOn, 0xffaa88ff);
  THEME_COLOR(TextButtonTextOnHover, 0xffbb99ff);

  THEME_IMPLEMENT_COLOR(ToggleButton, ToggleButtonDisabled, 0xff4c4f52);
  THEME_IMPLEMENT_COLOR(ToggleButton, ToggleButtonOff, 0xff848789);
  THEME_IMPLEMENT_COLOR(ToggleButton, ToggleButtonOffHover, 0xffaaacad);
  THEME_IMPLEMENT_COLOR(ToggleButton, ToggleButtonOn, 0xffaa88ff);
  THEME_IMPLEMENT_COLOR(ToggleButton, ToggleButtonOnHover, 0xffbb99ff);

  THEME_IMPLEMENT_VALUE(ToggleButton, ButtonTextHeight, 13.5f, ScaledHeight, true);

  THEME_COLOR(UiButtonBackground, 0xff4c4f52);
  THEME_COLOR(UiButtonBackgroundHover, 0xff606265);
  THEME_COLOR(UiButtonText, 0xffdddddd);
  THEME_COLOR(UiButtonTextHover, 0xffffffff);

  THEME_COLOR(UiActionButtonBackground, 0xff9977ee);
  THEME_COLOR(UiActionButtonBackgroundHover, 0xffaa88ff);
  THEME_COLOR(UiActionButtonText, 0xffdddddd);
  THEME_COLOR(UiActionButtonTextHover, 0xffffffff);

  THEME_VALUE(TextButtonRounding, 9.0f, ScaledHeight, false);
  THEME_VALUE(UiButtonRounding, 9.0f, ScaledHeight, false);
  THEME_VALUE(UiButtonHoverRoundingMult, 0.7f, Constant, false);

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
    if (toggle_on_mouse_down_)
      notify(toggle());

    redraw();
  }

  void Button::mouseUp(const MouseEvent& e) {
    if (!active_)
      return;

    if (localBounds().contains(e.position)) {
      hover_amount_.target(true, true);
      if (!toggle_on_mouse_down_)
        notify(toggle());

      redraw();
    }
  }

  UiButton::UiButton(const std::string& text) : text_(text, Font(10, fonts::Lato_Regular_ttf)) { }

  UiButton::UiButton(const std::string& text, const Font& font) : text_(text, font) { }

  void UiButton::drawBackground(Canvas& canvas, float hover_amount) {
    if (action_)
      canvas.setBlendedPaletteColor(kUiActionButtonBackground, kUiActionButtonBackgroundHover, hover_amount);
    else
      canvas.setBlendedPaletteColor(kUiButtonBackground, kUiButtonBackgroundHover, hover_amount);

    int w = width();
    int h = height();
    float rounding = canvas.value(kUiButtonRounding);

    if (isActive() || !border_when_inactive_) {
      float mult = hover_amount * canvas.value(kUiButtonHoverRoundingMult) + (1.0f - hover_amount);
      canvas.roundedRectangle(0, 0, w, h, rounding * mult);
    }
    else
      canvas.roundedRectangleBorder(0, 0, w, h, rounding, 2.0f);
  }

  void UiButton::draw(Canvas& canvas, float hover_amount) {
    drawBackground(canvas, hover_amount);

    if (border_when_inactive_ && !isActive()) {
      if (action_)
        canvas.setPaletteColor(kUiActionButtonBackground);
      else
        canvas.setPaletteColor(kUiButtonBackground);
    }
    else if (action_)
      canvas.setBlendedPaletteColor(kUiActionButtonText, kUiActionButtonTextHover, hover_amount);
    else
      canvas.setBlendedPaletteColor(kUiButtonText, kUiButtonTextHover, hover_amount);
    canvas.text(&text_, 0, 0, width(), height());
  }

  void IconButton::draw(Canvas& canvas, float hover_amount) {
    int x = getIconX();
    int y = getIconY();

    if (shadow_.blur_radius) {
      canvas.setPaletteColor(kButtonShadow);
      canvas.svg(shadow_, x, y);
    }

    if (isActive())
      canvas.setBlendedPaletteColor(ToggleButton::kToggleButtonOff,
                                    ToggleButton::kToggleButtonOffHover, hover_amount);
    else
      canvas.setPaletteColor(ToggleButton::kToggleButtonDisabled);

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
    int x = getIconX();
    int y = getIconY();
    if (shadow_.blur_radius) {
      canvas.setPaletteColor(kButtonShadow);
      canvas.svg(shadow_, x, y);
    }

    if (toggled())
      canvas.setBlendedPaletteColor(kToggleButtonOn, kToggleButtonOnHover, hover_amount);
    else
      canvas.setBlendedPaletteColor(kToggleButtonOff, kToggleButtonOffHover, hover_amount);
    canvas.svg(icon_, x, y);
  }

  ToggleTextButton::ToggleTextButton(const std::string& name) :
      ToggleButton(name), text_(name, Font(10, fonts::Lato_Regular_ttf)) { }

  ToggleTextButton::ToggleTextButton(const std::string& name, const Font& font) :
      ToggleButton(name), text_(name, font) { }

  void ToggleTextButton::drawBackground(Canvas& canvas, float hover_amount) {
    if (toggled())
      canvas.setBlendedPaletteColor(kTextButtonBackgroundOn, kTextButtonBackgroundOnHover, hover_amount);
    else
      canvas.setBlendedPaletteColor(kTextButtonBackgroundOff, kTextButtonBackgroundOffHover, hover_amount);

    float rounding = canvas.value(kTextButtonRounding);
    canvas.roundedRectangle(0, 0, width(), height(), rounding);
  }

  void ToggleTextButton::draw(Canvas& canvas, float hover_amount) {
    if (draw_background_)
      drawBackground(canvas, hover_amount);

    if (toggled())
      canvas.setBlendedPaletteColor(kTextButtonTextOn, kTextButtonTextOnHover, hover_amount);
    else
      canvas.setBlendedPaletteColor(kTextButtonTextOff, kTextButtonTextOffHover, hover_amount);
    canvas.text(&text_, 0, 0, width(), height());
  }

  void ToggleTextButton::resized() {
    ToggleButton::resized();
    float height = paletteValue(kButtonTextHeight);
    setFont(Font(height, fonts::Lato_Regular_ttf));
  }
}