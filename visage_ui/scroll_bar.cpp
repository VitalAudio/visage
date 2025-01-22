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

#include "scroll_bar.h"

#include "visage_graphics/theme.h"

namespace visage {
  THEME_COLOR(ScrollBarDefault, 0x22ffffff);
  THEME_COLOR(ScrollBarDown, 0x55ffffff);

  THEME_VALUE(ScrollBarWidth, 20.0f, ScaledDpi, true);

  void ScrollBar::draw(Canvas& canvas) {
    if (!active_ || range_ <= 0)
      return;

    float y_ratio = position_ / range_;
    float height_ratio = view_height_ * 1.0f / range_;
    int h = height();

    canvas.setBlendedPaletteColor(kScrollBarDefault, kScrollBarDown, color_.update());
    float w = width_.update();

    float rounding = std::min(width_.setSourceValue() / 2.0f, rounding_);
    float x = left_ ? 0.0f : width() - w;
    canvas.roundedRectangle(x, y_ratio * h, w, height_ratio * h, rounding);

    if (width_.isAnimating() || color_.isAnimating())
      redraw();
  }

  void ScrollBar::mouseEnter(const MouseEvent& e) {
    width_.target(true);
    redraw();
  }

  void ScrollBar::mouseExit(const MouseEvent& e) {
    width_.target(false);
    redraw();
  }

  void ScrollBar::mouseDown(const MouseEvent& e) {
    redraw();
    color_.target(true);

    int max_value = range_ - view_height_;
    if (!active_ || max_value <= 0 || range_ <= 0)
      return;

    last_drag_ = e.position.y;
  }

  void ScrollBar::mouseUp(const MouseEvent& e) {
    color_.target(false);
    redraw();
  }

  void ScrollBar::mouseDrag(const MouseEvent& e) {
    float delta = range_ * (e.position.y - last_drag_) / height();
    last_drag_ = e.position.y;

    position_ += delta;
    position_ = std::min<float>(range_ - view_height_, std::max(0.0f, position_));

    for (auto& callback : callbacks_)
      callback(std::round(position_));
    redraw();
  }

  void ScrollableFrame::resized() {
    int scroll_bar_width = paletteValue(kScrollBarWidth);
    int x = scroll_bar_left_ ? 0 : width() - scroll_bar_width;
    float_position_ = y_position_;
    scroll_bar_.setBounds(x, 0, scroll_bar_width, height());
    container_.setBounds(0, -y_position_, width(),
                         std::max(scroll_bar_.viewRange(), scroll_bar_.viewHeight()));
  }
}