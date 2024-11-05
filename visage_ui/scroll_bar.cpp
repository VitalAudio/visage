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
    scroll_bar_.setBounds(x, 0, scroll_bar_width, height());
    container_.setBounds(0, -y_position_, width(), scroll_bar_.viewHeight());
  }
}