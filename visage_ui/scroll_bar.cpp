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

  THEME_VALUE(ScrollBarWidth, 13.0f, ScaledHeight, true);

  void ScrollBar::draw(Canvas& canvas) {
    if (!active_ || range_ <= 0)
      return;

    float y_ratio = position_ / range_;
    float height_ratio = view_height_ * 1.0f / range_;
    int height = getHeight();

    canvas.setBlendedPaletteColor(kScrollBarDefault, kScrollBarDown, color_.update());
    float width = width_.update();

    float rounding = std::min(width_.setSourceValue() / 2.0f, rounding_);
    float x = left_ ? 0.0f : getWidth() - width;
    canvas.roundedRectangle(x, y_ratio * height, width, height_ratio * height, rounding);

    if (width_.isAnimating() || color_.isAnimating())
      redraw();
  }

  void ScrollBar::onMouseEnter(const MouseEvent& e) {
    width_.target(true);
    redraw();
  }

  void ScrollBar::onMouseExit(const MouseEvent& e) {
    width_.target(false);
    redraw();
  }

  void ScrollBar::onMouseDown(const MouseEvent& e) {
    redraw();
    color_.target(true);

    int max_value = range_ - view_height_;
    if (!active_ || max_value <= 0 || range_ <= 0)
      return;

    last_drag_ = e.getPosition().y;
  }

  void ScrollBar::onMouseUp(const MouseEvent& e) {
    color_.target(false);
    redraw();
  }

  void ScrollBar::onMouseDrag(const MouseEvent& e) {
    float height = getHeight();
    float delta = range_ * (e.getPosition().y - last_drag_) / height;
    last_drag_ = e.getPosition().y;

    position_ += delta;
    position_ = std::min<float>(range_ - view_height_, std::max(0.0f, position_));

    for (auto& callback : callbacks_)
      callback(std::round(position_));
    redraw();
  }

  void ScrollableComponent::resized() {
    int old_height = std::max(1, scroll_bar_.getHeight());
    int scroll_bar_width = getValue(kScrollBarWidth);
    int x = scroll_bar_left_ ? 0 : getWidth() - scroll_bar_width;
    scroll_bar_.setBounds(x, 0, scroll_bar_width, getHeight());
    container_.setBounds(0, -y_position_, getWidth(), scroll_bar_.viewHeight());
    setYPosition(getHeight() * float_position_ / old_height);
  }
}