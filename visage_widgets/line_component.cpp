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

#include "line_component.h"

namespace visage {
  THEME_IMPLEMENT_COLOR(LineComponent, LineColor, 0xffaa88ff);
  THEME_IMPLEMENT_COLOR(LineComponent, LineFillColor, 0x669f88ff);
  THEME_IMPLEMENT_COLOR(LineComponent, LineFillColor2, 0x669f88ff);
  THEME_IMPLEMENT_COLOR(LineComponent, LineDisabledColor, 0xff4c4f52);
  THEME_IMPLEMENT_COLOR(LineComponent, LineDisabledFillColor, 0x22666666);
  THEME_IMPLEMENT_COLOR(LineComponent, CenterPoint, 0xff1d2125);
  THEME_IMPLEMENT_COLOR(LineComponent, GridColor, 0x22ffffff);
  THEME_IMPLEMENT_COLOR(LineComponent, HoverColor, 0xffffffff);
  THEME_IMPLEMENT_COLOR(LineComponent, DragColor, 0x55ffffff);

  THEME_IMPLEMENT_VALUE(LineComponent, LineWidth, 1.5f, ScaledHeight, false);
  THEME_IMPLEMENT_VALUE(LineComponent, LineColorBoost, 1.0f, Constant, false);
  THEME_IMPLEMENT_VALUE(LineComponent, LineFillBoost, 1.0f, Constant, false);
  THEME_VALUE(PositionBulbWidth, 4.0f, ScaledHeight, true);

  void LineComponent::BoostBuffer::boostRange(float start, float end, float decay) {
    any_boost_value_ = true;

    int active_points = num_points_;
    int start_index = std::max(static_cast<int>(std::ceil(start * (active_points - 1))), 0);
    float end_position = end * (active_points - 1);
    int end_index = std::max(std::ceil(end_position), 0.0f);
    float progress = end_position - static_cast<int>(end_position);

    start_index %= active_points;
    end_index %= active_points;
    int num_points = end_index - start_index;
    int direction = 1;
    if (enable_backward_boost_) {
      if ((num_points < 0 && num_points > -num_points_ / 2) || (num_points == 0 && last_negative_boost_)) {
        num_points = -num_points;
        direction = -1;
      }
      else if (num_points > num_points_ / 2) {
        num_points -= active_points;
        num_points = -num_points;
        direction = -1;
      }
    }

    last_negative_boost_ = direction < 0;
    if (last_negative_boost_) {
      start_index = std::max(static_cast<int>(std::floor(start * (active_points - 1))), 0);
      end_index = std::max(std::floor(end_position), 0.0f);
      start_index %= active_points;
      end_index %= active_points;

      num_points = start_index - end_index;
      progress = 1.0f - progress;
    }

    float delta = (1.0f - decay) / num_points;
    float val = decay;

    for (int i = start_index; i != end_index; i = (i + active_points + direction) % active_points) {
      val += delta;
      val = std::min(1.0f, val);
      float last_value = values_[i];
      values_[i] = std::max(last_value, val);
    }

    float end_value = values_[end_index];
    values_[end_index] = std::max(end_value, progress * progress);
  }

  void LineComponent::BoostBuffer::decayBoosts(float mult) {
    bool any_boost = false;
    for (int i = 0; i < num_points_; ++i) {
      values_[i] *= mult;
      any_boost = any_boost || values_[i];
    }

    any_boost_value_ = any_boost;
  }

  LineComponent::LineComponent(int num_points, bool loop) :
      line_(num_points), boost_(line_.values.get(), num_points), fill_center_(kCenter), loop_(loop) {
    boost_.enableBackwardBoost(loop_);

    for (int i = 0; i < line_.num_points; ++i)
      setXAt(i, i / (line_.num_points - 1.0f));
  }

  LineComponent::~LineComponent() = default;

  void LineComponent::init() {
    line_.init();
    UiFrame::init();
  }

  int LineComponent::fillLocation() const {
    if (fill_center_ == kBottom)
      return height();
    if (fill_center_ == kTop)
      return 0;
    if (fill_center_ == kCustom)
      return custom_fill_center_;
    return height() / 2;
  }

  void LineComponent::draw(Canvas& canvas) {
    if (canvas.totallyClamped())
      return;

    if (fill_)
      drawFill(canvas, active_ ? kLineFillColor : kLineDisabledFillColor);
    drawLine(canvas, active_ ? kLineColor : kLineDisabledColor);
  }

  void LineComponent::drawLine(Canvas& canvas, unsigned int color_id) {
    line_.line_value_scale = canvas.value(kLineColorBoost);
    canvas.setPaletteColor(color_id);
    canvas.line(&line_, 0.0f, 0.0f, width(), height(), line_width_);
  }

  void LineComponent::drawFill(Canvas& canvas, unsigned int color_id) {
    QuadColor color = canvas.color(color_id);
    line_.fill_value_scale = canvas.value(kLineFillBoost);
    canvas.setColor(color.withMultipliedAlpha(fill_alpha_mult_));
    canvas.lineFill(&line_, 0.0f, 0.0f, width(), height(), fillLocation());
  }

  void LineComponent::drawPosition(Canvas& canvas, float x, float y) {
    float marker_width = canvas.value(kPositionBulbWidth);
    canvas.setColor(canvas.color(kLineColor).withMultipliedHdr(1.0f + canvas.value(kLineColorBoost)));
    canvas.circle(x - marker_width * 0.5f, y - marker_width * 0.5f, marker_width);
  }

  void LineComponent::resized() {
    line_width_ = paletteValue(kLineWidth);
    UiFrame::resized();
  }

  void LineComponent::destroy() {
    line_.destroy();
    UiFrame::destroy();
  }
}