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

#include "bar_component.h"

#include "visage_graphics/theme.h"

namespace visage {
  THEME_IMPLEMENT_COLOR(BarComponent, BarColor, 0xffaa88ff);

  BarComponent::BarComponent(int num_bars) : num_bars_(num_bars) {
    bars_ = std::make_unique<Bar[]>(num_bars);
  }

  void BarComponent::draw(Canvas& canvas) {
    QuadColor bar_color = canvas.color(kBarColor);

    float width_scale = 1.0f / width();
    float height_scale = 1.0f / height();
    for (int i = 0; i < num_bars_; ++i) {
      const Bar& bar = bars_[i];
      float left = bar.left * width_scale;
      float right = bar.right * width_scale;
      float top = bar.top * height_scale;
      float bottom = bar.bottom * height_scale;
      canvas.setColor(QuadColor(bar_color.sampleColor(left, top), bar_color.sampleColor(right, top),
                                bar_color.sampleColor(left, bottom), bar_color.sampleColor(right, bottom),
                                bar_color.sampleHdr(left, top), bar_color.sampleHdr(right, top),
                                bar_color.sampleHdr(left, bottom), bar_color.sampleHdr(right, bottom)));

      canvas.rectangle(bar.left, bar.top, bar.right - bar.left, bar.bottom - bar.top);
    }
  }
}