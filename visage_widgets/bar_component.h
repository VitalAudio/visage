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

#pragma once

#include "visage_ui/drawable_component.h"

namespace visage {
  class BarComponent : public DrawableComponent {
  public:
    THEME_DEFINE_COLOR(BarColor);

    struct Bar {
      float left = 0.0f;
      float top = 0.0f;
      float right = 0.0f;
      float bottom = 0.0f;
    };

    explicit BarComponent(int num_bars);
    ~BarComponent() override = default;

    void draw(Canvas& canvas) override;

    void setHorizontalAntiAliasing(bool anti_alias) { horizontal_aa_ = anti_alias; }
    void setVerticalAntiAliasing(bool anti_alias) { vertical_aa_ = anti_alias; }

    void setY(int index, float y) {
      if (!vertical_aa_)
        y = std::round(y);
      bars_[index].top = y;

      redraw();
    }

    void positionBar(int index, float x, float y, float width, float height) {
      float right = x + width;
      float bottom = y + height;
      if (!horizontal_aa_) {
        right = std::round(right);
        x = std::round(x);
      }
      if (!vertical_aa_) {
        bottom = std::round(bottom);
        y = std::round(y);
      }
      bars_[index] = { x, y, right, bottom };

      redraw();
    }

    int numBars() const { return num_bars_; }

  private:
    std::unique_ptr<Bar[]> bars_;
    int num_bars_ = 0;
    bool horizontal_aa_ = true;
    bool vertical_aa_ = true;

    VISAGE_LEAK_CHECKER(BarComponent)
  };
}