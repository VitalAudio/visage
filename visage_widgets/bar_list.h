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

#include "visage_ui/frame.h"

namespace visage {
  class BarList : public Frame {
  public:
    THEME_DEFINE_COLOR(BarColor);

    struct Bar {
      float left = 0.0f;
      float top = 0.0f;
      float right = 0.0f;
      float bottom = 0.0f;
    };

    explicit BarList(int num_bars);
    ~BarList() override = default;

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

    VISAGE_LEAK_CHECKER(BarList)
  };
}