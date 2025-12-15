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

#include "visage_graphics/theme.h"
#include "visage_ui/frame.h"

namespace visage {
  class HeatMap : public Frame {
  public:
    VISAGE_THEME_DEFINE_COLOR(HeatMapGradient);

    explicit HeatMap(int width, int height);
    ~HeatMap() override;

    void draw(Canvas& canvas) override;

    float at(int x, int y) const { return data_.at(x, y); }
    void set(int x, int y, float val) {
      data_.set(x, y, val);
      redraw();
    }

    int dataWidth() const { return data_.width(); }
    int dataHeight() const { return data_.height(); }

  private:
    void drawLine(Canvas& canvas, theme::ColorId color_id);
    void drawFill(Canvas& canvas, theme::ColorId color_id);

    HeatMapData data_;

    VISAGE_LEAK_CHECKER(HeatMap)
  };
}