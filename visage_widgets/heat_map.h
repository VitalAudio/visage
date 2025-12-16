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
    HeatMap();
    HeatMap(int width, int height);
    ~HeatMap() override;

    void setDimensions(int width, int height) {
      data_.setDimensions(width, height);
      redraw();
    }

    void setOctaves(float octaves) {
      data_.setOctaves(octaves);
      redraw();
    }

    void draw(Canvas& canvas) override;

    void setGradient(visage::Gradient gradient) {
      gradient_ = std::move(gradient);
      redraw();
    }

    float at(int x, int y) const { return data_.at(x, y); }
    void set(int x, int y, float val) {
      data_.set(x, y, val);
      redraw();
    }

    int dataWidth() const { return data_.width(); }
    int dataHeight() const { return data_.height(); }

  private:
    HeatMapData data_;
    visage::Gradient gradient_ = visage::Gradient::kMagma;

    VISAGE_LEAK_CHECKER(HeatMap)
  };
}