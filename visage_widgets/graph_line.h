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
  class GraphLine : public Frame {
  public:
    VISAGE_THEME_DEFINE_COLOR(LineColor);
    VISAGE_THEME_DEFINE_COLOR(LineFillColor);
    VISAGE_THEME_DEFINE_COLOR(LineFillColor2);
    VISAGE_THEME_DEFINE_COLOR(LineDisabledColor);
    VISAGE_THEME_DEFINE_COLOR(LineDisabledFillColor);
    VISAGE_THEME_DEFINE_COLOR(CenterPoint);
    VISAGE_THEME_DEFINE_COLOR(GridColor);
    VISAGE_THEME_DEFINE_COLOR(HoverColor);
    VISAGE_THEME_DEFINE_COLOR(DragColor);

    VISAGE_THEME_DEFINE_VALUE(LineWidth);

    enum FillCenter {
      kCenter,
      kBottom,
      kTop,
      kCustom
    };

    explicit GraphLine(int num_points, bool loop = false);
    ~GraphLine() override;

    void draw(Canvas& canvas) override;

    float at(int index) const { return data_[index]; }
    void set(int index, float val) {
      VISAGE_ASSERT(index < data_.numPoints() && index >= 0);
      data_[index] = val;
      redraw();
    }

    bool isFilled() const { return filled_; }
    void setFilled(bool fill) { filled_ = fill; }
    void setFillCenter(FillCenter fill_center) { fill_center_ = fill_center; }
    void setFillCenter(float center) {
      custom_fill_center_ = center;
      fill_center_ = kCustom;
      redraw();
    }
    float fillLocation() const;

    int numPoints() const { return data_.numPoints(); }

    bool active() const { return active_; }
    void setActive(bool active) { active_ = active; }
    void setFillAlphaMult(float mult) { fill_alpha_mult_ = mult; }

  private:
    void drawLine(Canvas& canvas, theme::ColorId color_id);
    void drawFill(Canvas& canvas, theme::ColorId color_id);

    GraphData data_;
    Dimension line_width_;

    bool filled_ = false;
    FillCenter fill_center_ = kCenter;
    float custom_fill_center_ = 0.0f;
    float fill_alpha_mult_ = 1.0f;

    bool active_ = true;
    bool loop_ = false;

    VISAGE_LEAK_CHECKER(GraphLine)
  };
}