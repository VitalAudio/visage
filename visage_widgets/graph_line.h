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

#include "visage_graphics/path.h"
#include "visage_graphics/theme.h"
#include "visage_ui/frame.h"

namespace visage {
  class GraphLine : public Frame {
  public:
    static constexpr int kLineVerticesPerPoint = 6;
    static constexpr int kFillVerticesPerPoint = 2;

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
    VISAGE_THEME_DEFINE_VALUE(LineSizeBoost);
    VISAGE_THEME_DEFINE_VALUE(LineColorBoost);
    VISAGE_THEME_DEFINE_VALUE(LineFillBoost);

    enum FillCenter {
      kCenter,
      kBottom,
      kTop,
      kCustom
    };

    explicit GraphLine(int num_points, bool loop = false);
    ~GraphLine() override;

    void draw(Canvas& canvas) override;

    // float boostAt(int index) const { return line_.values[index]; }
    // void setBoostAt(int index, float val) {
    //   VISAGE_ASSERT(index < line_.num_points && index >= 0);
    //   line_.values[index] = val;
    //   redraw();
    // }
    //
    // float yAt(int index) const { return line_.points[index].y; }
    // void setYAt(int index, float val) {
    //   VISAGE_ASSERT(index < line_.num_points && index >= 0);
    //   line_.points[index].y = val;
    //   redraw();
    // }
    //
    // float xAt(int index) const { return line_.points[index].x; }
    // void setXAt(int index, float val) {
    //   VISAGE_ASSERT(index < line_.num_points && index >= 0);
    //   line_.points[index].x = val;
    //   redraw();
    // }
    //
    // void setAt(int index, Point point) {
    //   VISAGE_ASSERT(index < line_.num_points && index >= 0);
    //   line_.points[index] = point;
    //   redraw();
    // }

    bool fill() const { return fill_; }

    void setFill(bool fill) { fill_ = fill; }
    void setFillCenter(FillCenter fill_center) { fill_center_ = fill_center; }
    void setFillCenter(float center) {
      custom_fill_center_ = center;
      fill_center_ = kCustom;
      redraw();
    }
    int fillLocation() const;

    int numPoints() const { return line_.numPoints(); }

    bool active() const { return active_; }
    void setActive(bool active) { active_ = active; }
    void setFillAlphaMult(float mult) { fill_alpha_mult_ = mult; }

  private:
    void drawLine(Canvas& canvas, theme::ColorId color_id);
    void drawFill(Canvas& canvas, theme::ColorId color_id);

    Path line_;
    Dimension line_width_;

    bool fill_ = false;
    FillCenter fill_center_ = kCenter;
    float custom_fill_center_ = 0.0f;
    float fill_alpha_mult_ = 1.0f;

    bool active_ = true;
    bool loop_ = false;

    VISAGE_LEAK_CHECKER(GraphLine)
  };
}