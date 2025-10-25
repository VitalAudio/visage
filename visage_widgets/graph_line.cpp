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

#include "graph_line.h"

namespace visage {
  VISAGE_THEME_IMPLEMENT_COLOR(GraphLine, LineColor, 0xffaa88ff);
  VISAGE_THEME_IMPLEMENT_COLOR(GraphLine, LineFillColor, 0x669f88ff);
  VISAGE_THEME_IMPLEMENT_COLOR(GraphLine, LineFillColor2, 0x669f88ff);
  VISAGE_THEME_IMPLEMENT_COLOR(GraphLine, LineDisabledColor, 0xff4c4f52);
  VISAGE_THEME_IMPLEMENT_COLOR(GraphLine, LineDisabledFillColor, 0x22666666);
  VISAGE_THEME_IMPLEMENT_COLOR(GraphLine, CenterPoint, 0xff1d2125);
  VISAGE_THEME_IMPLEMENT_COLOR(GraphLine, GridColor, 0x22ffffff);
  VISAGE_THEME_IMPLEMENT_COLOR(GraphLine, HoverColor, 0xffffffff);
  VISAGE_THEME_IMPLEMENT_COLOR(GraphLine, DragColor, 0x55ffffff);

  VISAGE_THEME_IMPLEMENT_VALUE(GraphLine, LineWidth, 2.0f);
  VISAGE_THEME_VALUE(PositionBulbWidth, 4.0f);

  GraphLine::GraphLine(int num_points, bool loop) : data_(num_points), loop_(loop) { }

  GraphLine::~GraphLine() = default;

  float GraphLine::fillLocation() const {
    if (fill_center_ == kBottom)
      return 0.0f;
    if (fill_center_ == kTop)
      return 1.0f;
    if (fill_center_ == kCustom)
      return custom_fill_center_;
    return 0.5f;
  }

  void GraphLine::draw(Canvas& canvas) {
    if (canvas.totallyClamped())
      return;

    if (filled_)
      drawFill(canvas, active_ ? LineFillColor : LineDisabledFillColor);
    drawLine(canvas, active_ ? LineColor : LineDisabledColor);
  }

  void GraphLine::drawLine(Canvas& canvas, theme::ColorId color_id) {
    canvas.setColor(color_id);
    float line_width = line_width_.compute(canvas.dpiScale(), width(), height(),
                                           canvas.dpiScale() * paletteValue(LineWidth));
    canvas.graphLine(data_, 0.0f, 0.0f, width(), height(), Dimension::nativePixels(line_width));
  }

  void GraphLine::drawFill(Canvas& canvas, theme::ColorId color_id) {
    if (fill_alpha_mult_ != 1.0f)
      canvas.setColor(canvas.color(color_id).withMultipliedAlpha(fill_alpha_mult_));
    else
      canvas.setColor(canvas.color(color_id));

    canvas.graphFill(data_, 0.0f, 0.0f, width(), height(), fillLocation());
  }
}