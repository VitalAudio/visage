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
  THEME_IMPLEMENT_COLOR(GraphLine, LineColor, 0xffaa88ff);
  THEME_IMPLEMENT_COLOR(GraphLine, LineFillColor, 0x669f88ff);
  THEME_IMPLEMENT_COLOR(GraphLine, LineFillColor2, 0x669f88ff);
  THEME_IMPLEMENT_COLOR(GraphLine, LineDisabledColor, 0xff4c4f52);
  THEME_IMPLEMENT_COLOR(GraphLine, LineDisabledFillColor, 0x22666666);
  THEME_IMPLEMENT_COLOR(GraphLine, CenterPoint, 0xff1d2125);
  THEME_IMPLEMENT_COLOR(GraphLine, GridColor, 0x22ffffff);
  THEME_IMPLEMENT_COLOR(GraphLine, HoverColor, 0xffffffff);
  THEME_IMPLEMENT_COLOR(GraphLine, DragColor, 0x55ffffff);

  THEME_IMPLEMENT_VALUE(GraphLine, LineWidth, 2.0f, ScaledDpi, false);
  THEME_IMPLEMENT_VALUE(GraphLine, LineColorBoost, 1.5f, Constant, false);
  THEME_IMPLEMENT_VALUE(GraphLine, LineFillBoost, 1.0f, Constant, false);
  THEME_VALUE(PositionBulbWidth, 4.0f, ScaledDpi, true);

  GraphLine::GraphLine(int num_points, bool loop) :
      line_(num_points), fill_center_(kCenter), loop_(loop) {
    for (int i = 0; i < line_.num_points; ++i)
      setXAt(i, i / (line_.num_points - 1.0f));
  }

  GraphLine::~GraphLine() = default;

  int GraphLine::fillLocation() const {
    if (fill_center_ == kBottom)
      return height();
    if (fill_center_ == kTop)
      return 0;
    if (fill_center_ == kCustom)
      return custom_fill_center_;
    return height() / 2;
  }

  void GraphLine::draw(Canvas& canvas) {
    if (canvas.totallyClamped())
      return;

    if (fill_)
      drawFill(canvas, active_ ? kLineFillColor : kLineDisabledFillColor);
    drawLine(canvas, active_ ? kLineColor : kLineDisabledColor);
  }

  void GraphLine::drawLine(Canvas& canvas, unsigned int color_id) {
    line_.line_value_scale = canvas.value(kLineColorBoost);
    canvas.setPaletteColor(color_id);
    canvas.line(&line_, 0.0f, 0.0f, width(), height(), line_width_);
  }

  void GraphLine::drawFill(Canvas& canvas, unsigned int color_id) {
    QuadColor color = canvas.color(color_id);
    line_.fill_value_scale = canvas.value(kLineFillBoost);
    canvas.setColor(color.withMultipliedAlpha(fill_alpha_mult_));
    canvas.lineFill(&line_, 0.0f, 0.0f, width(), height(), fillLocation());
  }

  void GraphLine::drawPosition(Canvas& canvas, float x, float y) {
    float marker_width = canvas.value(kPositionBulbWidth);
    canvas.setColor(canvas.color(kLineColor).withMultipliedHdr(1.0f + canvas.value(kLineColorBoost)));
    canvas.circle(x - marker_width * 0.5f, y - marker_width * 0.5f, marker_width);
  }

  void GraphLine::resized() {
    line_width_ = paletteValue(kLineWidth);
    Frame::resized();
  }
}