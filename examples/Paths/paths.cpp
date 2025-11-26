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

#include "embedded/example_fonts.h"

#include <visage/app.h>
#include <visage/graphics.h>
#include <visage/ui.h>

visage::Path starPath(float center_x, float center_y, float radius) {
  static constexpr float kPi = 3.14159265f;

  visage::Path path;
  int num_points = 10;
  for (int i = 0; i < num_points; ++i) {
    float angle = (float)i / (float)num_points * 2.0f * kPi;
    float r = (i % 2) ? radius : radius * 0.4f;
    auto point = visage::Point { center_x, center_y } +
                 r * visage::Point(std::sin(angle), std::cos(angle));
    if (i == 0)
      path.moveTo(point);
    else
      path.lineTo(point);
  }
  path.close();
  return path;
}

int runExample() {
  visage::ApplicationWindow app;

  app.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff222222);
    canvas.fill(0, 0, app.width(), app.height());

    canvas.setColor(0xffff44ff);

    float w = app.width() / 3.0f;
    float h = app.height();
    visage::Path star = starPath(w * 0.5f, h * 0.5f, std::min(w, h) * 0.4f);
    auto stroked = star.stroke(2);
    float segment = star.length() / 40.0f;
    auto dashed = star.stroke(2, visage::Path::Join::Round, visage::Path::EndCap::Round,
                              { segment }, canvas.time() * segment);
    canvas.fill(star);
    canvas.fill(stroked, w, 0);
    canvas.fill(dashed, 2.0f * w, 0);
    app.redraw();
  };

  app.setTitle("Visage Paths Example");
  app.show(900, 200);
  app.runEventLoop();
  return 0;
}
