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

int runExample() {
  visage::ApplicationWindow app;

  app.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff222222);
    canvas.fill(0, 0, app.width(), app.height());

    canvas.setColor(0xffff44ff);

    visage::Path path;
    float w = app.width() / 3.0f;
    float h = app.height();
    visage::Point center { w / 2.0f, h / 2.0f };
    float outer_radius = std::min(w, h) * 0.4f;
    float inner_radius = std::min(w, h) * 0.162f;
    int num_points = 10;
    for (int i = 0; i < num_points; ++i) {
      float angle = (float)i / (float)num_points * 2.f * 3.14159265f;
      float radius = (i % 2) ? outer_radius : inner_radius;
      auto point = center + radius * visage::Point(std::sin(angle), std::cos(angle));
      if (i == 0)
        path.moveTo(point);
      else
        path.lineTo(point);
    }
    path.close();

    canvas.fill(path, 0, 0, w, h);

    auto stroked = path.stroke(2);
    canvas.fill(stroked, w, 0, w, h);

    auto dashed = path.stroke(2, visage::Path::Join::Miter, visage::Path::EndCap::Butt,
                              { path.length() / 20.0f }, canvas.time() * 50.0f);
    canvas.fill(dashed, 2.0f * w, 0, w, h);
    app.redraw();
  };

  app.setTitle("Visage Paths Example");
  app.show(900, 200);
  app.runEventLoop();
  return 0;
}
