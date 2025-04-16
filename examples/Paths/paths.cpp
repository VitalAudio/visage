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

#include <complex>
#include <random>
#include <visage/app.h>
#include <visage/graphics.h>

static constexpr int kStarPoints = 5;
static constexpr float kPi = 3.14159265358979323846f;

visage::Path drawStar(visage::ApplicationWindow& app, double time, float ratio) {
  visage::Path path;
  std::complex<float> delta(cos(kPi / kStarPoints), sin(kPi / kStarPoints));
  std::complex<float> position(cos(time), sin(time));

  float center_x = app.width() / 2.0f;
  float center_y = app.height() / 2.0f;

  for (int i = 0; i < kStarPoints; ++i) {
    std::complex<float> inner_position = position * delta;
    path.lineTo(center_x + 100.0f * position.real(), center_y - 100.0f * position.imag());
    path.lineTo(center_x + 100.0f * ratio * inner_position.real(),
                center_y - 100.0f * ratio * inner_position.imag());
    position = inner_position * delta;
  }

  return path;
}

int runExample() {
  visage::ApplicationWindow app;
  float ratio = 0.5f;

  visage::Path path;

  app.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff662244);
    canvas.fill(0, 0, app.width(), app.height());
    path = drawStar(app, canvas.time(), ratio);

    canvas.setColor(0xffffffff);
    canvas.fill(&path, 0, 0, app.width(), app.height());
    app.redraw();
  };

  app.onMouseDown() = [&](const visage::MouseEvent& e) { app.redraw(); };

  app.setTitle("Visage Paths Example");
  app.show(800, 600);
  app.runEventLoop();
  return 0;
}
