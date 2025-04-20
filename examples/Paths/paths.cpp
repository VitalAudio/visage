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

visage::Path drawStar(visage::ApplicationWindow& app, double phase) {
  visage::Path path;
  std::complex<float> delta(cos(4.0f * kPi / kStarPoints), sin(4.0f * kPi / kStarPoints));
  std::complex<float> position(0, -1);

  float center_x = app.width() / 2.0f;
  float center_y = app.height() / 2.0f;

  for (int i = 0; i < kStarPoints; ++i) {
    path.lineTo(100.0f * position.real(), 100.0f * position.imag());
    position = position * delta;
  }

  path.close();
  path.rotate(phase);
  path.translate(center_x, center_y);
  //path.setFillRule(visage::Path::FillRule::NonZero);

  return path;
}

inline float randomFloat(float min, float max) {
  static std::random_device random_device;
  static std::mt19937 generator(random_device());
  std::uniform_real_distribution distribution(min, max);
  return distribution(generator);
}

int runExample() {
  visage::ApplicationWindow app;

  visage::Path star;

  visage::Path path1;
  visage::Path path2;

  app.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff442233);
    canvas.fill(0, 0, app.width(), app.height());

    std::complex<float> position(cos(-2.0f * kPi / kStarPoints), sin(-2.0f * kPi / kStarPoints));

    // while (true) {
    //   star.clear();
    //   star.moveTo(randomFloat(0.0f, app.width()), randomFloat(0.0f, app.height()));
    //   for (int i = 0; i < 3; ++i)
    //     star.lineTo(randomFloat(0.0f, app.width()), randomFloat(0.0f, app.height()));
    //   star.close();
    //
    //   star.moveTo(randomFloat(0.0f, app.width()), randomFloat(0.0f, app.height()));
    //   for (int i = 0; i < 3; ++i)
    //     star.lineTo(randomFloat(0.0f, app.width()), randomFloat(0.0f, app.height()));
    //   star.close();
    //   star.triangulate();
    // }
    star.clear();
    star.moveTo(randomFloat(0.0f, app.width()), randomFloat(0.0f, app.height()));
    for (int i = 0; i < 20; ++i)
      star.lineTo(randomFloat(0.0f, app.width()), randomFloat(0.0f, app.height()));
    star.close();

    path2.clear();
    path2.moveTo(0.3f * app.width(), 0);
    path2.lineTo(0.3f * app.width(), app.height());
    path2.lineTo(0.6f * app.width(), app.height());
    path2.lineTo(0.6f * app.width(), 0);
    path2.close();

    // star = star.computeXor(path2);
    canvas.setColor(visage::Brush::linear(0xffff00ff, 0xffffff00, visage::Point(0, 0),
                                          visage::Point(app.width(), app.height())));
    canvas.fill(&star, 0, 0, app.width(), app.height());

    canvas.setColor(0xffffffff);
    canvas.line(&star, 0, 0, app.width(), app.height(), 3);
    // app.redraw();
  };

  app.onMouseDown() = [&](const visage::MouseEvent& e) {
    app.redraw();
    // svg_path.translate(-0.5f * app.width(), -0.5f * app.height());
    // if (e.isRightButton())
    //   svg_path.rotate(-0.1f);
    // else
    //   svg_path.rotate(0.1f);
    // svg_path.translate(0.5f * app.width(), 0.5f * app.height());
  };

  app.setTitle("Visage Paths Example");
  app.show(800, 600);
  app.runEventLoop();
  return 0;
}
