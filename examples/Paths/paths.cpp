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

inline float randomFloat(float min, float max) {
  static std::random_device random_device;
  static std::mt19937 generator(random_device());
  std::uniform_real_distribution distribution(min, max);
  return distribution(generator);
}

int runExample() {
  visage::ApplicationWindow app;

  visage::Path path;
  path.moveTo(107.989151, 492.898651);
  path.lineTo(190.269104, 18.6316490);
  path.lineTo(754.102112, 208.022049);
  path.lineTo(279.940582, 84.3667450);
  path.lineTo(708.273193, 63.2528839);

  app.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff442233);
    canvas.fill(0, 0, app.width(), app.height());

    path.clear();
    path.moveTo(randomFloat(0, app.width()), randomFloat(0, app.height()));
    for (int i = 0; i < 20; ++i)
      path.lineTo(randomFloat(0, app.width()), randomFloat(0, app.height()));
    path.close();

    canvas.setColor(visage::Brush::linear(0xffff00ff, 0xffffff00, visage::Point(0, 0),
                                          visage::Point(app.width(), app.height())));
    canvas.fill(&path, 0, 0, app.width(), app.height());

    canvas.setColor(0xffffffff);
    canvas.line(&path, 0, 0, app.width(), app.height(), 3);
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
