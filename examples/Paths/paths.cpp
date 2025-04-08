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

inline float randomFloat() {
  static std::random_device random_device;
  static std::mt19937 generator(random_device());
  std::uniform_real_distribution distribution(0.0f, 1.0f);
  return distribution(generator);
}

static constexpr int kStarPoints = 5;
static constexpr int kFanPoints = 10;
static constexpr float kPi = 3.14159265358979323846f;

visage::Path drawFan(visage::ApplicationWindow& app, double time) {
  visage::Path path;
  std::complex<float> delta(cos(kPi * 0.5f / kFanPoints), sin(kPi * 0.5f / kFanPoints));
  std::complex<float> position(cos(time), sin(time));

  float center_x = app.width() / 2.0f;
  float center_y = app.height() / 2.0f;
  auto start_position = std::complex<float>(cos(time + kPi * 0.25f), sin(time + kPi * 0.25f));
  path.moveTo(center_x + 200.0f * start_position.real(), center_y + 200.0f * start_position.imag());

  for (int i = 1; i < kFanPoints; ++i) {
    path.lineTo(center_x + 100.0f * position.real(), center_y + 100.0f * position.imag());
    position = position * delta;
  }

  return path;
}

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

  int num_draw = 0;
  float ratio = 0.5f;

  std::vector<visage::Color> colors;
  visage::Path path;
  // path.moveTo(100, 400);
  // path.lineTo(400, 400);
  // path.lineTo(401, 100);
  // path.lineTo(101, 100);
  // path.lineTo(301, 300);
  // path.lineTo(300, 200);

  // path.moveTo(100, 300);
  // path.lineTo(400, 500);
  // path.lineTo(701, 100);
  // path.lineTo(900, 400);
  // path.lineTo(901, 100);
  // path.lineTo(700, 400);
  // path.lineTo(401, 100);

  // Star

  app.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff000066);
    canvas.fill(0, 0, app.width(), app.height());

    float center_x = app.width() / 2.0f;
    float center_y = app.height() / 2.0f;
    float radius = 300.0f;

    path.clear();
    path.moveTo(95.6908340, 84.7759018);
    path.lineTo(18.5086765, 86.3851242);
    path.lineTo(89.6078720, 76.3953705);
    path.lineTo(43.8916054, 60.7139359);
    path.lineTo(22.0712891, 25.0330086);
    path.scale(10.0f);
    canvas.setColor(0xffffffff);
    canvas.line(&path, 0, 0, app.width(), app.height(), 3.0f);

    // path.moveTo(center_x, center_y - radius);
    // std::complex<float> delta(cos(-2.0f * kPi * 2.0f / kStarPoints), sin(-2.0f * kPi * 2.0f / kStarPoints));
    // std::complex<float> position(0.0f, -1.0f);
    //
    // for (int i = 1; i < kStarPoints; ++i) {
    //   position = position * delta;
    //   path.lineTo(center_x + radius * position.real(), center_y + radius * position.imag());
    // }

    // path.reverse();

    // path.parseSvgPath("M9 3.881v-3.881l6 6-6 6v-3.966c-6.98-0.164-6.681 4.747-4.904 "
    //                   "7.966-4.386-4.741-3.455-12.337 4.904-12.119z");
    // path.scale(40.0f);

    visage::Path::Triangulation tri = path.triangulate();
    for (int i = 0; i < tri.triangles.size() / 3; ++i) {
      int num = num_draw % (tri.triangles.size() / 3 + 1);
      if (i >= colors.size())
        colors.push_back(visage::Color(1.0f, randomFloat(), randomFloat(), randomFloat()));

      canvas.setColor(colors[i]);
      int index = i * 3;
      canvas.triangle(tri.points[tri.triangles[index]].x, tri.points[tri.triangles[index]].y,
                      tri.points[tri.triangles[index + 1]].x, tri.points[tri.triangles[index + 1]].y,
                      tri.points[tri.triangles[index + 2]].x, tri.points[tri.triangles[index + 2]].y);
    }
    app.redraw();
  };

  app.onMouseDown() = [&](const visage::MouseEvent& e) {
    num_draw++;
    app.redraw();
  };

  app.onMouseMove() = [&](const visage::MouseEvent& e) { ratio = e.position.y / app.height(); };

  app.setTitle("Visage Basic Example");
  app.show(800, 600);
  app.runEventLoop();
  return 0;
}
