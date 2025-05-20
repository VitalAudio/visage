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

void drawTriangles(visage::Canvas& canvas, const visage::Path& path) {
  static std::vector<visage::Color> colors;
  auto triangulation = path.triangulate();
  for (int i = 0; i * 3 < triangulation.triangles.size(); ++i) {
    if (i <= colors.size())
      colors.push_back(visage::Color::fromAHSV(0.8f, randomFloat(0.0f, 360.0f), 1.0f, 1.0f));

    canvas.setColor(colors[i]);
    visage::Point p1 = triangulation.points[triangulation.triangles[i * 3]];
    visage::Point p2 = triangulation.points[triangulation.triangles[i * 3 + 1]];
    visage::Point p3 = triangulation.points[triangulation.triangles[i * 3 + 2]];
    canvas.triangle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
  }
}

void drawPointIndices(visage::Canvas& canvas, const visage::Path& path) {
  canvas.setColor(0xffffffff);
  visage::Font font(14, resources::fonts::Lato_Regular_ttf);
  int i = 0;
  for (const auto& sub_path : path.subPaths()) {
    for (const auto& point : sub_path.points) {
      canvas.text(std::to_string(i), font, visage::Font::Justification::kTopLeft, point.x, point.y, 50, 50);
      ++i;
    }
  }
}

void printPath(const visage::Path& path) {
  for (const auto& sub_path : path.subPaths()) {
    for (const auto& point : sub_path.points) {
      VISAGE_LOG("path.lineTo(%f, %f);", point.x, point.y);
    }
    VISAGE_LOG("path.close();");
  }
}

int runExample() {
  visage::ApplicationWindow app;

  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr int kStarPoints = 20;
  static constexpr int kWidth = 100;
  static constexpr float kRadius = 40.0f;

  visage::Path path;
  // path.parseSvgPath("M8 9l4-4h-3v-4zM11.636 7.364l-1.121 1.121 4.064 1.515-6.579 "
  //                   "2.453-6.579-2.453 4.064-1.515-1.121-1.121-4.364 1.636v4l8 3 8-3v-4z");
  // path.scale(30);
  // path.translate(30, 30);

  // path.lineTo(3.00000000, 4.00000000);
  // path.lineTo(4.00000000, 2.00000000);
  // path.lineTo(3.00000000, 1.00000000);
  // path.lineTo(0.00000000, 3.00000000);
  // path.lineTo(4.00000000, 2.00000000);
  // path.lineTo(3.00000000, 1.00000000);

  // path.lineTo(2.00000000, 0.00000000);
  // path.lineTo(3.00000000, 2.00000000);
  // path.lineTo(4.00000000, 2.00000000);
  // path.lineTo(1.00000000, 1.00000000);
  // path.lineTo(2.00000000, 4.00000000);
  // path.lineTo(3.00000000, 1.00000000);

  // path.lineTo(1.00000000, 4.00000000);
  // path.lineTo(4.00000000, 4.00000000);
  // path.lineTo(2.00000000, 2.00000000);
  // path.lineTo(2.00000000, 3.00000000);
  // path.lineTo(1.00000000, 0.00000000);
  // path.lineTo(3.00000000, 0.00000000);

  // path.lineTo(1.00000000, 0.00000000);
  // path.lineTo(3.00000000, 4.00000000);
  // path.lineTo(0.00000000, 1.00000000);
  // path.lineTo(4.00000000, 4.00000000);
  // path.lineTo(2.00000000, 3.00000000);
  // path.lineTo(4.00000000, 2.00000000);

  path.lineTo(0.00000000, 3.00000000);
  path.lineTo(2.00000000, 2.00000000);
  path.lineTo(3.00000000, 0.00000000);
  path.lineTo(1.00000000, 3.00000000);
  path.lineTo(0.00000000, 3.00000000);
  path.lineTo(4.00000000, 1.00000000);
  path.lineTo(4.00000000, 4.00000000);

  path.scale(100);
  float offset = 0.0f;

  std::vector<visage::Color> colors;

  app.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff442233);
    canvas.fill(0, 0, app.width(), app.height());

    double value = 50 * sin(3.5500089999999997 * 0.2);
    value = 50;
    // path2 = path.computeOffset(value, visage::Path::JoinType::Miter);
    canvas.setColor(visage::Brush::linear(0xffff00ff, 0xffffff00, visage::Point(0, 0),
                                          visage::Point(app.width(), app.height())));

    // drawTriangles(canvas, path);
    // drawPointIndices(canvas, path);
    canvas.fill(&path, 0, 0, app.width(), app.height());
    canvas.setColor(0xffffffff);

    canvas.line(&path, 0, 0, app.width(), app.height(), 3);
    app.redraw();
  };

  app.onMouseMove() = [&](const visage::MouseEvent& e) { offset = e.position.y * 0.1f; };
  app.onMouseDown() = [&](const visage::MouseEvent& e) {
    path.clear();
    for (int i = 0; i < 10; ++i) {
      path.lineTo(randomFloat(0.0f, app.width()), randomFloat(0.0f, app.height()));
    }
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
