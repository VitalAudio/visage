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
  static constexpr int kStarPoints = 10;
  static constexpr int kWidth = 100;
  static constexpr float kRadius = 40.0f;

  visage::Path path;
  // path.parseSvgPath("M8 9l4-4h-3v-4zM11.636 7.364l-1.121 1.121 4.064 1.515-6.579 "
  //                   "2.453-6.579-2.453 4.064-1.515-1.121-1.121-4.364 1.636v4l8 3 8-3v-4z");
  // path.scale(30);
  // path.translate(30, 30);

  std::vector<std::vector<visage::DPoint>> paths = {
    { { 0, 2 }, { 0, 4 }, { 4, 2 }, { 3, 4 }, { 3, 2 }, { 4, 3 }, { 2, 0 }, { 4, 0 }, { 2, 4 }, { 0, 4 }, { 2, 3 }, { 1, 4 } },
    { { 2, 3 }, { 2, 1 }, { 1, 4 }, { 3, 2 }, { 4, 0 }, { 1, 4 }, { 4, 1 }, { 1, 2 }, { 2, 4 }, { 0, 4 } },
    { { 2, 1 }, { 0, 3 }, { 4, 4 }, { 1, 3 }, { 3, 2 }, { 2, 0 }, { 0, 1 }, { 4, 1 }, { 3, 3 }, { 3, 0 }, { 3, 2 }, { 2, 4 } },
    { { 4, 3 }, { 3, 0 }, { 2, 2 }, { 3, 1 }, { 3, 2 }, { 0, 2 }, { 4, 3 }, { 4, 4 }, { 2, 2 } },
    { { 2, 4 },
      { 1, 2 },
      { 1, 4 },
      { 4, 1 },
      { 1, 2 },
      { 4, 3 },
      { 2, 0 },
      { 2, 4 },
      { 3, 4 },
      { 1, 0 },
      { 3, 4 },
      { 4, 1 },
      { 0, 4 },
      { 1, 1 },
      { 4, 1 } },
    { { 0, 4 },
      { 1, 0 },
      { 1, 4 },
      { 0, 2 },
      { 2, 4 },
      { 3, 4 },
      { 2, 0 },
      { 4, 1 },
      { 2, 3 },
      { 3, 3 },
      { 4, 0 },
      { 2, 0 },
      { 2, 3 },
      { 0, 1 },
      { 3, 4 },
      { 1, 3 },
      { 4, 1 } },
    { { 1, 2 }, { 4, 2 }, { 3, 4 }, { 4, 0 }, { 4, 4 }, { 3, 3 }, { 4, 1 } },
    { { 0, 4 }, { 4, 0 }, { 0, 3 }, { 1, 4 }, { 3, 1 }, { 1, 3 }, { 4, 4 } },
    { { 4, 1 }, { 4, 4 }, { 3, 3 }, { 0, 1 }, { 0, 2 }, { 4, 3 }, { 1, 4 } },
    { { 1, 2 }, { 2, 2 }, { 0, 1 }, { 4, 1 }, { 4, 0 }, { 0, 4 }, { 4, 1 } },
    { { 0, 2 }, { 4, 4 }, { 4, 0 }, { 1, 2 }, { 4, 4 }, { 4, 2 }, { 0, 3 } },
    { { 3, 4 }, { 3, 1 }, { 1, 2 }, { 3, 0 }, { 3, 2 }, { 4, 2 }, { 2, 0 } },
    { { 2, 2 }, { 4, 4 }, { 0, 4 }, { 1, 0 }, { 0, 0 }, { 0, 4 }, { 3, 4 } },
    { { 4, 1 }, { 2, 2 }, { 2, 1 }, { 0, 3 }, { 4, 1 }, { 1, 3 }, { 0, 1 } },
    { { 1, 0 }, { 3, 2 }, { 2, 0 }, { 4, 3 }, { 2, 1 }, { 4, 0 }, { 4, 1 } },
    { { 1, 3 }, { 0, 3 }, { 3, 2 }, { 2, 0 }, { 0, 3 }, { 0, 3 }, { 2, 2 } },
    { { 1, 1 }, { 1, 0 }, { 3, 4 }, { 4, 3 }, { 1, 0 }, { 1, 2 }, { 2, 0 } },
    { { 0, 3 }, { 2, 2 }, { 3, 0 }, { 1, 3 }, { 0, 3 }, { 4, 1 } },
    { { 4, 4 }, { 2, 0 }, { 2, 4 }, { 3, 4 }, { 4, 4 }, { 2, 0 }, { 0, 4 } },
    { { 0, 0 }, { 3, 4 }, { 4, 3 }, { 0, 3 }, { 4, 4 }, { 3, 3 }, { 4, 3 } },
    { { 2, 4 }, { 2, 2 }, { 1, 3 }, { 1, 2 }, { 1, 2 }, { 0, 4 }, { 3, 1 } },
    { { 1, 2 }, { 1, 1 }, { 1, 1 }, { 3, 0 }, { 1, 1 }, { 0, 4 }, { 4, 1 } },
    { { 3, 4 }, { 4, 2 }, { 3, 1 }, { 0, 3 }, { 4, 2 }, { 3, 1 } },
    { { 2, 0 }, { 3, 2 }, { 4, 2 }, { 1, 1 }, { 2, 4 }, { 3, 1 } },
    { { 1, 4 }, { 4, 4 }, { 2, 2 }, { 2, 3 }, { 1, 0 }, { 3, 0 } },
    { { 1, 0 }, { 3, 4 }, { 0, 1 }, { 4, 4 }, { 2, 3 }, { 4, 2 } },
    { { 4, 4 }, { 4, 0 }, { 0, 2 }, { 1, 0 }, { 3, 2 }, { 0, 2 }, { 2, 1 } },
    { { 4, 4 }, { 2, 0 }, { 4, 1 }, { 0, 1 }, { 0, 3 }, { 4, 0 }, { 3, 1 } },
    { { 0, 3 }, { 3, 1 }, { 0, 2 }, { 4, 4 }, { 1, 4 }, { 1, 1 }, { 2, 2 } },
    { { 2, 2 }, { 0, 4 }, { 0, 2 }, { 1, 4 }, { 1, 0 }, { 2, 4 }, { 0, 2 } },
    { { 1, 4 }, { 2, 3 }, { 4, 2 }, { 1, 1 }, { 2, 1 }, { 4, 4 }, { 3, 0 } },
    { { 4, 0 }, { 0, 2 }, { 3, 1 }, { 3, 3 }, { 4, 4 }, { 0, 3 }, { 0, 4 } },
    { { 4, 4 }, { 1, 1 }, { 2, 3 }, { 2, 2 }, { 3, 0 }, { 4, 0 }, { 1, 3 } },
    { { 2, 1 }, { 3, 0 }, { 1, 4 }, { 2, 3 }, { 2, 1 }, { 4, 2 }, { 2, 2 } },
    { { 3, 3 }, { 2, 2 }, { 4, 3 }, { 5, -2 }, { 0, 4 }, { 4, 2 }, { 3, 1 } },
  };

  float radius = 100.0f;
  float offset = 0.0f;
  for (const auto& point : paths[0])
    path.lineTo(point.x, point.y);
  path.close();

  // path.scale(100);

  std::vector<visage::Color> colors;

  app.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff442233);
    canvas.fill(0, 0, app.width(), app.height());

    double value = 50 * sin(3.5500089999999997 * 0.2);
    value = 50;
    // path2 = path.computeOffset(value, visage::Path::JoinType::Miter);

    // drawTriangles(canvas, path);
    drawPointIndices(canvas, path);

    canvas.setColor(visage::Brush::linear(0xffff00ff, 0xffffff00, visage::Point(0, 0),
                                          visage::Point(app.width(), app.height())));
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
