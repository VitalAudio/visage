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
#include "visage_widgets/shader_editor.h"

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
  visage::ShaderCompiler compiler;
  compiler.watchShaderFolder("C:/Users/matth/visage/visage_graphics/shaders");

  visage::ApplicationWindow app;

  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr int kStarPoints = 10;
  static constexpr int kWidth = 100;
  static constexpr float kRadius = 40.0f;

  visage::Path path;
  // path.parseSvgPath(
  //     "M417.03 387.52C405.88 380.67 396.85 370.88 390.91 359.22C376.72 331.39 343.25 265.7 329.07 "
  //     "237.86C325.29 230.45 314.7       230.46 310.93 237.87C296.74 265.81 263.23 331.78 249.05 "
  //     "359.71C243.12 371.37 234.08 381.16 222.93 387.99C186.34 410.39 93.63 467.15 57.04 "
  //     "489.56C47.57 495.36       35.2 492.51 29.21 483.15C24.97 476.51 23.01 473.44 18.76 "
  //     "466.8C12.87 457.59 15.78 445.32 25.18 439.74C60.74 418.6 150.23 365.4 185.8 344.26C197.02 "
  //     "337.59 206.09 327.83 211.93 316.15C229.54 280.93 273.43 193.15 291.04 157.93C296.53 "
  //     "146.97 307.73 140.04 319.99 140.04C323.99 140.04 315.99 140.04 319.99 140.04C332.25       "
  //     "140.04 343.47 146.95 348.99 157.9C366.59 192.83 410.39 279.75 427.99 314.68C433.87 326.35 "
  //     "442.94 336.11 454.14 342.83C489.91 364.27 580.07 418.32 615.83 439.76C625.18 445.36 "
  //     "627.97 457.63 621.96 466.73C617.57 473.38 615.41 476.66 611.01 483.31C604.87 492.62 592.45 "
  //     "495.36 582.95 489.53C546.36 467.03 453.62 410.01        417.03 387.52Z");
  //path.parseSvgPath("M632.44 336.35C642.52 327.78 642.52 312.22 632.44 303.65C603.02 278.66 534 "
  //                  "220.04 504.58 195.05C497.05 188.66 485.49        194.08 485.59 203.96C485.67 "
  //                  "212.74 485.89 234.68 486.24 269.8C459.05 269.87 442.05 269.92 435.25 "
  //                  "269.93C421.3 269.97 410 281.3 410 295.25C410 310.2 410 329.8       410 "
  //                  "344.75C410 358.71 421.29 370.04 435.25 370.08C442.05 370.1 459.05 370.15 "
  //                  "486.24 370.24C485.89 405.33 485.67 427.27 485.59 436.04C485.49 445.92 497.05 "
  //                  "451.34 504.58 444.95C534 419.96 603.02 361.34 632.44 336.35Z");
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
  // path.lineTo(0, 0);
  // path.lineTo(500, 0);
  // path.lineTo(500, 500);
  // path.lineTo(250, 250);
  // path.lineTo(0, 500);
  // path.close();
  // path.translate(100, 100);
  // path = path.offset(-35.0f, visage::Path::JoinType::Round);
  //
  // path.lineTo(112, 112);
  // path.lineTo(200, 112);
  // path.lineTo(200, 200);
  // path.lineTo(112, 200);
  // path.close();

  std::vector<visage::Color> colors;

  //loop over files in a directory and load them
  auto svgs = visage::searchForFiles("C:\\Users\\matth\\vital\\icons", ".*\\.svg");
  visage::Svg svg;
  int svg_index = 0;

  auto load_next_svg = [&] {
    std::string svg_data = visage::loadFileAsString(svgs[svg_index]);
    svg_index = (svg_index + 1) % svgs.size();
    svg = visage::Svg((unsigned char*)svg_data.c_str(), svg_data.length());
    svg.setDimensions(app.width(), app.height());
  };

  // path.moveTo(0.00000000, 0);
  // path.lineTo(-3.33066907e-16, 10);
  // path.lineTo(5, 10);
  // path.lineTo(5, 0);
  // path.close();
  //
  // visage::Path intersect;
  // intersect.addRectangle(0, 0, 400, 400);
  // path = path.combine(intersect, visage::Path::Operation::Intersection);
  // path.translate(50, 50);

  app.onDraw() = [&](visage::Canvas& canvas) {
    // auto gradient = visage::Gradient(0xffff00ff, 0xffffff00);
    // canvas.setColor(visage::Brush::radial(gradient, visage::Point(50.0f, 50.0f), 100.0f));
    // canvas.setColor(visage::Brush::horizontal(gradient));
    canvas.setColor(0xffffffff);
    canvas.fill(0, 0, app.width(), app.height());

    svg.draw(canvas, 0, 0);
    canvas.setColor(0xff00ff00);
    // canvas.fill(&path, 0, 0, 120, 150);
    app.redraw();
  };

  app.onResize() = [&]() {
    svg.setDimensions(app.width(), app.height());
    app.redraw();
  };

  app.onMouseMove() = [&](const visage::MouseEvent& e) { offset = e.position.y * 0.1f; };
  app.onMouseDown() = [&](const visage::MouseEvent& e) {
    load_next_svg();
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
