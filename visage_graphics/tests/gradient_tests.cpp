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

#include "visage_graphics/canvas.h"
#include "visage_graphics/color.h"
#include "visage_graphics/gradient.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <sstream>

using namespace visage;
using namespace Catch;

TEST_CASE("Gradient initialization", "[graphics]") {
  SECTION("Default constructor creates empty gradient") {
    Gradient gradient;
    REQUIRE(gradient.resolution() == 0);
    REQUIRE(gradient.colors().empty());
  }

  SECTION("Constructor with variadic colors") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color green(1.0f, 0.0f, 1.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);

    Gradient gradient(red, green, blue);

    REQUIRE(gradient.resolution() == 3);
    REQUIRE(gradient.colors()[0] == red);
    REQUIRE(gradient.colors()[1] == green);
    REQUIRE(gradient.colors()[2] == blue);
  }
}

TEST_CASE("Gradient fromSampleFunction", "[graphics]") {
  SECTION("Creates gradient with sample function") {
    auto sampleFunc = [](float t) -> Color { return { 1.0f, t, 0.0f, 0.0f }; };

    Gradient gradient = Gradient::fromSampleFunction(5, sampleFunc);

    REQUIRE(gradient.resolution() == 5);
    REQUIRE(gradient.colors()[0].red() == Approx(0.0f));
    REQUIRE(gradient.colors()[1].red() == Approx(0.25f));
    REQUIRE(gradient.colors()[2].red() == Approx(0.5f));
    REQUIRE(gradient.colors()[3].red() == Approx(0.75f));
    REQUIRE(gradient.colors()[4].red() == Approx(1.0f));
  }

  SECTION("Assert fires with zero resolution") {
    auto sampleFunc = [](float t) -> Color { return Color(); };

    Gradient gradient = Gradient::fromSampleFunction(1, sampleFunc);
    REQUIRE(gradient.resolution() == 1);
  }
}

TEST_CASE("Gradient color manipulation", "[graphics]") {
  SECTION("setResolution with empty gradient") {
    Gradient gradient;
    gradient.setResolution(5);

    REQUIRE(gradient.resolution() == 5);
    for (int i = 0; i < 5; ++i) {
      REQUIRE(gradient.colors()[i].alpha() == Approx(0.0f));
      REQUIRE(gradient.colors()[i].red() == Approx(0.0f));
      REQUIRE(gradient.colors()[i].green() == Approx(0.0f));
      REQUIRE(gradient.colors()[i].blue() == Approx(0.0f));
    }
  }

  SECTION("setResolution with existing gradient") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Gradient gradient(red);

    gradient.setResolution(3);

    REQUIRE(gradient.resolution() == 3);
    REQUIRE(gradient.colors()[0] == red);
    REQUIRE(gradient.colors()[1] == red);
    REQUIRE(gradient.colors()[2] == red);
  }

  SECTION("setResolution keeps positions in sync") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);
    Gradient gradient(red);

    gradient.setResolution(2);
    gradient.setColor(1, blue);
    REQUIRE(gradient.sample(0.0f) == red);
    REQUIRE(gradient.sample(1.0f) == blue);
    REQUIRE(gradient.sample(0.5f).red() == Approx(0.5f));
    REQUIRE(Gradient::compare(gradient, Gradient(red, blue)) == 0);

    Gradient decoded;
    decoded.decode(gradient.encode());
    REQUIRE(Gradient::compare(decoded, gradient) == 0);

    gradient.setResolution(1);
    REQUIRE(gradient.sample(0.5f) == red);
    REQUIRE(Gradient::compare(gradient, Gradient(red)) == 0);
  }

  SECTION("setColor modifies color at index") {
    Gradient gradient;
    gradient.setResolution(3);

    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    gradient.setColor(1, red);

    REQUIRE(gradient.colors()[0].red() == Approx(0.0f));
    REQUIRE(gradient.colors()[1].red() == Approx(1.0f));
    REQUIRE(gradient.colors()[2].red() == Approx(0.0f));
  }
}

TEST_CASE("Gradient sampling", "[graphics]") {
  SECTION("Sample empty gradient") {
    Gradient gradient;
    Color color = gradient.sample(0.5f);

    REQUIRE(color.alpha() == Approx(0.0f));
    REQUIRE(color.red() == Approx(0.0f));
    REQUIRE(color.green() == Approx(0.0f));
    REQUIRE(color.blue() == Approx(0.0f));
  }

  SECTION("Sample single color gradient") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Gradient gradient(red);

    Color color = gradient.sample(0.5f);
    REQUIRE(color == red);
  }

  SECTION("Sample multi-color gradient") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);
    Gradient gradient(red, blue);

    Color color = gradient.sample(0.5f);
    REQUIRE(color.alpha() == Approx(1.0f));
    REQUIRE(color.red() == Approx(0.5f));
    REQUIRE(color.green() == Approx(0.0f));
    REQUIRE(color.blue() == Approx(0.5f));

    color = gradient.sample(0.0f);
    REQUIRE(color == red);

    color = gradient.sample(1.0f);
    REQUIRE(color == blue);
  }

  SECTION("Stops sharing the first offset") {
    Color white(1.0f, 1.0f, 1.0f, 1.0f);
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);
    Gradient gradient;
    gradient.addColorStop(white, 0.0f);
    gradient.addColorStop(red, 0.0f);
    gradient.addColorStop(blue, 1.0f);

    REQUIRE(gradient.sample(0.0f) == white);
    REQUIRE(gradient.sample(0.5f).red() == Approx(0.5f));
    REQUIRE(gradient.sample(0.5f).blue() == Approx(0.5f));

    gradient.setRepeat(true);
    REQUIRE(gradient.sample(0.0f) == red);
  }
}

TEST_CASE("Gradient comparison", "[graphics]") {
  SECTION("Different resolution") {
    Gradient gradient1;
    gradient1.setResolution(3);

    Gradient gradient2;
    gradient2.setResolution(5);

    REQUIRE(Gradient::compare(gradient1, gradient2) < 0);
    REQUIRE(Gradient::compare(gradient2, gradient1) > 0);
  }

  SECTION("Same resolution, different colors") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color green(1.0f, 0.0f, 1.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);

    Gradient gradient1(red, green);
    Gradient gradient2(red, blue);

    int colorCompare = Color::compare(green, blue);
    REQUIRE(Gradient::compare(gradient1, gradient2) == colorCompare);
  }

  SECTION("Same resolution and colors") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);

    Gradient gradient1(red, blue);
    Gradient gradient2(red, blue);

    REQUIRE(Gradient::compare(gradient1, gradient2) == 0);
  }

  SECTION("Operator < works correctly") {
    Gradient gradient1;
    gradient1.setResolution(3);

    Gradient gradient2;
    gradient2.setResolution(5);

    REQUIRE(gradient1 < gradient2);
    REQUIRE(!(gradient2 < gradient1));
  }
}

TEST_CASE("Gradient interpolation", "[graphics]") {
  SECTION("Interpolate between gradients") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);
    Gradient gradient1(red);

    Color green(1.0f, 0.0f, 1.0f, 0.0f);
    Color yellow(1.0f, 1.0f, 1.0f, 0.0f);
    Gradient gradient2(green, yellow);

    Gradient result = Gradient::interpolate(gradient1, gradient2, 0.0f);
    REQUIRE(result.resolution() == 2);
    REQUIRE(result.colors()[0] == red);

    result = Gradient::interpolate(gradient1, gradient2, 1.0f);
    REQUIRE(result.resolution() == 2);
    REQUIRE(result.colors()[0] == green);
    REQUIRE(result.colors()[1] == yellow);

    result = Gradient::interpolate(gradient1, gradient2, 0.5f);
    REQUIRE(result.resolution() == 2);
    REQUIRE(result.colors()[0].red() == Approx(0.5f));
    REQUIRE(result.colors()[0].green() == Approx(0.5f));
    REQUIRE(result.colors()[0].blue() == Approx(0.0f));
  }

  SECTION("interpolateWith method") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);
    Gradient gradient1(red, blue);

    Color green(1.0f, 0.0f, 1.0f, 0.0f);
    Gradient gradient2(green);

    Gradient result = gradient1.interpolateWith(gradient2, 0.5f);
    REQUIRE(result.resolution() == 2);

    REQUIRE(result.colors()[0].red() == Approx(0.5f));
    REQUIRE(result.colors()[0].green() == Approx(0.5f));
    REQUIRE(result.colors()[0].blue() == Approx(0.0f));

    REQUIRE(result.colors()[1].red() == Approx(0.0f));
    REQUIRE(result.colors()[1].green() == Approx(0.5f));
    REQUIRE(result.colors()[1].blue() == Approx(0.5f));
  }
}

TEST_CASE("Gradient alpha manipulation", "[graphics]") {
  SECTION("withMultipliedAlpha") {
    Color red(0.8f, 1.0f, 0.0f, 0.0f);
    Color blue(0.6f, 0.0f, 0.0f, 1.0f);
    Gradient gradient(red, blue);

    Gradient result = gradient.withMultipliedAlpha(0.5f);

    REQUIRE(result.colors()[0].alpha() == Approx(0.4f));
    REQUIRE(result.colors()[1].alpha() == Approx(0.3f));

    REQUIRE(result.colors()[0].red() == Approx(1.0f));
    REQUIRE(result.colors()[1].blue() == Approx(1.0f));
  }
}

TEST_CASE("Gradient serialization", "[graphics]") {
  SECTION("encode/decode") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);
    Gradient original(red, blue);

    std::string encoded = original.encode();

    Gradient decoded;
    decoded.decode(encoded);

    REQUIRE(decoded.resolution() == original.resolution());
    REQUIRE(decoded.colors()[0] == original.colors()[0]);
    REQUIRE(decoded.colors()[1] == original.colors()[1]);
  }

  SECTION("Using stream operators") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);
    Gradient original(red, blue);

    std::ostringstream ostream;
    original.encode(ostream);

    Gradient decoded;
    std::istringstream istream(ostream.str());
    decoded.decode(istream);

    REQUIRE(decoded.resolution() == original.resolution());
    REQUIRE(decoded.colors()[0] == original.colors()[0]);
    REQUIRE(decoded.colors()[1] == original.colors()[1]);
  }
}

TEST_CASE("GradientPosition", "[graphics]") {
  SECTION("Initialization") {
    GradientPosition pos;
    REQUIRE(pos.shape == GradientPosition::InterpolationShape::Solid);

    GradientPosition horizontal(GradientPosition::InterpolationShape::Horizontal);
    REQUIRE(horizontal.shape == GradientPosition::InterpolationShape::Horizontal);

    Point from(10.0f, 20.0f);
    Point to(30.0f, 40.0f);
    GradientPosition linear(from, to);
    REQUIRE(linear.shape == GradientPosition::InterpolationShape::PointsLinear);
    REQUIRE(linear.point1 == from);
    REQUIRE(linear.point2 == to);
  }

  SECTION("Interpolation") {
    Point from1(0.0f, 0.0f);
    Point to1(100.0f, 100.0f);
    GradientPosition pos1(from1, to1);

    Point from2(100.0f, 0.0f);
    Point to2(0.0f, 100.0f);
    GradientPosition pos2(from2, to2);

    GradientPosition result = GradientPosition::interpolate(pos1, pos2, 0.5f);

    REQUIRE(result.shape == GradientPosition::InterpolationShape::PointsLinear);
    REQUIRE(result.point1.x == Approx(50.0f));
    REQUIRE(result.point1.y == Approx(0.0f));
    REQUIRE(result.point2.x == Approx(50.0f));
    REQUIRE(result.point2.y == Approx(100.0f));
  }

  SECTION("Serialization") {
    Point from(10.0f, 20.0f);
    Point to(30.0f, 40.0f);
    GradientPosition original(from, to);

    std::string encoded = original.encode();

    GradientPosition decoded;
    decoded.decode(encoded);

    REQUIRE(decoded.shape == original.shape);
    REQUIRE(decoded.point1.x == Approx(original.point1.x));
    REQUIRE(decoded.point1.y == Approx(original.point1.y));
    REQUIRE(decoded.point2.x == Approx(original.point2.x));
    REQUIRE(decoded.point2.y == Approx(original.point2.y));
  }

  SECTION("Scaling") {
    Point from(10.0f, 20.0f);
    Point to(30.0f, 40.0f);
    GradientPosition original(from, to);

    GradientPosition scaled = original * 2.0f;

    REQUIRE(scaled.shape == original.shape);
    REQUIRE(scaled.point1.x == Approx(20.0f));
    REQUIRE(scaled.point1.y == Approx(40.0f));
    REQUIRE(scaled.point2.x == Approx(60.0f));
    REQUIRE(scaled.point2.y == Approx(80.0f));
  }
}

TEST_CASE("Brush blends between a solid color and a gradient", "[graphics]") {
  using Shape = GradientPosition::InterpolationShape;
  Brush solid = Brush::solid(0xffff0000);
  Brush vertical = Brush::vertical(0xffff0000, 0xff0000ff);
  Brush linear = Brush::linear(0xffff0000, 0xff0000ff, { 0.0f, 0.0f }, { 20.0f, 0.0f });

  REQUIRE(Brush::interpolate(solid, vertical, 1.0f).position().shape == Shape::Vertical);
  REQUIRE(Brush::interpolate(vertical, solid, 0.0f).position().shape == Shape::Vertical);
  GradientPosition halfway = Brush::interpolate(solid, linear, 0.5f).position();
  REQUIRE(halfway.shape == Shape::PointsLinear);
  REQUIRE(halfway.point2.x == Approx(20.0f));

  Canvas canvas;
  canvas.setWindowless(20, 20);
  canvas.setColor(Brush::interpolate(solid, vertical, 1.0f));
  canvas.fill(0, 0, 20, 20);
  const Screenshot& screenshot = canvas.takeScreenshot();
  REQUIRE(screenshot.sample(10, 1).hexRed() > 0xe0);
  REQUIRE(screenshot.sample(10, 18).hexBlue() > 0xe0);
}

TEST_CASE("Brush creation", "[graphics]") {
  SECTION("Solid brush") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Brush brush = Brush::solid(red);

    REQUIRE(brush.gradient().resolution() == 1);
    REQUIRE(brush.gradient().colors()[0] == red);
    REQUIRE(brush.position().shape == GradientPosition::InterpolationShape::Solid);
  }

  SECTION("Horizontal brush") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);
    Brush brush = Brush::horizontal(red, blue);

    REQUIRE(brush.gradient().resolution() == 2);
    REQUIRE(brush.gradient().colors()[0] == red);
    REQUIRE(brush.gradient().colors()[1] == blue);
    REQUIRE(brush.position().shape == GradientPosition::InterpolationShape::Horizontal);
  }

  SECTION("Vertical brush") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);
    Brush brush = Brush::vertical(red, blue);

    REQUIRE(brush.gradient().resolution() == 2);
    REQUIRE(brush.gradient().colors()[0] == red);
    REQUIRE(brush.gradient().colors()[1] == blue);
    REQUIRE(brush.position().shape == GradientPosition::InterpolationShape::Vertical);
  }

  SECTION("Linear brush") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);
    Point from(10.0f, 20.0f);
    Point to(30.0f, 40.0f);

    Brush brush = Brush::linear(red, blue, from, to);

    REQUIRE(brush.gradient().resolution() == 2);
    REQUIRE(brush.gradient().colors()[0] == red);
    REQUIRE(brush.gradient().colors()[1] == blue);
    REQUIRE(brush.position().shape == GradientPosition::InterpolationShape::PointsLinear);
    REQUIRE(brush.position().point1 == from);
    REQUIRE(brush.position().point2 == to);
  }
}

TEST_CASE("Brush operations", "[graphics]") {
  SECTION("Interpolation") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Brush brush1 = Brush::solid(red);

    Color blue(1.0f, 0.0f, 0.0f, 1.0f);
    Point from(10.0f, 20.0f);
    Point to(30.0f, 40.0f);
    Brush brush2 = Brush::linear(blue, blue, from, to);

    Brush result = Brush::interpolate(brush1, brush2, 0.5f);

    REQUIRE(result.gradient().resolution() == 2);
    REQUIRE(result.gradient().colors()[0].red() == Approx(0.5f));
    REQUIRE(result.gradient().colors()[0].blue() == Approx(0.5f));

    REQUIRE(result.position().shape == GradientPosition::InterpolationShape::PointsLinear);
    REQUIRE(result.position().point1 == from);
    REQUIRE(result.position().point2 == to);
  }

  SECTION("Alpha multiplication") {
    Color red(0.8f, 1.0f, 0.0f, 0.0f);
    Brush brush = Brush::solid(red);

    Brush result = brush.withMultipliedAlpha(0.5f);

    REQUIRE(result.gradient().colors()[0].alpha() == Approx(0.4f));
  }

  SECTION("Serialization") {
    Color red(1.0f, 1.0f, 0.0f, 0.0f);
    Color blue(1.0f, 0.0f, 0.0f, 1.0f);
    Point from(10.0f, 20.0f);
    Point to(30.0f, 40.0f);

    Brush original = Brush::linear(red, blue, from, to);

    std::string encoded = original.encode();

    Brush decoded;
    decoded.decode(encoded);

    REQUIRE(decoded.gradient().resolution() == original.gradient().resolution());
    REQUIRE(decoded.gradient().colors()[0] == original.gradient().colors()[0]);
    REQUIRE(decoded.gradient().colors()[1] == original.gradient().colors()[1]);
    REQUIRE(decoded.position().shape == original.position().shape);
    REQUIRE(decoded.position().point1.x == Approx(original.position().point1.x));
    REQUIRE(decoded.position().point2.y == Approx(original.position().point2.y));

    original = Brush::radial(visage::Gradient(red, blue), from, 100.0f, 200.0f, to);
    encoded = original.encode();
    decoded.decode(encoded);

    REQUIRE(decoded.gradient().resolution() == original.gradient().resolution());
    REQUIRE(decoded.gradient().colors()[0] == original.gradient().colors()[0]);
    REQUIRE(decoded.gradient().colors()[1] == original.gradient().colors()[1]);
    REQUIRE(decoded.position().shape == original.position().shape);
    REQUIRE(decoded.position().point1.x == Approx(original.position().point1.x));
    REQUIRE(decoded.position().point2.y == Approx(original.position().point2.y));
    REQUIRE(decoded.position().coefficientx2 == Approx(original.position().coefficientx2).margin(0.0001f));
    REQUIRE(decoded.position().coefficienty2 == Approx(original.position().coefficienty2).margin(0.0001f));
    REQUIRE(decoded.position().coefficientxy == Approx(original.position().coefficientxy).margin(0.0001f));
  }

  SECTION("Radial Transform") {
    GradientPosition position = GradientPosition::radial(Point(50.0f, 50.0f), 1.0f, 2.0f);
    position = position.transformed(Transform::rotation(90.0f));
    REQUIRE(position.shape == GradientPosition::InterpolationShape::Radial);
    position = position.transformed(Transform::rotation(45.0f));
    REQUIRE(position.shape == GradientPosition::InterpolationShape::Radial);
    REQUIRE(position.point1.x == Approx(-50.0f * std::sqrt(2.0f)));
    REQUIRE(position.point1.y == Approx(0.0f).margin(0.001f));

    position = position.transformed(Transform::rotation(45.0f));
    position = position.transformed(Transform::scale(4.0f, 3.0f));

    GradientPosition end_position = GradientPosition::radial(Point(-200.0f, -150.0f), 4.0f, 6.0f);
    REQUIRE(position.point1.x == Approx(end_position.point1.x).margin(0.01f));
    REQUIRE(position.point1.y == Approx(end_position.point1.y).margin(0.01f));
    REQUIRE(position.point2.x == Approx(end_position.point2.x).margin(0.01f));
    REQUIRE(position.point2.y == Approx(end_position.point2.y).margin(0.01f));
    REQUIRE(position.coefficientx2 == Approx(end_position.coefficientx2).margin(0.01f));
    REQUIRE(position.coefficienty2 == Approx(end_position.coefficienty2).margin(0.01f));
    REQUIRE(position.coefficientxy == Approx(end_position.coefficientxy).margin(0.01f));
  }
}

TEST_CASE("Gradient atlas ignores leftover GPU memory", "[graphics]") {
  {
    Canvas previous;
    previous.setWindowless(256, 256);
    previous.setColor(0xff336699);
    previous.fill(0, 0, previous.width(), previous.height());
    previous.submit();
    previous.takeScreenshot();
  }

  Canvas canvas;
  canvas.setWindowless(64, 64);
  canvas.setColor(0xffff0000);
  canvas.fill(0, 0, canvas.width(), canvas.height());
  canvas.submit();
  const Screenshot& screenshot = canvas.takeScreenshot();

  for (int y = 0; y < screenshot.height(); y += 9) {
    for (int x = 0; x < screenshot.width(); x += 9)
      REQUIRE(screenshot.sample(x, y).toARGB() == 0xffff0000);
  }
}

TEST_CASE("Radial brushes keep their shape through encoding", "[graphics]") {
  Brush radial = Brush::radial(0xffff0000, 0xff0000ff, { 10.0f, 20.0f }, 5.0f, 8.0f,
                               { 12.0f, 21.0f }, 0.25f);
  Brush vertical = Brush::vertical(0xff00ff00, 0xffffffff);

  std::ostringstream stream;
  radial.encode(stream);
  vertical.encode(stream);

  Brush decoded_radial;
  Brush decoded_vertical;
  std::istringstream input(stream.str());
  decoded_radial.decode(input);
  decoded_vertical.decode(input);

  const GradientPosition& original = radial.position();
  const GradientPosition& decoded = decoded_radial.position();
  REQUIRE(decoded.shape == GradientPosition::InterpolationShape::Radial);
  REQUIRE(decoded.point1 == original.point1);
  REQUIRE(decoded.point2 == original.point2);
  REQUIRE(decoded.focal_radius == Approx(original.focal_radius));
  REQUIRE(decoded.coefficientx2 == Approx(original.coefficientx2));
  REQUIRE(decoded.coefficienty2 == Approx(original.coefficienty2));
  REQUIRE(decoded.coefficientxy == Approx(original.coefficientxy));
  REQUIRE(decoded_vertical.position().shape == GradientPosition::InterpolationShape::Vertical);
  REQUIRE(decoded_vertical.gradient().colors()[1] == vertical.gradient().colors()[1]);
}

TEST_CASE("Brushes saved before radial coefficients were encoded still decode", "[graphics]") {
  int radial = static_cast<int>(GradientPosition::InterpolationShape::Radial);
  std::string old_radial = Gradient(0xffff0000, 0xff0000ff).encode() + std::to_string(radial) +
                           "\n10\n20\n10\n20\n";
  Brush vertical = Brush::vertical(0xff00ff00, 0xffffffff);
  std::istringstream input(old_radial + vertical.encode());

  Brush decoded_radial;
  Brush decoded_vertical;
  decoded_radial.decode(input);
  decoded_vertical.decode(input);
  REQUIRE(decoded_vertical.position().shape == GradientPosition::InterpolationShape::Vertical);
  REQUIRE(decoded_vertical.gradient().colors()[1] == vertical.gradient().colors()[1]);
}
