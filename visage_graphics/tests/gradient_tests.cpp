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
    auto sampleFunc = [](float t) -> Color { return Color(1.0f, t, 0.0f, 0.0f); };

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
    REQUIRE(linear.point_from == from);
    REQUIRE(linear.point_to == to);
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
    REQUIRE(result.point_from.x == Approx(50.0f));
    REQUIRE(result.point_from.y == Approx(0.0f));
    REQUIRE(result.point_to.x == Approx(50.0f));
    REQUIRE(result.point_to.y == Approx(100.0f));
  }

  SECTION("Serialization") {
    Point from(10.0f, 20.0f);
    Point to(30.0f, 40.0f);
    GradientPosition original(from, to);

    std::string encoded = original.encode();

    GradientPosition decoded;
    decoded.decode(encoded);

    REQUIRE(decoded.shape == original.shape);
    REQUIRE(decoded.point_from.x == Approx(original.point_from.x));
    REQUIRE(decoded.point_from.y == Approx(original.point_from.y));
    REQUIRE(decoded.point_to.x == Approx(original.point_to.x));
    REQUIRE(decoded.point_to.y == Approx(original.point_to.y));
  }

  SECTION("Scaling") {
    Point from(10.0f, 20.0f);
    Point to(30.0f, 40.0f);
    GradientPosition original(from, to);

    GradientPosition scaled = original * 2.0f;

    REQUIRE(scaled.shape == original.shape);
    REQUIRE(scaled.point_from.x == Approx(20.0f));
    REQUIRE(scaled.point_from.y == Approx(40.0f));
    REQUIRE(scaled.point_to.x == Approx(60.0f));
    REQUIRE(scaled.point_to.y == Approx(80.0f));
  }
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
    REQUIRE(brush.position().point_from == from);
    REQUIRE(brush.position().point_to == to);
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

    REQUIRE(result.position().shape == GradientPosition::InterpolationShape::Solid);
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
    REQUIRE(decoded.position().point_from.x == Approx(original.position().point_from.x));
    REQUIRE(decoded.position().point_to.y == Approx(original.position().point_to.y));
  }
}