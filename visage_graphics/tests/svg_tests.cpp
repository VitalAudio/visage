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
#include "visage_graphics/svg.h"

#include <catch2/catch_test_macros.hpp>

using namespace visage;

static Svg parseSvg(const std::string& data) {
  Svg svg(reinterpret_cast<const unsigned char*>(data.data()), data.size());
  svg.setDimensions(10, 10, 1.0f);
  return svg;
}

static int numShapes(const SvgDrawable* drawable) {
  int count = drawable->command_list.empty() ? 0 : 1;
  for (const auto& child : drawable->children)
    count += numShapes(child.get());
  return count;
}

TEST_CASE("Svg use", "[graphics]") {
  Svg without_use = parseSvg(R"svg(<svg viewBox="0 0 10 10"><defs><rect id="r" width="10" height="10"/></defs><g><g></g></g></svg>)svg");
  Svg with_use = parseSvg(R"svg(<svg viewBox="0 0 10 10"><defs><rect id="r" width="10" height="10"/></defs><g><g><use href="#r"/></g></g></svg>)svg");
  REQUIRE(numShapes(with_use.drawable()) == numShapes(without_use.drawable()) + 1);
}

static const SvgDrawable* firstShape(const SvgDrawable* drawable) {
  if (!drawable->command_list.empty())
    return drawable;
  for (const auto& child : drawable->children) {
    if (const SvgDrawable* shape = firstShape(child.get()))
      return shape;
  }
  return nullptr;
}

TEST_CASE("Svg fill rule", "[graphics]") {
  Svg default_rule = parseSvg(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10"/></svg>)svg");
  REQUIRE(firstShape(default_rule.drawable())->path.fillRule() == Path::FillRule::NonZero);

  Svg even_odd = parseSvg(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" fill-rule="evenodd"/></svg>)svg");
  REQUIRE(firstShape(even_odd.drawable())->path.fillRule() == Path::FillRule::EvenOdd);
}

TEST_CASE("Svg malformed input", "[graphics]") {
  SECTION("Inset border radius with no x radius") {
    Svg svg = parseSvg(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" clip-path="inset(1px round / 2px)"/></svg>)svg");
    REQUIRE(svg.drawable());
  }

  SECTION("Polyline with no points") {
    Svg svg = parseSvg(R"svg(<svg viewBox="0 0 10 10"><polyline stroke="red"/></svg>)svg");
    REQUIRE(svg.drawable());
  }

  SECTION("Curve with non-finite transform") {
    Svg nan_scale = parseSvg(R"svg(<svg viewBox="0 0 10 10"><g transform="scale(nan)"><path d="M0 0C10 0 10 10 0 10"/></g></svg>)svg");
    REQUIRE(nan_scale.drawable());
    Svg inf_scale = parseSvg(R"svg(<svg viewBox="0 0 10 10"><g transform="scale(inf)"><path d="M0 0C10 0 10 10 0 10"/></g></svg>)svg");
    REQUIRE(inf_scale.drawable());
  }

  SECTION("Empty css selector") {
    Svg svg = parseSvg(R"svg(<svg viewBox="0 0 10 10"><style>a,>{fill:red}</style><rect width="10" height="10"/></svg>)svg");
    REQUIRE(svg.drawable());
  }

  SECTION("Use that references itself") {
    Svg svg = parseSvg(R"svg(<svg viewBox="0 0 10 10"><g id="a"><use href="#a"/></g></svg>)svg");
    REQUIRE(svg.drawable());
  }

  SECTION("Uses that reference each other") {
    Svg svg = parseSvg(R"svg(<svg viewBox="0 0 10 10"><g id="a"><use href="#b"/></g><g id="b"><use href="#a"/></g></svg>)svg");
    REQUIRE(svg.drawable());
  }
}

static Color renderSvgCenter(const std::string& data) {
  Svg svg(reinterpret_cast<const unsigned char*>(data.data()), data.size());
  svg.setDimensions(20, 20, 1.0f);
  Canvas canvas;
  canvas.setWindowless(20, 20);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, 20, 20);
  canvas.svg(svg, 0, 0);
  canvas.submit();
  return canvas.takeScreenshot().sample(10, 10);
}

TEST_CASE("Svg rendering", "[graphics]") {
  SECTION("Shapes are drawn") {
    Color color = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" fill="red"/></svg>)svg");
    REQUIRE(color.hexRed() == 0xff);
  }

  SECTION("Defs are not drawn") {
    Color color = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><defs><rect width="10" height="10" fill="red"/></defs></svg>)svg");
    REQUIRE(color.hexRed() == 0x00);
  }

  SECTION("Hidden group is not drawn") {
    Color color = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><g opacity="0"><rect width="10" height="10" fill="red"/></g></svg>)svg");
    REQUIRE(color.hexRed() == 0x00);
  }

  SECTION("CDATA style") {
    Color color = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><style><![CDATA[ .a { fill: red; } ]]></style><rect class="a" width="10" height="10"/></svg>)svg");
    REQUIRE(color.hexRed() == 0xff);
  }
}
