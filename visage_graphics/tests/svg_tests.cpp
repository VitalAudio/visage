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

TEST_CASE("Svg bounding box excludes undrawn content", "[graphics]") {
  Svg svg = parseSvg(R"svg(<svg viewBox="0 0 10 10"><clipPath id="c"><rect x="-1000" width="5000" height="5000"/></clipPath><defs><rect x="-1000" width="5000" height="5000"/></defs><rect width="10" height="10"/></svg>)svg");
  REQUIRE(svg.drawable()->post_bounding_box.width() == 10.0f);
  REQUIRE(svg.drawable()->post_bounding_box.height() == 10.0f);
}

TEST_CASE("Svg translate with one argument", "[graphics]") {
  Svg svg = parseSvg(R"svg(<svg viewBox="0 0 10 10"><g transform="translate(5)"><rect width="5" height="10"/></g></svg>)svg");
  REQUIRE(svg.drawable()->post_bounding_box.x() == 5.0f);
  REQUIRE(svg.drawable()->post_bounding_box.y() == 0.0f);
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

  SECTION("Deeply nested groups") {
    std::string data = R"svg(<svg viewBox="0 0 10 10">)svg";
    for (int i = 0; i < 10000; ++i)
      data += "<g>";
    data += R"svg(<rect width="10" height="10"/>)svg";
    for (int i = 0; i < 10000; ++i)
      data += "</g>";
    data += "</svg>";
    Svg svg = parseSvg(data);
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

  SECTION("Clip paths are not drawn") {
    Color color = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><clipPath id="c"><rect width="10" height="10" fill="red"/></clipPath></svg>)svg");
    REQUIRE(color.hexRed() == 0x00);
  }

  SECTION("Display none hides visible children") {
    Color color = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><g display="none"><rect width="10" height="10" fill="red" visibility="visible"/></g></svg>)svg");
    REQUIRE(color.hexRed() == 0x00);
  }

  SECTION("Hidden group is not drawn") {
    Color color = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><g opacity="0"><rect width="10" height="10" fill="red"/></g></svg>)svg");
    REQUIRE(color.hexRed() == 0x00);
  }

  SECTION("Colors with alpha") {
    Color long_hex = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" fill="#ff000080"/></svg>)svg");
    REQUIRE(long_hex.hexRed() >= 0x7f);
    REQUIRE(long_hex.hexRed() <= 0x81);
    REQUIRE(long_hex.hexBlue() == 0x00);

    Color short_hex = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" fill="#f008"/></svg>)svg");
    REQUIRE(short_hex.hexRed() >= 0x87);
    REQUIRE(short_hex.hexRed() <= 0x89);
    REQUIRE(short_hex.hexBlue() == 0x00);
  }

  SECTION("Color values") {
    std::vector<std::string> red_fills = {
      R"svg(<g fill="red"><rect width="10" height="10" fill="inherit"/></g>)svg",
      R"svg(<g fill="red"><rect width="10" height="10" fill="#qqq"/></g>)svg",
      R"svg(<rect width="10" height="10" fill="rgb(255 0 0)"/>)svg",
      R"svg(<rect width="10" height="10" fill="url(#missing) red"/>)svg",
      R"svg(<rect width="10" height="10" fill="RED"/>)svg",
      R"svg(<rect width="10" height="10" fill="red icc-color(profile, 0.1)"/>)svg",
      R"svg(<rect width="10" height="10" fill="currentColor" color="red"/>)svg",
      R"svg(<linearGradient id="g"><stop stop-color="rgb(255, 0, 0)"/></linearGradient><rect width="10" height="10" fill="url(#g)"/>)svg",
    };
    for (const auto& fill : red_fills) {
      INFO(fill);
      Color color = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10">)svg" + fill + "</svg>");
      REQUIRE(color.hexRed() == 0xff);
      REQUIRE(color.hexGreen() == 0x00);
    }

    Color missing = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" fill="url(#missing)"/></svg>)svg");
    REQUIRE(missing.hexRed() == 0x00);
  }

  SECTION("Gradient stops") {
    std::vector<std::string> red_gradients = {
      R"svg(<linearGradient id="g"><stop offset="0.6" stop-color="red"/><stop offset="0" stop-color="blue"/></linearGradient>)svg",
      R"svg(<linearGradient id="g"><stop offset="5mm" stop-color="blue"/><stop stop-color="red"/></linearGradient>)svg",
      R"svg(<linearGradient id="g" stop-color="red"><stop stop-color="inherit"/></linearGradient>)svg",
      R"svg(<linearGradient id="g" color="red"><stop stop-color="currentColor"/></linearGradient>)svg",
      R"svg(<g color="red"><linearGradient id="g"><stop stop-color="currentColor"/></linearGradient></g>)svg",
    };
    for (const auto& gradient : red_gradients) {
      INFO(gradient);
      Color color = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10">)svg" + gradient +
                                    R"svg(<rect width="10" height="10" fill="url(#g)"/></svg>)svg");
      REQUIRE(color.hexRed() == 0xff);
      REQUIRE(color.hexBlue() == 0x00);
    }
  }

  SECTION("Gradient references") {
    std::vector<std::string> red_gradients = {
      R"svg(<linearGradient id="a"><stop stop-color="red"/></linearGradient><linearGradient id="b" href="#a"/><linearGradient id="g" xlink:href="#b"/>)svg",
      R"svg(<linearGradient id="a"><stop stop-color="red"/></linearGradient><linearGradient id="b"><stop stop-color="blue"/></linearGradient><linearGradient id="g" href="#a" xlink:href="#b"/>)svg",
      R"svg(<linearGradient id="a" color="red"><stop stop-color="currentColor"/></linearGradient><linearGradient id="g" href="#a" color="blue"/>)svg",
      R"svg(<g color="red"><linearGradient id="a"><stop stop-color="currentColor"/></linearGradient></g><g color="blue"><linearGradient id="g" href="#a"/></g>)svg",
      R"svg(<linearGradient id="a" stop-color="red"><stop stop-color="inherit"/></linearGradient><linearGradient id="g" href="#a" stop-color="blue"/>)svg",
    };
    for (const auto& gradient : red_gradients) {
      INFO(gradient);
      Color color = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10">)svg" + gradient +
                                    R"svg(<rect width="10" height="10" fill="url(#g)"/></svg>)svg");
      REQUIRE(color.hexRed() == 0xff);
      REQUIRE(color.hexBlue() == 0x00);
    }

    Color not_inherited = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><linearGradient id="a" stop-color="blue"/><linearGradient id="g" href="#a"><stop stop-color="inherit"/></linearGradient><rect width="10" height="10" fill="url(#g)"/></svg>)svg");
    REQUIRE(not_inherited.hexBlue() == 0x00);
  }

  SECTION("User space gradients are relative to the view box") {
    std::vector<std::string> red_gradients = {
      R"svg(<linearGradient id="g" gradientUnits="userSpaceOnUse"><stop offset="0.9" stop-color="red"/><stop offset="0.9" stop-color="blue"/></linearGradient>)svg",
      R"svg(<linearGradient id="g" gradientUnits="userSpaceOnUse" x1="60%" x2="100%"><stop offset="0.9" stop-color="red"/><stop offset="0.9" stop-color="blue"/></linearGradient>)svg",
      R"svg(<radialGradient id="g" gradientUnits="userSpaceOnUse" cx="0" cy="0" r="100%"><stop offset="0.9" stop-color="red"/><stop offset="0.9" stop-color="blue"/></radialGradient>)svg",
    };
    for (const auto& gradient : red_gradients) {
      INFO(gradient);
      Color color = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10">)svg" + gradient +
                                    R"svg(<rect width="10" height="10" fill="url(#g)"/></svg>)svg");
      REQUIRE(color.hexRed() == 0xff);
      REQUIRE(color.hexBlue() == 0x00);
    }
  }

  SECTION("Gradient stops use canvas current color") {
    std::string data = R"svg(<svg viewBox="0 0 10 10"><linearGradient id="g"><stop stop-color="currentColor"/></linearGradient><rect width="10" height="10" fill="url(#g)"/></svg>)svg";
    Svg svg(reinterpret_cast<const unsigned char*>(data.data()), data.size());
    svg.setDimensions(20, 20, 1.0f);
    Canvas canvas;
    canvas.setWindowless(20, 20);
    canvas.setColor(0xffffffff);
    canvas.fill(0, 0, 20, 20);
    canvas.setColor(0xffff0000);
    canvas.svg(svg, 0, 0);
    canvas.submit();
    Color color = canvas.takeScreenshot().sample(10, 10);
    REQUIRE(color.hexRed() == 0xff);
    REQUIRE(color.hexGreen() == 0x00);
  }

  SECTION("Hsl colors") {
    Color green = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" fill="hsl(120, 100%, 50%)"/></svg>)svg");
    REQUIRE(green.hexRed() == 0x00);
    REQUIRE(green.hexGreen() == 0xff);
    REQUIRE(green.hexBlue() == 0x00);

    Color pale = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" fill="hsl(240, 100%, 75%)"/></svg>)svg");
    REQUIRE(pale.hexRed() >= 0x7f);
    REQUIRE(pale.hexRed() <= 0x81);
    REQUIRE(pale.hexBlue() == 0xff);
  }

  SECTION("Polygon with offset view box") {
    Color color = renderSvgCenter(R"svg(<svg viewBox="10 10 10 10"><polygon points="10,10 20,10 20,20 10,20" fill="red"/></svg>)svg");
    REQUIRE(color.hexRed() == 0xff);
  }

  SECTION("CDATA style") {
    Color color = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><style><![CDATA[ .a { fill: red; } ]]></style><rect class="a" width="10" height="10"/></svg>)svg");
    REQUIRE(color.hexRed() == 0xff);
  }
}
