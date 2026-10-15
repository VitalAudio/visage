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

#include <catch2/catch_approx.hpp>
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

TEST_CASE("Svg comments containing tags", "[graphics]") {
  Svg comment = parseSvg(R"svg(<svg viewBox="0 0 10 10"><!-- a > b <rect width="10" height="10"/> --><rect x="2" width="4" height="4"/></svg>)svg");
  REQUIRE(numShapes(comment.drawable()) == 1);
  REQUIRE(comment.drawable()->post_bounding_box.x() == 2.0f);

  Svg description = parseSvg(R"svg(<svg viewBox="0 0 10 10"><desc>text <!-- <g opacity="0.5"><rect width="10" height="10"/></g> --> more</desc><rect x="2" width="4" height="4"/></svg>)svg");
  REQUIRE(numShapes(description.drawable()) == 1);
  REQUIRE(description.drawable()->post_bounding_box.width() == 4.0f);
}

TEST_CASE("Svg use", "[graphics]") {
  Svg without_use = parseSvg(R"svg(<svg viewBox="0 0 10 10"><defs><rect id="r" width="10" height="10"/></defs><g><g></g></g></svg>)svg");
  Svg with_use = parseSvg(R"svg(<svg viewBox="0 0 10 10"><defs><rect id="r" width="10" height="10"/></defs><g><g><use href="#r"/></g></g></svg>)svg");
  REQUIRE(numShapes(with_use.drawable()) == numShapes(without_use.drawable()) + 1);
}

TEST_CASE("Svg use references its ancestor", "[graphics]") {
  Svg svg = parseSvg(R"svg(<svg viewBox="0 0 10 10"><g id="g"><use href="#g" x="2"/><rect width="10" height="10"/></g></svg>)svg");
  REQUIRE(numShapes(svg.drawable()) == 1);
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

TEST_CASE("Svg transform origin", "[graphics]") {
  Svg rotated = parseSvg(R"svg(<svg viewBox="0 0 10 10"><rect x="5" width="5" height="2" transform="rotate(90)" transform-origin="5 5"/></svg>)svg");
  REQUIRE(rotated.drawable()->post_bounding_box.x() == Catch::Approx(8.0f));
  REQUIRE(rotated.drawable()->post_bounding_box.y() == Catch::Approx(5.0f));

  Svg keywords = parseSvg(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" transform="scale(0.5)" transform-origin="bottom right"/></svg>)svg");
  REQUIRE(keywords.drawable()->post_bounding_box.x() == 5.0f);
  REQUIRE(keywords.drawable()->post_bounding_box.y() == 5.0f);

  Svg single = parseSvg(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" transform="scale(0.5)" transform-origin="0"/></svg>)svg");
  REQUIRE(single.drawable()->post_bounding_box.x() == 0.0f);
  REQUIRE(single.drawable()->post_bounding_box.y() == 2.5f);
}

TEST_CASE("Svg nested viewports", "[graphics]") {
  Svg view_box = parseSvg(R"svg(<svg viewBox="0 0 10 10"><svg x="2" y="2" width="4" height="4" viewBox="0 0 2 2"><rect width="2" height="2"/></svg></svg>)svg");
  REQUIRE(view_box.drawable()->post_bounding_box.x() == 2.0f);
  REQUIRE(view_box.drawable()->post_bounding_box.width() == 4.0f);

  Svg percent = parseSvg(R"svg(<svg viewBox="0 0 10 10"><svg x="50%" width="50%" height="50%"><rect width="100%" height="100%"/></svg></svg>)svg");
  REQUIRE(percent.drawable()->post_bounding_box.x() == Catch::Approx(5.0f));
  REQUIRE(percent.drawable()->post_bounding_box.width() == Catch::Approx(5.0f));
  REQUIRE(percent.drawable()->post_bounding_box.height() == Catch::Approx(5.0f));
}

TEST_CASE("Svg use of symbol and svg viewports", "[graphics]") {
  Svg symbol = parseSvg(R"svg(<svg viewBox="0 0 10 10"><symbol id="s" viewBox="0 0 1 1"><rect width="1" height="1"/></symbol><use href="#s" width="4" height="4"/></svg>)svg");
  REQUIRE(symbol.drawable()->post_bounding_box.width() == 4.0f);

  Svg svg = parseSvg(R"svg(<svg viewBox="0 0 10 10"><defs><svg id="v" width="2" height="2" viewBox="0 0 1 1"><rect width="1" height="1"/></svg></defs><use href="#v" width="6" height="6"/></svg>)svg");
  REQUIRE(svg.drawable()->post_bounding_box.width() == 6.0f);
}

static bool hasMarkerReferences(const SvgDrawable* drawable) {
  if (drawable->state.marker_start || drawable->state.marker_mid || drawable->state.marker_end)
    return true;
  for (const auto& child : drawable->children) {
    if (hasMarkerReferences(child.get()))
      return true;
  }
  return false;
}

TEST_CASE("Svg keeps no marker references after parsing", "[graphics]") {
  Svg svg = parseSvg(R"svg(<svg viewBox="0 0 10 10"><marker id="m"><path d="M0 0L1 1" marker-end="url(#m)"/><rect width="2" height="1"/></marker><g marker-mid="url(#m)"><path d="M1 1L5 5L9 1" marker-start="url(#m)" marker-end="url(#m)"/></g></svg>)svg");
  REQUIRE(numShapes(svg.drawable()) > 1);
  REQUIRE_FALSE(hasMarkerReferences(svg.drawable()));
}

TEST_CASE("Svg marker orientation and units", "[graphics]") {
  Svg stroke_units = parseSvg(R"svg(<svg viewBox="0 0 10 10"><marker id="m"><rect width="2" height="1"/></marker><path d="M5 5 L5 9" stroke-width="2" marker-start="url(#m)"/></svg>)svg");
  REQUIRE(stroke_units.drawable()->post_bounding_box.width() == 4.0f);
  REQUIRE(stroke_units.drawable()->post_bounding_box.height() == 2.0f);

  Svg user_units = parseSvg(R"svg(<svg viewBox="0 0 10 10"><marker id="m" markerUnits="userSpaceOnUse"><rect width="2" height="1"/></marker><path d="M5 5 L5 9" stroke-width="2" marker-start="url(#m)"/></svg>)svg");
  REQUIRE(user_units.drawable()->post_bounding_box.width() == 2.0f);
  REQUIRE(user_units.drawable()->post_bounding_box.height() == 1.0f);

  for (const std::string& orient : { "0.25turn", "1.5707964rad", "100grad", "90deg" }) {
    INFO(orient);
    Svg rotated = parseSvg(R"svg(<svg viewBox="0 0 10 10"><marker id="m" orient=")svg" + orient +
                           R"svg("><rect width="2" height="1"/></marker><path d="M5 5 L5 9" marker-start="url(#m)"/></svg>)svg");
    REQUIRE(std::abs(rotated.drawable()->post_bounding_box.width() - 1.0f) < 0.001f);
    REQUIRE(std::abs(rotated.drawable()->post_bounding_box.height() - 2.0f) < 0.001f);
  }
}

TEST_CASE("Svg marker percentages use the viewport", "[graphics]") {
  Svg svg = parseSvg(R"svg(<svg viewBox="0 0 20 20"><marker id="m" viewBox="0 0 1 1" markerWidth="10%" markerHeight="10%"><rect width="1" height="1"/></marker><path d="M10 10 L10 18" marker-start="url(#m)"/></svg>)svg");
  REQUIRE(svg.drawable()->post_bounding_box.width() == 1.0f);
  REQUIRE(svg.drawable()->post_bounding_box.height() == 1.0f);
}

TEST_CASE("Svg marker reference point with viewBox", "[graphics]") {
  Svg svg = parseSvg(R"svg(<svg viewBox="0 0 10 10"><marker id="m" viewBox="2 2 4 4" markerWidth="2" markerHeight="2" refX="4" refY="4"><rect x="4" y="4" width="2" height="2"/></marker><path d="M5 5 L5 9" marker-start="url(#m)"/></svg>)svg");
  REQUIRE(svg.drawable()->post_bounding_box.x() == 5.0f);
  REQUIRE(svg.drawable()->post_bounding_box.y() == 5.0f);
  REQUIRE(svg.drawable()->post_bounding_box.width() == 1.0f);
}

TEST_CASE("Svg marker properties", "[graphics]") {
  Svg inherited = parseSvg(R"svg(<svg viewBox="0 0 10 10"><marker id="m"><rect width="2" height="1"/></marker><g marker-start="url(#m)"><path d="M5 5 L5 9"/></g></svg>)svg");
  REQUIRE(inherited.drawable()->post_bounding_box.width() == 2.0f);

  Svg attribute = parseSvg(R"svg(<svg viewBox="0 0 10 10"><marker id="m"><rect width="2" height="1"/></marker><path d="M5 5 L5 9" marker="url(#m)"/></svg>)svg");
  REQUIRE(attribute.drawable()->post_bounding_box.width() == 0.0f);

  Svg style = parseSvg(R"svg(<svg viewBox="0 0 10 10"><marker id="m"><rect width="2" height="1"/></marker><path d="M5 5 L5 9" style="marker: url(#m)"/></svg>)svg");
  REQUIRE(style.drawable()->post_bounding_box.width() == 2.0f);
}

TEST_CASE("Svg rect sizes and radii", "[graphics]") {
  Svg negative = parseSvg(R"svg(<svg viewBox="0 0 10 10"><rect width="-10" height="10"/><rect width="10" height="-10"/></svg>)svg");
  REQUIRE(numShapes(negative.drawable()) == 0);

  Svg auto_radius = parseSvg(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" rx="-2" ry="3"/></svg>)svg");
  const auto& corner = firstShape(auto_radius.drawable())->command_list[2];
  REQUIRE(corner.type == 'A');
  REQUIRE(corner.control1 == Point(3.0f, 3.0f));
}

TEST_CASE("Svg ellipse radii", "[graphics]") {
  Svg negative = parseSvg(R"svg(<svg viewBox="0 0 10 10"><circle cx="5" cy="5" r="-2"/><ellipse cx="5" cy="5" rx="-2" ry="-3"/></svg>)svg");
  REQUIRE(numShapes(negative.drawable()) == 0);

  Svg auto_radius = parseSvg(R"svg(<svg viewBox="0 0 10 10"><ellipse cx="5" cy="5" ry="2"/></svg>)svg");
  REQUIRE(auto_radius.drawable()->post_bounding_box.width() == 4.0f);

  Svg markers = parseSvg(R"svg(<svg viewBox="0 0 10 10"><marker id="m"><rect width="1" height="1"/></marker><circle cx="5" cy="5" r="2" marker-mid="url(#m)"/></svg>)svg");
  REQUIRE(numShapes(markers.drawable()) == 5);
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

  SECTION("Symbols are only drawn through use") {
    Color hidden = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><symbol id="s"><rect width="10" height="10" fill="red"/></symbol></svg>)svg");
    REQUIRE(hidden.hexRed() == 0x00);
    Color used = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><symbol id="s"><rect width="10" height="10" fill="red"/></symbol><use href="#s"/></svg>)svg");
    REQUIRE(used.hexRed() == 0xff);
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

  SECTION("Context paint") {
    Color in_use = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><defs><rect id="r" width="10" height="10" fill="context-fill"/></defs><use href="#r" fill="red"/></svg>)svg");
    REQUIRE(in_use.hexRed() == 0xff);

    Color in_marker = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><marker id="m" overflow="visible"><path d="M-5 -5 L-4 -4" stroke="lime"/><rect x="-5" y="-5" width="10" height="10" fill="context-stroke"/></marker><path d="M5 5 L5 6" stroke="red" marker-start="url(#m)"/></svg>)svg");
    REQUIRE(in_marker.hexRed() == 0xff);
    REQUIRE(in_marker.hexGreen() == 0x00);
  }

  SECTION("Namespaces and unknown elements") {
    Color prefixed = renderSvgCenter(R"svg(<svg:svg xmlns:svg="http://www.w3.org/2000/svg" viewBox="0 0 10 10"><svg:rect width="10" height="10" fill="red"/></svg:svg>)svg");
    REQUIRE(prefixed.hexRed() == 0xff);

    Color foreign = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><g xmlns="http://example.org"><rect width="10" height="10" fill="red"/></g></svg>)svg");
    REQUIRE(foreign.hexRed() == 0x00);

    Color unknown = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><random><rect width="10" height="10" fill="red"/></random></svg>)svg");
    REQUIRE(unknown.hexRed() == 0x00);
  }

  SECTION("Root background color") {
    Color root = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10" style="background-color: red"></svg>)svg");
    REQUIRE(root.hexRed() == 0xff);

    Color element = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><rect width="1" height="1" style="background-color: red"/><svg><g/></svg></svg>)svg");
    REQUIRE(element.hexRed() == 0x00);
  }

  SECTION("Use references") {
    Color duplicate = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><defs><rect id="r" width="10" height="10" fill="red"/><rect id="r" width="10" height="10" fill="blue"/></defs><use href="#r"/></svg>)svg");
    REQUIRE(duplicate.hexRed() == 0xff);

    Color css = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><style>#r { fill: red; }</style><defs><rect id="r" width="10" height="10"/></defs><use href="#r"/></svg>)svg");
    REQUIRE(css.hexRed() == 0xff);
  }

  SECTION("Bounding box gradients on shapes without area") {
    Color removed = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><linearGradient id="g"><stop stop-color="red"/></linearGradient><line x1="0" y1="5" x2="10" y2="5" stroke="url(#g)" stroke-width="4"/></svg>)svg");
    REQUIRE(removed.hexRed() == 0x00);

    Color fallback = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><linearGradient id="g"><stop stop-color="blue"/></linearGradient><line x1="0" y1="5" x2="10" y2="5" stroke="url(#g) red" stroke-width="4"/></svg>)svg");
    REQUIRE(fallback.hexRed() == 0xff);
    REQUIRE(fallback.hexBlue() == 0x00);
  }

  SECTION("Color matrix filters") {
    Color filtered = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><filter id="f"><feColorMatrix values="0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0"/></filter><rect width="10" height="10" fill="blue" filter="url(#f)"/></svg>)svg");
    REQUIRE(filtered.hexRed() == 0xff);
    REQUIRE(filtered.hexBlue() == 0x00);
  }

  SECTION("Entity references") {
    Color attribute = renderSvgCenter(R"svg(<!DOCTYPE svg [<!ENTITY red "fill:red;">]><svg viewBox="0 0 10 10"><rect width="10" height="10" style="&red;"/></svg>)svg");
    REQUIRE(attribute.hexRed() == 0xff);

    Color element = renderSvgCenter(R"svg(<!DOCTYPE svg [<!ENTITY r "<rect width='10' height='10' fill='red'/>">]><svg viewBox="0 0 10 10"><g transform="translate(20 0)">&r;</g></svg>)svg");
    REQUIRE(element.hexRed() == 0x00);
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

    Color mixed = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" fill="rgb(255, 50%, 0)"/></svg>)svg");
    REQUIRE(mixed.hexRed() == 0x00);
    Color modern_mixed = renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10" fill="rgb(255 0% 0)"/></svg>)svg");
    REQUIRE(modern_mixed.hexRed() == 0xff);

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

TEST_CASE("Svg element transforms apply to curve resolution and stroke width", "[graphics]") {
  auto first_shape = [](const std::string& data) {
    Svg svg(reinterpret_cast<const unsigned char*>(data.data()), data.size());
    svg.setDimensions(100, 100, 1.0f);
    const SvgDrawable* drawable = svg.drawable();
    while (drawable->command_list.empty())
      drawable = drawable->children.front().get();
    return std::make_pair(drawable->path.numPoints(), drawable->stroke_path.boundingBox().height());
  };

  auto scaled_circle = first_shape(R"svg(<svg viewBox="0 0 100 100"><circle r="2" transform="translate(50 50) scale(20)"/></svg>)svg");
  auto large_circle = first_shape(R"svg(<svg viewBox="0 0 100 100"><circle cx="50" cy="50" r="40"/></svg>)svg");
  REQUIRE(scaled_circle.first == large_circle.first);

  auto own_scale = first_shape(R"svg(<svg viewBox="0 0 100 100"><path d="M0 0.5L1 0.5" stroke="red" stroke-width="0.05" transform="scale(100)"/></svg>)svg");
  auto parent_scale = first_shape(R"svg(<svg viewBox="0 0 100 100"><g transform="scale(100)"><path d="M0 0.5L1 0.5" stroke="red" stroke-width="0.05"/></g></svg>)svg");
  REQUIRE(own_scale.second == Catch::Approx(5.0f));
  REQUIRE(own_scale.second == Catch::Approx(parent_scale.second));
}

TEST_CASE("Svg length units", "[graphics]") {
  auto rect_width = [](const std::string& width) {
    Svg svg = parseSvg(R"svg(<svg viewBox="0 0 10 10"><rect width=")svg" + width +
                       R"svg(" height="10"/></svg>)svg");
    return svg.drawable()->post_bounding_box.width();
  };

  REQUIRE(rect_width("4") == Catch::Approx(4.0f));
  REQUIRE(rect_width(" 4px ") == Catch::Approx(4.0f));
  REQUIRE(rect_width("8Q") == Catch::Approx(96.0f / 25.4f * 2.0f));
  REQUIRE(rect_width("4mmx") == 0.0f);
}

TEST_CASE("Svg stray closing tags", "[graphics]") {
  Svg nested = parseSvg(R"svg(<svg viewBox="0 0 10 10"><g></g></g><rect width="10" height="10"/></svg>)svg");
  REQUIRE(numShapes(nested.drawable()) == 1);
  REQUIRE(nested.drawable()->post_bounding_box.width() == Catch::Approx(10.0f));

  Svg root = parseSvg(R"svg(<svg viewBox="0 0 10 10"><rect width="10" height="10"/></svg></svg>)svg");
  REQUIRE(root.drawable()->post_bounding_box.width() == Catch::Approx(10.0f));

  REQUIRE(renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><g></g></g><rect width="10" height="10" fill="red"/></svg>)svg")
              .hexRed() == 0xff);
}

TEST_CASE("Svg whitespace around attribute equals", "[graphics]") {
  Svg svg = parseSvg("<svg viewBox=\"0 0 10 10\"><rect x = \"2\" width\n=\n\"2\" height\t= "
                     "\"2\"/><rect x=\"6\" width=\"2\" height=\"2\"/></svg>");
  REQUIRE(numShapes(svg.drawable()) == 2);
  REQUIRE(svg.drawable()->post_bounding_box.x() == 2.0f);
  REQUIRE(svg.drawable()->post_bounding_box.width() == 6.0f);
}

TEST_CASE("Svg css combinators", "[graphics]") {
  auto red = [](const std::string& style, const std::string& body) {
    return renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><style>)svg" + style +
                           "{fill:red}</style>" + body + "</svg>")
               .hexRed() == 0xff;
  };

  std::string nested = R"svg(<g class="a"><g><rect width="10" height="10"/></g></g>)svg";
  REQUIRE(red("g rect", nested));
  REQUIRE(red(".a rect", nested));
  REQUIRE(red("svg .a\n\tg  >  rect", nested));
  REQUIRE(red("g > rect", nested));
  REQUIRE_FALSE(red(".a > rect", nested));
  REQUIRE_FALSE(red("svg > rect", nested));
  REQUIRE_FALSE(red("rect g", nested));
  REQUIRE_FALSE(red("g rect", R"svg(<rect width="10" height="10"/>)svg"));
}

TEST_CASE("Svg css cascade order", "[graphics]") {
  auto red = [](const std::string& style, const std::string& body) {
    return renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><style>)svg" + style + "</style>" + body + "</svg>")
               .hexRed() == 0xff;
  };

  REQUIRE(red(".a{fill:red}", R"svg(<rect class="a" width="10" height="10" fill="blue"/>)svg"));
  REQUIRE(red("g rect{fill:red}", R"svg(<g><rect width="10" height="10" fill="blue"/></g>)svg"));
  REQUIRE(red("rect{fill:blue}", R"svg(<rect width="10" height="10" style="fill:red"/>)svg"));
  REQUIRE(red("#r{fill:red} rect{fill:blue}", R"svg(<rect id="r" width="10" height="10"/>)svg"));
  REQUIRE(red(".a{fill:red} rect{fill:blue}", R"svg(<rect class="a" width="10" height="10"/>)svg"));
  REQUIRE(red("g rect{fill:red} rect{fill:blue}", R"svg(<g><rect width="10" height="10"/></g>)svg"));
  REQUIRE(red("rect{fill:blue} rect{fill:red}", R"svg(<rect width="10" height="10"/>)svg"));
  REQUIRE(red("g{fill:blue}", R"svg(<g><rect width="10" height="10" fill="red"/></g>)svg"));

  REQUIRE(renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><style type="text/other">rect{fill:blue}</style><rect width="10" height="10" fill="red"/></svg>)svg")
              .hexRed() == 0xff);
  REQUIRE(renderSvgCenter(R"svg(<svg viewBox="0 0 10 10"><style type="Text/CSS">rect{fill:red}</style><rect width="10" height="10" fill="blue"/></svg>)svg")
              .hexRed() == 0xff);
}

TEST_CASE("Svg attribute character references", "[graphics]") {
  Svg numeric = parseSvg(R"svg(<svg viewBox="0 0 10 10"><path d="M&#x20;1&#x20;1&#32;L&#32;9&#x9;9z" stroke="red"/></svg>)svg");
  REQUIRE(numeric.drawable()->post_bounding_box.width() == Catch::Approx(8.0f).margin(1.0f));
  REQUIRE(numeric.drawable()->post_bounding_box.height() == Catch::Approx(8.0f).margin(1.0f));

  Svg named = parseSvg(R"svg(<svg viewBox="0 0 10 10"><rect id="a&amp;b" x="&#50;" width="4" height="4"/></svg>)svg");
  REQUIRE(named.drawable()->post_bounding_box.x() == 2.0f);
}
