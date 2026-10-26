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

#include "test_helpers.h"

#include <filesystem>
#include <visage_utils/child_process.h>
#include <visage_utils/file_system.h>

#if VISAGE_EMSCRIPTEN
#include <emscripten.h>
#endif

namespace manual {
  namespace {
    struct SvgCase {
      std::string name;
      std::string expected;
      std::string svg;
    };

    const char* kHeader = R"svg(<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" )svg"
                          R"svg(viewBox="0 0 100 100" width="100" height="100">)svg";

    std::string svg(const std::string& body) {
      return std::string(kHeader) + body + "</svg>";
    }

    std::string checker(float y = 0, float height = 100) {
      std::string result = R"svg(<rect x="0" y=")svg" + std::to_string(y) +
                           R"svg(" width="100" height=")svg" + std::to_string(height) +
                           R"svg(" fill="#bbb"/>)svg";
      for (int row = 0; row * 5 < height; ++row) {
        for (int column = row % 2; column < 20; column += 2) {
          result += R"svg(<rect x=")svg" + std::to_string(column * 5) + R"svg(" y=")svg" +
                    std::to_string(y + row * 5) + R"svg(" width="5" height="5" fill="#fff"/>)svg";
        }
      }
      return result;
    }

    std::string deepNesting() {
      std::string body;
      for (int i = 0; i < 200; ++i)
        body += "<g>";
      body += R"svg(<rect x="5" y="5" width="40" height="40" fill="green"/>)svg";
      for (int i = 0; i < 200; ++i)
        body += "</g>";
      body += R"svg(<g id="loop"><use href="#loop"/></g>)svg";
      body += R"svg(<use id="a" href="#b"/><use id="b" href="#a"/>)svg";
      body += R"svg(<rect x="55" y="55" width="40" height="40" fill="green"/>)svg";
      return svg(body);
    }

    std::vector<SvgCase> paintCases() {
      return {
        { "Linear gradients",
          "Top left: red to blue, left to right. Top right: the same stops through an href chain, "
          "top to "
          "bottom. Bottom: yellow until 80% then a hard edge to green (out-of-order stops).",
          svg(R"svg(<defs>
  <linearGradient id="base"><stop offset="0" stop-color="red"/><stop offset="1" stop-color="blue"/></linearGradient>
  <linearGradient id="middle" href="#base"/>
  <linearGradient id="chained" href="#middle" x1="0" y1="0" x2="0" y2="1"/>
  <linearGradient id="user" gradientUnits="userSpaceOnUse" x1="0%" y1="0" x2="100%" y2="0">
    <stop offset="0.8" stop-color="yellow"/><stop offset="0.2" stop-color="green"/></linearGradient>
</defs>
<rect x="5" y="5" width="40" height="40" fill="url(#base)"/>
<rect x="55" y="5" width="40" height="40" fill="url(#chained)"/>
<rect x="5" y="55" width="90" height="40" fill="url(#user)"/>)svg") },
        { "Radial gradients",
          "Left: a sphere lit from the top left (focal point). Right: orange and purple rings "
          "reflecting "
          "outward from the center.",
          svg(R"svg(<defs>
  <radialGradient id="r" cx="50%" cy="50%" r="50%" fx="30%" fy="30%">
    <stop offset="0" stop-color="white"/><stop offset="1" stop-color="#204080"/></radialGradient>
  <radialGradient id="rep" r="20%" spreadMethod="reflect">
    <stop offset="0" stop-color="orange"/><stop offset="1" stop-color="purple"/></radialGradient>
</defs>
<circle cx="27" cy="50" r="24" fill="url(#r)"/>
<rect x="55" y="5" width="40" height="90" fill="url(#rep)"/>)svg") },
        { "Color syntaxes",
          "Top row: six identical orange squares. Middle row: three identical half-transparent "
          "orange squares "
          "over a checkerboard. Bottom: green (currentColor), blue (inherit) and purple "
          "(fallback).",
          svg(checker(35, 30) + R"svg(
<rect x="2" y="5" width="14" height="25" fill="#f80"/>
<rect x="18" y="5" width="14" height="25" fill="#ff8800"/>
<rect x="34" y="5" width="14" height="25" fill="rgb(255,136,0)"/>
<rect x="50" y="5" width="14" height="25" fill="rgb(100% 53.3% 0%)"/>
<rect x="66" y="5" width="14" height="25" fill="hsl(32, 100%, 50%)"/>
<rect x="82" y="5" width="14" height="25" fill="#ff8800ff"/>
<rect x="5" y="38" width="25" height="24" fill="#ff880080"/>
<rect x="37" y="38" width="25" height="24" fill="rgba(255,136,0,0.5)"/>
<rect x="69" y="38" width="25" height="24" fill="hsla(32,100%,50%,0.5)"/>
<g color="green"><rect x="5" y="70" width="25" height="25" fill="currentColor"/></g>
<g fill="blue"><rect x="37" y="70" width="25" height="25" fill="inherit"/></g>
<rect x="69" y="70" width="25" height="25" fill="url(#missing) purple"/>)svg") },
        { "CSS styling", "Every shape is green. Each one has fill=\"red\" as an attribute that CSS rules must override.",
          svg(R"svg(<style type="TEXT/CSS"><![CDATA[
  .a { fill: green; }
  #b { fill: green; }
  g.parent rect { fill: green; }
  g.parent > circle { fill: green; }
  rect.attr { fill: green; }
]]></style>
<rect class="a" x="5" y="5" width="40" height="25" fill="red"/>
<rect id="b" x="55" y="5" width="40" height="25" fill="red"/>
<g class="parent"><g><rect x="5" y="38" width="40" height="25" fill="red"/></g>
  <circle cx="75" cy="50" r="12" fill="red"/></g>
<rect class="attr" x="5" y="70" width="90" height="25" fill="red"/>)svg") },
        { "Transforms",
          "Each green shape exactly covers its thin white outline: translate with one argument, "
          "rotate with "
          "transform-origin, rotate around a point and translate+scale.",
          svg(R"svg(
<rect x="5" y="5" width="20" height="20" fill="green" transform="translate(30)"/>
<rect x="15" y="50" width="20" height="20" fill="green" transform-origin="25 60" transform="rotate(45)"/>
<rect x="55" y="50" width="20" height="20" fill="green" transform="rotate(45 65 60)"/>
<rect x="0" y="0" width="10" height="5" fill="green" transform="translate(55 80) scale(4 3)"/>
<g fill="none" stroke="white" stroke-width="0.6">
  <rect x="35" y="5" width="20" height="20"/>
  <polygon points="25,45.86 39.14,60 25,74.14 10.86,60"/>
  <polygon points="65,45.86 79.14,60 65,74.14 50.86,60"/>
  <rect x="55" y="80" width="40" height="15"/>
</g>)svg") },
        { "use, symbol and nested svg",
          "Four green shapes: a circle from a symbol, a square cut from a circle by a nested svg "
          "viewport, a "
          "square from the first of two duplicate ids, and a circle from a custom xlink prefix. "
          "Nothing red.",
          svg(R"svg(
<symbol id="sym" viewBox="0 0 10 10"><circle cx="5" cy="5" r="5" fill="green"/></symbol>
<use href="#sym" x="5" y="5" width="40" height="40"/>
<svg x="55" y="5" width="40" height="40" viewBox="0 0 20 20"><circle cx="10" cy="10" r="14" fill="green"/></svg>
<defs><rect id="dup" width="40" height="40" fill="green"/><rect id="dup" width="40" height="40" fill="red"/></defs>
<use href="#dup" x="5" y="55"/>
<g xmlns:xl="http://www.w3.org/1999/xlink"><use xl:href="#sym" x="55" y="55" width="40" height="40"/></g>)svg") },
        { "Opacity",
          "Top left: red and blue squares at group opacity 0.5, the overlap is plain "
          "half-transparent blue. "
          "Top right: opacity=\"50%\". Bottom left: solid fill with a half-transparent stroke. "
          "Bottom right: "
          "opacity=\"2\" is fully opaque.",
          svg(checker() + R"svg(
<g opacity="0.5"><rect x="5" y="5" width="28" height="28" fill="red"/><rect x="17" y="17" width="28" height="28" fill="blue"/></g>
<rect x="55" y="5" width="40" height="40" fill="green" opacity="50%"/>
<rect x="10" y="60" width="30" height="30" fill="green" stroke="black" stroke-width="8" stroke-opacity="0.4"/>
<rect x="55" y="55" width="40" height="40" fill="green" opacity="2"/>)svg") },
        { "Strokes",
          "Top: dashes with butt, round and square caps. Middle: miter, round and bevel joins. "
          "Bottom: a circle "
          "scaled 3x horizontally has a stroke 3x thicker on the left and right.",
          svg(R"svg(
<g stroke="green" stroke-width="5" fill="none">
  <line x1="10" y1="8" x2="90" y2="8" stroke-dasharray="10 6"/>
  <line x1="10" y1="20" x2="90" y2="20" stroke-dasharray="10 8" stroke-linecap="round"/>
  <line x1="10" y1="32" x2="90" y2="32" stroke-dasharray="10 10" stroke-linecap="square" stroke-dashoffset="5"/>
  <polyline points="8,62 18,44 28,62" stroke-linejoin="miter"/>
  <polyline points="38,62 48,44 58,62" stroke-linejoin="round"/>
  <polyline points="68,62 78,44 88,62" stroke-linejoin="bevel"/>
  <circle cx="0" cy="0" r="8" stroke-width="2" transform="translate(50 82) scale(3 1)"/>
</g>)svg") },
        { "Path data",
          "Top left: a half circle from an arc with radii too small. Top right: a smooth S curve. "
          "Bottom left: "
          "a zigzag from exponents and implicit line commands. Bottom right: a rounded bump from "
          "compact arc "
          "flags.",
          svg(R"svg(
<path d="M5 30 A1 1 0 0 1 45 30" fill="none" stroke="green" stroke-width="4"/>
<path d="M55 30 C55 5 75 5 75 18 S95 30 95 5" fill="none" stroke="green" stroke-width="4"/>
<path d="M5 90 1e1 60 15 9e1 2e1 60 25 90 30 60 35 90 40 60 45 90" fill="none" stroke="green" stroke-width="3"/>
<path d="M55 90a20 20 0 0140 0z" fill="green"/>)svg") },
        { "Parsing: entities and namespaces",
          "Four green squares. The entity, the character reference, spaces around = and the svg: "
          "prefix all "
          "work. The comment and the unknown element draw nothing red.",
          R"svg(<?xml version="1.0"?>
<!DOCTYPE svg [ <!ENTITY green "#00a000"> ]>
)svg" + svg(R"svg(
<!-- comment with > inside <rect fill="red" width="100" height="100"/> -->
<rect x="5" y="5" width="40" height="40" fill="&green;"/>
<rect x="55" y="5" width="40" height="40" fill="&#x23;00a000"/>
<rect x = "5" y = "55" width = "40" height = "40" fill = "#00a000"/>
<svg:rect xmlns:svg="http://www.w3.org/2000/svg" x="55" y="55" width="40" height="40" fill="#00a000"/>
<unknown><rect width="100" height="100" fill="red"/></unknown>)svg") },
      };
    }

    std::vector<SvgCase> clipCases() {
      return {
        { "clipPath basics",
          "Top left: a green disk (square clipped by a circle). Top right: a hollow star "
          "(clip-rule evenodd). "
          "Bottom left: two separate disks (two clip children union). Bottom right: a disk whose "
          "clip is "
          "defined after it.",
          svg(R"svg(
<defs>
  <clipPath id="circle"><circle cx="25" cy="25" r="20"/></clipPath>
  <clipPath id="star"><path clip-rule="evenodd" d="M75 5 L87 43 L55 20 L95 20 L63 43 Z"/></clipPath>
  <clipPath id="two"><circle cx="15" cy="75" r="10"/><circle cx="35" cy="75" r="10"/></clipPath>
</defs>
<rect x="0" y="0" width="50" height="50" fill="green" clip-path="url(#circle)"/>
<rect x="50" y="0" width="50" height="50" fill="green" clip-path="url(#star)"/>
<rect x="0" y="50" width="50" height="50" fill="green" clip-path="url(#two)"/>
<rect x="50" y="50" width="50" height="50" fill="green" clip-path="url(#late)"/>
<clipPath id="late"><circle cx="75" cy="75" r="20"/></clipPath>)svg") },
        { "Clip units and nesting",
          "Top left: a disk from an objectBoundingBox clip. Top right: a disk with a thin diagonal "
          "gap (two "
          "rotated rects clipped by a circle that rotates with them). Bottom: a lens shape where "
          "two clip circles intersect (clipPath with its own "
          "clip-path).",
          svg(R"svg(
<defs>
  <clipPath id="bbox" clipPathUnits="objectBoundingBox"><circle cx="0.5" cy="0.5" r="0.5"/></clipPath>
  <clipPath id="local"><circle cx="0" cy="0" r="16"/></clipPath>
  <clipPath id="left"><circle cx="40" cy="75" r="22"/></clipPath>
  <clipPath id="both" clip-path="url(#left)"><circle cx="60" cy="75" r="22"/></clipPath>
</defs>
<rect x="5" y="5" width="40" height="40" fill="green" clip-path="url(#bbox)"/>
<g transform="translate(75 25) rotate(30)"><rect x="-20" y="-20" width="40" height="10" fill="green" clip-path="url(#local)"/>
  <rect x="-20" y="-6" width="40" height="26" fill="green" clip-path="url(#local)"/></g>
<rect x="0" y="50" width="100" height="50" fill="green" clip-path="url(#both)"/>)svg") },
        { "CSS clip-path shapes",
          "Top row: circle() and ellipse(). Bottom row: inset() with rounded corners and polygon() "
          "as a "
          "triangle.",
          svg(R"svg(
<rect x="5" y="5" width="40" height="40" fill="green" style="clip-path: circle(40%)"/>
<rect x="55" y="5" width="40" height="40" fill="green" style="clip-path: ellipse(50% 25%)"/>
<rect x="5" y="55" width="40" height="40" fill="green" style="clip-path: inset(5px 10px round 8px)"/>
<rect x="55" y="55" width="40" height="40" fill="green" style="clip-path: polygon(50% 0, 100% 100%, 0 100%)"/>)svg") },
        { "Masks",
          "Top: a green bar fading out left to right (luminance mask). Bottom left: a green disk "
          "(white circle "
          "luminance mask). Bottom right: a disk from an alpha mask.",
          svg(R"svg(
<defs>
  <linearGradient id="fade"><stop offset="0" stop-color="white"/><stop offset="1" stop-color="black"/></linearGradient>
  <mask id="m1"><rect x="0" y="0" width="100" height="45" fill="url(#fade)"/></mask>
  <mask id="m2"><circle cx="25" cy="75" r="20" fill="white"/></mask>
  <mask id="m3" mask-type="alpha"><circle cx="75" cy="75" r="20" fill="black"/></mask>
</defs>
<rect x="5" y="5" width="90" height="40" fill="green" mask="url(#m1)"/>
<rect x="0" y="50" width="50" height="50" fill="green" mask="url(#m2)"/>
<rect x="50" y="50" width="50" height="50" fill="green" style="mask-type: alpha" mask="url(#m3)"/>)svg") },
        { "Markers",
          "Top: arrows at the start, middle and end of a zigzag, all pointing along the line. "
          "Middle: start "
          "arrow reversed (auto-start-reverse). Bottom: arrows on an arc and a smooth curve follow "
          "the curve, "
          "and get bigger with the thicker stroke.",
          svg(R"svg(
<defs>
  <marker id="arrow" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="4" markerHeight="4" orient="auto">
    <path d="M0 0 L10 5 L0 10 z" fill="green"/></marker>
  <marker id="rev" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="4" markerHeight="4" orient="auto-start-reverse">
    <path d="M0 0 L10 5 L0 10 z" fill="green"/></marker>
</defs>
<polyline points="10,25 30,8 50,25 70,8 90,25" fill="none" stroke="gray" stroke-width="1.5"
  marker-start="url(#arrow)" marker-mid="url(#arrow)" marker-end="url(#arrow)"/>
<line x1="15" y1="45" x2="85" y2="45" stroke="gray" stroke-width="1.5" marker-start="url(#rev)" marker-end="url(#rev)"/>
<path d="M10 90 A20 20 0 0 1 45 70" fill="none" stroke="gray" stroke-width="3" marker-end="url(#arrow)"/>
<path d="M55 90 C60 60 70 60 75 75 S90 90 92 62" fill="none" stroke="gray" stroke-width="3"
  marker-start="url(#arrow)" marker-end="url(#arrow)"/>)svg") },
        { "context-stroke markers",
          "Each arrow head is the same color as its line: red, green and blue.", svg(R"svg(
<defs>
  <marker id="ctx" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="5" markerHeight="5" orient="auto">
    <path d="M0 0 L10 5 L0 10 z" fill="context-stroke"/></marker>
</defs>
<line x1="10" y1="20" x2="85" y2="20" stroke="red" stroke-width="2" marker-end="url(#ctx)"/>
<line x1="10" y1="50" x2="85" y2="50" stroke="green" stroke-width="2" marker-end="url(#ctx)"/>
<line x1="10" y1="80" x2="85" y2="80" stroke="blue" stroke-width="2" marker-end="url(#ctx)"/>)svg") },
        { "Hidden content",
          "Only one green square in the middle. Nothing red: display none (inherited), visibility "
          "hidden, defs, "
          "symbol, clip path contents and an element using an empty clip path are all hidden.",
          svg(R"svg(
<g display="none"><g><rect width="100" height="100" fill="red"/></g></g>
<rect width="100" height="100" fill="red" visibility="hidden"/>
<defs><rect width="100" height="100" fill="red"/></defs>
<symbol id="s"><rect width="100" height="100" fill="red"/></symbol>
<clipPath id="c"><rect width="100" height="100" fill="red"/></clipPath>
<clipPath id="empty"/>
<rect width="100" height="100" fill="red" clip-path="url(#empty)"/>
<rect x="30" y="30" width="40" height="40" fill="green"/>)svg") },
        { "Background and viewBox",
          "Dark blue background from the root's background-color. A green diamond centered in a "
          "viewBox that "
          "starts at -50,-50.",
          R"svg(<svg xmlns="http://www.w3.org/2000/svg" viewBox="-50 -50 100 100" width="100" height="100" )svg"
          R"svg(style="background-color: #203050">
<polygon points="0,-40 40,0 0,40 -40,0" fill="green"/></svg>)svg" },
        { "Deep nesting and loops",
          "Two green squares: one inside 200 nested groups and one after self-referencing and "
          "circular use "
          "elements. No crash or hang.",
          deepNesting() },
        { "Malformed markup",
          "The valid parts still draw: a green square top left and a green triangle from path data "
          "that ends "
          "with a broken command. Stray closing tags and unquoted attributes don't blank the image "
          "or crash. "
          "Browsers reject malformed XML, so skip the browser comparison here.",
          std::string(kHeader) + R"svg(
<rect x="5" y="5" width="40" height="40" fill="green"/>
</g></rect>
<rect x=55 y=5 width=40 height=40 fill=green/>
<path d="M55 95 L75 55 L95 95 Z M 10 10 L" fill="green"/>
<rect x="5" y="55" width="40" height="40" fill="green" stroke=>
</svg>)svg" },
      };
    }

    class SvgCell : public Box {
    public:
      explicit SvgCell(const SvgCase& svg_case) :
          Box(svg_case.name), description_(svg_case.expected, 11, kDimText) {
        description_.setMultiLine(true);
        addChild(&description_);
        frame_.load(reinterpret_cast<const uint8_t*>(svg_case.svg.data()), svg_case.svg.size());
        addChild(&frame_);
      }

      void resized() override {
        visage::Bounds content = contentBounds();
        float size = std::min(content.width(), content.height() - 70);
        visage::Bounds square(content.x() + (content.width() - size) * 0.5f, content.y(), size, size);
        frame_.setBounds(square);
        description_.setBounds(content.x(), square.bottom() + 6, content.width(),
                               content.bottom() - square.bottom() - 6);
      }

    private:
      Label description_;
      visage::SvgFrame frame_;
    };

    class SvgPage : public visage::Frame {
    public:
      explicit SvgPage(std::vector<SvgCase> cases) : cases_(std::move(cases)) {
        for (const SvgCase& svg_case : cases_) {
          cells_.push_back(std::make_unique<SvgCell>(svg_case));
          addChild(cells_.back().get());
        }
        export_ = makeButton("Export all SVGs and open them in a browser", [this] { exportSvgs(); });
        addChild(export_.get());
        addChild(&status_);
        status_.setMono(true);
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        visage::Bounds top = takeTop(bounds, 34);
        export_->setBounds(takeLeft(top, std::min(380.0f, top.width() * 0.5f)));
        status_.setBounds(top);
        auto cells = grid(bounds, 5, 2, 8);
        for (int i = 0; i < cells_.size(); ++i)
          cells_[i]->setBounds(cells[i]);
      }

    private:
      void exportSvgs() {
#if VISAGE_EMSCRIPTEN
        std::vector<SvgCase> web_cases = paintCases();
        std::vector<SvgCase> web_clips = clipCases();
        web_cases.insert(web_cases.end(), web_clips.begin(), web_clips.end());
        std::string web_html =
            "<!doctype html><meta charset=\"utf-8\"><title>Visage SVG tests</title>"
            "<style>body{font-family:sans-serif;background:#222;color:#ddd}"
            ".c{display:inline-block;width:240px;margin:8px;vertical-align:top}"
            "img{width:200px;height:200px;background:#333}</style>";
        for (int i = 0; i < web_cases.size(); ++i) {
          EM_ASM(
              {
                Module.manualSvgUrls = Module.manualSvgUrls || [];
                Module.manualSvgUrls[$0] = URL.createObjectURL(new Blob([UTF8ToString($1)], { type: 'image/svg+xml'}));
              },
              i, web_cases[i].svg.c_str());
          web_html += "<div class=c><b>" + web_cases[i].name + "</b><br><img src=\"@@SVG" +
                      std::to_string(i) + "@@\"><br><small>" + web_cases[i].expected + "</small></div>";
        }
        EM_ASM(
            {
              var html = UTF8ToString($0);
              for (var i = 0; i < Module.manualSvgUrls.length; ++i)
                html = html.split('@@SVG' + i + '@@').join(Module.manualSvgUrls[i]);
              window.open(URL.createObjectURL(new Blob([html], { type: 'text/html'})), '_blank');
            },
            web_html.c_str());
        status_.setText("Opened the SVGs in a new tab. Allow pop-ups for this page if nothing "
                        "opened.");
        return;
#endif
        visage::File directory = std::filesystem::temp_directory_path() / "visage_svg_tests";
        visage::createDirectory(directory);
        std::vector<SvgCase> all = paintCases();
        std::vector<SvgCase> clips = clipCases();
        all.insert(all.end(), clips.begin(), clips.end());

        std::string html = "<!doctype html><meta charset=\"utf-8\"><title>Visage SVG tests</title>"
                           "<style>body{font-family:sans-serif;background:#222;color:#ddd}"
                           ".c{display:inline-block;width:240px;margin:8px;vertical-align:top}"
                           "img{width:200px;height:200px;background:#333}</style>";
        for (int i = 0; i < all.size(); ++i) {
          std::string file_name = (i < 9 ? "0" : "") + std::to_string(i + 1) + ".svg";
          visage::replaceFileWithText(directory / file_name, all[i].svg);
          html += "<div class=c><b>" + all[i].name + "</b><br><img src=\"" + file_name +
                  "\"><br><small>" + all[i].expected + "</small></div>";
        }
        visage::File index = directory / "index.html";
        visage::replaceFileWithText(index, html);

        std::string output;
#if VISAGE_MAC
        visage::spawnChildProcess("/usr/bin/open", "\"" + pathString(index) + "\"", output, 5000);
#elif VISAGE_WINDOWS
        visage::spawnChildProcess("explorer.exe", "\"" + pathString(index) + "\"", output, 5000);
#else
        visage::spawnChildProcess("xdg-open", "\"" + pathString(index) + "\"", output, 5000);
#endif
        status_.setText(pathString(index));
      }

      std::vector<SvgCase> cases_;
      std::vector<std::unique_ptr<SvgCell>> cells_;
      std::unique_ptr<visage::UiButton> export_;
      Label status_ { "", 11, kDimText };
    };

    std::vector<std::string> cellChecks(const std::vector<SvgCase>& cases) {
      std::vector<std::string> checks;
      for (const SvgCase& svg_case : cases)
        checks.push_back(svg_case.name + " matches its description and the browser.");
      return checks;
    }
  }

  void addSvgTests(std::vector<TestDefinition>& tests) {
    std::vector<std::string> paint_checks = cellChecks(paintCases());
    paint_checks.push_back("\"Export all SVGs\" opens a browser page with all 20 SVGs and their "
                           "descriptions.");
    tests.push_back({ "svg_paint", "SVG", "SVG Paint and Structure",
                      "Each cell renders an SVG with SvgFrame and says what you should see. Export "
                      "opens the same "
                      "files in your browser so you can compare side by side.",
                      paint_checks, [] { return std::make_unique<SvgPage>(paintCases()); } });

    std::vector<std::string> clip_checks = cellChecks(clipCases());
    clip_checks
        .push_back("Leaving and returning to the SVG pages repeatedly doesn't slow down or leak.");
    tests.push_back({ "svg_clip", "SVG", "SVG Clips, Masks and Markers",
                      "Clipping, masking, markers, hidden content and malformed input. Compare "
                      "with the browser "
                      "using Export on either SVG page.",
                      clip_checks, [] { return std::make_unique<SvgPage>(clipCases()); } });
  }
}
