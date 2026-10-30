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

#include "visage_app/application_editor.h"
#include "visage_widgets/graph_line.h"

#include <catch2/catch_test_macros.hpp>
#include <functional>

using namespace visage;

static int redrawsFrom(const std::function<void(GraphLine&)>& change) {
  int redraws = 0;
  FrameEventHandler handler;
  handler.request_redraw = [&redraws](Frame*) { redraws++; };

  GraphLine graph_line(4);
  graph_line.setEventHandler(&handler);
  change(graph_line);
  graph_line.setEventHandler(nullptr);
  return redraws;
}

TEST_CASE("GraphLine setters that change its look request a redraw", "[widgets]") {
  REQUIRE(redrawsFrom([](GraphLine& line) { line.setFilled(true); }) == 1);
  REQUIRE(redrawsFrom([](GraphLine& line) { line.setFillCenter(GraphLine::kBottom); }) == 1);
  REQUIRE(redrawsFrom([](GraphLine& line) { line.setActive(false); }) == 1);
  REQUIRE(redrawsFrom([](GraphLine& line) { line.setFillAlphaMult(0.5f); }) == 1);
}

static Screenshot renderFilled(GraphLine::FillCenter fill_center) {
  ApplicationEditor editor;
  editor.onDraw() = [&editor](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, editor.width(), editor.height());
  };

  GraphLine graph_line(2);
  graph_line.set(0, 0.5f);
  graph_line.set(1, 0.5f);
  graph_line.setFilled(true);
  graph_line.setFillCenter(fill_center);
  editor.addChild(&graph_line);
  editor.onResize() += [&]() { graph_line.setBounds(editor.localBounds()); };
  editor.setWindowless(40, 40);
  return editor.takeScreenshot();
}

TEST_CASE("GraphLine fills toward the edge its fill center names", "[widgets]") {
  Screenshot bottom = renderFilled(GraphLine::kBottom);
  REQUIRE(bottom.sample(20, 5).hexRed() == 0);
  REQUIRE(bottom.sample(20, 35).hexRed() > 0x20);

  Screenshot top = renderFilled(GraphLine::kTop);
  REQUIRE(top.sample(20, 5).hexRed() > 0x20);
  REQUIRE(top.sample(20, 35).hexRed() == 0);
}
