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

#include "visage/app.h"

#include <algorithm>
#include <bgfx/bgfx.h>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <embedded/test_shaders.h>
#include <visage/ui.h>
#include <visage/widgets.h>

using namespace visage;
using namespace Catch;

TEST_CASE("Screenshot solid color", "[integration]") {
  Canvas canvas;
  canvas.setWindowless(10, 5);
  canvas.setColor(0xffddaa88);
  canvas.fill(0, 0, canvas.width(), canvas.height());
  canvas.submit();

  canvas.takeScreenshot();
  const Screenshot& screenshot = canvas.screenshot();

  REQUIRE(screenshot.width() == 10);
  REQUIRE(screenshot.height() == 5);
  const uint8_t* data = screenshot.data();
  int i = 0;
  for (int y = 0; y < 5; ++y) {
    for (int x = 0; x < 10; ++x) {
      REQUIRE(data[i++] == 0xdd);
      REQUIRE(data[i++] == 0xaa);
      REQUIRE(data[i++] == 0x88);
      REQUIRE(data[i++] == 0xff);
    }
  }
}

TEST_CASE("Screenshot vertical gradient", "[integration]") {
  Color source = 0xff345678;
  Color destination = 0xff88aacc;
  ApplicationEditor editor;
  editor.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(Brush::vertical(source, destination));
    canvas.fill(0, 0, editor.width(), editor.height());
  };

  editor.setWindowless(10, 5);
  Screenshot screenshot = editor.takeScreenshot();
  REQUIRE(screenshot.width() == 10);
  REQUIRE(screenshot.height() == 5);
  uint8_t* data = screenshot.data();
  int i = 0;
  for (int y = 0; y < 5; ++y) {
    float t = y / 4.0f;
    Color sample = source.interpolateWith(destination, t);

    for (int x = 0; x < 10; ++x) {
      REQUIRE(data[i++] == sample.hexRed());
      REQUIRE(data[i++] == sample.hexGreen());
      REQUIRE(data[i++] == sample.hexBlue());
      REQUIRE(data[i++] == 0xff);
    }
  }
}

TEST_CASE("Screenshot horizontal gradient", "[integration]") {
  Color source = 0xff123456;
  Color destination = 0xff88aacc;
  ApplicationEditor editor;
  editor.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(Brush::horizontal(source, destination));
    canvas.fill(0, 0, editor.width(), editor.height());
  };

  editor.setWindowless(10, 5);
  Screenshot screenshot = editor.takeScreenshot();
  REQUIRE(screenshot.width() == 10);
  REQUIRE(screenshot.height() == 5);
  uint8_t* data = screenshot.data();
  for (int x = 0; x < 10; ++x) {
    float t = x / 9.0f;
    Color sample = source.interpolateWith(destination, t);

    for (int y = 0; y < 5; ++y) {
      int index = (y * 10 + x) * 4;
      REQUIRE(std::abs(data[index] - (int)sample.hexRed()) <= 1);
      REQUIRE(std::abs(data[index + 1] - (int)sample.hexGreen()) <= 1);
      REQUIRE(std::abs(data[index + 2] - (int)sample.hexBlue()) <= 1);
      REQUIRE(std::abs(data[index + 3] - 0xff) <= 1);
    }
  }
}

TEST_CASE("Animated graph lines", "[integration]") {
  Color source = 0xff123456;
  Color destination = 0xff88aacc;
  ApplicationEditor editor;
  editor.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(Brush::horizontal(source, destination));
    canvas.fill(0, 0, editor.width(), editor.height());
  };

  visage::GraphLine graph_line1(254);
  visage::GraphLine graph_line2(250);
  editor.addChild(&graph_line1);
  editor.addChild(&graph_line2);

  editor.onResize() += [&]() {
    graph_line1.setBounds(editor.localBounds());
    graph_line2.setBounds(editor.localBounds());
  };

  editor.setWindowless(100, 100);

  for (int i = 0; i < 90; ++i) {
    graph_line1.redraw();
    graph_line2.redraw();
    editor.drawWindow();
  }
}

TEST_CASE("Grandchild overlapping order", "[integration]") {
  ApplicationEditor editor;
  visage::Frame trigger;

  visage::Frame container;
  visage::Frame wrapper;
  visage::Frame bottom;
  visage::Frame top;

  editor.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff333333);
    canvas.fill();
  };

  trigger.setBounds(20, 20, 100, 100);
  trigger.onDraw() = [](visage::Canvas& c) {
    c.setColor(0xFFFF0000);
    c.fill();
  };

  container.setBounds(150, 20, 100, 100);
  wrapper.setBounds(0, 0, 100, 100);
  bottom.setBounds(0, 0, 100, 100);
  top.setBounds(0, 0, 100, 100);

  container.addChild(&wrapper);
  wrapper.addChild(&bottom);
  container.addChild(&top);

  bottom.onDraw() = [](visage::Canvas& c) {
    c.setColor(0xFF00FF00);
    c.roundedRectangle(0, 0, 100, 100, 5);
  };

  top.onDraw() = [](visage::Canvas& c) {
    c.setColor(0xFFFFFF00);
    c.roundedRectangle(0, 0, 100, 100, 5);
  };

  editor.addChild(&trigger);
  editor.addChild(&container);
  editor.setWindowless(300, 150);
  Screenshot screenshot = editor.takeScreenshot();
  uint8_t* data = screenshot.data();

  int y = 30;
  int x = 160;
  int index = (y * 300 + x) * 4;
  REQUIRE(data[index] == 0xff);
  REQUIRE(data[index + 1] == 0xff);
  REQUIRE(data[index + 2] == 0x00);
  REQUIRE(data[index + 3] == 0xff);
}

TEST_CASE("Add/remove child multiple times", "[integration]") {
  ApplicationEditor app;
  visage::Frame child;

  app.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };

  child.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };

  child.setBounds(0, 0, 50, 50);
  app.addChild(child);
  app.setWindowless(50, 50);

  for (int i = 0; i < 10; ++i) {
    app.drawWindow();
    Screenshot screenshot = app.takeScreenshot();
    uint8_t* data = screenshot.data();

    child.redraw();
    int index = 100;
    REQUIRE(data[index + 3] == 0xff);
    if (i % 2) {
      REQUIRE(data[index] == 0x00);
      REQUIRE(data[index + 1] == 0x00);
      REQUIRE(data[index + 2] == 0x00);
      app.addChild(child);
    }
    else {
      REQUIRE(data[index] == 0xff);
      REQUIRE(data[index + 1] == 0xff);
      REQUIRE(data[index + 2] == 0xff);
      app.removeChild(child);
    }
  }
}

TEST_CASE("Remove and re-add child with layer", "[integration]") {
  ApplicationEditor app;
  visage::Frame child;

  app.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };

  child.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };

  child.setBounds(0, 0, 50, 50);
  child.setAlphaTransparency(0.5f);
  app.addChild(child);
  app.setWindowless(50, 50);

  for (int i = 0; i < 6; ++i) {
    app.drawWindow();
    Screenshot screenshot = app.takeScreenshot();
    Color sample = screenshot.sample(10, 10);

    if (i % 2) {
      REQUIRE(sample.hexRed() == 0x00);
      app.addChild(child);
    }
    else {
      REQUIRE(sample.hexRed() >= 0x7f);
      REQUIRE(sample.hexRed() <= 0x80);
      app.removeChild(child);
      child.setBounds(0, 0, 20 + i, 20 + i);
    }
  }
}

TEST_CASE("Remove and re-add parent of child with layer", "[integration]") {
  ApplicationEditor app;
  visage::Frame parent;
  visage::Frame child;

  app.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };

  child.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };

  parent.setBounds(0, 0, 50, 50);
  child.setBounds(0, 0, 50, 50);
  child.setAlphaTransparency(0.5f);
  parent.addChild(child);
  app.addChild(parent);
  app.setWindowless(50, 50);

  for (int i = 0; i < 6; ++i) {
    app.drawWindow();
    Screenshot screenshot = app.takeScreenshot();
    Color sample = screenshot.sample(10, 10);

    if (i % 2) {
      REQUIRE(sample.hexRed() == 0x00);
      app.addChild(parent);
    }
    else {
      REQUIRE(sample.hexRed() >= 0x7f);
      REQUIRE(sample.hexRed() <= 0x80);
      app.removeChild(parent);
      child.setBounds(0, 0, 20 + i, 20 + i);
    }
  }
}

TEST_CASE("Overlap test - grouping with overlaps bug", "[integration]") {
  visage::ApplicationWindow app;

  visage::Frame trigger;
  visage::Frame container;
  visage::Frame wrapper;
  visage::Frame bottom;
  visage::Frame top;

  app.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff333333);
    canvas.fill(0, 0, app.width(), app.height());
  };

  trigger.setBounds(0, 0, 25, 25);
  trigger.onDraw() = [](visage::Canvas& c) {
    c.setColor(0xFFFF0000);
    c.fill();
  };

  container.setBounds(30, 0, 20, 25);
  wrapper.setBounds(0, 0, 20, 25);
  bottom.setBounds(0, 0, 20, 25);
  top.setBounds(0, 0, 20, 25);

  container.addChild(&wrapper);
  wrapper.addChild(&bottom);
  container.addChild(&top);

  bottom.onDraw() = [](visage::Canvas& c) {
    c.setColor(0xFF00FF00);
    c.fill();
  };

  top.onDraw() = [&top](visage::Canvas& c) {
    c.setColor(0xFFFF00FF);
    c.fill();
  };

  app.addChild(&trigger);
  app.addChild(&container);
  app.setWindowless(50, 50);

  Screenshot screenshot = app.takeScreenshot();
  uint8_t* data = screenshot.data();
  int index = (5 * 50 + 35) * 4;
  REQUIRE(data[index] == 0xff);
  REQUIRE(data[index + 1] == 0x00);
  REQUIRE(data[index + 2] == 0xff);
  REQUIRE(data[index + 3] == 0xff);
}

TEST_CASE("Overlap test - several overlapping siblings", "[integration]") {
  visage::ApplicationWindow app;

  visage::Frame one;
  visage::Frame two;
  visage::Frame three;

  one.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xffcf6944);
    canvas.fill();
  };

  two.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff8d9f8c);
    canvas.fill();
  };

  three.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };

  one.setBounds(0, 0, 50, 50);
  two.setBounds(0, 0, 50, 50);
  three.setBounds(0, 0, 50, 50);

  app.addChild(one);
  app.addChild(two);
  app.addChild(three);

  app.setWindowless(50, 50);

  Screenshot screenshot = app.takeScreenshot();
  uint8_t* data = screenshot.data();
  int index = 100;
  REQUIRE(data[index] == 0xff);
  REQUIRE(data[index + 1] == 0xff);
  REQUIRE(data[index + 2] == 0xff);
  REQUIRE(data[index + 3] == 0xff);
}

TEST_CASE("Overlap test - several overlapping children", "[integration]") {
  visage::ApplicationWindow app;

  visage::Frame parent;
  visage::Frame one;
  visage::Frame two;
  visage::Frame three;

  one.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xffcf6944);
    canvas.fill();
  };

  two.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff8d9f8c);
    canvas.fill();
  };

  three.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };

  parent.setBounds(0, 0, 50, 50);

  one.setBounds(0, 0, 50, 50);
  two.setBounds(0, 0, 50, 50);
  three.setBounds(0, 0, 50, 50);

  parent.addChild(one);
  parent.addChild(two);
  parent.addChild(three);

  app.addChild(parent);
  app.setWindowless(50, 50);

  Screenshot screenshot = app.takeScreenshot();
  uint8_t* data = screenshot.data();
  int index = 100;
  REQUIRE(data[index] == 0xff);
  REQUIRE(data[index + 1] == 0xff);
  REQUIRE(data[index + 2] == 0xff);
  REQUIRE(data[index + 3] == 0xff);
}

TEST_CASE("Overlap test - niece/nephew overlap order", "[integration]") {
  visage::ApplicationWindow app;

  visage::Frame parent;
  visage::Frame child;
  visage::Frame overlay;
  visage::Frame modal;

  parent.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };

  child.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff8d9f8c);
    canvas.fill();
  };

  overlay.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff222222);
    canvas.fill();
  };

  modal.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };

  parent.setBounds(0, 0, 50, 50);
  child.setBounds(0, 0, 50, 50);
  overlay.setBounds(0, 0, 50, 50);
  modal.setBounds(0, 0, 50, 50);

  parent.addChild(&child);
  parent.addChild(&overlay);

  app.addChild(&parent);
  app.addChild(&modal);

  app.setWindowless(50, 50);

  Screenshot screenshot = app.takeScreenshot();
  uint8_t* data = screenshot.data();
  int index = 100;
  REQUIRE(data[index] == 0xff);
  REQUIRE(data[index + 1] == 0xff);
  REQUIRE(data[index + 2] == 0xff);
  REQUIRE(data[index + 3] == 0xff);
}

TEST_CASE("Overlap test - niece/nephew overlap order 2", "[integration]") {
  visage::ApplicationWindow app;

  visage::Frame parent;
  visage::Frame child;
  visage::Frame overlay;
  visage::Frame modal;

  child.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff8d9f8c);
    canvas.fill();
  };

  overlay.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff222222);
    canvas.fill();
  };

  modal.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };

  parent.setBounds(0, 0, 50, 50);
  child.setBounds(0, 0, 50, 50);
  overlay.setBounds(0, 0, 50, 50);
  modal.setBounds(0, 0, 50, 50);

  parent.addChild(&child);
  parent.addChild(&overlay);

  app.addChild(&parent);
  app.addChild(&modal);

  app.setWindowless(50, 50);

  Screenshot screenshot = app.takeScreenshot();
  uint8_t* data = screenshot.data();
  int index = 100;
  REQUIRE(data[index] == 0xff);
  REQUIRE(data[index + 1] == 0xff);
  REQUIRE(data[index + 2] == 0xff);
  REQUIRE(data[index + 3] == 0xff);
}

TEST_CASE("Overlap test - non transitive overlap", "[integration]") {
  visage::ApplicationWindow app;

  visage::Frame one;
  visage::Frame two;
  visage::Frame three;

  one.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xffff0000);
    canvas.fill();
  };

  two.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff00ff00);
    canvas.fill();
  };

  three.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff0000ff);
    canvas.fill();
  };

  one.setBounds(0, 0, 20, 50);
  two.setBounds(10, 0, 30, 50);
  three.setBounds(30, 0, 20, 50);

  app.addChild(one);
  app.addChild(two);
  app.addChild(three);

  app.setWindowless(50, 50);

  Screenshot screenshot = app.takeScreenshot();
  uint8_t* data = screenshot.data();
  int y = 30;
  int index1 = (y * 50 + 5) * 4;
  REQUIRE(data[index1] == 0xff);
  REQUIRE(data[index1 + 1] == 0x00);
  REQUIRE(data[index1 + 2] == 0x00);
  REQUIRE(data[index1 + 3] == 0xff);

  int index2 = (y * 50 + 15) * 4;
  REQUIRE(data[index2] == 0x00);
  REQUIRE(data[index2 + 1] == 0xff);
  REQUIRE(data[index2 + 2] == 0x00);
  REQUIRE(data[index2 + 3] == 0xff);

  int index3 = (y * 50 + 35) * 4;
  REQUIRE(data[index3] == 0x00);
  REQUIRE(data[index3 + 1] == 0x00);
  REQUIRE(data[index3 + 2] == 0xff);
  REQUIRE(data[index3 + 3] == 0xff);
}

TEST_CASE("Overlap test - non transitive overlap after overlap", "[integration]") {
  visage::ApplicationWindow app;

  visage::Frame zero;
  visage::Frame one;
  visage::Frame two;
  visage::Frame three;

  zero.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };

  one.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xffff0000);
    canvas.fill();
  };

  two.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff00ff00);
    canvas.fill();
  };

  three.onDraw() = [&](visage::Canvas& canvas) {
    canvas.setColor(0xff0000ff);
    canvas.fill();
  };

  zero.setBounds(0, 0, 50, 50);
  one.setBounds(0, 0, 20, 50);
  two.setBounds(10, 0, 30, 50);
  three.setBounds(30, 0, 20, 50);

  app.addChild(zero);
  app.addChild(one);
  app.addChild(two);
  app.addChild(three);

  app.setWindowless(50, 50);

  Screenshot screenshot = app.takeScreenshot();
  uint8_t* data = screenshot.data();
  int y = 30;
  int index1 = (y * 50 + 5) * 4;
  REQUIRE(data[index1] == 0xff);
  REQUIRE(data[index1 + 1] == 0x00);
  REQUIRE(data[index1 + 2] == 0x00);
  REQUIRE(data[index1 + 3] == 0xff);

  int index2 = (y * 50 + 15) * 4;
  REQUIRE(data[index2] == 0x00);
  REQUIRE(data[index2 + 1] == 0xff);
  REQUIRE(data[index2 + 2] == 0x00);
  REQUIRE(data[index2 + 3] == 0xff);

  int index3 = (y * 50 + 35) * 4;
  REQUIRE(data[index3] == 0x00);
  REQUIRE(data[index3 + 1] == 0x00);
  REQUIRE(data[index3 + 2] == 0xff);
  REQUIRE(data[index3 + 3] == 0xff);
}
TEST_CASE("Redraw requested while drawing stale children", "[integration]") {
  static constexpr int kNumChildren = 16;
  ApplicationEditor editor;
  Frame first;
  Frame second;
  Frame children[kNumChildren];
  int draw_counts[kNumChildren] {};

  first.onDraw() = [&](Canvas& canvas) { second.redraw(); };
  second.onDraw() = [&](Canvas& canvas) {
    for (auto& child : children)
      child.redraw();
  };

  editor.addChild(first);
  editor.addChild(second);
  first.setBounds(0, 0, 10, 10);
  second.setBounds(10, 0, 10, 10);
  for (int i = 0; i < kNumChildren; ++i) {
    editor.addChild(children[i]);
    children[i].setBounds(i, 10, 1, 1);
    children[i].onDraw() = [&draw_counts, i](Canvas& canvas) { draw_counts[i]++; };
  }

  editor.setWindowless(20, 20);
  editor.drawWindow();
  std::fill(std::begin(draw_counts), std::end(draw_counts), 0);

  first.redraw();
  editor.drawWindow();
  for (int count : draw_counts)
    REQUIRE(count == 1);
}

TEST_CASE("Frame redrawing itself after being redrawn by another frame", "[integration]") {
  ApplicationEditor editor;
  Frame first;
  Frame second;
  bool animating = false;
  int draw_count = 0;

  first.onDraw() = [&](Canvas& canvas) {
    animating = true;
    second.redraw();
  };
  second.onDraw() = [&](Canvas& canvas) {
    draw_count++;
    if (animating)
      second.redraw();
  };

  editor.addChild(first);
  editor.addChild(second);
  first.setBounds(0, 0, 10, 10);
  second.setBounds(10, 0, 10, 10);
  editor.setWindowless(20, 10);
  editor.drawWindow();
  animating = false;
  editor.drawWindow();

  first.redraw();
  for (int i = 0; i < 3; ++i) {
    draw_count = 0;
    editor.drawWindow();
    REQUIRE(draw_count == 1);
  }
}

TEST_CASE("Translucent content in child with layer", "[integration]") {
  ApplicationEditor app;
  Frame child;
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  child.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0x80ffffff);
    canvas.fill();
  };

  child.setBounds(0, 0, 50, 50);
  child.setCached(true);
  app.addChild(child);
  app.setWindowless(50, 50);
  app.drawWindow();
  Screenshot screenshot = app.takeScreenshot();
  REQUIRE(screenshot.data()[0] >= 0x7f);
  REQUIRE(screenshot.data()[0] <= 0x81);
}

TEST_CASE("Frame draw mask", "[integration]") {
  ApplicationEditor app;
  Frame frame;
  Frame child;
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  child.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };
  frame.onDrawMask() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill(0, 0, frame.width() / 2, frame.height());
    canvas.setColor(0x80ffffff);
    canvas.fill(frame.width() / 2, 0, frame.width() / 4, frame.height());
  };

  frame.addChild(child);
  child.setBounds(0, 0, 80, 20);
  frame.setBounds(0, 0, 40, 20);
  app.addChild(frame);
  app.setWindowless(80, 20);
  app.drawWindow();
  Screenshot screenshot = app.takeScreenshot();
  REQUIRE(screenshot.sample(5, 10).hexRed() == 0xff);
  REQUIRE(screenshot.sample(25, 10).hexRed() >= 0x7f);
  REQUIRE(screenshot.sample(25, 10).hexRed() <= 0x81);
  REQUIRE(screenshot.sample(35, 10).hexRed() == 0x00);

  frame.setBounds(0, 0, 80, 20);
  app.drawWindow();
  REQUIRE(app.takeScreenshot().sample(35, 10).hexRed() == 0xff);
}

TEST_CASE("Frame mask types", "[integration]") {
  for (MaskType mask_type : { MaskType::Alpha, MaskType::Luminance }) {
    ApplicationEditor app;
    Frame frame;
    frame.onDraw() = [&](Canvas& canvas) {
      canvas.setColor(0xffffffff);
      canvas.fill();
    };
    frame.setMaskType(mask_type);
    frame.onDrawMask() = [&](Canvas& canvas) {
      canvas.setColor(0xffffffff);
      canvas.fill(0, 0, frame.width() / 2, frame.height());
      canvas.setColor(0xff000000);
      canvas.fill(frame.width() / 2, 0, frame.width() / 2, frame.height());
    };

    frame.setBounds(0, 0, 40, 20);
    app.addChild(frame);
    app.setWindowless(40, 20);
    app.drawWindow();
    Screenshot screenshot = app.takeScreenshot();
    REQUIRE(screenshot.sample(10, 10).hexRed() == 0xff);
    REQUIRE(screenshot.sample(30, 10).hexRed() == (mask_type == MaskType::Alpha ? 0xff : 0x00));
  }
}

TEST_CASE("Layer is cleared when it stops drawing", "[integration]") {
  ApplicationEditor app;
  Frame frame;
  bool draw = true;
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  frame.onDraw() = [&](Canvas& canvas) {
    if (draw) {
      canvas.setColor(0xffffffff);
      canvas.fill();
    }
  };

  frame.setBounds(0, 0, 20, 20);
  frame.setAlphaTransparency(0.5f);
  app.addChild(frame);
  app.setWindowless(20, 20);
  app.drawWindow();
  REQUIRE(app.takeScreenshot().sample(10, 10).hexRed() >= 0x7f);

  draw = false;
  frame.redraw();
  app.drawWindow();
  REQUIRE(app.takeScreenshot().sample(10, 10).hexRed() == 0x00);
}

TEST_CASE("Hiding a frame releases the layers of the frames inside it", "[integration]") {
  ApplicationEditor app;
  Frame parent;
  Frame child;
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  child.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };

  parent.setBounds(0, 0, 20, 20);
  child.setBounds(0, 0, 20, 20);
  child.setAlphaTransparency(0.5f);
  parent.addChild(child);
  app.addChild(parent);
  app.setWindowless(20, 20);
  app.drawWindow();
  REQUIRE(child.region()->needsLayer());

  parent.setVisible(false);
  app.drawWindow();
  REQUIRE_FALSE(child.region()->needsLayer());

  parent.setVisible(true);
  app.drawWindow();
  REQUIRE(child.region()->needsLayer());
  REQUIRE(app.takeScreenshot().sample(10, 10).hexRed() == Approx(0x80).margin(1));
}

TEST_CASE("Hiding the parent of a masked backdrop frame releases its mask group layer",
          "[integration]") {
  ApplicationEditor app;
  Frame parent;
  Frame glass;
  BlurPostEffect blur;
  blur.setBlurRadius(7.0f);
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  glass.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };
  glass.onDrawMask() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill(0, 0, glass.width() / 2, glass.height());
  };
  glass.setBackdropEffect(&blur);

  parent.setBounds(0, 0, 20, 20);
  glass.setBounds(0, 0, 20, 20);
  parent.addChild(glass);
  app.addChild(parent);
  app.setWindowless(20, 20);
  app.drawWindow();
  Region* mask_group = glass.region()->parent();
  REQUIRE(mask_group->needsLayer());
  REQUIRE(app.takeScreenshot().sample(5, 10).hexRed() == 0xff);

  parent.setVisible(false);
  app.drawWindow();
  REQUIRE_FALSE(mask_group->needsLayer());
  REQUIRE(app.takeScreenshot().sample(5, 10).hexRed() == 0x00);

  parent.setVisible(true);
  app.drawWindow();
  REQUIRE(mask_group->needsLayer());
  Screenshot screenshot = app.takeScreenshot();
  REQUIRE(screenshot.sample(5, 10).hexRed() == 0xff);
  REQUIRE(screenshot.sample(15, 10).hexRed() == 0x00);
}

TEST_CASE("Backdrop frame keeps its content over a redrawing sibling", "[integration]") {
  ApplicationEditor app;
  Frame background;
  Frame glass;
  BlurPostEffect blur;
  blur.setBlurRadius(10.0f);

  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  background.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
    canvas.setColor(0xff0000ff);
    canvas.circle(0, 0, 20);
    background.redraw();
  };
  glass.setBackdropEffect(&blur);
  glass.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xffff0000);
    canvas.fill(5, 5, 10, 10);
  };
  glass.onDrawMask() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };

  app.addChild(background);
  app.addChild(glass);
  background.setBounds(0, 0, 40, 40);
  glass.setBounds(10, 10, 20, 20);
  app.setWindowless(40, 40);
  for (int i = 0; i < 3; ++i) {
    app.drawWindow();
    REQUIRE(app.takeScreenshot().sample(20, 20).hexRed() == 0xff);
  }
}

TEST_CASE("Frame mask applies to on-top children added after it", "[integration]") {
  ApplicationEditor app;
  Frame frame;
  Frame added_on_top;
  Frame made_on_top;
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  frame.onDrawMask() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill(0, 0, frame.width() / 2, frame.height());
  };
  for (Frame* child : { &added_on_top, &made_on_top }) {
    child->onDraw() = [](Canvas& canvas) {
      canvas.setColor(0xffffffff);
      canvas.fill();
    };
  }

  added_on_top.setOnTop(true);
  frame.addChild(added_on_top);
  frame.addChild(made_on_top);
  made_on_top.setOnTop(true);
  added_on_top.setBounds(0, 0, 40, 10);
  made_on_top.setBounds(0, 10, 40, 10);
  frame.setBounds(0, 0, 40, 20);
  app.addChild(frame);
  app.setWindowless(40, 20);
  app.drawWindow();
  Screenshot screenshot = app.takeScreenshot();
  REQUIRE(screenshot.sample(10, 5).hexRed() == 0xff);
  REQUIRE(screenshot.sample(30, 5).hexRed() == 0x00);
  REQUIRE(screenshot.sample(10, 15).hexRed() == 0xff);
  REQUIRE(screenshot.sample(30, 15).hexRed() == 0x00);
}

TEST_CASE("Frame mask applies after its blur", "[integration]") {
  ApplicationEditor app;
  Frame frame;
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  frame.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };
  frame.onDrawMask() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill(0, 0, frame.width() / 2, frame.height());
  };

  frame.setBlurRadius(8.0f);
  frame.setBounds(0, 0, 80, 80);
  app.addChild(frame);
  app.setWindowless(80, 80);
  app.drawWindow();
  Screenshot screenshot = app.takeScreenshot();
  REQUIRE(screenshot.sample(38, 40).hexRed() >= 0xfd);
  REQUIRE(screenshot.sample(42, 40).hexRed() <= 0x02);
}

TEST_CASE("Frame mask clips its bloom glow", "[integration]") {
  auto render = [](int mask_width) {
    ApplicationEditor app;
    Frame frame;
    BloomPostEffect bloom;
    bloom.setBloomSize(16.0f);
    bloom.setBloomIntensity(0.6f);
    app.onDraw() = [&](Canvas& canvas) {
      canvas.setColor(0xff000000);
      canvas.fill();
    };
    frame.onDraw() = [&](Canvas& canvas) {
      canvas.setColor(Color(0xffff8040, 1.5f));
      canvas.fill(24, 16, 14, 32);
    };
    frame.onDrawMask() = [mask_width](Canvas& canvas) {
      canvas.setColor(0xffffffff);
      canvas.fill(0, 0, mask_width, 64);
    };

    frame.setPostEffect(&bloom);
    frame.setBounds(0, 0, 80, 64);
    app.addChild(frame);
    app.setWindowless(80, 64);
    app.drawWindow();
    return app.takeScreenshot();
  };

  Screenshot full = render(80);
  Screenshot masked = render(40);
  REQUIRE(full.sample(44, 32).hexRed() > 0x10);
  for (int x : { 41, 44, 50 })
    REQUIRE(masked.sample(x, 32).hexRed() == 0x00);
  for (int x : { 16, 20, 39 }) {
    INFO("x " << x);
    REQUIRE(masked.sample(x, 32).hexRed() == Approx(full.sample(x, 32).hexRed()).margin(2));
    REQUIRE(masked.sample(x, 32).hexGreen() == Approx(full.sample(x, 32).hexGreen()).margin(2));
  }
}

TEST_CASE("Masked frames blend their effect's masked output with their layer blend mode",
          "[integration]") {
  static constexpr int kWidth = 60;
  static constexpr int kHeight = 40;
  enum Effect {
    kBlur,
    kBloom,
    kShader
  };
  auto render = [](Effect effect, BlendMode blend_mode, int mask_width, bool backdrop, bool with_frame) {
    ApplicationEditor app;
    Frame frame;
    BloomPostEffect bloom;
    bloom.setBloomSize(8.0f);
    bloom.setBloomIntensity(0.3f);
    ShaderPostEffect opaque(test_shaders::vs_test_post_effect, test_shaders::fs_test_opaque);
    if (backdrop) {
      app.onDraw() = [](Canvas& canvas) {
        canvas.setColor(0xff304050);
        canvas.fill(0, 0, kWidth / 2, kHeight);
        canvas.setColor(0xffa0c0e0);
        canvas.fill(kWidth / 2, 0, kWidth / 2, kHeight);
      };
    }
    frame.onDraw() = [](Canvas& canvas) {
      canvas.setColor(0x80c06040);
      canvas.fill(0, 0, kWidth * 3 / 4, kHeight);
      canvas.setColor(0xff2040c0);
      canvas.fill(10, 10, 20, 20);
    };
    frame.onDrawMask() = [mask_width](Canvas& canvas) {
      canvas.setColor(0xffffffff);
      canvas.fill(0, 0, mask_width, kHeight);
    };

    if (effect == kBlur)
      frame.setBlurRadius(6.0f);
    else if (effect == kBloom)
      frame.setPostEffect(&bloom);
    else
      frame.setPostEffect(&opaque);
    frame.setLayerBlendMode(blend_mode);
    frame.setBounds(0, 0, kWidth, kHeight);
    if (with_frame)
      app.addChild(frame);
    app.setWindowless(kWidth, kHeight);
    app.drawWindow();
    return app.takeScreenshot();
  };

  auto blend = [](BlendMode blend_mode, float backdrop, float source, float source_alpha) {
    float result = source + backdrop * (1.0f - source_alpha);
    if (blend_mode == BlendMode::Add)
      result = source + backdrop;
    else if (blend_mode == BlendMode::Sub)
      result = backdrop - source;
    else if (blend_mode == BlendMode::Mult)
      result = source * backdrop + backdrop * (1.0f - source_alpha);
    return std::clamp(result, 0.0f, 1.0f);
  };

  Screenshot backdrop = render(kBlur, BlendMode::Alpha, kWidth, true, false);
  for (Effect effect : { kBlur, kBloom, kShader }) {
    Screenshot source = render(effect, BlendMode::Alpha, kWidth, false, true);
    for (BlendMode blend_mode : { BlendMode::Alpha, BlendMode::Add, BlendMode::Sub, BlendMode::Mult }) {
      Screenshot screenshot = render(effect, blend_mode, kWidth / 2, true, true);
      for (int y = 1; y < kHeight; y += 2) {
        for (int x = 1; x < kWidth; x += 2) {
          Color source_color = x < kWidth / 2 ? source.sample(x, y) : Color(0);
          Color background = backdrop.sample(x, y);
          Color result = screenshot.sample(x, y);
          float source_alpha = source_color.hexAlpha() / 255.0f;
          INFO("effect " << effect << " blend mode " << static_cast<int>(blend_mode) << " x " << x
                         << " y " << y);
          REQUIRE(result.hexRed() == Approx(255.0f * blend(blend_mode, background.hexRed() / 255.0f,
                                                           source_color.hexRed() / 255.0f, source_alpha))
                                         .margin(2));
          REQUIRE(result.hexGreen() == Approx(255.0f * blend(blend_mode, background.hexGreen() / 255.0f,
                                                             source_color.hexGreen() / 255.0f, source_alpha))
                                           .margin(2));
          REQUIRE(result.hexBlue() == Approx(255.0f * blend(blend_mode, background.hexBlue() / 255.0f,
                                                            source_color.hexBlue() / 255.0f, source_alpha))
                                          .margin(2));
        }
      }
    }
  }
}

TEST_CASE("Masking a frame keeps its place among its siblings", "[integration]") {
  ApplicationEditor app;
  Frame below;
  Frame above;
  below.onDraw() = [](Canvas& canvas) {
    canvas.setColor(0xffff0000);
    canvas.fill();
  };
  above.onDraw() = [](Canvas& canvas) {
    canvas.setColor(0xff0000ff);
    canvas.fill();
  };

  below.setBounds(0, 0, 30, 20);
  above.setBounds(20, 0, 20, 20);
  app.addChild(below);
  app.addChild(above);
  below.onDrawMask() = [](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };
  app.setWindowless(40, 20);
  app.drawWindow();
  Screenshot screenshot = app.takeScreenshot();
  REQUIRE(screenshot.sample(10, 10).hexRed() == 0xff);
  REQUIRE(screenshot.sample(25, 10).hexBlue() == 0xff);
  REQUIRE(screenshot.sample(25, 10).hexRed() == 0x00);
}

TEST_CASE("Luminance frame mask applies to a bloom frame's output", "[integration]") {
  ApplicationEditor app;
  Frame frame;
  BloomPostEffect bloom;
  bloom.setBloomIntensity(0.0f);
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  frame.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };
  frame.setMaskType(MaskType::Luminance);
  frame.onDrawMask() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill(0, 0, 20, 20);
    canvas.setColor(0xff808080);
    canvas.fill(20, 0, 20, 20);
    canvas.setColor(0xff000000);
    canvas.fill(40, 0, 20, 20);
  };

  frame.setPostEffect(&bloom);
  frame.setBounds(0, 0, 60, 20);
  app.addChild(frame);
  app.setWindowless(60, 20);
  app.drawWindow();
  Screenshot screenshot = app.takeScreenshot();
  REQUIRE(screenshot.sample(10, 10).hexRed() == Approx(0xff).margin(2));
  REQUIRE(screenshot.sample(30, 10).hexRed() == Approx(0x80).margin(2));
  REQUIRE(screenshot.sample(50, 10).hexRed() == 0x00);
}

TEST_CASE("Luminance frame mask keeps its strength next to a bloom frame", "[integration]") {
  auto render = [](bool bloom_neighbor) {
    ApplicationEditor app;
    Frame masked;
    Frame neighbor;
    BloomPostEffect bloom;
    bloom.setBloomIntensity(0.0f);
    app.onDraw() = [](Canvas& canvas) {
      canvas.setColor(0xff000000);
      canvas.fill();
    };
    masked.onDraw() = [](Canvas& canvas) {
      canvas.setColor(0xffffffff);
      canvas.fill();
    };
    masked.setMaskType(MaskType::Luminance);
    masked.onDrawMask() = [](Canvas& canvas) {
      canvas.setColor(0xffffffff);
      canvas.fill(0, 0, 20, 20);
      canvas.setColor(0xff808080);
      canvas.fill(20, 0, 20, 20);
    };
    neighbor.onDraw() = [](Canvas& canvas) {
      canvas.setColor(0xff000000);
      canvas.fill();
    };
    if (bloom_neighbor)
      neighbor.setPostEffect(&bloom);

    masked.setBounds(0, 0, 40, 20);
    neighbor.setBounds(60, 0, 20, 20);
    app.addChild(masked);
    app.addChild(neighbor);
    app.setWindowless(80, 20);
    app.drawWindow();
    return app.takeScreenshot();
  };

  Screenshot alone = render(false);
  Screenshot next_to_bloom = render(true);
  REQUIRE(alone.sample(10, 10).hexRed() == Approx(0xff).margin(2));
  REQUIRE(alone.sample(30, 10).hexRed() == Approx(0x80).margin(2));
  REQUIRE(next_to_bloom.sample(10, 10).hexRed() == Approx(0xff).margin(2));
  REQUIRE(next_to_bloom.sample(30, 10).hexRed() == Approx(0x80).margin(2));
}

TEST_CASE("Masked frames blur what's behind them with their own backdrop effect", "[integration]") {
  auto render = [](int mask_corner) {
    ApplicationEditor app;
    Frame glass;
    BlurPostEffect blur;
    blur.setBlurRadius(12.0f);
    app.onDraw() = [](Canvas& canvas) {
      canvas.setColor(0xffff0000);
      canvas.fill(0, 0, 40, 60);
      canvas.setColor(0xff0000ff);
      canvas.fill(40, 0, 40, 60);
    };
    glass.setBackdropEffect(&blur);
    glass.onDraw() = [](Canvas& canvas) {
      canvas.setColor(0x20ffffff);
      canvas.fill();
    };
    if (mask_corner >= 0) {
      glass.onDrawMask() = [&glass, mask_corner](Canvas& canvas) {
        canvas.setColor(0xffffffff);
        canvas.roundedRectangle(0, 0, glass.width(), glass.height(), mask_corner);
      };
    }

    app.addChild(glass);
    glass.setBounds(10, 10, 60, 40);
    app.setWindowless(80, 60);
    app.drawWindow();
    return app.takeScreenshot();
  };

  Screenshot unmasked = render(-1);
  REQUIRE(unmasked.sample(38, 30).hexRed() < 0xc0);
  REQUIRE(unmasked.sample(42, 30).hexRed() > 0x40);
  for (int mask_corner : { 0, 12 }) {
    Screenshot masked = render(mask_corner);
    for (int x : { 34, 38, 42, 46 }) {
      INFO("mask corner " << mask_corner << " x " << x);
      REQUIRE(masked.sample(x, 30).hexRed() == Approx(unmasked.sample(x, 30).hexRed()).margin(2));
      REQUIRE(masked.sample(x, 30).hexBlue() == Approx(unmasked.sample(x, 30).hexBlue()).margin(2));
    }
  }
}

TEST_CASE("A full mask keeps a frame's HDR output for an outer bloom", "[integration]") {
  static constexpr int kWidth = 96;
  static constexpr int kHeight = 64;
  enum Inner {
    kPlain,
    kBloom,
    kShader
  };
  auto render = [](Inner inner_kind, bool masked) {
    ApplicationEditor app;
    Frame outer;
    Frame inner;
    BloomPostEffect outer_bloom;
    outer_bloom.setBloomSize(10.0f);
    outer_bloom.setBloomIntensity(0.8f);
    BloomPostEffect inner_bloom;
    inner_bloom.setBloomSize(6.0f);
    inner_bloom.setBloomIntensity(0.5f);
    ShaderPostEffect shader(test_shaders::vs_test_post_effect, test_shaders::fs_test_opaque);
    app.onDraw() = [](Canvas& canvas) {
      for (int x = 0; x < kWidth; x += 8) {
        canvas.setColor(x % 16 ? 0xff304050 : 0xffa0c0e0);
        canvas.fill(x, 0, 8, kHeight);
      }
    };
    outer.onDraw() = [&outer](Canvas& canvas) {
      canvas.setColor(0xc0204080);
      canvas.fill(0, 0, outer.width() / 3, outer.height());
    };
    inner.onDraw() = [](Canvas& canvas) {
      canvas.setColor(Color(0xffff8040, 2.5f));
      canvas.fill(8, 8, 16, 12);
      canvas.setColor(0x80ffffff);
      canvas.fill(28, 8, 12, 20);
    };
    if (masked) {
      inner.onDrawMask() = [](Canvas& canvas) {
        canvas.setColor(0xffffffff);
        canvas.fill();
      };
    }

    outer.setPostEffect(&outer_bloom);
    if (inner_kind == kBloom)
      inner.setPostEffect(&inner_bloom);
    else if (inner_kind == kShader)
      inner.setPostEffect(&shader);
    app.addChild(outer);
    outer.setBounds(8, 4, 80, 56);
    outer.addChild(inner);
    inner.setBounds(16, 8, 48, 40);
    app.setWindowless(kWidth, kHeight);
    app.drawWindow();
    return app.takeScreenshot();
  };

  for (Inner inner_kind : { kPlain, kBloom, kShader }) {
    Screenshot unmasked = render(inner_kind, false);
    Screenshot masked = render(inner_kind, true);
    for (int y = 1; y < kHeight; y += 2) {
      for (int x = 1; x < kWidth; x += 2) {
        INFO("inner " << inner_kind << " x " << x << " y " << y);
        REQUIRE(masked.sample(x, y).hexRed() == Approx(unmasked.sample(x, y).hexRed()).margin(2));
        REQUIRE(masked.sample(x, y).hexGreen() == Approx(unmasked.sample(x, y).hexGreen()).margin(2));
        REQUIRE(masked.sample(x, y).hexBlue() == Approx(unmasked.sample(x, y).hexBlue()).margin(2));
      }
    }
  }
}

TEST_CASE("Backdrop inside a masked frame only blurs the frame's content", "[integration]") {
  ApplicationEditor app;
  Frame group;
  Frame glass;
  BlurPostEffect blur;
  blur.setBlurRadius(4.0f);

  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xffff0000);
    canvas.fill(0, 0, 32, 32);
    canvas.setColor(0xff0000ff);
    canvas.fill(32, 0, 32, 32);
  };
  group.onDrawMask() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };
  glass.setBackdropEffect(&blur);

  app.addChild(group);
  group.addChild(glass);
  group.setBounds(0, 0, 64, 32);
  glass.setBounds(16, 8, 32, 16);
  app.setWindowless(64, 32);
  app.drawWindow();
  Color edge = app.takeScreenshot().sample(31, 16);
  REQUIRE(edge.hexRed() > 0xf0);
  REQUIRE(edge.hexBlue() < 0x10);
}

TEST_CASE("Alpha transparency applies to children", "[integration]") {
  ApplicationEditor app;
  Frame parent;
  Frame child;
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  child.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };

  parent.setBounds(0, 0, 50, 50);
  child.setBounds(0, 0, 50, 50);
  parent.addChild(child);
  parent.setAlphaTransparency(0.5f);
  app.addChild(parent);
  app.setWindowless(50, 50);
  app.drawWindow();
  REQUIRE(app.takeScreenshot().data()[0] >= 0x7f);
  REQUIRE(app.takeScreenshot().data()[0] <= 0x81);

  parent.setAlphaTransparency(0.25f);
  app.drawWindow();
  REQUIRE(app.takeScreenshot().data()[0] >= 0x3f);
  REQUIRE(app.takeScreenshot().data()[0] <= 0x41);
}

TEST_CASE("Frame opacity applies after its post effect", "[integration]") {
  ApplicationEditor app;
  Frame frame;
  ShaderPostEffect opaque(test_shaders::vs_test_post_effect, test_shaders::fs_test_opaque);
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  frame.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xffffffff);
    canvas.fill();
  };

  frame.setBounds(0, 0, 20, 20);
  frame.setPostEffect(&opaque);
  frame.setAlphaTransparency(0.5f);
  app.addChild(frame);
  app.setWindowless(20, 20);
  app.drawWindow();
  REQUIRE(app.takeScreenshot().sample(10, 10).hexRed() == Approx(0x80).margin(1));

  frame.setAlphaTransparency(0.25f);
  app.drawWindow();
  REQUIRE(app.takeScreenshot().sample(10, 10).hexRed() == Approx(0x40).margin(1));

  frame.setAlphaTransparency(1.0f);
  app.drawWindow();
  REQUIRE(app.takeScreenshot().sample(10, 10).hexRed() == 0xff);

  frame.setAlphaTransparency(0.5f);
  app.drawWindow();
  REQUIRE(app.takeScreenshot().sample(10, 10).hexRed() == Approx(0x80).margin(1));

  frame.removePostEffect();
  app.drawWindow();
  REQUIRE(app.takeScreenshot().sample(10, 10).hexRed() == Approx(0x80).margin(1));

  frame.setPostEffect(&opaque);
  app.drawWindow();
  REQUIRE(app.takeScreenshot().sample(10, 10).hexRed() == Approx(0x80).margin(1));
}

TEST_CASE("Translucent frames keep their shader effect's own blend state", "[integration]") {
  ApplicationEditor app;
  Frame frame;
  ShaderPostEffect opaque(test_shaders::vs_test_post_effect, test_shaders::fs_test_opaque);
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff202020);
    canvas.fill();
  };
  frame.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff808080);
    canvas.fill();
  };

  frame.setBounds(0, 0, 20, 20);
  frame.setPostEffect(&opaque);
  frame.setAlphaTransparency(0.5f);
  app.addChild(frame);
  app.setWindowless(20, 20);
  app.drawWindow();
  REQUIRE(app.takeScreenshot().sample(10, 10).hexRed() == Approx(0x50).margin(1));

  opaque.setState(BlendMode::Add);
  frame.redraw();
  app.drawWindow();
  REQUIRE(app.takeScreenshot().sample(10, 10).hexRed() == Approx(0xa0).margin(1));

  opaque.setState(BlendMode::Alpha);
  frame.redraw();
  app.drawWindow();
  REQUIRE(app.takeScreenshot().sample(10, 10).hexRed() == Approx(0x50).margin(1));
}

TEST_CASE("Fading or masking a drawn bloom frame keeps its sibling's glow", "[integration]") {
  for (bool mask : { false, true }) {
    ApplicationEditor app;
    Frame a;
    Frame b;
    BloomPostEffect bloom_a;
    BloomPostEffect bloom_b;
    for (BloomPostEffect* bloom : { &bloom_a, &bloom_b }) {
      bloom->setBloomSize(8.0f);
      bloom->setBloomIntensity(0.4f);
    }
    app.onDraw() = [](Canvas& canvas) {
      canvas.setColor(0xff304050);
      canvas.fill();
    };
    auto draw = [](Canvas& canvas) {
      canvas.setColor(Color(0xffffe0a0, 2.5f));
      canvas.fill(20, 14, 12, 12);
    };
    a.onDraw() = draw;
    b.onDraw() = draw;
    a.setPostEffect(&bloom_a);
    b.setPostEffect(&bloom_b);
    a.setBounds(0, 0, 56, 40);
    b.setBounds(60, 0, 56, 40);
    app.addChild(a);
    app.addChild(b);
    app.setWindowless(120, 40);
    app.drawWindow();
    Screenshot before = app.takeScreenshot();

    if (mask) {
      a.onDrawMask() = [](Canvas& canvas) {
        canvas.setColor(0xffffffff);
        canvas.fill();
      };
    }
    else {
      a.setAlphaTransparency(0.5f);
      app.drawWindow();
      a.setAlphaTransparency(1.0f);
    }
    app.drawWindow();
    INFO("mask " << mask);
    REQUIRE(app.takeScreenshot().sample(94, 20).hexRed() == before.sample(94, 20).hexRed());
  }
}

TEST_CASE("Fading a drawn shader frame leaves its parent's layer alone", "[integration]") {
  ApplicationEditor app;
  Frame frame;
  ShaderPostEffect opaque(test_shaders::vs_test_post_effect, test_shaders::fs_test_opaque);
  app.onDraw() = [](Canvas& canvas) {
    canvas.setColor(0xff304050);
    canvas.fill();
  };
  frame.onDraw() = [](Canvas& canvas) {
    canvas.setColor(0xffc06040);
    canvas.fill();
  };
  frame.setPostEffect(&opaque);
  frame.setBounds(0, 0, 56, 40);
  app.addChild(frame);
  app.setWindowless(120, 40);
  app.drawWindow();

  Layer* layer = app.region()->layer();
  int width = layer->width();
  int height = layer->height();
  frame.setAlphaTransparency(0.5f);
  app.drawWindow();
  frame.setAlphaTransparency(1.0f);
  app.drawWindow();
  REQUIRE(layer->width() == width);
  REQUIRE(layer->height() == height);
}

TEST_CASE("Translucent bloom frames don't change with their siblings", "[integration]") {
  auto render = [](bool sibling) {
    ApplicationEditor app;
    Frame a;
    Frame b;
    BloomPostEffect bloom_a;
    BloomPostEffect bloom_b;
    auto draw = [](Canvas& canvas) {
      canvas.setColor(Color(0xffffe0a0, 2.5f));
      canvas.fill(20, 14, 12, 12);
    };
    app.onDraw() = [](Canvas& canvas) {
      canvas.setColor(0xff304050);
      canvas.fill();
    };
    a.onDraw() = draw;
    b.onDraw() = draw;
    bloom_a.setBloomSize(8.0f);
    bloom_a.setBloomIntensity(0.4f);
    a.setPostEffect(&bloom_a);
    b.setPostEffect(&bloom_b);
    a.setAlphaTransparency(0.5f);
    a.setBounds(0, 0, 56, 40);
    b.setBounds(60, 0, 56, 40);
    app.addChild(a);
    if (sibling)
      app.addChild(b);
    app.setWindowless(120, 40);
    app.drawWindow();
    return app.takeScreenshot();
  };

  Screenshot alone = render(false);
  Screenshot with_sibling = render(true);
  REQUIRE(with_sibling.sample(34, 20).hexRed() == alone.sample(34, 20).hexRed());
  REQUIRE(with_sibling.sample(26, 20).hexBlue() == alone.sample(26, 20).hexBlue());
}

TEST_CASE("Fading a shader frame inside a bloom frame keeps its glow", "[integration]") {
  ApplicationEditor app;
  Frame bloom_frame;
  Frame shader_frame;
  BloomPostEffect bloom;
  bloom.setBloomSize(8.0f);
  bloom.setBloomIntensity(0.4f);
  ShaderPostEffect shader(test_shaders::vs_test_post_effect, test_shaders::fs_test_color_mult);
  app.onDraw() = [](Canvas& canvas) {
    canvas.setColor(0xff304050);
    canvas.fill();
  };
  shader_frame.onDraw() = [](Canvas& canvas) {
    canvas.setColor(0xffc06040);
    canvas.fill(0, 0, 30, 40);
    canvas.setColor(Color(0xffffe0a0, 2.0f));
    canvas.fill(20, 14, 12, 12);
  };
  shader_frame.setPostEffect(&shader);
  bloom_frame.setPostEffect(&bloom);
  bloom_frame.setBounds(0, 0, 56, 40);
  shader_frame.setBounds(0, 0, 56, 40);
  bloom_frame.addChild(shader_frame);
  app.addChild(bloom_frame);
  app.setWindowless(56, 40);
  app.drawWindow();
  Screenshot before = app.takeScreenshot();

  shader_frame.setAlphaTransparency(0.5f);
  app.drawWindow();
  shader_frame.setAlphaTransparency(1.0f);
  app.drawWindow();
  const Screenshot& after = app.takeScreenshot();
  for (int y = 0; y < 40; ++y) {
    for (int x = 0; x < 56; ++x) {
      INFO("x " << x << " y " << y);
      REQUIRE(after.sample(x, y).hexRed() == Approx(before.sample(x, y).hexRed()).margin(1));
      REQUIRE(after.sample(x, y).hexGreen() == Approx(before.sample(x, y).hexGreen()).margin(1));
      REQUIRE(after.sample(x, y).hexBlue() == Approx(before.sample(x, y).hexBlue()).margin(1));
    }
  }
}

static Screenshot svgFrameScreenshot(const std::string& data) {
  ApplicationEditor app;
  SvgFrame frame;
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  frame.load(reinterpret_cast<const uint8_t*>(data.data()), data.size());
  frame.setBounds(0, 0, 20, 20);
  app.addChild(frame);
  app.setWindowless(20, 20);
  app.drawWindow();
  return app.takeScreenshot();
}

static Color svgFrameCenter(const std::string& data) {
  return svgFrameScreenshot(data).sample(10, 10);
}

TEST_CASE("Svg frame opacity", "[integration]") {
  Color translucent = svgFrameCenter(R"svg(<svg viewBox="0 0 10 10"><g opacity="0.5"><rect width="10" height="10" fill="red"/><rect width="10" height="10" fill="white" fill-opacity="0.5"/></g></svg>)svg");
  REQUIRE(translucent.hexRed() >= 0x7f);
  REQUIRE(translucent.hexRed() <= 0x81);
  REQUIRE(translucent.hexGreen() >= 0x3f);
  REQUIRE(translucent.hexGreen() <= 0x41);

  Color clamped = svgFrameCenter(R"svg(<svg viewBox="0 0 10 10"><g opacity="1.5"><rect width="10" height="10" fill="red" fill-opacity="0.5"/></g></svg>)svg");
  REQUIRE(clamped.hexRed() >= 0x7f);
  REQUIRE(clamped.hexRed() <= 0x81);

  Color percent = svgFrameCenter(R"svg(<svg viewBox="0 0 10 10"><g opacity="50%"><rect width="10" height="10" fill="red"/></g></svg>)svg");
  REQUIRE(percent.hexRed() >= 0x7f);
  REQUIRE(percent.hexRed() <= 0x81);

  REQUIRE(svgFrameCenter(R"svg(<svg viewBox="0 0 10 10"><g opacity="0"><rect width="10" height="10" fill="white"/></g></svg>)svg")
              .hexRed() == 0x00);

  Color stacked = svgFrameCenter(R"svg(<svg viewBox="0 0 10 10"><g><rect width="10" height="10" fill="white" opacity="0.5"/></g><rect width="10" height="10" fill="red"/></svg>)svg");
  REQUIRE(stacked.hexRed() == 0xff);
  REQUIRE(stacked.hexGreen() == 0x00);

  REQUIRE(svgFrameCenter(R"svg(<svg viewBox="0 0 10 10"><defs><rect width="10" height="10" fill="white"/></defs></svg>)svg")
              .hexRed() == 0x00);
  REQUIRE(svgFrameCenter(R"svg(<svg viewBox="0 0 10 10"><clipPath id="c"><rect width="10" height="10" fill="white"/></clipPath></svg>)svg")
              .hexRed() == 0x00);
}

TEST_CASE("Svg frame positioning", "[integration]") {
  Screenshot stroked = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><rect x="1" y="1" width="18" height="18" fill="none" stroke="white"/><rect x="5" y="5" width="10" height="10" fill="red"/></svg>)svg");
  REQUIRE(stroked.sample(4, 10).hexRed() == 0x00);
  REQUIRE(stroked.sample(5, 10).hexRed() == 0xff);
  REQUIRE(stroked.sample(14, 10).hexRed() == 0xff);
  REQUIRE(stroked.sample(15, 10).hexRed() == 0x00);

  Color offscreen = svgFrameCenter(R"svg(<svg viewBox="0 0 20 20"><rect x="-500" width="10" height="10" fill="white"/><rect width="20" height="20" fill="red"/></svg>)svg");
  REQUIRE(offscreen.hexRed() == 0xff);
}

TEST_CASE("Svg frame marker clipping", "[integration]") {
  Screenshot clipped = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><marker id="m" markerUnits="userSpaceOnUse" markerWidth="5" markerHeight="5"><rect width="20" height="20" fill="white"/></marker><path d="M0 0 L0 20" marker-start="url(#m)"/></svg>)svg");
  REQUIRE(clipped.sample(2, 2).hexRed() == 0xff);
  REQUIRE(clipped.sample(10, 10).hexRed() == 0x00);

  Screenshot visible = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><marker id="m" markerUnits="userSpaceOnUse" markerWidth="5" markerHeight="5" overflow="visible"><rect width="20" height="20" fill="white"/></marker><path d="M0 0 L0 20" marker-start="url(#m)"/></svg>)svg");
  REQUIRE(visible.sample(10, 10).hexRed() == 0xff);
}

TEST_CASE("Svg frame nested viewport clipping", "[integration]") {
  Screenshot clipped = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><svg width="5" height="5"><rect width="20" height="20" fill="white"/></svg></svg>)svg");
  REQUIRE(clipped.sample(2, 2).hexRed() == 0xff);
  REQUIRE(clipped.sample(10, 10).hexRed() == 0x00);
}

TEST_CASE("Svg frame masks", "[integration]") {
  Screenshot luminance = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><mask id="m"><rect width="10" height="20" fill="white"/><rect x="10" width="10" height="20" fill="black"/></mask><rect width="20" height="20" fill="white" mask="url(#m)"/></svg>)svg");
  REQUIRE(luminance.sample(5, 10).hexRed() == 0xff);
  REQUIRE(luminance.sample(15, 10).hexRed() == 0x00);

  Screenshot region = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><mask id="m" maskUnits="userSpaceOnUse" x="0" y="0" width="10" height="20"><rect width="20" height="20" fill="white"/></mask><rect width="20" height="20" fill="white" mask="url(#m)"/></svg>)svg");
  REQUIRE(region.sample(5, 10).hexRed() == 0xff);
  REQUIRE(region.sample(15, 10).hexRed() == 0x00);

  Screenshot filtered = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><filter id="f"><feColorMatrix values="0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0"/></filter><mask id="m"><rect width="10" height="20" filter="url(#f)"/></mask><rect width="20" height="20" fill="white" mask="url(#m)"/></svg>)svg");
  REQUIRE(filtered.sample(5, 10).hexRed() == 0xff);
  REQUIRE(filtered.sample(15, 10).hexRed() == 0x00);

  Screenshot occluded = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><mask id="m"><rect width="20" height="20" fill="white"/><rect x="10" width="10" height="20" fill="black"/></mask><rect width="20" height="20" fill="white" mask="url(#m)"/></svg>)svg");
  REQUIRE(occluded.sample(5, 10).hexRed() == 0xff);
  REQUIRE(occluded.sample(15, 10).hexRed() == 0x00);

  Screenshot clipped = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="c"><rect width="20" height="10"/></clipPath><mask id="m"><rect width="10" height="20" fill="white"/></mask><rect width="20" height="20" fill="white" clip-path="url(#c)" mask="url(#m)"/></svg>)svg");
  REQUIRE(clipped.sample(5, 5).hexRed() == 0xff);
  REQUIRE(clipped.sample(15, 5).hexRed() == 0x00);
  REQUIRE(clipped.sample(5, 15).hexRed() == 0x00);

  for (const char* mask_type : { R"(mask-type="alpha")", R"(style="mask-type: alpha")" }) {
    Screenshot alpha = svgFrameScreenshot(std::string(R"svg(<svg viewBox="0 0 20 20"><mask id="m" )svg") + mask_type +
                                          R"svg(><rect width="10" height="20" fill="black"/></mask><rect width="20" height="20" fill="white" mask="url(#m)"/></svg>)svg");
    REQUIRE(alpha.sample(5, 10).hexRed() == 0xff);
    REQUIRE(alpha.sample(15, 10).hexRed() == 0x00);
  }
}

TEST_CASE("Svg frame clip paths clipped by other clip paths", "[integration]") {
  Screenshot nested = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="top"><rect width="20" height="10"/></clipPath><clipPath id="left" clip-path="url(#top)"><rect width="10" height="20"/></clipPath><rect width="20" height="20" fill="white" clip-path="url(#left)"/></svg>)svg");
  REQUIRE(nested.sample(5, 5).hexRed() == 0xff);
  REQUIRE(nested.sample(15, 5).hexRed() == 0x00);
  REQUIRE(nested.sample(5, 15).hexRed() == 0x00);
  REQUIRE(nested.sample(15, 15).hexRed() == 0x00);

  Screenshot empty = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="empty"/><clipPath id="left" clip-path="url(#empty)"><rect width="10" height="20"/></clipPath><rect width="20" height="20" fill="white" clip-path="url(#left)"/></svg>)svg");
  REQUIRE(empty.sample(5, 5).hexRed() == 0x00);

  Screenshot cycle = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="left" clip-path="url(#top)"><rect width="10" height="20"/></clipPath><clipPath id="top" clip-path="url(#left)"><rect width="20" height="10"/></clipPath><rect width="20" height="20" fill="white" clip-path="url(#left)"/></svg>)svg");
  REQUIRE(cycle.sample(5, 5).hexRed() == 0xff);
  REQUIRE(cycle.sample(15, 5).hexRed() == 0x00);
  REQUIRE(cycle.sample(5, 15).hexRed() == 0x00);
}

TEST_CASE("Svg frame clip paths defined after use", "[integration]") {
  Screenshot forward = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><rect width="20" height="20" fill="white" clip-path="url(#c)"/><clipPath id="c"><rect width="10" height="20"/></clipPath></svg>)svg");
  REQUIRE(forward.sample(5, 10).hexRed() == 0xff);
  REQUIRE(forward.sample(15, 10).hexRed() == 0x00);

  Screenshot nested = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><g><rect width="20" height="20" fill="white" clip-path="url(#c)"/></g><g><defs><clipPath id="c"><circle cx="10" cy="10" r="5"/></clipPath></defs></g></svg>)svg");
  REQUIRE(nested.sample(10, 10).hexRed() == 0xff);
  REQUIRE(nested.sample(2, 2).hexRed() == 0x00);

  Screenshot duplicate = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="c"><rect width="10" height="20"/></clipPath><clipPath id="c"><rect x="10" width="10" height="20"/></clipPath><rect width="20" height="20" fill="white" clip-path="url(#c)"/></svg>)svg");
  REQUIRE(duplicate.sample(5, 10).hexRed() == 0xff);
  REQUIRE(duplicate.sample(15, 10).hexRed() == 0x00);
}

TEST_CASE("Svg frame clip paths use the referencing element's coordinates", "[integration]") {
  Screenshot outside = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><g transform="translate(4 0)"><clipPath id="c"><rect width="4" height="20"/></clipPath></g><rect width="20" height="20" fill="white" clip-path="url(#c)"/></svg>)svg");
  REQUIRE(outside.sample(2, 10).hexRed() == 0xff);
  REQUIRE(outside.sample(6, 10).hexRed() == 0x00);

  Screenshot used = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><g id="g" transform="translate(2 0)"><clipPath id="c"><rect width="4" height="4"/></clipPath><rect width="8" height="8" fill="white" clip-path="url(#c)"/></g><use href="#g" x="10" y="10"/></svg>)svg");
  REQUIRE(used.sample(3, 2).hexRed() == 0xff);
  REQUIRE(used.sample(7, 2).hexRed() == 0x00);
  REQUIRE(used.sample(13, 12).hexRed() == 0xff);
  REQUIRE(used.sample(17, 12).hexRed() == 0x00);

  Screenshot shared = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="c"><rect width="4" height="20"/></clipPath><rect width="20" height="10" fill="white" clip-path="url(#c)"/><g transform="translate(10 0)"><rect y="10" width="10" height="10" fill="white" clip-path="url(#c)"/></g></svg>)svg");
  REQUIRE(shared.sample(2, 5).hexRed() == 0xff);
  REQUIRE(shared.sample(6, 5).hexRed() == 0x00);
  REQUIRE(shared.sample(12, 15).hexRed() == 0xff);
  REQUIRE(shared.sample(16, 15).hexRed() == 0x00);

  Screenshot bounding = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="c" clipPathUnits="objectBoundingBox"><rect width="0.5" height="1"/></clipPath><rect width="10" height="10" fill="white" clip-path="url(#c)"/><rect x="10" y="10" width="10" height="10" fill="white" clip-path="url(#c)"/></svg>)svg");
  REQUIRE(bounding.sample(2, 5).hexRed() == 0xff);
  REQUIRE(bounding.sample(7, 5).hexRed() == 0x00);
  REQUIRE(bounding.sample(12, 15).hexRed() == 0xff);
  REQUIRE(bounding.sample(17, 15).hexRed() == 0x00);

  Screenshot rotated = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="c"><rect x="10" width="5" height="20"/></clipPath><rect x="10" width="10" height="10" fill="white" clip-path="url(#c)"/><rect x="10" y="10" width="10" height="10" fill="white" clip-path="url(#c)" transform="rotate(90 15 15)"/></svg>)svg");
  REQUIRE(rotated.sample(12, 5).hexRed() == 0xff);
  REQUIRE(rotated.sample(17, 5).hexRed() == 0x00);
  REQUIRE(rotated.sample(15, 12).hexRed() == 0xff);
  REQUIRE(rotated.sample(15, 17).hexRed() == 0x00);
}

TEST_CASE("Svg frame inset clip shapes", "[integration]") {
  auto inset = [](const std::string& shape) {
    return svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><rect width="20" height="20" fill="white" clip-path=")svg" +
                              shape + R"svg("/></svg>)svg");
  };

  Screenshot bottom_right = inset("inset(0 round 0 0 10px 0)");
  REQUIRE(bottom_right.sample(19, 19).hexRed() == 0x00);
  REQUIRE(bottom_right.sample(0, 19).hexRed() == 0xff);

  Screenshot two_radii = inset("inset(0 round 0 10px)");
  REQUIRE(two_radii.sample(19, 0).hexRed() == 0x00);
  REQUIRE(two_radii.sample(0, 19).hexRed() == 0x00);
  REQUIRE(two_radii.sample(0, 0).hexRed() == 0xff);
  REQUIRE(two_radii.sample(19, 19).hexRed() == 0xff);

  Screenshot rounded = inset("inset(5px round 2px)");
  REQUIRE(rounded.sample(2, 10).hexRed() == 0x00);
  REQUIRE(rounded.sample(10, 17).hexRed() == 0x00);
  REQUIRE(rounded.sample(10, 10).hexRed() == 0xff);

  Screenshot uniform = inset("inset(5px)");
  REQUIRE(uniform.sample(2, 10).hexRed() == 0x00);
  REQUIRE(uniform.sample(17, 10).hexRed() == 0x00);
  REQUIRE(uniform.sample(10, 17).hexRed() == 0x00);
  REQUIRE(uniform.sample(10, 10).hexRed() == 0xff);

  Screenshot vertical_horizontal = inset("inset(2px 5px)");
  REQUIRE(vertical_horizontal.sample(10, 1).hexRed() == 0x00);
  REQUIRE(vertical_horizontal.sample(3, 10).hexRed() == 0x00);
  REQUIRE(vertical_horizontal.sample(10, 3).hexRed() == 0xff);

  Screenshot three = inset("inset(0 0 0 round 0 10px 0)");
  REQUIRE(three.sample(10, 10).hexRed() == 0xff);
  REQUIRE(three.sample(19, 0).hexRed() == 0x00);
  REQUIRE(three.sample(0, 19).hexRed() == 0x00);
  REQUIRE(three.sample(19, 19).hexRed() == 0xff);

  Screenshot overlapping = inset("inset(60%)");
  REQUIRE(overlapping.sample(10, 10).hexRed() == 0x00);
  REQUIRE(overlapping.sample(9, 9).hexRed() == 0x00);
}

TEST_CASE("Svg frame rect clip shapes", "[integration]") {
  auto clipped = [](const std::string& clip_path) {
    return svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><rect width="20" height="20" fill="white" clip-path=")svg" +
                              clip_path + R"svg("/></svg>)svg");
  };

  Screenshot edges = clipped("rect(2px 15px 18px 5px)");
  REQUIRE(edges.sample(6, 10).hexRed() == 0xff);
  REQUIRE(edges.sample(13, 10).hexRed() == 0xff);
  REQUIRE(edges.sample(3, 10).hexRed() == 0x00);
  REQUIRE(edges.sample(17, 10).hexRed() == 0x00);
  REQUIRE(edges.sample(10, 1).hexRed() == 0x00);

  Screenshot automatic = clipped("rect(auto auto 50% 25%)");
  REQUIRE(automatic.sample(18, 2).hexRed() == 0xff);
  REQUIRE(automatic.sample(3, 2).hexRed() == 0x00);
  REQUIRE(automatic.sample(10, 15).hexRed() == 0x00);

  Screenshot rounded = clipped("rect(2px 18px 18px 2px round 4px)");
  REQUIRE(rounded.sample(10, 10).hexRed() == 0xff);
  REQUIRE(rounded.sample(2, 2).hexRed() == 0x00);

  REQUIRE(clipped("rect(10px 5px 18px 15px)").sample(10, 14).hexRed() == 0x00);
  REQUIRE(clipped("rect(2px 15px 18px)").sample(1, 1).hexRed() == 0xff);

  Screenshot slash = clipped("inset(0 round 10px/4px)");
  REQUIRE(slash.sample(0, 6).hexRed() == 0xff);
  REQUIRE(slash.sample(2, 0).hexRed() == 0x00);

  Screenshot upper_case_round = clipped("inset(2px ROUND 8px)");
  REQUIRE(upper_case_round.sample(10, 10).hexRed() == 0xff);
  REQUIRE(upper_case_round.sample(3, 3).hexRed() == 0x00);
  Screenshot upper_case_auto = clipped("rect(AUTO Auto auto 5px)");
  REQUIRE(upper_case_auto.sample(10, 10).hexRed() == 0xff);
  REQUIRE(upper_case_auto.sample(2, 10).hexRed() == 0x00);
  for (const char* clip_path : { "inset(5PX)", "inset(4q)", "inset(4Q)" }) {
    INFO(clip_path);
    Screenshot unit_case = clipped(clip_path);
    REQUIRE(unit_case.sample(10, 10).hexRed() == 0xff);
    REQUIRE(unit_case.sample(2, 10).hexRed() == 0x00);
  }

  std::vector<std::pair<std::string, IPoint>> invalid = {
    { "rect(1px 18px 18px 4px round)", { 1, 1 } },
    { "rect(1px 2px 3px 4px/5px)", { 10, 10 } },
    { "inset(0 round 10px/)", { 0, 0 } },
    { "inset(0 round 10px//4px)", { 0, 0 } },
    { "rect(a b c d)", { 10, 10 } },
    { "rect(1px,2px,3px,4px)", { 10, 10 } },
    { "inset(5foo 3px)", { 1, 10 } },
    { "inset(5e 3px)", { 10, 1 } },
    { "inset(3px 5%%)", { 10, 1 } },
    { "inset(3px round 5zz)", { 1, 1 } },
    { "rect(1px 2qq 18px 4px)", { 10, 10 } },
  };
  for (const auto& [clip_path, point] : invalid) {
    INFO(clip_path);
    REQUIRE(clipped(clip_path).sample(point.x, point.y).hexRed() == 0xff);
  }
}

TEST_CASE("Svg frame clip paths", "[integration]") {
  Screenshot united = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="c"><rect width="5" height="20"/><rect x="15" width="5" height="20"/></clipPath><rect width="20" height="20" fill="white" clip-path="url(#c)"/></svg>)svg");
  REQUIRE(united.sample(2, 10).hexRed() == 0xff);
  REQUIRE(united.sample(10, 10).hexRed() == 0x00);
  REQUIRE(united.sample(17, 10).hexRed() == 0xff);

  Screenshot transformed = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="c"><rect width="10" height="10"/></clipPath><g transform="scale(0.5)" clip-path="url(#c)"><rect width="40" height="40" fill="white"/></g><g transform="translate(10 10) scale(0.5)" clip-path="url(#c)"><rect width="40" height="40" fill="red"/></g></svg>)svg");
  REQUIRE(transformed.sample(2, 2).hexRed() == 0xff);
  REQUIRE(transformed.sample(7, 7).hexRed() == 0x00);
  REQUIRE(transformed.sample(12, 12).hexRed() == 0xff);
  REQUIRE(transformed.sample(12, 12).hexGreen() == 0x00);
  REQUIRE(transformed.sample(17, 17).hexRed() == 0x00);

  Screenshot even_odd = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><g clip-rule="evenodd"><clipPath id="c"><path d="M0 0h20v20h-20z M5 5h10v10h-10z"/></clipPath></g><rect width="20" height="20" fill="white" clip-path="url(#c)"/></svg>)svg");
  REQUIRE(even_odd.sample(2, 2).hexRed() == 0xff);
  REQUIRE(even_odd.sample(10, 10).hexRed() == 0x00);

  std::vector<std::string> hidden = {
    R"svg(<clipPath id="c"/>)svg",
    R"svg(<clipPath id="c"><path d=""/></clipPath>)svg",
    R"svg(<clipPath id="c"><rect width="20" height="20" visibility="hidden"/></clipPath>)svg",
    R"svg(<clipPath id="c"><rect width="20" height="20" display="none"/></clipPath>)svg",
    R"svg(<clipPath id="c"><g><rect width="20" height="20"/></g></clipPath>)svg",
  };
  for (const auto& clip_path : hidden) {
    INFO(clip_path);
    REQUIRE(svgFrameCenter(R"svg(<svg viewBox="0 0 20 20">)svg" + clip_path +
                           R"svg(<rect width="20" height="20" fill="white" clip-path="url(#c)"/></svg>)svg")
                .hexRed() == 0x00);
  }

  REQUIRE(svgFrameCenter(R"svg(<svg viewBox="0 0 20 20"><clipPath id="c" display="none"><rect width="20" height="20"/></clipPath><rect width="20" height="20" fill="white" clip-path="url(#c)"/></svg>)svg")
              .hexRed() == 0xff);
  REQUIRE(svgFrameCenter(R"svg(<svg viewBox="0 0 20 20"><rect width="20" height="20" fill="white" clip-path="none"/></svg>)svg")
              .hexRed() == 0xff);

  Screenshot bounding_box = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="c" clipPathUnits="objectBoundingBox" transform="translate(10 10)"><rect width="0.5" height="0.5"/></clipPath><rect width="20" height="20" fill="white" clip-path="url(#c)"/></svg>)svg");
  REQUIRE(bounding_box.sample(5, 5).hexRed() == 0x00);
  REQUIRE(bounding_box.sample(15, 15).hexRed() == 0xff);
}

TEST_CASE("Svg frame clip shapes with a reference box", "[integration]") {
  auto clipped = [](const std::string& clip_path) {
    return svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><rect x="10" width="10" height="20" fill="white" clip-path=")svg" +
                              clip_path + R"svg("/></svg>)svg");
  };

  Screenshot view_box = clipped("inset(0 0 0 50%) view-box");
  REQUIRE(view_box.sample(12, 10).hexRed() == 0xff);
  REQUIRE(view_box.sample(17, 10).hexRed() == 0xff);

  Screenshot no_space = clipped("inset(0 0 0 50%)view-box");
  REQUIRE(no_space.sample(12, 10).hexRed() == 0xff);

  Screenshot fill_box = clipped("inset(0 0 0 50%) fill-box");
  REQUIRE(fill_box.sample(12, 10).hexRed() == 0x00);
  REQUIRE(fill_box.sample(17, 10).hexRed() == 0xff);

  Screenshot unstroked_stroke_box = clipped("inset(0 0 0 50%) stroke-box");
  REQUIRE(unstroked_stroke_box.sample(12, 10).hexRed() == 0x00);
  REQUIRE(unstroked_stroke_box.sample(17, 10).hexRed() == 0xff);

  auto stroked = [](const std::string& box) {
    return svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><rect x="10" width="10" height="20" fill="white" stroke="white" stroke-width="8" clip-path="inset(0 0 0 25%) )svg" +
                              box + R"svg("/></svg>)svg");
  };
  REQUIRE(stroked("stroke-box").sample(11, 10).hexRed() == 0xff);
  REQUIRE(stroked("fill-box").sample(11, 10).hexRed() == 0x00);
  REQUIRE(stroked("border-box").sample(11, 10).hexRed() == 0xff);
  REQUIRE(stroked("content-box").sample(11, 10).hexRed() == 0x00);

  Screenshot view_box_first = clipped("view-box inset(0 0 0 50%)");
  REQUIRE(view_box_first.sample(12, 10).hexRed() == 0xff);
  Screenshot fill_box_first = clipped("fill-box inset(0 0 0 50%)");
  REQUIRE(fill_box_first.sample(12, 10).hexRed() == 0x00);
  REQUIRE(fill_box_first.sample(17, 10).hexRed() == 0xff);

  Screenshot upper_case = clipped("inset(0 0 0 75%) VIEW-BOX");
  REQUIRE(upper_case.sample(12, 10).hexRed() == 0x00);
  REQUIRE(upper_case.sample(17, 10).hexRed() == 0xff);
  Screenshot mixed_case_first = clipped("Fill-Box inset(0 0 0 50%)");
  REQUIRE(mixed_case_first.sample(12, 10).hexRed() == 0x00);
  REQUIRE(mixed_case_first.sample(17, 10).hexRed() == 0xff);

  Screenshot unfilled = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><rect x="10" width="10" height="20" fill="none" stroke="white" stroke-width="8" clip-path="inset(0 0 0 25%) fill-box"/></svg>)svg");
  REQUIRE(unfilled.sample(11, 10).hexRed() == 0x00);
  REQUIRE(unfilled.sample(13, 10).hexRed() == 0xff);

  Screenshot bounding = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="c" clipPathUnits="objectBoundingBox"><rect width="0.5" height="1"/></clipPath><rect x="5" y="2" width="10" height="16" fill="white" stroke="white" stroke-width="6" clip-path="url(#c)"/></svg>)svg");
  REQUIRE(bounding.sample(3, 10).hexRed() == 0x00);
  REQUIRE(bounding.sample(7, 10).hexRed() == 0xff);
  REQUIRE(bounding.sample(12, 10).hexRed() == 0x00);

  std::vector<std::pair<std::string, IPoint>> invalid = {
    { "view-box url(#c)", { 15, 10 } },
    { "url(#c) view-box", { 15, 10 } },
    { "view-box inset(10%) fill-box", { 1, 1 } },
    { "foo inset(10%)", { 1, 1 } },
    { "inset(10%) foo", { 1, 1 } },
  };
  for (const auto& [clip_path, point] : invalid) {
    INFO(clip_path);
    Screenshot ignored = svgFrameScreenshot(R"svg(<svg viewBox="0 0 20 20"><clipPath id="c"><rect width="10" height="20"/></clipPath><rect width="20" height="20" fill="white" clip-path=")svg" +
                                            clip_path + R"svg("/></svg>)svg");
    REQUIRE(ignored.sample(point.x, point.y).hexRed() == 0xff);
  }
}

namespace {
#if VISAGE_LINUX
  bool hasDisplay() {
    const char* display_name = std::getenv("DISPLAY");
    return display_name && display_name[0];
  }
#endif

  class TestWindow : public Window {
  public:
    TestWindow(int width, int height) : Window(width, height) { }

    void runEventLoop() override { }
    void* nativeHandle() const override { return nullptr; }
    void windowContentsResized(int width, int height) override { }
    void show() override { }
    void showMaximized() override { }
    void hide() override { }
    void close() override { }
    bool isShowing() const override { return true; }
    void setWindowTitle(const std::string& title) override { }
    IPoint maxWindowDimensions() const override { return { 1000, 1000 }; }
  };
}

TEST_CASE("Resizing a window many times keeps its swap chain valid", "[integration]") {
#if VISAGE_LINUX
  if (!hasDisplay())
    SKIP("No display");
#endif

  ApplicationEditor app;
  app.onDraw() = [](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  std::unique_ptr<Window> window = createWindow(Dimension::nativePixels(100), Dimension::nativePixels(100));
  app.addToWindow(window.get());
  for (int i = 0; i < 100; ++i) {
    window->handleResized(100 + i, 100 + i);
    app.drawWindow();
  }
  REQUIRE(app.nativeWidth() == 199);
  app.removeFromWindow();
}

TEST_CASE("Destroying a window right after resizing it leaves no swap chain work behind",
          "[integration]") {
#if VISAGE_LINUX
  if (!hasDisplay())
    SKIP("No display");
#endif

  {
    ApplicationEditor app;
    app.onDraw() = [](Canvas& canvas) {
      canvas.setColor(0xff000000);
      canvas.fill();
    };
    std::unique_ptr<Window> window = createWindow(Dimension::nativePixels(100),
                                                  Dimension::nativePixels(100));
    app.addToWindow(window.get());
    for (int i = 0; i < 20; ++i) {
      window->handleResized(100 + i, 100 + i);
      app.drawWindow();
    }
    app.removeFromWindow();
  }

  Canvas canvas;
  canvas.setWindowless(10, 10);
  canvas.setColor(0xffff0000);
  canvas.fill(0, 0, 10, 10);
  REQUIRE(canvas.takeScreenshot().sample(5, 5).hexRed() == 0xff);
}

TEST_CASE("Adding an editor to the window it's already in keeps its swap chain valid",
          "[integration]") {
#if VISAGE_LINUX
  if (!hasDisplay())
    SKIP("No display");
#endif

  ApplicationEditor app;
  app.onDraw() = [](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  std::unique_ptr<Window> window = createWindow(Dimension::nativePixels(100),
                                                Dimension::nativePixels(100));
  for (int i = 0; i < 10; ++i) {
    app.addToWindow(window.get());
    app.drawWindow();
    app.drawWindow();
  }
  REQUIRE(app.nativeWidth() == 100);
  app.removeFromWindow();
}

TEST_CASE("Window screenshots show the frame they were taken of", "[integration]") {
#if VISAGE_LINUX
  if (!hasDisplay())
    SKIP("No display");
#endif

  unsigned int color = 0;
  ApplicationEditor app;
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(color);
    canvas.fill();
  };
  std::unique_ptr<Window> window = createWindow(Dimension::nativePixels(40), Dimension::nativePixels(40));
  app.addToWindow(window.get());
  for (unsigned int next : { 0xffff0000, 0xff00ff00, 0xff0000ff }) {
    color = next;
    app.redraw();
    app.drawWindow();
    const Screenshot& screenshot = app.takeScreenshot();
    REQUIRE(screenshot.width() > 0);
    REQUIRE(screenshot.sample(20, 20).hexRed() == ((next >> 16) & 0xff));
    REQUIRE(screenshot.sample(20, 20).hexGreen() == ((next >> 8) & 0xff));
    REQUIRE(screenshot.sample(20, 20).hexBlue() == (next & 0xff));
  }
  app.removeFromWindow();
}

TEST_CASE("Window opens after rendering offscreen", "[integration]") {
#if VISAGE_LINUX
  if (!hasDisplay())
    SKIP("No display");
#endif

  Canvas canvas;
  canvas.setWindowless(10, 10);
  canvas.setColor(0xffffffff);
  canvas.fill(0, 0, 10, 10);
  REQUIRE(canvas.takeScreenshot().sample(5, 5).hexRed() == 0xff);

  ApplicationEditor app;
  app.onDraw() = [](Canvas& canvas) {
    canvas.setColor(0xff000000);
    canvas.fill();
  };
  std::unique_ptr<Window> window = createWindow(Dimension::nativePixels(100), Dimension::nativePixels(100));
  app.addToWindow(window.get());
  app.drawWindow();
  app.drawWindow();
  REQUIRE(app.nativeWidth() == 100);
  app.removeFromWindow();
}

TEST_CASE("Mouse wheel scrolling a frame under the mouse sends exit and enter", "[integration]") {
  ApplicationEditor app;
  Frame top;
  Frame bottom;
  int top_enters = 0, top_exits = 0, bottom_enters = 0, bottom_exits = 0;
  top.onMouseEnter() = [&](const MouseEvent&) { top_enters++; };
  top.onMouseExit() = [&](const MouseEvent&) { top_exits++; };
  bottom.onMouseEnter() = [&](const MouseEvent&) { bottom_enters++; };
  bottom.onMouseExit() = [&](const MouseEvent&) { bottom_exits++; };
  app.onMouseWheel() = [&](const MouseEvent&) {
    top.setBounds(0, 20, 40, 20);
    bottom.setBounds(0, 0, 40, 20);
    return true;
  };

  app.addChild(top);
  app.addChild(bottom);
  app.setBounds(0, 0, 40, 40);
  top.setBounds(0, 0, 40, 20);
  bottom.setBounds(0, 20, 40, 20);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  window.handleMouseMove(20, 10, 0, 0);
  REQUIRE(top_enters == 1);

  window.handleMouseWheel(0.0f, 1.0f, 20, 10, 0, 0);
  REQUIRE(top_exits == 1);
  REQUIRE(bottom_enters == 1);

  window.handleMouseMove(21, 10, 0, 0);
  REQUIRE(top_exits == 1);
  REQUIRE(bottom_enters == 1);
  REQUIRE(bottom_exits == 0);
  app.removeFromWindow();
}

TEST_CASE("Backdrop frames redraw what's under them when their layer repacks", "[integration]") {
  auto render = [](bool fade_after_first_draw, unsigned int glass_color) {
    ApplicationEditor app;
    Frame capture;
    Frame glass;
    Frame overlapping_glass;
    Frame fading;
    BlurPostEffect blur;
    BlurPostEffect overlapping_blur;
    blur.setBlurRadius(10.0f);
    overlapping_blur.setBlurRadius(16.0f);
    Canvas* canvas = nullptr;
    capture.onDraw() = [&](Canvas& capture_canvas) {
      canvas = &capture_canvas;
      capture_canvas.layer(0)->requestScreenshot();
      capture_canvas.setColor(0xff00ffff);
      capture_canvas.fill(0, 0, 2, 2);
    };
    auto fill = [](Frame& frame, unsigned int color) {
      frame.onDraw() = [&frame, color](Canvas& canvas) {
        canvas.setColor(color);
        canvas.fill(0, 0, frame.width(), frame.height());
      };
    };
    fill(glass, glass_color);
    fill(overlapping_glass, 0x20000000);
    fill(fading, 0xff808080);

    capture.setBounds(0, 0, 2, 2);
    glass.setBounds(100, 80, 180, 120);
    glass.setBackdropEffect(&blur);
    overlapping_glass.setBounds(220, 120, 180, 120);
    overlapping_glass.setBackdropEffect(&overlapping_blur);
    fading.setBounds(260, 220, 160, 80);
    if (!fade_after_first_draw)
      fading.setAlphaTransparency(0.6f);
    for (Frame* frame : { &capture, &glass, &overlapping_glass, &fading })
      app.addChild(frame);

    app.setWindowless(480, 320);
    fading.setAlphaTransparency(0.6f);
    capture.redraw();
    app.drawWindow();
    return canvas->layer(0)->screenshot();
  };

  render(false, 0xff000000);
  Screenshot screenshot = render(true, 0x30ffffff);
  Screenshot expected = render(false, 0x30ffffff);
  for (int y = 2; y < 320; y += 2) {
    for (int x = 2; x < 480; x += 2) {
      INFO("x " << x << " y " << y);
      REQUIRE(screenshot.sample(x, y).hexRed() == Approx(expected.sample(x, y).hexRed()).margin(2));
    }
  }
}

TEST_CASE("Frames needing more bgfx views than one frame allows still render", "[integration]") {
  static constexpr int kNumFrames = 400;
  static constexpr int kColumns = 20;
  static constexpr int kSize = 10;
  static constexpr int kSpacing = 12;
  static constexpr int kWidth = kColumns * kSpacing;
  static constexpr int kHeight = (kNumFrames / kColumns) * kSpacing;

  ApplicationEditor app;
  Color left_color = 0xffff0000;
  auto check = [&](const Screenshot& screenshot) {
    REQUIRE(screenshot.width() == app.nativeWidth());
    float scale = screenshot.width() / static_cast<float>(kWidth);
    for (int i = 0; i < kNumFrames; ++i) {
      int x = ((i % kColumns) * kSpacing + kSize / 2) * scale;
      int y = ((i / kColumns) * kSpacing + kSize / 2) * scale;
      Color center = screenshot.sample(x, y);
      INFO("frame " << i);
      REQUIRE(center.hexBlue() > 0x20);
      REQUIRE(center.hexBlue() < 0xe0);
      if (left_color.hexRed()) {
        REQUIRE(center.hexRed() > 0x20);
        REQUIRE(center.hexGreen() < 0x10);
      }
      else {
        REQUIRE(center.hexGreen() > 0x20);
        REQUIRE(center.hexRed() < 0x10);
      }
    }
  };

  std::vector<std::unique_ptr<Frame>> frames;
  for (int i = 0; i < kNumFrames; ++i) {
    auto frame = std::make_unique<Frame>();
    frame->onDraw() = [&left_color](Canvas& canvas) {
      canvas.setColor(left_color);
      canvas.rectangle(0, 0, kSize / 2, kSize);
      canvas.setColor(0xff0000ff);
      canvas.rectangle(kSize / 2, 0, kSize - kSize / 2, kSize);
    };
    frame->setBlurRadius(300.0f);
    frame->setBounds((i % kColumns) * kSpacing, (i / kColumns) * kSpacing, kSize, kSize);
    app.addChild(frame.get());
    frames.push_back(std::move(frame));
  }

  SECTION("Windowless") {
    app.setWindowless(kWidth, kHeight);
    app.drawWindow();
    check(app.takeScreenshot());
  }

#if VISAGE_LINUX
  if (!hasDisplay())
    return;
#endif
  SECTION("Window") {
    std::unique_ptr<Window> window = createWindow(Dimension::logicalPixels(kWidth),
                                                  Dimension::logicalPixels(kHeight));
    app.addToWindow(window.get());
    app.drawWindow();

    left_color = 0xff00ff00;
    for (auto& frame : frames)
      frame->redraw();
    app.drawStaleChildren();
    check(app.takeScreenshot());
    app.removeFromWindow();
  }
}

TEST_CASE("Refreshing programs by shader file rebuilds them without leaking handles",
          "[integration]") {
  Canvas canvas;
  canvas.setWindowless(10, 10);
  canvas.takeScreenshot();

  const EmbeddedFile& vertex = test_shaders::vs_test_post_effect;
  const EmbeddedFile& fragment = test_shaders::fs_test_opaque;
  const EmbeddedFile& replacement = test_shaders::fs_test_passthrough;
  bgfx::ProgramHandle original = ProgramCache::programHandle(vertex, fragment);
  auto flush = []() {
    bgfx::frame();
    bgfx::frame();
  };
  flush();
  int num_shaders = bgfx::getStats()->numShaders;
  int num_programs = bgfx::getStats()->numPrograms;

  for (int i = 0; i < 4; ++i) {
    REQUIRE(ShaderCache::swapShader(fragment, reinterpret_cast<const char*>(replacement.data),
                                    replacement.size));
    ProgramCache::refreshAllProgramsWithShader(fragment);
    REQUIRE(ProgramCache::programHandle(vertex, fragment).idx != original.idx);
  }
  REQUIRE_FALSE(ShaderCache::swapShader("not_a_shader", reinterpret_cast<const char*>(replacement.data),
                                        replacement.size));
  flush();
  REQUIRE(bgfx::getStats()->numShaders == num_shaders + 1);
  REQUIRE(bgfx::getStats()->numPrograms == num_programs + 1);

  ShaderCache::restoreShader(fragment);
  ProgramCache::restoreProgram(vertex, fragment);
  REQUIRE(ProgramCache::programHandle(vertex, fragment).idx == original.idx);
  flush();
  REQUIRE(bgfx::getStats()->numShaders == num_shaders);
  REQUIRE(bgfx::getStats()->numPrograms == num_programs);
}
