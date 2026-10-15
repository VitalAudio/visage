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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
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

namespace {
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
