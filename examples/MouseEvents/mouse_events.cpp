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

#include <visage/app.h>
#include <visage_ui/popup_menu.h>

class ExampleEditor : public visage::ApplicationWindow {
public:
  void draw(visage::Canvas& canvas) override {
    static constexpr float kRadius = 30.0f;
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, width(), height());
    if (down_)
      canvas.setColor(0xff00ffff);
    else
      canvas.setColor(0xffffffff);
    canvas.circle(x_ - kRadius, y_ - kRadius, 2.0f * kRadius);
  }

  void setPosition(const visage::Point& point) {
    x_ = point.x;
    y_ = point.y;
    redraw();
  }

  void setDown(bool down) {
    down_ = down;
    redraw();
  }

  void mouseMove(const visage::MouseEvent& e) override { setPosition(e.position); }
  void mouseDrag(const visage::MouseEvent& e) override { setPosition(e.position); }
  void mouseExit(const visage::MouseEvent& e) override { setPosition({ -100, -100 }); }
  void mouseDown(const visage::MouseEvent& e) override {
    if (e.isMiddleButton()) {
      setMouseRelativeMode(true);
      setCursorVisible(false);
    }

    if (!e.shouldTriggerPopup()) {
      setDown(true);
      return;
    }
    visage::PopupMenu menu;
    menu.addOption(1, "Option 1");
    menu.addOption(2, "Option 2");
    menu.show(this, { x_, y_ });
  }
  void mouseUp(const visage::MouseEvent& e) override {
    if (e.isMiddleButton()) {
      setCursorVisible(true);
      setMouseRelativeMode(false);
    }
    setDown(false);
  }

private:
  bool down_ = false;
  float x_ = -100.0f;
  float y_ = -100.0f;
};

int runExample() {
  ExampleEditor editor;
  editor.show(visage::Dimension::logicalPixels(800), visage::Dimension::logicalPixels(600));
  editor.runEventLoop();

  return 0;
}
