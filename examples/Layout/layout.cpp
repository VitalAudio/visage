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

using namespace visage::dimension;

class ExampleEditor : public visage::ApplicationWindow {
public:
  static constexpr int kNumFrames = 10;

  ExampleEditor() {
    setFlexLayout(true);
    layout().setPadding(10_px);
    layout().setFlexGap(10_px);
    layout().setFlexWrap(true);
    layout().setFlexReverseDirection(true);
    layout().setFlexWrapReverse(true);

    int i = 0;
    for (Frame& frame : frames_) {
      addChild(&frame);
      frame.layout().setHeight(100);
      frame.layout().setWidth(100 + i * 10);
      frame.layout().setFlexGrow(1.0f);

      frame.onDraw() = [&frame](visage::Canvas& canvas) {
        canvas.setColor(0xff888888);
        canvas.roundedRectangle(0, 0, frame.width(), frame.height(), 16);
      };
      ++i;
    }
  }

  void draw(visage::Canvas& canvas) override {
    canvas.setColor(0xff222026);
    canvas.fill(0, 0, width(), height());
  }

private:
  Frame frames_[kNumFrames];
};

int runExample() {
  ExampleEditor editor;
  editor.show(visage::Dimension::logicalPixels(800), visage::Dimension::logicalPixels(600));
  editor.runEventLoop();

  return 0;
}
