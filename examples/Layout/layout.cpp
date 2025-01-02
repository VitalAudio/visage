/* Copyright Vital Audio, LLC
*
* visage is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* visage is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with visage.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "embedded/example_fonts.h"

#include <visage_app/application_editor.h>
#include <visage_windowing/windowing.h>

using namespace visage::dimension;

class ExampleEditor : public visage::WindowedEditor {
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

      frame.onDraw() = [this, &frame](visage::Canvas& canvas) {
        canvas.setColor(0xff888888);
        canvas.roundedRectangle(0, 0, frame.width(), frame.height(), 16);
      };
      ++i;
    }
  }

  void draw(visage::Canvas& canvas) override {
    static constexpr int kColumns = 12;

    canvas.setColor(0xff222026);
    canvas.fill(0, 0, width(), height());
  }

private:
  Frame frames_[kNumFrames];
  visage::Font font_;
};

int runExample() {
  ExampleEditor editor;
  editor.show(800, 600);
  editor.runEventLoop();

  return 0;
}
