/* Copyright Vital Audio, LLC
*
* va is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* va is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with va.  If not, see <http://www.gnu.org/licenses/>.
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
    layout().setPadding(10);
    layout().setFlexGap(10);
    layout().setFlexRows(false);
    layout().setFlexWrapReverse(true);
    layout().setFlexReverseDirection(true);

    for (Frame& frame : frames_) {
      addChild(&frame);
      frame.layout().setHeight(100);
      frame.layout().setWidth(100);
      frame.layout().setFlexGrow(1.0f);

      frame.onDraw() = [this, &frame](visage::Canvas& canvas) {
        canvas.setColor(0xff888888);
        canvas.fill(0, 0, frame.width(), frame.height());
      };
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
  editor.showWithEventLoop(0.5f);

  return 0;
}
