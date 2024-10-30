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

#include <visage_app/application_editor.h>
#include <visage_ui/popup_menu.h>
#include <visage_windowing/windowing.h>

class ExampleEditor : public visage::WindowedEditor {
public:
  void draw(visage::Canvas& canvas) override {
    static constexpr float kRadius = 50.0f;
    canvas.clearArea(0, 0, width(), height());
    if (down_)
      canvas.setColor(0xff00ffff);
    else
      canvas.setColor(0xffffffff);
    canvas.circle(x_ - kRadius, y_ - kRadius, 2.0f * kRadius);
  };

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
    if (!e.shouldTriggerPopup()) {
      setDown(true);
      return;
    }
    visage::PopupMenu menu;
    menu.addOption(1, "Option 1");
    menu.addOption(2, "Option 2");
    menu.show(this, { x_, y_ });
  }
  void mouseUp(const visage::MouseEvent& e) override { setDown(false); }

private:
  bool down_ = false;
  int x_ = -100;
  int y_ = -100;
};

int runExample() {
  ExampleEditor editor;
  editor.showWithEventLoop(0.5f);

  return 0;
}
