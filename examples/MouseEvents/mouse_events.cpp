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
#include <visage_windowing/windowing.h>

class TestEditor : public visage::WindowedEditor {
public:
  void draw(visage::Canvas& canvas) override {
    static constexpr float kRadius = 50.0f;
    canvas.clearArea(0, 0, getWidth(), getHeight());
    if (down_)
      canvas.setColor(0xff00ffff);
    else
      canvas.setColor(0xffffffff);
    canvas.circle(x_ - kRadius, y_ - kRadius, 2.0f * kRadius);
    redraw();
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

  void onMouseMove(const visage::MouseEvent& e) override { setPosition(e.position); }
  void onMouseDrag(const visage::MouseEvent& e) override { setPosition(e.position); }
  void onMouseExit(const visage::MouseEvent& e) override { setPosition({ -100, -100 }); }
  void onMouseDown(const visage::MouseEvent& e) override { setDown(true); }
  void onMouseUp(const visage::MouseEvent& e) override { setDown(false); }

private:
  bool down_ = false;
  int x_ = -100;
  int y_ = -100;
};

int runExample() {
  TestEditor editor;
  editor.show(0.5f);

  return 0;
}
