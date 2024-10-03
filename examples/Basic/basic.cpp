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
  void draw(visage::Canvas& canvas) {
    canvas.clearArea(0, 0, getWidth(), getHeight());
    canvas.setColor(0xff00ffff);
    canvas.circle(200.0f + cosf(canvas.time()) * 200.0f, 500.0f + sinf(canvas.time()) * 200.0f, 100.0f);

    redraw();
  };
};

int runExample() {
  TestEditor editor;
  editor.show(0.5f);

  return 0;
}
