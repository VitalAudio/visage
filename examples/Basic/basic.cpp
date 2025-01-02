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

#include <visage_app/application_editor.h>
#include <visage_windowing/windowing.h>

int runExample() {
  visage::WindowedEditor editor;

  editor.onDraw() = [&editor](visage::Canvas& canvas) {
    canvas.setColor(0xff000066);
    canvas.fill(0, 0, editor.width(), editor.height());

    float circle_radius = editor.height() * 0.1f;
    float x = editor.width() * 0.5f - circle_radius;
    float y = editor.height() * 0.5f - circle_radius;
    canvas.setColor(0xff00ffff);
    canvas.circle(x, y, 2.0f * circle_radius);
  };

  editor.show(800, 600);
  editor.runEventLoop();
  return 0;
}
