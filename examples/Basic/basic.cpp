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

int runExample() {
  visage::WindowedEditor editor;

  editor.setDrawFunction([&editor](visage::Canvas& canvas) {
    canvas.clearArea(0, 0, editor.width(), editor.height());
    canvas.setColor(0xff00ffff);

    float circle_radius = editor.height() * 0.1f;
    float movement_radius = editor.height() * 0.3f;
    float center_x = editor.width() * 0.5f - circle_radius;
    float center_y = editor.height() * 0.5f - circle_radius;
    canvas.circle(center_x + movement_radius * cosf(canvas.time()),
                  center_y + movement_radius * sinf(canvas.time()), 2.0f * circle_radius);

    editor.redraw();
  });

  editor.showWithEventLoop(0.5f);
  return 0;
}
