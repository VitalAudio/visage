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

#include <visage_graphics/canvas.h>
#include <visage_graphics/renderer.h>
#include <visage_windowing/windowing.h>

int runExample() {
  std::unique_ptr<visage::Window> window = visage::createScaledWindow(1.0f);
  visage::Canvas canvas;

  visage::Renderer::instance().checkInitialization(window->initWindow(), window->globalDisplay());
  canvas.pairToWindow(window->nativeHandle(), window->clientWidth(), window->clientHeight());
  canvas.setColor(0xff223333);
  canvas.fill(0, 0, window->clientWidth(), window->clientHeight());
  canvas.setColor(0xffaa99ff);
  canvas.ring(50, 50, window->clientWidth() - 100.0f, window->clientWidth() * 0.1f);
  canvas.submit();
  canvas.render();

  window->show();
  window->runEventLoop();

  return 0;
}
