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

#include <visage/graphics.h>
#include <visage/windowing.h>

int runExample() {
  std::unique_ptr<visage::Window> window = visage::createWindow(800, 800);
  visage::Renderer::instance().initialize(window->initWindow(), window->globalDisplay());

  visage::Canvas canvas;
  canvas.pairToWindow(window->nativeHandle(), window->clientWidth(), window->clientHeight());
  canvas.setColor(0xff223333);
  canvas.fill(0, 0, window->clientWidth(), window->clientHeight());
  canvas.setColor(0xffaa99ff);
  canvas.ring(50, 50, window->clientWidth() - 100.0f, window->clientWidth() * 0.1f);

  window->show();
  canvas.submit();

  window->runEventLoop();

  return 0;
}
