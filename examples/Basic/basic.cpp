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

#include <visage_app/application_editor.h>

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
