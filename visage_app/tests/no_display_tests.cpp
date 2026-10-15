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

#include "visage/app.h"

#include <catch2/catch_test_macros.hpp>
#include <cstdlib>

using namespace visage;

TEST_CASE("Windowing without an X display fails without crashing", "[integration]") {
#if VISAGE_LINUX
  const char* display = std::getenv("DISPLAY");
  if (display && display[0]) {
    SUCCEED("An X display is available");
    return;
  }

  REQUIRE(defaultDpiScale() > 0.0f);
  computeWindowBounds({}, {}, Dimension::logicalPixels(100), Dimension::logicalPixels(100));
  REQUIRE(createWindow({}, {}, Dimension::logicalPixels(100), Dimension::logicalPixels(100)) == nullptr);
  REQUIRE(createPluginWindow(Dimension::logicalPixels(100), Dimension::logicalPixels(100), nullptr) ==
          nullptr);
  REQUIRE(readClipboardText().empty());
  setClipboardText("text");
  setCursorStyle(MouseCursor::IBeam);
  setCursorVisible(false);
  cursorPosition();
  setCursorPosition({ 1, 1 });
  setCursorScreenPosition({ 1, 1 });

  ApplicationWindow window;
  window.show(Dimension::logicalPixels(100), Dimension::logicalPixels(100));
  REQUIRE_FALSE(window.isShowing());
  window.runEventLoop();
#endif
}
