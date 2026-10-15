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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdlib>
#include <cstring>

#if VISAGE_LINUX
#include <X11/extensions/Xrandr.h>
#include <X11/Xlib.h>

static bool onlyUnsizedOutputs(Display* display) {
  XRRScreenResources* resources = XRRGetScreenResourcesCurrent(display, DefaultRootWindow(display));
  if (resources == nullptr)
    return false;

  bool connected = false;
  bool sized = false;
  for (int i = 0; i < resources->noutput; ++i) {
    XRROutputInfo* output = XRRGetOutputInfo(display, resources, resources->outputs[i]);
    if (output == nullptr)
      continue;
    if (output->connection == RR_Connected) {
      connected = true;
      sized = sized || output->mm_width != 0;
    }
    XRRFreeOutputInfo(output);
  }
  XRRFreeScreenResources(resources);
  return connected && !sized;
}

TEST_CASE("X11 DPI falls back to the screen DPI when outputs report no size", "[integration]") {
  Display* display = std::getenv("DISPLAY") ? XOpenDisplay(nullptr) : nullptr;
  if (display == nullptr)
    SKIP("No display");

  const char* resources = XResourceManagerString(display);
  bool xft_dpi = resources && std::strstr(resources, "Xft.dpi:");
  bool unsized = onlyUnsizedOutputs(display);
  int screen = DefaultScreen(display);
  float screen_dpi = DisplayWidth(display, screen) * 25.4f / DisplayWidthMM(display, screen);
  XCloseDisplay(display);
  if (xft_dpi || !unsized)
    SKIP("The display sets Xft.dpi or its outputs report a size");

  float expected_dpi = std::round(screen_dpi / 24.0f) * 24.0f;
  REQUIRE(visage::defaultDpiScale() * visage::Window::kDefaultDpi == Catch::Approx(expected_dpi));
}

TEST_CASE("X11 ignores an Xft.dpi setting outside the sane range", "[integration]") {
  Display* display = std::getenv("DISPLAY") ? XOpenDisplay(nullptr) : nullptr;
  if (display == nullptr)
    SKIP("No display");

  const char* resources = XResourceManagerString(display);
  const char* setting = resources ? std::strstr(resources, "Xft.dpi:") : nullptr;
  float xft_dpi = setting ? std::strtof(setting + std::strlen("Xft.dpi:"), nullptr) : 0.0f;
  bool unsized = onlyUnsizedOutputs(display);
  int screen = DefaultScreen(display);
  float screen_dpi = DisplayWidth(display, screen) * 25.4f / DisplayWidthMM(display, screen);
  XCloseDisplay(display);
  if (setting == nullptr || (xft_dpi >= 50.0f && xft_dpi <= 500.0f) || !unsized)
    SKIP("The display has no out of range Xft.dpi or its outputs report a size");

  float expected_dpi = std::round(screen_dpi / 24.0f) * 24.0f;
  REQUIRE(visage::defaultDpiScale() * visage::Window::kDefaultDpi == Catch::Approx(expected_dpi));
}
#endif
