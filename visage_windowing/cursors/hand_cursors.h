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

#pragma once

namespace visage::hand_cursors {
  static constexpr int kSize = 16;
  static constexpr int kHotSpotX = 8;
  static constexpr int kHotSpotY = 8;

  static constexpr const char* kOpenHand[kSize] = {
    "       XX       ",
    "   XX XooXX     ",
    "  XooXXooXoXX   ",
    "  XooXXooXooXX  ",
    "   XooXooXooXoX ",
    "   XooXooXooXoX ",
    " XXXooooooooXoX ",
    "XooXooooooooooX ",
    "XoooXoooooooooX ",
    " XooooooooooooX ",
    "  XoooooooooooX ",
    "  XooooooooooX  ",
    "   XoooooooooX  ",
    "    XoooooooX   ",
    "     XooooooX   ",
    "     XXXXXXXX   ",
  };

  static constexpr const char* kClosedHand[kSize] = {
    "                ",
    "                ",
    "                ",
    "    XX XX XX    ",
    "   XooXooXooXX  ",
    "   XooooooooXoX ",
    "    XoooooooooX ",
    "   XXoooooooooX ",
    "  XoooooooooooX ",
    "  XooooooooooX  ",
    "   XoooooooooX  ",
    "    XoooooooX   ",
    "     XooooooX   ",
    "     XooooooX   ",
    "     XXXXXXXX   ",
    "                ",
  };
}
