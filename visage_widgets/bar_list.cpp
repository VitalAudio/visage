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

#include "bar_list.h"

#include "visage_graphics/theme.h"

namespace visage {
  VISAGE_THEME_IMPLEMENT_COLOR(BarList, BarColor, 0xffaa88ff);

  BarList::BarList(int num_bars) : num_bars_(num_bars), bars_(std::make_unique<Bar[]>(num_bars)) { }

  void BarList::draw(Canvas& canvas) {
    canvas.setColor(BarColor);

    for (int i = 0; i < num_bars_; ++i) {
      const Bar& bar = bars_[i];
      canvas.rectangle(bar.left, bar.top, bar.right - bar.left, bar.bottom - bar.top);
    }
  }
}