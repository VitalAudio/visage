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

#include "client_window_decoration.h"

namespace visage {

  static Bounds iconBounds(Bounds bounds) {
    float height = bounds.height();
    float width = bounds.width();
    float icon_y = (height + 2.0f) / 3.0f;
    float icon_width = height - 2.0f * icon_y;
    return { (width - icon_width) / 2.0f, icon_y, icon_width, icon_width };
  }

  ClientWindowDecoration::ClientWindowDecoration() :
      close_button_(HitTestResult::CloseButton), maximize_button_(HitTestResult::MaximizeButton),
      minimize_button_(HitTestResult::MinimizeButton) {
    static constexpr int kIconColor = 0xffffffff;

    addChild(&close_button_);
    close_button_.setColor(kCloseButtonColor);
    addChild(&maximize_button_);
    addChild(&minimize_button_);

    close_button_.onDraw() += [this](Canvas& canvas) {
      canvas.setColor(kIconColor);
      Bounds bounds = iconBounds(close_button_.localBounds());
      canvas.segment(bounds.x(), bounds.y(), bounds.right(), bounds.bottom(), 1.0f, true);
      canvas.segment(bounds.x(), bounds.bottom(), bounds.right(), bounds.y(), 1.0f, true);
    };

    maximize_button_.onDraw() += [this](Canvas& canvas) {
      canvas.setColor(kIconColor);
      Bounds bounds = iconBounds(close_button_.localBounds());
      canvas.roundedRectangleBorder(bounds.x(), bounds.y(), bounds.width(), bounds.height(), 2.0f, 1.0f);
    };

    minimize_button_.onDraw() += [this](Canvas& canvas) {
      canvas.setColor(kIconColor);
      Bounds bounds = iconBounds(close_button_.localBounds());
      canvas.rectangle(bounds.x(), bounds.yCenter() - 1.0f, bounds.width(), 1.0f);
    };
  }
}
