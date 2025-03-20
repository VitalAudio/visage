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

#include "visage_graphics/animation.h"
#include "visage_ui/frame.h"

namespace visage {
  class ClientDecoratorButton : public Frame {
  public:
    static constexpr int kDefaultHoverColor = 0x88888888;

    ClientDecoratorButton(HitTestResult hit_test_result) :
        hit_test_result_(hit_test_result), color_(kDefaultHoverColor) {
      hover_animation_.setSourceValue(0.0f);
      hover_animation_.setTargetValue(1.0f);
    }

    void mouseEnter(const MouseEvent& e) override {
      hover_animation_.target(true);
      redraw();
    }

    void mouseExit(const MouseEvent& e) override {
      hover_animation_.target(false);
      redraw();
    }

    void draw(Canvas& canvas) override {
      canvas.setColor(color_.withAlpha(color_.alpha() * hover_animation_.update()));
      canvas.fill(0, 0, width(), height());

      if (hover_animation_.isAnimating())
        redraw();
    }

    void setColor(const Color& color) { color_ = color; }

    HitTestResult hitTest(const Point& position) const override { return hit_test_result_; }

  private:
    Animation<float> hover_animation_;
    HitTestResult hit_test_result_ = HitTestResult::Client;
    Color color_;
  };

  class ClientWindowDecoration : public Frame {
  public:
    static constexpr int kButtonWidth = 46;
    static constexpr int kButtonHeight = 28;
    static constexpr int kCloseButtonColor = 0xffc42b1c;

    static int requiredWidth() { return 3 * kButtonWidth; }
    static int requiredHeight() { return kButtonHeight; }

    ClientWindowDecoration();

    void resized() override {
      close_button_.setBounds(width() - kButtonWidth, 0, kButtonWidth, height());
      maximize_button_.setBounds(close_button_.x() - kButtonWidth, 0, kButtonWidth, height());
      minimize_button_.setBounds(maximize_button_.x() - kButtonWidth, 0, kButtonWidth, height());
    }

  private:
    ClientDecoratorButton close_button_;
    ClientDecoratorButton maximize_button_;
    ClientDecoratorButton minimize_button_;

    VISAGE_LEAK_CHECKER(ClientWindowDecoration)
  };
}
