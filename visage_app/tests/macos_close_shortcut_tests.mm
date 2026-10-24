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
#include <functional>
#include <vector>

#import <Cocoa/Cocoa.h>

using namespace visage;

namespace {
  class Steps : public EventTimer {
  public:
    explicit Steps(std::vector<std::function<bool()>> steps, std::function<void()> on_timeout) :
        steps_(std::move(steps)), on_timeout_(std::move(on_timeout)) {
      startTimer(20);
    }

    void timerCallback() override {
      if (++ticks_ > kMaxTicks) {
        stopTimer();
        timed_out_ = true;
        on_timeout_();
        return;
      }
      if (next_ < steps_.size() && steps_[next_]())
        next_++;
    }

    bool finished() const { return next_ == steps_.size(); }
    bool timedOut() const { return timed_out_; }

  private:
    static constexpr int kMaxTicks = 500;

    std::vector<std::function<bool()>> steps_;
    std::function<void()> on_timeout_;
    size_t next_ = 0;
    int ticks_ = 0;
    bool timed_out_ = false;
  };

  void sendCommandKey(ApplicationWindow& window, NSString* character, unsigned short key_code) {
    NSView* view = (__bridge NSView*)window.window()->nativeHandle();
    NSEvent* event = [NSEvent keyEventWithType:NSEventTypeKeyDown
                                      location:NSZeroPoint
                                 modifierFlags:NSEventModifierFlagCommand
                                     timestamp:0
                                  windowNumber:view.window.windowNumber
                                       context:nil
                                    characters:character
                   charactersIgnoringModifiers:character
                                     isARepeat:NO
                                       keyCode:key_code];
    [view keyDown:event];
  }
}

TEST_CASE("Cmd+W closes only the window that receives it and Cmd+Q closes every window",
          "[integration]") {
  static constexpr unsigned short kKeyCodeW = 13;
  static constexpr unsigned short kKeyCodeQ = 12;

  ApplicationWindow app;
  ApplicationWindow other;
  app.show(Dimension::nativePixels(100), Dimension::nativePixels(80));
  other.show(Dimension::nativePixels(60), Dimension::nativePixels(40));

  bool app_showing_after_w = false;
  bool other_showing_after_w = true;
  Steps steps({ [&] { return app.isShowing() && other.isShowing(); },
                [&] {
                  sendCommandKey(other, @"w", kKeyCodeW);
                  return true;
                },
                [&] {
                  app_showing_after_w = app.isShowing();
                  other_showing_after_w = other.isShowing();
                  return true;
                },
                [&] {
                  other.show(Dimension::nativePixels(60), Dimension::nativePixels(40));
                  return true;
                },
                [&] { return other.isShowing(); },
                [&] {
                  sendCommandKey(app, @"q", kKeyCodeQ);
                  return true;
                } },
              [&] {
                other.close();
                app.close();
              });
  app.runEventLoop();

  REQUIRE_FALSE(steps.timedOut());
  REQUIRE(steps.finished());
  REQUIRE(app_showing_after_w);
  REQUIRE_FALSE(other_showing_after_w);
  REQUIRE_FALSE(app.isShowing());
  REQUIRE_FALSE(other.isShowing());
}
