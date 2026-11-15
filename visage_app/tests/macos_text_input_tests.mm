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
#include "visage_widgets/text_editor.h"

#include <catch2/catch_test_macros.hpp>
#include <functional>
#include <string>
#include <vector>

#import <Cocoa/Cocoa.h>
#import <objc/runtime.h>

using namespace visage;

namespace {
  constexpr unsigned short kKeyCodeE = 14;

  int interpret_calls = 0;

  void countInterpretKeyEvents(NSView* view) {
    static bool installed = false;
    if (installed)
      return;

    installed = true;
    SEL selector = @selector(interpretKeyEvents:);
    IMP original = class_getMethodImplementation([NSResponder class], selector);
    IMP counting = imp_implementationWithBlock(^(id receiver, NSArray* events) {
      interpret_calls++;
      reinterpret_cast<void (*)(id, SEL, NSArray*)>(original)(receiver, selector, events);
    });
    class_addMethod([view class], selector, counting, "v@:@");
  }

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

  class KeyLogFrame : public Frame {
  public:
    KeyLogFrame() { setAcceptsKeystrokes(true); }

    bool keyPress(const KeyEvent& e) override {
      presses++;
      return true;
    }

    bool keyRelease(const KeyEvent& e) override {
      releases++;
      return true;
    }

    int presses = 0;
    int releases = 0;
  };

  NSView* viewFor(ApplicationWindow& window) {
    return (__bridge NSView*)window.window()->nativeHandle();
  }

  void sendKey(ApplicationWindow& window, NSEventType type, bool repeat) {
    NSView* view = viewFor(window);
    NSEvent* event = [NSEvent keyEventWithType:type
                                      location:NSZeroPoint
                                 modifierFlags:0
                                     timestamp:0
                                  windowNumber:view.window.windowNumber
                                       context:nil
                                    characters:@"e"
                   charactersIgnoringModifiers:@"e"
                                     isARepeat:repeat
                                       keyCode:kKeyCodeE];
    if (type == NSEventTypeKeyDown)
      [view keyDown:event];
    else
      [view keyUp:event];
  }

  void holdKey(ApplicationWindow& window) {
    sendKey(window, NSEventTypeKeyDown, false);
    sendKey(window, NSEventTypeKeyDown, true);
    sendKey(window, NSEventTypeKeyDown, true);
    sendKey(window, NSEventTypeKeyUp, false);
  }
}

TEST_CASE("Keys only go through macOS text input while a text editor has focus", "[integration]") {
  bool composing = false;
  SECTION("Not composing") { }
  SECTION("Text entry ends while composing") {
    composing = true;
  }

  ApplicationWindow app;
  KeyLogFrame keys;
  TextEditor editor;
  app.addChild(keys);
  app.addChild(editor);
  keys.setBounds(0, 0, 50, 80);
  editor.setBounds(50, 0, 50, 80);
  app.show(Dimension::nativePixels(100), Dimension::nativePixels(80));

  int interpret_calls_without_editor = -1;
  int presses_without_editor = 0;
  int releases_without_editor = 0;
  int interpret_calls_with_editor = 0;
  std::string editor_text;

  Steps steps({ [&] { return app.isShowing(); },
                [&] {
                  countInterpretKeyEvents(viewFor(app));
                  if (composing) {
                    editor.requestKeyboardFocus();
                    [(id<NSTextInputClient>)viewFor(app) setMarkedText:@"\u00B4"
                                                         selectedRange:NSMakeRange(1, 0)
                                                      replacementRange:NSMakeRange(NSNotFound, 0)];
                  }
                  keys.requestKeyboardFocus();
                  interpret_calls = 0;
                  holdKey(app);
                  interpret_calls_without_editor = interpret_calls;
                  presses_without_editor = keys.presses;
                  releases_without_editor = keys.releases;
                  return true;
                },
                [&] {
                  editor.requestKeyboardFocus();
                  interpret_calls = 0;
                  sendKey(app, NSEventTypeKeyDown, false);
                  sendKey(app, NSEventTypeKeyUp, false);
                  interpret_calls_with_editor = interpret_calls;
                  editor_text = editor.text().toUtf8();
                  return true;
                },
                [&] {
                  app.close();
                  return true;
                } },
              [&] { app.close(); });
  app.runEventLoop();

  REQUIRE_FALSE(steps.timedOut());
  REQUIRE(steps.finished());
  CHECK(interpret_calls_without_editor == 0);
  CHECK(presses_without_editor == 3);
  CHECK(releases_without_editor == 1);
  CHECK(interpret_calls_with_editor == 1);
  CHECK(editor_text == "e");
}
