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

#include "visage_ui/frame.h"

#include <set>

namespace visage {
  class ApplicationEditor;
  class Canvas;
  class Window;
  class WindowEventHandler;
  class ClientWindowDecoration;

  class TopLevelFrame : public Frame {
  public:
    explicit TopLevelFrame(ApplicationEditor* editor);
    ~TopLevelFrame() override;

    void resized() override;
    void addClientDecoration();
    bool hasClientDecoration() const { return client_decoration_ != nullptr; }

  private:
    ApplicationEditor* editor_ = nullptr;
    std::unique_ptr<ClientWindowDecoration> client_decoration_;
  };

  class ApplicationEditor : public Frame {
  public:
    static constexpr int kDefaultClientTitleBarHeight = 30;

    ApplicationEditor();
    ~ApplicationEditor() override;

    const Screenshot& takeScreenshot();

    void setCanvasDetails();

    void addToWindow(Window* handle);
    void setWindowless(int width, int height);
    void removeFromWindow();
    void drawWindow();

    bool isFixedAspectRatio() const { return fixed_aspect_ratio_; }
    void setFixedAspectRatio(bool fixed);
    float aspectRatio() const override {
      if (height() && width())
        return width() * 1.0f / height();
      return 1.0f;
    }

    Window* window() const { return window_; }

    void drawStaleChildren();

    void setDimensions(float width, float height) { setBounds(x(), y(), width, height); }
    void setNativeDimensions(int width, int height) {
      setNativeBounds(nativeX(), nativeY(), width, height);
    }

    void addClientDecoration() { top_level_->addClientDecoration(); }
    HitTestResult hitTest(const Point& position) const override {
      if (position.y < kDefaultClientTitleBarHeight && top_level_->hasClientDecoration())
        return HitTestResult::TitleBar;

      return HitTestResult::Client;
    }

  private:
    Window* window_ = nullptr;
    FrameEventHandler event_handler_;
    std::unique_ptr<Canvas> canvas_;
    std::unique_ptr<TopLevelFrame> top_level_;
    std::unique_ptr<WindowEventHandler> window_event_handler_;
    bool fixed_aspect_ratio_ = false;

    std::set<Frame*> stale_children_;
    std::set<Frame*> drawing_children_;

    VISAGE_LEAK_CHECKER(ApplicationEditor)
  };
}
