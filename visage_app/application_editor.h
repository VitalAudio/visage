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

    void setReferenceDimensions(int width, int height) {
      reference_width_ = width;
      reference_height_ = height;
    }

    int referenceWidth() const { return reference_width_; }
    int referenceHeight() const { return reference_height_; }

    bool isFixedAspectRatio() const { return fixed_aspect_ratio_ > 0.0f; }
    void setFixedAspectRatio(float aspect_ratio) { fixed_aspect_ratio_ = aspect_ratio; }
    float aspectRatio() const {
      if (fixed_aspect_ratio_)
        return fixed_aspect_ratio_;
      if (height() && width())
        return width() * 1.0f / height();
      return 1.0f;
    }

    Window* window() const { return window_; }

    void drawStaleChildren();

    void setPixelScale(float scale) { pixel_scale_ = scale; }
    int logicalWidth() const { return std::round(width() / pixel_scale_); }
    int logicalHeight() const { return std::round(height() / pixel_scale_); }

    void setLogicalDimensions(int logical_width, int logical_height) {
      setBounds(x(), y(), std::round(logical_width * pixel_scale_),
                std::round(logical_height * pixel_scale_));
    }

    void addClientDecoration() { top_level_.addClientDecoration(); }
    HitTestResult hitTest(const Point& position) const override {
      if (position.y < kDefaultClientTitleBarHeight * dpiScale() && top_level_.hasClientDecoration())
        return HitTestResult::TitleBar;

      return HitTestResult::Client;
    }

  private:
    Window* window_ = nullptr;
    TopLevelFrame top_level_;
    FrameEventHandler event_handler_;
    std::unique_ptr<Canvas> canvas_;
    std::unique_ptr<WindowEventHandler> window_event_handler_;
    float fixed_aspect_ratio_ = 0.0f;
    float pixel_scale_ = 1.0f;

    int reference_width_ = 0;
    int reference_height_ = 0;

    std::set<Frame*> stale_children_;
    std::set<Frame*> drawing_children_;

    VISAGE_LEAK_CHECKER(ApplicationEditor)
  };
}
