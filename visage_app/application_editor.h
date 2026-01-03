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

    auto& onShow() { return on_show_; }
    auto& onHide() { return on_hide_; }
    auto& onCloseRequested() { return on_close_requested_; }
    auto& onWindowContentsResized() { return on_window_contents_resized_; }

    void notifyContentsResized();
    const Screenshot& takeScreenshot();
    void setCanvasDetails();

    void addToWindow(Window* window);
    void setWindowless(int width, int height);
    void removeFromWindow();
    void drawWindow();

    bool isFixedAspectRatio() const { return fixed_aspect_ratio_ != 0.0f; }
    void setFixedAspectRatio(bool fixed);
    float aspectRatio() const override {
      if (height() && width())
        return width() * 1.0f / height();
      return 1.0f;
    }

    Window* window() const { return window_; }

    void drawStaleChildren();

    void setMinimumDimensions(float width, float height) {
      min_width_ = std::max(0.0f, width);
      min_height_ = std::max(0.0f, height);
    }

    void checkFixedAspectRatio() {
      if (fixed_aspect_ratio_ && width() && height())
        fixed_aspect_ratio_ = aspectRatio();
    }

    void adjustWindowDimensions(int* width, int* height, bool horizontal_resize, bool vertical_resize) const;

    void adjustWindowDimensions(uint32_t* width, uint32_t* height, bool horizontal_resize,
                                bool vertical_resize) const {
      int w = *width;
      int h = *height;
      adjustWindowDimensions(&w, &h, horizontal_resize, vertical_resize);
      *width = w;
      *height = h;
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
    CallbackList<void()> on_show_;
    CallbackList<void()> on_hide_;
    CallbackList<bool()> on_close_requested_;
    CallbackList<void()> on_window_contents_resized_;

    float fixed_aspect_ratio_ = 0.0f;
    float min_width_ = 0.0f;
    float min_height_ = 0.0f;

    std::vector<Frame*> stale_children_;
    std::vector<Frame*> drawing_children_;

    VISAGE_LEAK_CHECKER(ApplicationEditor)
  };
}
