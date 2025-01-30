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

#include "application_editor.h"

#include <visage_windowing/windowing.h>

namespace visage {

  class ApplicationWindow : public ApplicationEditor {
  public:
    ApplicationWindow();
    ~ApplicationWindow() override;

    const std::string& title() const { return title_; }
    void setTitle(std::string title) { title_ = std::move(title); }
    void setWindowDecoration(Window::Decoration decoration) {
      decoration_ = decoration;
      if (decoration_ == Window::Decoration::Client)
        addClientDecoration();
    }

    void show(const Dimension& width, const Dimension& height);
    void show(const Dimension& x, const Dimension& y, const Dimension& width, const Dimension& height);
    void showMaximized();
    void hide();
    bool isShowing() const;

    auto& onShow() { return on_show_; }
    auto& onHide() { return on_hide_; }

    void runEventLoop();

  private:
    void showWindow(bool maximized);

    Window::Decoration decoration_ = Window::Decoration::Native;
    CallbackList<void()> on_show_;
    CallbackList<void()> on_hide_;
    std::string title_;
    std::unique_ptr<Window> window_;
  };
}
