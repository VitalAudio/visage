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

#include "application_window.h"

namespace visage {
  ApplicationWindow::ApplicationWindow() = default;

  ApplicationWindow::~ApplicationWindow() {
    removeFromWindow();
  }

  void ApplicationWindow::setWindowDimensions(const Dimension& width, const Dimension& height) {
    if (window_ == nullptr)
      setDpiScale(defaultDpiScale());

    IBounds bounds = computeWindowBounds(width, height);
    setNativeBounds({ nativeX(), nativeY(), bounds.width(), bounds.height() });
    initial_position_ = bounds.topLeft();
  }

  void ApplicationWindow::setWindowDimensions(const Dimension& x, const Dimension& y,
                                              const Dimension& width, const Dimension& height) {
    if (window_ == nullptr)
      setDpiScale(defaultDpiScale());

    IBounds bounds = computeWindowBounds(x, y, width, height);
    setNativeBounds({ nativeX(), nativeY(), bounds.width(), bounds.height() });
    initial_position_ = bounds.topLeft();
  }

  void ApplicationWindow::setTitle(std::string title) {
    title_ = std::move(title);
    if (window_)
      window_->setWindowTitle(title_);
  }

  void ApplicationWindow::setWindowOnTop(bool on_top) {
    always_on_top_ = on_top;
    if (window_)
      window_->setAlwaysOnTop(on_top);
  }

  void ApplicationWindow::show() {
    show(Dimension::nativePixels(initial_position_.x), Dimension::nativePixels(initial_position_.y),
         Dimension::nativePixels(nativeWidth()), Dimension::nativePixels(nativeHeight()));
  }

  void ApplicationWindow::show(void* parent_window) {
    VISAGE_ASSERT(width() && height());
    show(Dimension::nativePixels(nativeWidth()), Dimension::nativePixels(nativeHeight()), parent_window);
  }

  void ApplicationWindow::show(const Dimension& width, const Dimension& height, void* parent_window) {
    removeFromWindow();
    window_ = createPluginWindow(width, height, parent_window);
    showWindow(false);
  }

  void ApplicationWindow::show(const Dimension& width, const Dimension& height) {
    show({}, {}, width, height);
  }

  void ApplicationWindow::show(const Dimension& x, const Dimension& y, const Dimension& width,
                               const Dimension& height) {
    removeFromWindow();
    window_ = createWindow(x, y, width, height, decoration_);
    showWindow(false);
  }

  void ApplicationWindow::showMaximized() {
    static constexpr float kUnmaximizedWidthPercent = 85.0f;

    removeFromWindow();
    window_ = createWindow({}, {}, Dimension::widthPercent(kUnmaximizedWidthPercent),
                           Dimension::heightPercent(kUnmaximizedWidthPercent), decoration_);
    showWindow(true);
  }

  void ApplicationWindow::hide() {
    if (window_ && onCloseRequested().callback())
      return window_->hide();
  }

  void ApplicationWindow::close() {
    if (window_ && onCloseRequested().callback()) {
      removeFromWindow();
      window_ = nullptr;
    }
  }

  bool ApplicationWindow::isShowing() const {
    return window_ && window_->isShowing();
  }

  void ApplicationWindow::runEventLoop() {
    window_->runEventLoop();
  }

  void ApplicationWindow::showWindow(bool maximized) {
    if (!title_.empty())
      window_->setWindowTitle(title_);
    window_->setAlwaysOnTop(always_on_top_);

    addToWindow(window_.get());
    if (maximized)
      window_->showMaximized();
    else
      window_->show();
  }
}
