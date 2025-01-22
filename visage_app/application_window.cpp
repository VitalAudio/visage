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

#include <visage_windowing/windowing.h>

namespace visage {
  ApplicationWindow::ApplicationWindow() = default;

  ApplicationWindow::~ApplicationWindow() {
    removeFromWindow();
  }

  void ApplicationWindow::show(const Dimension& width, const Dimension& height) {
    show({}, {}, width, height);
  }

  void ApplicationWindow::show(const Dimension& x, const Dimension& y, const Dimension& width,
                               const Dimension& height) {
    show(x, y, width, height, false);
  }

  void ApplicationWindow::showPopup(const Dimension& width, const Dimension& height) {
    showPopup({}, {}, width, height);
  }

  void ApplicationWindow::showPopup(const Dimension& x, const Dimension& y, const Dimension& width,
                                    const Dimension& height) {
    show(x, y, width, height, true);
  }

  void ApplicationWindow::show(const Dimension& x, const Dimension& y, const Dimension& width,
                               const Dimension& height, bool popup) {
    removeFromWindow();
    window_ = createWindow(x, y, width, height, popup);
    showWindow(false);
  }

  void ApplicationWindow::showMaximized() {
    removeFromWindow();
    window_ = createWindow({}, {}, Dimension::widthPercent(85.0f), Dimension::heightPercent(85.0f), false);
    showWindow(true);
  }

  void ApplicationWindow::runEventLoop() {
    window_->runEventLoop();
  }

  void ApplicationWindow::showWindow(bool maximized) {
    if (!title_.empty())
      window_->setWindowTitle(title_);

    addToWindow(window_.get());
    if (maximized)
      window_->showMaximized();
    else
      window_->show();
  }
}
