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

#include "application_editor.h"

#include "client_window_decoration.h"
#include "visage_graphics/canvas.h"
#include "visage_graphics/renderer.h"
#include "visage_windowing/windowing.h"
#include "window_event_handler.h"

namespace visage {
  TopLevelFrame::TopLevelFrame(ApplicationEditor* editor) : editor_(editor) { }

  TopLevelFrame::~TopLevelFrame() = default;

  void TopLevelFrame::resized() {
    if (editor_->window())
      setDpiScale(editor_->window()->dpiScale());

    editor_->setNativeBounds(nativeLocalBounds());
    editor_->setCanvasDetails();

    if (client_decoration_) {
      int decoration_width = client_decoration_->requiredWidth();
      client_decoration_->setBounds(width() - decoration_width, 0, decoration_width,
                                    client_decoration_->requiredHeight());
    }
  }

  void TopLevelFrame::addClientDecoration() {
#if !VISAGE_MAC && !VISAGE_EMSCRIPTEN
    client_decoration_ = std::make_unique<ClientWindowDecoration>();
    addChild(client_decoration_.get());
    client_decoration_->setOnTop(true);
#endif
  }

  ApplicationEditor::ApplicationEditor() : top_level_(this), canvas_(std::make_unique<Canvas>()) {
    canvas_->addRegion(top_level_.region());
    top_level_.addChild(this);

    event_handler_.request_redraw = [this](Frame* frame) { stale_children_.insert(frame); };
    event_handler_.request_keyboard_focus = [this](Frame* frame) {
      if (window_event_handler_)
        window_event_handler_->setKeyboardFocus(frame);
    };
    event_handler_.remove_from_hierarchy = [this](Frame* frame) {
      // Do not edit the hierarchy during draw() calls
      VISAGE_ASSERT(drawing_children_.empty());

      if (window_event_handler_)
        window_event_handler_->giveUpFocus(frame);
      stale_children_.erase(frame);
    };
    event_handler_.set_mouse_relative_mode = [this](bool relative) {
      window_->setMouseRelativeMode(relative);
    };
    event_handler_.set_cursor_style = visage::setCursorStyle;
    event_handler_.set_cursor_visible = visage::setCursorVisible;
    event_handler_.read_clipboard_text = visage::readClipboardText;
    event_handler_.set_clipboard_text = visage::setClipboardText;
    top_level_.setEventHandler(&event_handler_);
    onResize() += [this] { top_level_.setNativeBounds(nativeLocalBounds()); };
  }

  ApplicationEditor::~ApplicationEditor() {
    top_level_.setEventHandler(nullptr);
  }

  const Screenshot& ApplicationEditor::takeScreenshot() {
    canvas_->requestScreenshot();
    redraw();
    drawWindow();
    return canvas_->screenshot();
  }

  void ApplicationEditor::setCanvasDetails() {
    canvas_->setDimensions(nativeWidth(), nativeHeight());

    if (window_)
      canvas_->setDpiScale(window_->dpiScale());
  }

  void ApplicationEditor::addToWindow(Window* window) {
    window_ = window;
    window_->setFixedAspectRatio(fixed_aspect_ratio_);

    Renderer::instance().checkInitialization(window_->initWindow(), window->globalDisplay());
    canvas_->pairToWindow(window_->nativeHandle(), window->clientWidth(), window->clientHeight());
    top_level_.setDpiScale(window_->dpiScale());
    top_level_.setNativeBounds(0, 0, window->clientWidth(), window->clientHeight());

    window_event_handler_ = std::make_unique<WindowEventHandler>(window, &top_level_);

    window->setDrawCallback([this](double time) {
      canvas_->updateTime(time);
      EventManager::instance().checkEventTimers();
      drawWindow();
    });

    drawWindow();
    drawWindow();
    redraw();
  }

  void ApplicationEditor::setWindowless(int width, int height) {
    canvas_->removeFromWindow();
    window_ = nullptr;
    Renderer::instance().checkInitialization(headlessWindowHandle(), nullptr);
    setBounds(0, 0, width, height);
    canvas_->setWindowless(width, height);
    drawWindow();
  }

  void ApplicationEditor::removeFromWindow() {
    window_event_handler_ = nullptr;
    window_ = nullptr;
    canvas_->removeFromWindow();
  }

  void ApplicationEditor::drawWindow() {
    if (window_ && !window_->isVisible())
      return;

    if (width() == 0 || height() == 0)
      return;

    if (!initialized())
      init();

    drawStaleChildren();
    canvas_->submit();
  }

  void ApplicationEditor::setFixedAspectRatio(bool fixed) {
    fixed_aspect_ratio_ = fixed;
    if (window_)
      window_->setFixedAspectRatio(fixed);
  }

  void ApplicationEditor::drawStaleChildren() {
    drawing_children_.clear();
    std::swap(stale_children_, drawing_children_);
    for (Frame* child : drawing_children_) {
      if (child->isDrawing())
        child->drawToRegion(*canvas_);
    }
    for (auto it = stale_children_.begin(); it != stale_children_.end();) {
      Frame* child = *it;
      if (drawing_children_.count(child) == 0) {
        child->drawToRegion(*canvas_);
        it = stale_children_.erase(it);
      }
      else
        ++it;
    }
    drawing_children_.clear();
  }
}
