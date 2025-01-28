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
    float dpi_scale = editor_->window() ? editor_->window()->dpiScale() : 1.0f;

    float width_scale = 1.0f;
    if (editor_->referenceWidth())
      width_scale = width() * 1.0f / editor_->referenceWidth();
    float height_scale = 1.0f;
    if (editor_->referenceHeight())
      height_scale = height() * 1.0f / editor_->referenceHeight();

    setDimensionScaling(dpi_scale, width_scale, height_scale);
    editor_->setBounds(localBounds());
    editor_->setCanvasDetails();

    if (client_decoration_) {
      int decoration_width = client_decoration_->requiredWidth();
      client_decoration_->setBounds(width() - decoration_width, 0, decoration_width,
                                    client_decoration_->requiredHeight());
    }
  }

  void TopLevelFrame::addClientDecoration() {
    client_decoration_ = std::make_unique<ClientWindowDecoration>();
    addChild(client_decoration_.get());
    client_decoration_->setOnTop(true);
  }

  ApplicationEditor::ApplicationEditor() : top_level_(this) {
    pixel_scale_ = windowPixelScale();
    canvas_ = std::make_unique<Canvas>();
    canvas_->addRegion(top_level_.region());
    top_level_.addChild(this);

    event_handler_.request_redraw = [this](Frame* frame) { stale_children_.insert(frame); };
    event_handler_.request_keyboard_focus = [this](Frame* frame) {
      if (window_event_handler_)
        window_event_handler_->setKeyboardFocus(frame);
    };
    event_handler_.remove_from_hierarchy = [this](Frame* frame) {
      if (window_event_handler_)
        window_event_handler_->giveUpFocus(frame);
      stale_children_.erase(frame);
      drawing_children_.erase(frame);
    };
    event_handler_.set_mouse_relative_mode = [this](bool relative) {
      window_->setMouseRelativeMode(relative);
    };
    event_handler_.set_cursor_style = visage::setCursorStyle;
    event_handler_.set_cursor_visible = visage::setCursorVisible;
    event_handler_.read_clipboard_text = visage::readClipboardText;
    event_handler_.set_clipboard_text = visage::setClipboardText;
    top_level_.setEventHandler(&event_handler_);
    onResize() += [this] { top_level_.setBounds(localBounds()); };
  }

  ApplicationEditor::~ApplicationEditor() {
    top_level_.removeAllChildren();
  }

  void ApplicationEditor::takeScreenshot(const std::string& filename) {
    canvas_->takeScreenshot(filename);
  }

  void ApplicationEditor::setCanvasDetails() {
    canvas_->setDimensions(width(), height());
    if (referenceWidth())
      canvas_->setWidthScale(width() * 1.0f / referenceWidth());
    if (referenceHeight())
      canvas_->setHeightScale(height() * 1.0f / referenceHeight());

    if (window_)
      canvas_->setDpiScale(window_->dpiScale());
  }

  void ApplicationEditor::addToWindow(Window* window) {
    window_ = window;
    pixel_scale_ = window_->pixelScale();

    Renderer::instance().checkInitialization(window_->initWindow(), window->globalDisplay());
    canvas_->pairToWindow(window_->nativeHandle(), window->clientWidth(), window->clientHeight());
    top_level_.setBounds(0, 0, window->clientWidth(), window->clientHeight());

    window_event_handler_ = std::make_unique<WindowEventHandler>(window, &top_level_);

    window->setDrawCallback([this](double time) {
      canvas_->updateTime(time);
      EventManager::instance().checkEventTimers();
      drawWindow();
    });

#if !VISAGE_LINUX
    drawWindow();
    drawWindow();
    redraw();
#endif
  }

  void ApplicationEditor::removeFromWindow() {
    pixel_scale_ = windowPixelScale();
    window_event_handler_ = nullptr;
    window_ = nullptr;
    canvas_->removeFromWindow();
  }

  void ApplicationEditor::drawWindow() {
    if (window_ == nullptr || !window_->isVisible())
      return;

    if (!initialized())
      init();

    drawStaleChildren();
    canvas_->submit();
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
