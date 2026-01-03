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
    editor_->notifyContentsResized();

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

  ApplicationEditor::ApplicationEditor() :
      canvas_(std::make_unique<Canvas>()), top_level_(std::make_unique<TopLevelFrame>(this)) {
    canvas_->addRegion(top_level_->region());
    top_level_->addChild(this);

    event_handler_.request_redraw = [this](Frame* frame) {
      if (std::find(stale_children_.begin(), stale_children_.end(), frame) == stale_children_.end())
        stale_children_.push_back(frame);
    };
    event_handler_.request_keyboard_focus = [this](Frame* frame) {
      if (window_event_handler_)
        window_event_handler_->setKeyboardFocus(frame);
    };
    event_handler_.remove_from_hierarchy = [this](Frame* frame) {
      // Do not edit the hierarchy during draw() calls
      VISAGE_ASSERT(drawing_children_.empty());

      if (window_event_handler_)
        window_event_handler_->giveUpFocus(frame);
      auto pos = std::find(stale_children_.begin(), stale_children_.end(), frame);
      if (pos != stale_children_.end())
        stale_children_.erase(pos);
    };
    event_handler_.set_mouse_relative_mode = [this](bool relative) {
      if (window_)
        window_->setMouseRelativeMode(relative);
    };
    event_handler_.set_cursor_style = visage::setCursorStyle;
    event_handler_.set_cursor_visible = visage::setCursorVisible;
    event_handler_.read_clipboard_text = visage::readClipboardText;
    event_handler_.set_clipboard_text = visage::setClipboardText;
    top_level_->setEventHandler(&event_handler_);
    onResize() += [this] { top_level_->setNativeBounds(nativeLocalBounds()); };
  }

  ApplicationEditor::~ApplicationEditor() {
    top_level_->setEventHandler(nullptr);
  }

  void ApplicationEditor::notifyContentsResized() {
    if (window_)
      window_->setInternalWindowSize(nativeWidth(), nativeHeight());
    on_window_contents_resized_.callback();
  }

  const Screenshot& ApplicationEditor::takeScreenshot() {
    return canvas_->takeScreenshot();
  }

  void ApplicationEditor::setCanvasDetails() {
    canvas_->setDimensions(nativeWidth(), nativeHeight());

    if (window_)
      canvas_->setDpiScale(window_->dpiScale());
  }

  void ApplicationEditor::addToWindow(Window* window) {
    window_ = window;

    Renderer::instance().initialize(window_->initWindow(), window->globalDisplay());
    canvas_->pairToWindow(window_->nativeHandle(), window->clientWidth(), window->clientHeight());
    top_level_->setDpiScale(window_->dpiScale());
    top_level_->setNativeBounds(0, 0, window->clientWidth(), window->clientHeight());
    window_->setFixedAspectRatio(fixed_aspect_ratio_ != 0.0f);

    window_event_handler_ = std::make_unique<WindowEventHandler>(this, top_level_.get());
    checkFixedAspectRatio();

    window_->setDrawCallback([this](double time) {
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
    fixed_aspect_ratio_ = fixed ? aspectRatio() : 0.0f;
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
      if (std::find(drawing_children_.begin(), drawing_children_.end(), child) == drawing_children_.end()) {
        child->drawToRegion(*canvas_);
        it = stale_children_.erase(it);
      }
      else
        ++it;
    }
    drawing_children_.clear();
  }

  void ApplicationEditor::adjustWindowDimensions(int* width, int* height, bool horizontal_resize,
                                                 bool vertical_resize) const {
    int min_width = min_width_ * dpiScale();
    int min_height = min_height_ * dpiScale();
    Point min_dimensions(min_width, min_height);
    if (isFixedAspectRatio()) {
      auto max_dimensions = window_ ? Point(window_->maxWindowDimensions()) : Point(FLT_MAX, FLT_MAX);
      auto adjusted = adjustBoundsForAspectRatio({ *width * 1.0f, *height * 1.0f }, min_dimensions,
                                                 max_dimensions, fixed_aspect_ratio_,
                                                 horizontal_resize, vertical_resize);
      *width = adjusted.x;
      *height = adjusted.y;
    }
    else {
      *width = std::max(min_width, *width);
      *height = std::max(min_height, *height);
    }
  }
}
