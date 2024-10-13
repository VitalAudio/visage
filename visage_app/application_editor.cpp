/* Copyright Vital Audio, LLC
 *
 * visage is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * visage is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with visage.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "application_editor.h"

#include "visage_graphics/canvas.h"
#include "visage_graphics/renderer.h"
#include "visage_windowing/windowing.h"
#include "window_event_handler.h"

namespace visage {
  ApplicationEditor::ApplicationEditor() {
    canvas_ = std::make_unique<Canvas>();
    top_level_.setCanvas(canvas_.get());
    canvas_->addRegion(top_level_.region());
    top_level_.addChild(this);

    event_handler_.request_redraw = [this](Frame* component) { stale_children_.insert(component); };
    event_handler_.request_keyboard_focus = [this](Frame* frame) {
      if (window_event_handler_)
        window_event_handler_->setKeyboardFocus(frame);
    };
    event_handler_.set_mouse_relative_mode = [this](bool relative) {
      window_->setMouseRelativeMode(relative);
    };
    event_handler_.set_cursor_style = visage::setCursorStyle;
    event_handler_.set_cursor_visible = visage::setCursorVisible;
    event_handler_.read_clipboard_text = visage::readClipboardText;
    event_handler_.set_clipboard_text = visage::setClipboardText;
    top_level_.setEventHandler(&event_handler_);
  }

  ApplicationEditor::~ApplicationEditor() = default;

  void ApplicationEditor::setCanvasDetails() {
    canvas_->setDimensions(width(), height());
    canvas_->setWidthScale(width() * 1.0f / defaultWidth());
    canvas_->setHeightScale(height() * 1.0f / defaultHeight());

    if (window_)
      canvas_->setDpiScale(window_->pixelScale());
  }

  void ApplicationEditor::resized() {
    top_level_.setBounds(0, 0, width(), height());
    setCanvasDetails();

    float dpi_scale = window_ ? window_->pixelScale() : 1.0f;
    float width_scale = width() * 1.0f / defaultWidth();
    float height_scale = height() * 1.0f / defaultHeight();
    setDimensionScaling(dpi_scale, width_scale, height_scale);
    editorResized();
  }

  void ApplicationEditor::addToWindow(Window* window) {
    window_ = window;
    Renderer::instance().checkInitialization(window_->initWindow(), window->globalDisplay());
    canvas_->pairToWindow(window_->nativeHandle(), window->clientWidth(), window->clientHeight());
    setBounds(0, 0, window->clientWidth(), window->clientHeight());

    window_event_handler_ = std::make_unique<WindowEventHandler>(window, this);

    window->setDrawCallback([this](double time) {
      canvas_->updateTime(time);
      EventManager::instance().checkEventTimers();
      drawWindow();
    });

    drawWindow();
    drawWindow();
  }

  void ApplicationEditor::removeFromWindow() {
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
    canvas_->render();
  }

  void ApplicationEditor::drawStaleChildren() {
    drawing_children_.clear();
    std::swap(stale_children_, drawing_children_);
    for (Frame* child : drawing_children_) {
      if (child->isDrawing())
        child->drawToRegion();
    }
    for (auto it = stale_children_.begin(); it != stale_children_.end();) {
      Frame* child = *it;
      if (drawing_children_.count(child) == 0) {
        child->drawToRegion();
        it = stale_children_.erase(it);
      }
      else
        ++it;
    }
    drawing_children_.clear();
  }

  WindowedEditor::~WindowedEditor() {
    removeFromWindow();
  }

  void WindowedEditor::show(float window_scale) {
    removeFromWindow();
    window_ = createScaledWindow(defaultAspectRatio(), window_scale);
    showWindow();
  }

  void WindowedEditor::show(int width, int height) {
    removeFromWindow();
    window_ = createWindow(width, height);
    showWindow();
  }

  void WindowedEditor::show(int x, int y, int width, int height) {
    removeFromWindow();
    window_ = createWindow(x, y, width, height);
    showWindow();
  }

  void WindowedEditor::showWithEventLoop(float window_scale) {
    show(window_scale);
    window_->runEventLoop();
  }

  void WindowedEditor::showWithEventLoop(int x, int y, int width, int height) {
    show(x, y, width, height);
    window_->runEventLoop();
  }

  void WindowedEditor::showWithEventLoop(int width, int height) {
    show(width, height);
    window_->runEventLoop();
  }

  void WindowedEditor::showWindow() {
    if (!title_.empty())
      window_->setWindowTitle(title_);

    addToWindow(window_.get());
    window_->show();
  }
}
