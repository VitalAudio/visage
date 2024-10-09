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
    setCanvas(canvas_.get());
    canvas_->addRegion(region());
  }

  ApplicationEditor::~ApplicationEditor() = default;

  void ApplicationEditor::resized() {
    canvas_->setDimensions(getWidth(), getHeight());
    canvas_->setWidthScale(getWidth() * 1.0f / getDefaultWidth());
    canvas_->setHeightScale(getHeight() * 1.0f / getDefaultHeight());

    if (window_)
      canvas_->setDpiScale(window_->getPixelScale());
    editorResized();

    drawWindow();
  }

  void ApplicationEditor::addToWindow(Window* window) {
    window_ = window;
    Renderer::getInstance().checkInitialization(window_->getInitWindow(), window->getGlobalDisplay());
    canvas_->pairToWindow(window_->getNativeHandle(), window->clientWidth(), window->clientHeight());
    setBounds(0, 0, window->clientWidth(), window->clientHeight());

    window_event_handler_ = std::make_unique<WindowEventHandler>(window, this);

    window->setDrawCallback([this](double time) {
      canvas_->updateTime(time);
      EventManager::getInstance().checkEventTimers();
      drawWindow();
    });

    drawWindow();
    drawWindow();
  }

  void ApplicationEditor::removeFromWindow() {
    window_event_handler_ = nullptr;
    window_ = nullptr;
    canvas_->clearWindowHandle();
  }

  void ApplicationEditor::drawWindow() {
    if (window_ == nullptr || !window_->isVisible())
      return;

    if (!initialized())
      init();

    drawing_children_.clear();
    std::swap(stale_children_, drawing_children_);
    for (DrawableComponent* child : drawing_children_) {
      if (child->isDrawing())
        child->drawToRegion();
    }
    for (auto it = stale_children_.begin(); it != stale_children_.end();) {
      DrawableComponent* child = *it;
      if (drawing_children_.count(child) == 0) {
        child->drawToRegion();
        it = stale_children_.erase(it);
      }
      else
        ++it;
    }
    drawing_children_.clear();

    canvas_->submit();
    canvas_->render();
  }

  float ApplicationEditor::getDpiScale() {
    if (window_ == nullptr)
      return 1.0f;
    return window_->getPixelScale();
  }

  void ApplicationEditor::requestKeyboardFocus(UiFrame* frame) {
    if (window_event_handler_)
      window_event_handler_->setKeyboardFocus(frame);
  }

  void ApplicationEditor::setCursorStyle(MouseCursor style) {
    visage::setCursorStyle(style);
  }

  void ApplicationEditor::setCursorVisible(bool visible) {
    visage::setCursorVisible(visible);
  }

  std::string ApplicationEditor::getClipboardText() {
    return visage::getClipboardText();
  }

  void ApplicationEditor::setClipboardText(const std::string& text) {
    visage::setClipboardText(text);
  }

  void ApplicationEditor::setMouseRelativeMode(bool relative) {
    window_->setMouseRelativeMode(relative);
  }

  bool ApplicationEditor::requestRedraw(DrawableComponent* component) {
    stale_children_.insert(component);
    return true;
  }

  WindowedEditor::~WindowedEditor() {
    removeFromWindow();
  }

  void WindowedEditor::show(float window_scale) {
    removeFromWindow();
    window_ = createScaledWindow(getDefaultAspectRatio(), window_scale);
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

  void WindowedEditor::showWindow() {
    if (!title_.empty())
      window_->setWindowTitle(title_);

    addToWindow(window_.get());
    window_->show();
    window_->runEventThread();
  }
}
