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
#include "embedded/fonts.h"
#include "visage_graphics/canvas.h"
#include "visage_graphics/renderer.h"
#include "visage_windowing/windowing.h"
#include "window_event_handler.h"

#include <cstdio>

namespace visage {
  static constexpr float kTitleFontPoints = 11.0f;
  static constexpr float kPointsToPixels = 96.0f / 72.0f;

  static std::string commandOutput(const char* command) {
    std::string result;
#if VISAGE_LINUX
    if (FILE* pipe = popen(command, "r")) {
      char buffer[256];
      while (fgets(buffer, sizeof(buffer), pipe))
        result += buffer;
      pclose(pipe);
    }
#endif
    while (!result.empty() && (result.back() == '\n' || result.back() == '\''))
      result.pop_back();
    if (!result.empty() && result.front() == '\'')
      result.erase(0, 1);
    return result;
  }

  static Font titleFont(float dpi_scale) {
    static const std::string font_path = [] {
      std::string family =
          commandOutput("gsettings get org.gnome.desktop.interface font-name 2>/dev/null");
      size_t size_start = family.find_last_of(' ');
      if (size_start != std::string::npos)
        family = family.substr(0, size_start);
      if (family.empty())
        family = "sans-serif";
      std::string command = "fc-match -f '%{file}' '" + family + ":bold' 2>/dev/null";
      return commandOutput(command.c_str());
    }();

    float size = kTitleFontPoints * kPointsToPixels * dpi_scale;
    if (font_path.empty())
      return Font(size, fonts::Lato_Regular_ttf);
    return Font(size, font_path);
  }

  static void drawFrameButton(Canvas& canvas, const Window::FrameDecoration& decoration,
                              Window::FrameButton button, IBounds bounds, unsigned int icon_color) {
    if (bounds.width() == 0)
      return;

    float scale = decoration.dpi_scale;
    float alpha = 0.1f;
    if (decoration.pressed_button == button)
      alpha = 0.3f;
    else if (decoration.hovered_button == button)
      alpha = 0.15f;
    unsigned int shade = decoration.dark ? 0xffffff : 0x000000;
    canvas.setColor(Color((static_cast<unsigned int>(alpha * 255.0f) << 24) | shade));
    canvas.circle(bounds.x(), bounds.y(), bounds.width());

    float icon = 8.0f * scale;
    float thickness = 1.5f * scale;
    float x = bounds.x() + (bounds.width() - icon) * 0.5f;
    float y = bounds.y() + (bounds.height() - icon) * 0.5f;
    canvas.setColor(icon_color);
    if (button == Window::FrameButton::Close) {
      canvas.segment(x, y, x + icon, y + icon, thickness, true);
      canvas.segment(x, y + icon, x + icon, y, thickness, true);
    }
    else if (button == Window::FrameButton::Minimize)
      canvas.rectangle(x, y + icon - thickness, icon, thickness);
    else if (decoration.maximized) {
      float inset = 2.0f * scale;
      canvas.roundedRectangleBorder(x, y + inset, icon - inset, icon - inset, scale, thickness);
      canvas.roundedRectangleBorder(x + inset, y, icon - inset, icon - inset, scale, thickness);
    }
    else
      canvas.roundedRectangleBorder(x, y, icon, icon, scale, thickness);
  }

  static void drawFrameDecoration(Canvas& canvas, const Window::FrameDecoration& decoration) {
    static constexpr float kCornerRadius = 12.0f;

    canvas.clearDrawnShapes();
    canvas.setDimensions(decoration.width, decoration.height);
    canvas.setDpiScale(1.0f);

    float scale = decoration.dpi_scale;
    IBounds window = decoration.window;
    float radius = decoration.maximized ? 0.0f : kCornerRadius * scale;
    if (!decoration.maximized) {
      float spread = (decoration.active ? 26.0f : 14.0f) * scale;
      float offset = (decoration.active ? 4.0f : 2.0f) * scale;
      canvas.setColor(decoration.active ? 0x38000000 : 0x2a000000);
      canvas.roundedRectangleShadow(window.x(), window.y() + offset, window.width(),
                                    window.height(), radius, 2.0f * spread);
      canvas.setColor(decoration.active ? 0x24000000 : 0x18000000);
      canvas.roundedRectangleShadow(window.x(), window.y() + 0.5f * offset, window.width(),
                                    window.height(), radius, spread);
      canvas.setColor(decoration.dark ? 0x80000000 : 0x24000000);
      canvas.roundedRectangle(window.x() - scale, window.y() - scale, window.width() + 2.0f * scale,
                              window.height() + 2.0f * scale, radius + scale);
    }

    unsigned int header = decoration.dark ? (decoration.active ? 0xff303030 : 0xff242424) :
                                            (decoration.active ? 0xffebebeb : 0xfffafafa);
    unsigned int window_color = decoration.dark ? 0xff242424 : 0xfffafafa;
    canvas.setColor(window_color);
    canvas.roundedRectangle(window.x(), window.y(), window.width(), window.height(), radius);
    canvas.setColor(header);
    IBounds title_bar = decoration.title_bar;
    canvas.roundedRectangle(title_bar.x(), title_bar.y(), title_bar.width(),
                            title_bar.height() + radius, radius);
    canvas.setColor(decoration.dark ? 0xff1f1f1f : 0xffd9d9d9);
    canvas.rectangle(title_bar.x(), title_bar.bottom() - scale, title_bar.width(), scale);

    unsigned int foreground = decoration.dark ? 0xffffffff : 0xff000000;
    unsigned int title_color = (decoration.active ? 0xcc000000 : 0x80000000) | (foreground & 0xffffff);
    canvas.setColor(title_color);
    canvas.text(decoration.title, titleFont(scale), Font::kCenter, title_bar.x(), title_bar.y(),
                title_bar.width(), title_bar.height());

    unsigned int icon_color = (decoration.active ? 0xff000000 : 0x80000000) | (foreground & 0xffffff);
    drawFrameButton(canvas, decoration, Window::FrameButton::Minimize, decoration.minimize_button,
                    icon_color);
    drawFrameButton(canvas, decoration, Window::FrameButton::Maximize, decoration.maximize_button,
                    icon_color);
    drawFrameButton(canvas, decoration, Window::FrameButton::Close, decoration.close_button, icon_color);
    canvas.submit();
  }

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
    VISAGE_ASSERT(!hasStaticStorage(this));
    canvas_->addRegion(top_level_->region());
    top_level_->addChild(this);

    event_handler_.request_redraw = [this](Frame* frame) {
      if (stale_set_.insert(frame).second)
        stale_children_.push_back(frame);
    };
    event_handler_.request_keyboard_focus = [this](Frame* frame) {
      if (window_event_handler_)
        window_event_handler_->setKeyboardFocus(frame);
    };
    event_handler_.keyboard_focused_frame = [this]() -> Frame* {
      return window_event_handler_ ? window_event_handler_->keyboardFocusedFrame() : nullptr;
    };
    event_handler_.remove_from_hierarchy = [this](Frame* frame) {
      // Do not edit the hierarchy during draw() calls
      VISAGE_ASSERT(drawing_children_.empty());

      if (window_event_handler_)
        window_event_handler_->giveUpFocus(frame);
      if (stale_set_.erase(frame))
        stale_children_.erase(std::find(stale_children_.begin(), stale_children_.end(), frame));
    };
    event_handler_.set_mouse_relative_mode = [this](bool relative) {
      if (window_)
        window_->setMouseRelativeMode(relative);
    };
    event_handler_.take_mouse_drag = [this](Frame* frame) {
      return window_event_handler_ && window_event_handler_->takeMouseDrag(frame);
    };
    event_handler_.set_cursor_style = visage::setCursorStyle;
    event_handler_.set_cursor_visible = visage::setCursorVisible;
    event_handler_.read_clipboard_text = visage::readClipboardText;
    event_handler_.set_clipboard_text = visage::setClipboardText;
    top_level_->setEventHandler(&event_handler_);
    onResize() += [this] { top_level_->setNativeBounds(nativeLocalBounds()); };
  }

  ApplicationEditor::~ApplicationEditor() {
    removeFromWindow();
    top_level_->setEventHandler(nullptr);
  }

  void ApplicationEditor::notifyContentsResized() {
    if (window_ == nullptr ||
        (window_->clientWidth() == nativeWidth() && window_->clientHeight() == nativeHeight()))
      return;

    window_->setInternalWindowSize(nativeWidth(), nativeHeight());
    on_window_contents_resized_.callback();
  }

  const Screenshot& ApplicationEditor::takeScreenshot() {
    return canvas_->takeScreenshot();
  }

  void ApplicationEditor::setCanvasDetails() {
    canvas_->setDimensions(nativeWidth(), nativeHeight());
    canvas_->setDpiScale(window_ ? window_->dpiScale() : dpiScale());
  }

  void ApplicationEditor::addToWindow(Window* window) {
    window_ = window;

    Renderer::instance().initialize(window_->initWindow(), window->globalDisplay(),
                                    window->waylandDisplay());
    canvas_->pairToWindow(window_->graphicsHandle(), window->clientWidth(), window->clientHeight());
    if (void* frame_handle = window_->frameDecorationHandle()) {
      frame_canvas_ = std::make_unique<Canvas>();
      frame_canvas_->pairToWindow(frame_handle, 1, 1, true);
      window_->setFrameDecorationCallback([this](const Window::FrameDecoration& decoration) {
        if (frame_canvas_)
          drawFrameDecoration(*frame_canvas_, decoration);
      });
    }
    top_level_->setDpiScale(window_->dpiScale());
    top_level_->setNativeBounds(0, 0, window->clientWidth(), window->clientHeight());
    window_->setFixedAspectRatio(fixed_aspect_ratio_ != 0.0f);

    window_event_handler_ = std::make_unique<WindowEventHandler>(this, top_level_.get());
    checkFixedAspectRatio();

    window_->setDrawCallback([this, alive = aliveToken()](double time) {
      canvas_->updateTime(time);
      EventManager::instance().checkEventTimers();
      if (!alive.expired() && window_)
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
    canvas_->setWindowless(nativeWidth(), nativeHeight());
    drawWindow();
  }

  void ApplicationEditor::removeFromWindow() {
    window_event_handler_ = nullptr;
    if (window_) {
      window_->setFrameWaiter(nullptr);
      window_->setDrawCallback(nullptr);
      window_->setFrameDecorationCallback(nullptr);
    }
    if (frame_canvas_) {
      frame_canvas_->removeFromWindow();
      frame_canvas_ = nullptr;
    }
    window_ = nullptr;
    waited_frame_buffer_id_ = -1;
    canvas_->removeFromWindow();
  }

  void ApplicationEditor::drawWindow() {
    if (window_ && !window_->isVisible())
      return;

    if (width() == 0 || height() == 0)
      return;

    if (!initialized())
      init();

    if (window_)
      window_->waitForFrame();

    int frame = canvas_->frameCount();
    drawStaleChildren();
    canvas_->submit();
    if (window_ == nullptr)
      return;

    if (canvas_->frameCount() != frame)
      window_->framePresented();

    int frame_buffer_id = canvas_->frameBufferId();
    if (frame_buffer_id != waited_frame_buffer_id_) {
      waited_frame_buffer_id_ = frame_buffer_id;
      window_->setFrameWaiter([frame_buffer_id](int timeout_ms) {
        return Canvas::waitForFrame(frame_buffer_id, timeout_ms);
      });
    }
  }

  void ApplicationEditor::setFixedAspectRatio(bool fixed) {
    fixed_aspect_ratio_ = fixed ? aspectRatio() : 0.0f;
    if (window_)
      window_->setFixedAspectRatio(fixed);
  }

  void ApplicationEditor::drawStaleChildren() {
    drawing_children_.clear();
    drawing_set_.clear();
    std::swap(stale_children_, drawing_children_);
    std::swap(stale_set_, drawing_set_);
    for (Frame* child : drawing_children_) {
      if (child->isDrawing())
        child->drawToRegion(*canvas_);
    }
    for (int i = 0; i < stale_children_.size();) {
      Frame* child = stale_children_[i];
      if (drawing_set_.insert(child).second) {
        stale_children_.erase(stale_children_.begin() + i);
        stale_set_.erase(child);
        drawing_children_.push_back(child);
        child->drawToRegion(*canvas_);
      }
      else
        ++i;
    }
    drawing_children_.clear();
    drawing_set_.clear();
  }

  void ApplicationEditor::adjustWindowDimensions(int* width, int* height, bool horizontal_resize,
                                                 bool vertical_resize) const {
    float dpi_scale = window_ ? window_->dpiScale() : dpiScale();
    int min_width = min_width_ * dpi_scale;
    int min_height = min_height_ * dpi_scale;
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
