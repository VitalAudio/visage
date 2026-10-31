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

#include <algorithm>

namespace visage {
  static Color withAlpha(unsigned int color, float alpha) {
    return Color(color).withAlpha(Color(color).alpha() * alpha);
  }

  static Font titleFont(const Window::FrameDecoration& decoration) {
    if (decoration.title_font.empty())
      return Font(decoration.title_font_size, fonts::Lato_Regular_ttf, 1.0f);
    return Font(decoration.title_font_size, decoration.title_font, 1.0f);
  }

  static void drawFrameButton(Canvas& canvas, const Window::FrameDecoration& decoration,
                              Window::FrameButton button, IBounds bounds, unsigned int foreground,
                              float opacity) {
    if (bounds.width() == 0)
      return;

    float scale = decoration.dpi_scale;
    float alpha = 0.1f;
    if (decoration.pressed_button == button)
      alpha = 0.3f;
    else if (decoration.hovered_button == button)
      alpha = 0.15f;
    canvas.setColor(withAlpha(foreground, alpha * opacity));
    canvas.circle(bounds.x(), bounds.y(), bounds.width());

    float unit = scale;
    float x = bounds.x() + (bounds.width() - 16.0f * unit) * 0.5f;
    float y = bounds.y() + (bounds.height() - 16.0f * unit) * 0.5f;
    auto frame = [&](float left, float top, float right, float bottom) {
      canvas.rectangle(x + left * unit, y + top * unit, (right - left) * unit, unit);
      canvas.rectangle(x + left * unit, y + (bottom - 1.0f) * unit, (right - left) * unit, unit);
      canvas.rectangle(x + left * unit, y + (top + 1.0f) * unit, unit, (bottom - top - 2.0f) * unit);
      canvas.rectangle(x + (right - 1.0f) * unit, y + (top + 1.0f) * unit, unit,
                       (bottom - top - 2.0f) * unit);
    };

    canvas.setColor(withAlpha(foreground, opacity));
    if (button == Window::FrameButton::Close) {
      float thickness = 1.25f * unit;
      float center_x = x + 8.0f * unit;
      float center_y = y + 8.0f * unit;
      float arm = 3.65f * unit;
      float gap = 0.3536f * thickness;
      canvas.segment(center_x - arm, center_y - arm, center_x + arm, center_y + arm, thickness, false);
      canvas.segment(center_x - arm, center_y + arm, center_x - gap, center_y + gap, thickness, false);
      canvas.segment(center_x + gap, center_y - gap, center_x + arm, center_y - arm, thickness, false);
    }
    else if (button == Window::FrameButton::Minimize)
      canvas.rectangle(x + 4.0f * unit, y + 10.0f * unit, 8.0f * unit, unit);
    else if (decoration.maximized) {
      frame(4.0f, 6.0f, 10.0f, 12.0f);
      canvas.setColor(withAlpha(foreground, 0.5f * opacity));
      canvas.rectangle(x + 6.0f * unit, y + 4.0f * unit, 6.0f * unit, unit);
      canvas.rectangle(x + 11.0f * unit, y + 5.0f * unit, unit, 5.0f * unit);
    }
    else
      frame(4.0f, 4.0f, 12.0f, 12.0f);
  }

  static void drawWindowShadow(Canvas& canvas, IBounds window, float radius, float scale,
                               float blur, float spread, float alpha) {
    canvas.setColor(withAlpha(0xff000000, alpha));
    canvas.roundedRectangleShadow(window.x() - spread * scale, window.y() - spread * scale,
                                  window.width() + 2.0f * spread * scale,
                                  window.height() + 2.0f * spread * scale, radius + spread * scale,
                                  2.0f * blur * scale);
  }

  static void drawFrameDecoration(Canvas& canvas, const Window::FrameDecoration& decoration) {
    canvas.clearDrawnShapes();
    canvas.setDimensions(decoration.width, decoration.height);
    canvas.setDpiScale(1.0f);

    float scale = decoration.dpi_scale;
    float radius = decoration.corner_radius;
    IBounds window = decoration.window;
    if (decoration.shadow) {
      if (decoration.active) {
        drawWindowShadow(canvas, window, radius, scale, 14.0f, 5.0f, 0.15f);
        drawWindowShadow(canvas, window, radius, scale, 5.0f, 2.0f, 0.1f);
      }
      else
        drawWindowShadow(canvas, window, radius, scale, 10.0f, 5.0f, 0.08f);
      canvas.setColor(withAlpha(0xff000000, 0.05f));
      canvas.roundedRectangle(window.x() - scale, window.y() - scale, window.width() + 2.0f * scale,
                              window.height() + 2.0f * scale, radius + scale);
    }

    canvas.setColor(decoration.window_color);
    canvas.roundedRectangle(window.x(), window.y(), window.width(), window.height(), radius);

    IBounds title_bar = decoration.title_bar;
    canvas.setColor(decoration.header_color);
    canvas.roundedRectangle(title_bar.x(), title_bar.y(), title_bar.width(),
                            title_bar.height() + radius, radius);
    if (decoration.shadow) {
      canvas.setColor(withAlpha(decoration.foreground_color, 0.15f));
      canvas.roundedRectangleBorder(title_bar.x(), title_bar.y(), title_bar.width(),
                                    title_bar.height() + radius, radius, scale);
    }
    canvas.setColor(decoration.shade_color);
    canvas.rectangle(title_bar.x(), title_bar.bottom() - scale, title_bar.width(), scale);

    canvas.setClampBounds(window.x(), window.y(), window.width(), window.height());
    float opacity = decoration.active ? 1.0f : 0.5f;
    int title_right = title_bar.right();
    for (IBounds button :
         { decoration.minimize_button, decoration.maximize_button, decoration.close_button }) {
      if (button.width())
        title_right = std::min(title_right, button.x());
    }
    float padding = 12.0f * scale;
    float title_left = title_bar.x() + padding;
    float available = std::max(0.0f, title_right - padding - title_left);
    Font font = titleFont(decoration);
    float text_width = std::min(available, font.stringWidth(String(decoration.title).toUtf32()));
    float text_x = std::clamp(title_bar.x() + 0.5f * (title_bar.width() - text_width), title_left,
                              title_left + available - text_width);
    canvas.setClampBounds(title_left, title_bar.y(), available, title_bar.height());
    canvas.setColor(withAlpha(decoration.foreground_color, opacity));
    canvas.text(decoration.title, font, text_width < available ? Font::kCenter : Font::kLeft,
                text_x, title_bar.y(), text_width, title_bar.height());
    canvas.setClampBounds(window.x(), window.y(), window.width(), window.height());

    drawFrameButton(canvas, decoration, Window::FrameButton::Minimize, decoration.minimize_button,
                    decoration.foreground_color, opacity);
    drawFrameButton(canvas, decoration, Window::FrameButton::Maximize, decoration.maximize_button,
                    decoration.foreground_color, opacity);
    drawFrameButton(canvas, decoration, Window::FrameButton::Close, decoration.close_button,
                    decoration.foreground_color, opacity);
    canvas.submit();
  }

  class WindowCornerMask : public Frame {
  public:
    WindowCornerMask() { setIgnoresMouseEvents(true, false); }

    void setRadius(float radius) {
      if (radius == radius_)
        return;
      radius_ = radius;
      redraw();
    }

    void draw(Canvas& canvas) override {
      if (radius_ <= 0.0f)
        return;

      canvas.setBlendMode(BlendMode::MaskMult);
      canvas.setColor(0xffffffff);
      canvas.roundedRectangle(0, -radius_, width(), height() + radius_, radius_);
    }

  private:
    float radius_ = 0.0f;
  };

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
    if (corner_mask_)
      corner_mask_->setBounds(localBounds());
  }

  void TopLevelFrame::setCornerRadius(float radius) {
    if (corner_mask_ == nullptr) {
      if (radius <= 0.0f)
        return;
      corner_mask_ = std::make_unique<WindowCornerMask>();
      addChild(corner_mask_.get());
      corner_mask_->setOnTop(true);
      corner_mask_->setBounds(localBounds());
    }
    corner_mask_->setRadius(radius);
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
    void* frame_handle = window_->frameDecorationHandle();
    canvas_->pairToWindow(window_->graphicsHandle(), window->clientWidth(), window->clientHeight(),
                          frame_handle != nullptr);
    if (frame_handle) {
      frame_canvas_ = std::make_unique<Canvas>();
      frame_canvas_->pairToWindow(frame_handle, 1, 1, true);
      window_->setFrameDecorationCallback([this](const Window::FrameDecoration& decoration) {
        top_level_->setCornerRadius(decoration.corner_radius / decoration.dpi_scale);
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
    top_level_->setCornerRadius(0.0f);
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
