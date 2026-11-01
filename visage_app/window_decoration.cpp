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

#include "window_decoration.h"

#include "embedded/fonts.h"
#include "visage_graphics/canvas.h"

#include <algorithm>

namespace visage {

  static Bounds iconBounds(Bounds bounds) {
    float height = bounds.height();
    float width = bounds.width();
    float icon_y = (height + 2.0f) / 3.0f;
    float icon_width = height - 2.0f * icon_y;
    return { (width - icon_width) / 2.0f, icon_y, icon_width, icon_width };
  }

  ClientWindowDecoration::ClientWindowDecoration() :
      close_button_(HitTestResult::CloseButton), maximize_button_(HitTestResult::MaximizeButton),
      minimize_button_(HitTestResult::MinimizeButton) {
    static constexpr int kIconColor = 0xffffffff;

    addChild(&close_button_);
    close_button_.setColor(kCloseButtonColor);
    addChild(&maximize_button_);
    addChild(&minimize_button_);

    close_button_.onDraw() += [this](Canvas& canvas) {
      canvas.setColor(kIconColor);
      Bounds bounds = iconBounds(close_button_.localBounds());
      canvas.segment(bounds.x(), bounds.y(), bounds.right(), bounds.bottom(), 1.0f, true);
      canvas.segment(bounds.x(), bounds.bottom(), bounds.right(), bounds.y(), 1.0f, true);
    };

    maximize_button_.onDraw() += [this](Canvas& canvas) {
      canvas.setColor(kIconColor);
      Bounds bounds = iconBounds(close_button_.localBounds());
      canvas.roundedRectangleBorder(bounds.x(), bounds.y(), bounds.width(), bounds.height(), 2.0f, 1.0f);
    };

    minimize_button_.onDraw() += [this](Canvas& canvas) {
      canvas.setColor(kIconColor);
      Bounds bounds = iconBounds(close_button_.localBounds());
      canvas.rectangle(bounds.x(), bounds.yCenter() - 1.0f, bounds.width(), 1.0f);
    };
  }

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

  static void drawFrame(Canvas& canvas, const Window::FrameDecoration& decoration) {
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

  WindowFrameDecoration::WindowFrameDecoration(Window* window) :
      window_(window), canvas_(std::make_unique<Canvas>()) {
    setIgnoresMouseEvents(true, false);
    canvas_->pairToWindow(window_->frameDecorationHandle(), 1, 1, true);
    window_->setFrameDecorationCallback([this](const Window::FrameDecoration& decoration) {
      drawDecoration(decoration);
    });
  }

  WindowFrameDecoration::~WindowFrameDecoration() {
    window_->setFrameDecorationCallback(nullptr);
    canvas_->removeFromWindow();
  }

  void WindowFrameDecoration::drawDecoration(const Window::FrameDecoration& decoration) {
    float corner_radius = decoration.corner_radius / decoration.dpi_scale;
    if (corner_radius != corner_radius_) {
      corner_radius_ = corner_radius;
      redraw();
    }
    drawFrame(*canvas_, decoration);
  }

  void WindowFrameDecoration::draw(Canvas& canvas) {
    if (corner_radius_ <= 0.0f)
      return;

    canvas.setBlendMode(BlendMode::MaskMult);
    canvas.setColor(0xffffffff);
    canvas.roundedRectangle(0, -corner_radius_, width(), height() + corner_radius_, corner_radius_);
  }
}
