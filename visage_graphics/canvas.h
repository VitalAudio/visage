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

#pragma once

#include "font.h"
#include "graphics_utils.h"
#include "layer.h"
#include "region.h"
#include "screenshot.h"
#include "shape_batcher.h"
#include "text.h"
#include "theme.h"
#include "visage_utils/dimension.h"
#include "visage_utils/space.h"
#include "visage_utils/time_utils.h"

namespace visage {
  class Palette;
  class Shader;

  class Canvas {
  public:
    static constexpr float kDefaultSquirclePower = 4.0f;

    static bool swapChainSupported();

    struct State {
      float x = 0;
      float y = 0;
      float scale = 1.0f;
      theme::OverrideId palette_override;
      const PackedBrush* brush = nullptr;
      ClampBounds clamp;
      BlendMode blend_mode = BlendMode::Alpha;
      Region* current_region = nullptr;
    };

    Canvas();
    Canvas(const Canvas& other) = delete;
    Canvas& operator=(const Canvas&) = delete;

    void clearDrawnShapes();
    int submit(int submit_pass = 0);

    void requestScreenshot();
    const Screenshot& screenshot() const;

    void ensureLayerExists(int layer);
    Layer* layer(int index) {
      ensureLayerExists(index);
      return layers_[index];
    }

    void invalidateRectInRegion(IBounds rect, const Region* region, int layer);
    void addToPackedLayer(Region* region, int layer_index);
    void removeFromPackedLayer(const Region* region, int layer_index);
    void changePackedLayer(Region* region, int from, int to);

    void pairToWindow(void* window_handle, int width, int height) {
      VISAGE_ASSERT(swapChainSupported());
      composite_layer_.pairToWindow(window_handle, width, height);
      setDimensions(width, height);
    }

    void setWindowless(int width, int height) { composite_layer_.setHeadlessRender(width, height); }

    void removeFromWindow() { composite_layer_.removeFromWindow(); }

    void setDimensions(int width, int height);
    void setDpiScale(float scale) { dpi_scale_ = scale; }
    void setNativePixelScale() { state_.scale = 1.0f; }
    void setLogicalPixelScale() { state_.scale = dpi_scale_; }

    float dpiScale() const { return dpi_scale_; }
    void updateTime(double time);
    double time() const { return render_time_; }
    double deltaTime() const { return delta_time_; }
    int frameCount() const { return render_frame_; }

    void setBlendMode(BlendMode blend_mode) { state_.blend_mode = blend_mode; }
    void setBrush(const Brush& brush) {
      state_.brush = state_.current_region->addBrush(&gradient_atlas_, brush.gradient(),
                                                     brush.position() * state_.scale);
    }
    void setColor(const Brush& brush) { setBrush(brush); }
    void setColor(unsigned int color) { setBrush(Brush::solid(color)); }
    void setColor(const Color& color) { setBrush(Brush::solid(color)); }
    void setColor(theme::ColorId color_id) { setBrush(color(color_id)); }

    void setBlendedColor(theme::ColorId color_from, theme::ColorId color_to, float t) {
      setBrush(blendedColor(color_from, color_to, t));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void fill(const T1& x, const T2& y, const T3& width, const T4& height) {
      float fill_x = pixels(x);
      float fill_y = pixels(y);
      float fill_w = pixels(width);
      float fill_h = pixels(height);
      addShape(Fill(state_.clamp.clamp(fill_x, fill_y, fill_w, fill_h), state_.brush,
                    state_.x + fill_x, state_.y + fill_y, fill_w, fill_h));
    }

    template<typename T1, typename T2, typename T3>
    void circle(const T1& x, const T2& y, const T3& width) {
      addShape(Circle(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y),
                      pixels(width)));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void fadeCircle(const T1& x, const T2& y, const T3& width, const T4& pixel_width) {
      Circle circle(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y), pixels(width));
      circle.pixel_width = pixels(pixel_width);
      addShape(circle);
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void ring(const T1& x, const T2& y, const T3& width, const T4& thickness) {
      Circle circle(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y), pixels(width));
      circle.thickness = pixels(thickness);
      addShape(circle);
    }

    template<typename T1, typename T2, typename T3>
    void squircle(const T1& x, const T2& y, const T3& width, float power = kDefaultSquirclePower) {
      float w = pixels(width);
      addShape(Squircle(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y), w, w, power));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void squircleBorder(const T1& x, const T2& y, const T3& width, float power, const T5& thickness) {
      float w = pixels(width);
      Squircle squircle(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y), w, w, power);
      squircle.thickness = pixels(thickness);
      addShape(squircle);
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void superEllipse(const T1& x, const T2& y, const T3& width, const T4& height, const T5& power) {
      addShape(Squircle(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y),
                        pixels(width), pixels(height), pixels(power)));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void roundedArc(const T1& x, const T2& y, const T3& width, const T4& thickness,
                    float center_radians, float radians) {
      float w = pixels(width);
      addShape(RoundedArc(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y), w,
                          w, pixels(thickness) + 1.0f, center_radians, radians));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void flatArc(const T1& x, const T2& y, const T3& width, const T4& thickness,
                 float center_radians, float radians) {
      float w = pixels(width);
      addShape(FlatArc(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y), w, w,
                       pixels(thickness) + 1.0f, center_radians, radians));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void arc(const T1& x, const T2& y, const T3& width, const T4& thickness, float center_radians,
             float radians, bool rounded = false) {
      if (rounded)
        roundedArc(x, y, width, thickness, center_radians, radians);
      else
        flatArc(x, y, width, thickness, center_radians, radians);
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void roundedArcShadow(const T1& x, const T2& y, const T3& width, const T4& thickness,
                          float center_radians, float radians, const T5& shadow_width) {
      float shadow = pixels(shadow_width);
      float full_width = pixels(width) + 2.0f * shadow;
      RoundedArc arc(state_.clamp, state_.brush, state_.x + pixels(x) - shadow,
                     state_.y + pixels(y) - shadow, full_width, full_width,
                     pixels(thickness) + 1.0f + 2.0f * shadow, center_radians, radians);
      arc.pixel_width = shadow;
      addShape(arc);
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void flatArcShadow(const T1& x, const T2& y, const T3& width, const T4& thickness,
                       float center_radians, float radians, const T5& shadow_width) {
      float shadow = pixels(shadow_width);
      float full_width = pixels(width) + 2.0f * shadow;
      FlatArc arc(state_.clamp, state_.brush, state_.x + pixels(x) - shadow,
                  state_.y + pixels(y) - shadow, full_width, full_width,
                  pixels(thickness) + 1.0f + 2.0f * shadow, center_radians, radians);
      arc.pixel_width = shadow;
      addShape(arc);
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void segment(const T1& a_x, const T2& a_y, const T3& b_x, const T4& b_y, const T5& thickness,
                 bool rounded) {
      addSegment(pixels(a_x), pixels(a_y), pixels(b_x), pixels(b_y), pixels(thickness), rounded);
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
    void quadratic(const T1& a_x, const T2& a_y, const T3& b_x, const T4& b_y, const T5& c_x,
                   const T6& c_y, const T7& thickness) {
      addQuadratic(pixels(a_x), pixels(a_y), pixels(b_x), pixels(b_y), pixels(c_x), pixels(c_y),
                   pixels(thickness));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void rectangle(const T1& x, const T2& y, const T3& width, const T4& height) {
      addShape(Rectangle(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y),
                         pixels(width), pixels(height)));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void rectangleBorder(const T1& x, const T2& y, const T3& width, const T4& height, const T5& thickness) {
      Rectangle border(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y),
                       pixels(width), pixels(height));
      border.thickness = pixels(thickness) + 1.0f;
      addShape(border);
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void roundedRectangle(const T1& x, const T2& y, const T3& width, const T4& height, const T5& rounding) {
      addShape(RoundedRectangle(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y),
                                pixels(width), pixels(height), std::max(1.0f, pixels(rounding))));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void diamond(const T1& x, const T2& y, const T3& width, const T4& rounding) {
      float w = pixels(width);
      addShape(Diamond(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y), w, w,
                       std::max(1.0f, pixels(rounding))));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void leftRoundedRectangle(const T1& x, const T2& y, const T3& width, const T4& height,
                              const T5& rounding) {
      addLeftRoundedRectangle(pixels(x), pixels(y), pixels(width), pixels(height), pixels(rounding));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void rightRoundedRectangle(const T1& x, const T2& y, const T3& width, const T4& height,
                               const T5& rounding) {
      addRightRoundedRectangle(pixels(x), pixels(y), pixels(width), pixels(height), pixels(rounding));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void topRoundedRectangle(const T1& x, const T2& y, const T3& width, const T4& height, const T5& rounding) {
      addTopRoundedRectangle(pixels(x), pixels(y), pixels(width), pixels(height), pixels(rounding));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void bottomRoundedRectangle(const T1& x, const T2& y, const T3& width, const T4& height,
                                const T5& rounding) {
      addBottomRoundedRectangle(pixels(x), pixels(y), pixels(width), pixels(height), pixels(rounding));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void rectangleShadow(const T1& x, const T2& y, const T3& width, const T4& height, const T5& blur_radius) {
      addRectangleShadow(pixels(x), pixels(y), pixels(width), pixels(height), pixels(blur_radius));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
    void roundedRectangleShadow(const T1& x, const T2& y, const T3& width, const T4& height,
                                const T5& rounding, const T6& blur_radius) {
      addRoundedRectangleShadow(pixels(x), pixels(y), pixels(width), pixels(height),
                                pixels(rounding), pixels(blur_radius));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
    void roundedRectangleBorder(const T1& x, const T2& y, const T3& width, const T4& height,
                                const T5& rounding, const T6& thickness) {
      addRoundedRectangleBorder(pixels(x), pixels(y), pixels(width), pixels(height),
                                pixels(rounding), pixels(thickness));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
    void triangle(const T1& a_x, const T2& a_y, const T3& b_x, const T4& b_y, const T5& c_x, const T6& c_y) {
      outerRoundedTriangleBorder(pixels(a_x), pixels(a_y), pixels(b_x), pixels(b_y), pixels(c_x),
                                 pixels(c_y), 0.0f);
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
    void triangleBorder(const T1& a_x, const T2& a_y, const T3& b_x, const T4& b_y, const T5& c_x,
                        const T6& c_y, const T7& thickness) {
      outerRoundedTriangleBorder(pixels(a_x), pixels(a_y), pixels(b_x), pixels(b_y), pixels(c_x),
                                 pixels(c_y), 0.0f, pixels(thickness));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7, typename T8>
    void roundedTriangleBorder(const T1& a_x, const T2& a_y, const T3& b_x, const T4& b_y,
                               const T5& c_x, const T6& c_y, const T7& rounding, const T8& thickness) {
      addRoundedTriangleBorder(pixels(a_x), pixels(a_y), pixels(b_x), pixels(b_y), pixels(c_x),
                               pixels(c_y), pixels(rounding), pixels(thickness));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
    void roundedTriangle(const T1& a_x, const T2& a_y, const T3& b_x, const T4& b_y, const T5& c_x,
                         const T6& c_y, const T7& rounding) {
      addRoundedTriangleBorder(pixels(a_x), pixels(a_y), pixels(b_x), pixels(b_y), pixels(c_x),
                               pixels(c_y), pixels(rounding), -1.0f);
    }

    template<typename T1, typename T2, typename T3>
    void triangleLeft(const T1& triangle_x, const T2& triangle_y, const T3& triangle_width) {
      float x = pixels(triangle_x);
      float y = pixels(triangle_y);
      float width = pixels(triangle_width);
      float h = width * 2.0f;
      outerRoundedTriangleBorder(x + width, y, x + width, y + h, x, y + h * 0.5f, 0.0f, width);
    }

    template<typename T1, typename T2, typename T3>
    void triangleRight(const T1& triangle_x, const T2& triangle_y, const T3& triangle_width) {
      float x = pixels(triangle_x);
      float y = pixels(triangle_y);
      float width = pixels(triangle_width);
      float h = width * 2.0f;
      outerRoundedTriangleBorder(x, y, x, y + h, x + width, y + h * 0.5f, 0.0f, width);
    }

    template<typename T1, typename T2, typename T3>
    void triangleUp(const T1& triangle_x, const T2& triangle_y, const T3& triangle_width) {
      float x = pixels(triangle_x);
      float y = pixels(triangle_y);
      float width = pixels(triangle_width);
      float w = width * 2.0f;
      outerRoundedTriangleBorder(x, y + width, x + w, y + width, x + w * 0.5f, y, 0.0f, width);
    }

    template<typename T1, typename T2, typename T3>
    void triangleDown(const T1& triangle_x, const T2& triangle_y, const T3& triangle_width) {
      float x = pixels(triangle_x);
      float y = pixels(triangle_y);
      float width = pixels(triangle_width);
      float w = width * 2.0f;
      outerRoundedTriangleBorder(x, y, x + w, y, x + w * 0.5f, y + width, 0.0f, width);
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void text(Text* text, const T1& x, const T2& y, const T3& width, const T4& height,
              Direction dir = Direction::Up) {
      TextBlock text_block(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y),
                           pixels(width), pixels(height), text,
                           text->font().withDpiScale(state_.scale), dir);
      addShape(std::move(text_block));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void text(const String& string, const Font& font, Font::Justification justification, const T1& x,
              const T2& y, const T3& width, const T4& height, Direction dir = Direction::Up) {
      if (!string.isEmpty()) {
        Text* stored_text = state_.current_region->addText(string, font, justification);
        text(stored_text, x, y, width, height, dir);
      }
    }

    template<typename T1, typename T2>
    void svg(const Svg& svg, const T1& x, const T2& y) {
      int radius = std::round(pixels(svg.blur_radius));
      int w = std::round(pixels(svg.width));
      int h = std::round(pixels(svg.height));
      addSvg({ svg.data, svg.data_size, w, h, radius }, pixels(x), pixels(y));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void svg(const unsigned char* svg_data, int svg_size, const T1& x, const T2& y, const T3& width,
             const T4& height, const T5& blur_radius) {
      int w = std::round(pixels(width));
      int h = std::round(pixels(height));
      int radius = std::round(pixels(blur_radius));
      addSvg({ svg_data, svg_size, w, h, radius }, pixels(x), pixels(y));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void svg(const unsigned char* svg_data, int svg_size, const T1& x, const T2& y, const T3& width,
             const T4& height) {
      int w = std::round(pixels(width));
      int h = std::round(pixels(height));
      addSvg({ svg_data, svg_size, w, h, 0 }, pixels(x), pixels(y));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void svg(const EmbeddedFile& file, const T1& x, const T2& y, const T3& width, const T4& height,
             const T5& blur_radius) {
      svg(file.data, file.size, x, y, width, height, blur_radius);
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void svg(const EmbeddedFile& file, const T1& x, const T2& y, const T3& width, const T4& height) {
      svg(file.data, file.size, x, y, width, height);
    }

    template<typename T1, typename T2>
    void image(const Image& image, const T1& x, const T2& y) {
      int radius = std::round(pixels(image.blur_radius));
      int w = std::round(pixels(image.width));
      int h = std::round(pixels(image.height));
      addImage({ image.data, image.data_size, w, h, radius }, pixels(x), pixels(y));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void image(const char* image_data, int image_size, const T1& x, const T2& y, const T3& width,
               const T4& height) {
      int w = std::round(pixels(width));
      int h = std::round(pixels(height));
      addImage({ image_data, image_size, w, h }, pixels(x), pixels(y));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void image(const EmbeddedFile& image_file, const T1& x, const T2& y, const T3& width, const T4& height) {
      image(image_file.data, image_file.size, x, y, width, height);
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void shader(Shader* shader, const T1& x, const T2& y, const T3& width, const T4& height) {
      addShape(ShaderWrapper(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y),
                             pixels(width), pixels(height), shader));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void line(Line* line, const T1& x, const T2& y, const T3& width, const T4& height, const T5& line_width) {
      addShape(LineWrapper(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y),
                           pixels(width), pixels(height), line, pixels(line_width), state_.scale));
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void lineFill(Line* line, const T1& x, const T2& y, const T3& width, const T4& height,
                  const T5& fill_position) {
      addShape(LineFillWrapper(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y),
                               pixels(width), pixels(height), line, pixels(fill_position), state_.scale));
    }

    void saveState() { state_memory_.push_back(state_); }

    void restoreState() {
      state_ = state_memory_.back();
      state_memory_.pop_back();
    }

    void setPosition(float x, float y) {
      state_.x += x * state_.scale;
      state_.y += y * state_.scale;
    }

    void addRegion(Region* region) {
      default_region_.addRegion(region);
      region->setCanvas(this);
    }

    void beginRegion(Region* region) {
      region->clear();
      saveState();
      state_.x = 0;
      state_.y = 0;
      setLogicalPixelScale();
      state_.brush = nullptr;
      state_.blend_mode = BlendMode::Alpha;
      setClampBounds(0, 0, region->width(), region->height());
      state_.current_region = region;
    }

    void endRegion() { restoreState(); }

    void setPalette(Palette* palette) { palette_ = palette; }
    void setPaletteOverride(theme::OverrideId override_id) {
      state_.palette_override = override_id;
    }

    void setClampBounds(float x, float y, float width, float height) {
      VISAGE_ASSERT(width >= 0);
      VISAGE_ASSERT(height >= 0);
      state_.clamp.left = state_.x + x * state_.scale;
      state_.clamp.top = state_.y + y * state_.scale;
      state_.clamp.right = state_.clamp.left + width * state_.scale;
      state_.clamp.bottom = state_.clamp.top + height * state_.scale;
    }

    void setClampBounds(const ClampBounds& bounds) { state_.clamp = bounds; }

    void trimClampBounds(int x, int y, int width, int height) {
      state_.clamp = state_.clamp.clamp(state_.x + x * state_.scale, state_.y + y * state_.scale,
                                        width * state_.scale, height * state_.scale);
    }

    const ClampBounds& currentClampBounds() const { return state_.clamp; }
    bool totallyClamped() const { return state_.clamp.totallyClamped(); }

    Brush color(theme::ColorId color_id);
    Brush blendedColor(theme::ColorId color_from, theme::ColorId color_to, float t) {
      return color(color_from).interpolateWith(color(color_to), t);
    }

    float value(theme::ValueId value_id);
    std::vector<std::string> debugInfo() const;

    ImageAtlas* imageAtlas() { return &image_atlas_; }
    GradientAtlas* gradientAtlas() { return &gradient_atlas_; }

    State* state() { return &state_; }

  private:
    template<typename T>
    constexpr float pixels(T&& value) {
      if constexpr (std::is_same_v<std::decay_t<T>, Dimension>)
        return value.compute(state_.scale, state_.current_region->width(),
                             state_.current_region->height());
      else
        return state_.scale * value;
    }

    template<typename T>
    void addShape(T shape) {
      state_.current_region->shape_batcher_.addShape(std::move(shape), state_.blend_mode);
    }

    void addSegment(float a_x, float a_y, float b_x, float b_y, float thickness,
                    bool rounded = false, float pixel_width = 1.0f) {
      float x = std::min(a_x, b_x) - thickness;
      float width = std::max(a_x, b_x) + thickness - x;
      float y = std::min(a_y, b_y) - thickness;
      float height = std::max(a_y, b_y) + thickness - y;

      float x1 = 2.0f * (a_x - x) / width - 1.0f;
      float y1 = 2.0f * (a_y - y) / height - 1.0f;
      float x2 = 2.0f * (b_x - x) / width - 1.0f;
      float y2 = 2.0f * (b_y - y) / height - 1.0f;

      if (rounded) {
        addShape(RoundedSegment(state_.clamp, state_.brush, state_.x + x, state_.y + y, width,
                                height, x1, y1, x2, y2, thickness + 1.0f, pixel_width));
      }
      else {
        addShape(FlatSegment(state_.clamp, state_.brush, state_.x + x, state_.y + y, width, height,
                             x1, y1, x2, y2, thickness + 1.0f, pixel_width));
      }
    }

    void addQuadratic(float a_x, float a_y, float b_x, float b_y, float c_x, float c_y,
                      float thickness, float pixel_width = 1.0f) {
      if (tryDrawCollinearQuadratic(a_x, a_y, b_x, b_y, c_x, c_y, thickness, pixel_width))
        return;

      float x = std::min(std::min(a_x, b_x), c_x) - thickness;
      float width = std::max(std::max(a_x, b_x), c_x) + thickness - x;
      float y = std::min(std::min(a_y, b_y), c_y) - thickness;
      float height = std::max(std::max(a_y, b_y), c_y) + thickness - y;

      float x1 = 2.0f * (a_x - x) / width - 1.0f;
      float y1 = 2.0f * (a_y - y) / height - 1.0f;
      float x2 = 2.0f * (b_x - x) / width - 1.0f;
      float y2 = 2.0f * (b_y - y) / height - 1.0f;
      float x3 = 2.0f * (c_x - x) / width - 1.0f;
      float y3 = 2.0f * (c_y - y) / height - 1.0f;

      addShape(QuadraticBezier(state_.clamp, state_.brush, state_.x + x, state_.y + y, width,
                               height, x1, y1, x2, y2, x3, y3, thickness + 1.0f, pixel_width));
    }

    void addLeftRoundedRectangle(float x, float y, float width, float height, float rounding) {
      ClampBounds clamp = state_.clamp;
      clamp.right = std::min(clamp.right, state_.x + x + width);
      addShape(RoundedRectangle(clamp, state_.brush, state_.x + x, state_.y + y,
                                width + rounding + 1.0f, height, std::max(1.0f, rounding)));
    }

    void addRightRoundedRectangle(float x, float y, float width, float height, float rounding) {
      ClampBounds clamp = state_.clamp;
      clamp.left = std::max(clamp.left, state_.x + x);
      float growth = rounding + 1.0f;
      addShape(RoundedRectangle(clamp, state_.brush, state_.x + x - growth, state_.y + y,
                                width + growth, height, std::max(1.0f, rounding)));
    }

    void addTopRoundedRectangle(float x, float y, float width, float height, float rounding) {
      ClampBounds clamp = state_.clamp;
      clamp.bottom = std::min(clamp.bottom, state_.y + y + height);
      addShape(RoundedRectangle(clamp, state_.brush, state_.x + x, state_.y + y, width,
                                height + rounding + 1.0f, std::max(1.0f, rounding)));
    }

    void addBottomRoundedRectangle(float x, float y, float width, float height, float rounding) {
      ClampBounds clamp = state_.clamp;
      clamp.top = std::max(clamp.top, state_.y + y);
      float growth = rounding + 1.0f;
      addShape(RoundedRectangle(clamp, state_.brush, state_.x + x, state_.y + y - growth, width,
                                height + growth, std::max(1.0f, rounding)));
    }

    void addRectangleShadow(float x, float y, float width, float height, float blur_radius) {
      if (blur_radius > 0.0f) {
        Rectangle rectangle(state_.clamp, state_.brush, state_.x + x, state_.y + y, width, height);
        rectangle.pixel_width = blur_radius;
        addShape(rectangle);
      }
    }

    void addRoundedRectangleShadow(float x, float y, float width, float height, float rounding,
                                   float blur_radius) {
      if (blur_radius <= 0.0f)
        return;

      float offset = -blur_radius * 0.5f;
      if (rounding <= 1.0f)
        rectangleShadow(state_.x + x + offset, state_.y + y + offset, width + blur_radius,
                        height + blur_radius, blur_radius);
      else {
        RoundedRectangle shadow(state_.clamp, state_.brush, state_.x + x + offset, state_.y + y + offset,
                                width + blur_radius, height + blur_radius, rounding);
        shadow.pixel_width = blur_radius;
        addShape(shadow);
      }
    }

    void addRoundedRectangleBorder(float x, float y, float width, float height, float rounding,
                                   float thickness) {
      saveState();
      float left = state_.clamp.left;
      float right = state_.clamp.right;
      float top = state_.clamp.top;
      float bottom = state_.clamp.bottom;

      float part = std::max(rounding, thickness);
      state_.clamp.right = std::min(right, state_.x + x + part + 1.0f);
      fullRoundedRectangleBorder(x, y, width, height, rounding, thickness);
      state_.clamp.right = right;
      state_.clamp.left = std::max(left, state_.x + x + width - part - 1.0f);
      fullRoundedRectangleBorder(x, y, width, height, rounding, thickness);

      state_.clamp.left = std::max(left, state_.x + x + part + 1.0f);
      state_.clamp.right = std::min(right, state_.x + x + width - part - 1.0f);
      state_.clamp.bottom = std::min(bottom, state_.y + y + part + 1.0f);
      fullRoundedRectangleBorder(x, y, width, height, rounding, thickness);
      state_.clamp.bottom = bottom;
      state_.clamp.top = std::max(top, state_.y + y + height - part - 1.0f);
      fullRoundedRectangleBorder(x, y, width, height, rounding, thickness);

      restoreState();
    }

    void addRoundedTriangleBorder(float a_x, float a_y, float b_x, float b_y, float c_x, float c_y,
                                  float rounding, float thickness) {
      float d_ab = sqrtf((a_x - b_x) * (a_x - b_x) + (a_y - b_y) * (a_y - b_y));
      float d_bc = sqrtf((b_x - c_x) * (b_x - c_x) + (b_y - c_y) * (b_y - c_y));
      float d_ca = sqrtf((c_x - a_x) * (c_x - a_x) + (c_y - a_y) * (c_y - a_y));
      float perimeter = d_ab + d_bc + d_ca;
      float inscribed_circle_x = (d_bc * a_x + d_ca * b_x + d_ab * c_x) / perimeter;
      float inscribed_circle_y = (d_bc * a_y + d_ca * b_y + d_ab * c_y) / perimeter;
      float s = perimeter * 0.5f;
      float inscribed_circle_radius = sqrtf(s * (s - d_ab) * (s - d_bc) * (s - d_ca)) / s;

      rounding = std::min(rounding, inscribed_circle_radius);
      float shrinking = rounding / inscribed_circle_radius;
      outerRoundedTriangleBorder(a_x + (inscribed_circle_x - a_x) * shrinking,
                                 a_y + (inscribed_circle_y - a_y) * shrinking,
                                 b_x + (inscribed_circle_x - b_x) * shrinking,
                                 b_y + (inscribed_circle_y - b_y) * shrinking,
                                 c_x + (inscribed_circle_x - c_x) * shrinking,
                                 c_y + (inscribed_circle_y - c_y) * shrinking, rounding, thickness);
    }

    void fullRoundedRectangleBorder(float x, float y, float width, float height, float rounding,
                                    float thickness) {
      RoundedRectangle border(state_.clamp, state_.brush, state_.x + x, state_.y + y, width, height,
                              rounding);
      border.thickness = thickness;
      addShape(border);
    }

    void outerRoundedTriangleBorder(float a_x, float a_y, float b_x, float b_y, float c_x,
                                    float c_y, float rounding, float thickness = -1.0f) {
      if (thickness < 0.0f)
        thickness = std::abs(a_x - b_x) + std::abs(a_y - b_y) + std::abs(a_x - c_x) + std::abs(a_y - c_y);
      float pad = rounding;
      float x = std::min(std::min(a_x, b_x), c_x) - pad;
      float width = std::max(std::max(a_x, b_x), c_x) - x + 2.0f * pad;
      float y = std::min(std::min(a_y, b_y), c_y) - pad;
      float height = std::max(std::max(a_y, b_y), c_y) - y + 2.0f * pad;

      float x1 = 2.0f * (a_x - x) / width - 1.0f;
      float y1 = 2.0f * (a_y - y) / height - 1.0f;
      float x2 = 2.0f * (b_x - x) / width - 1.0f;
      float y2 = 2.0f * (b_y - y) / height - 1.0f;
      float x3 = 2.0f * (c_x - x) / width - 1.0f;
      float y3 = 2.0f * (c_y - y) / height - 1.0f;

      addShape(Triangle(state_.clamp, state_.brush, state_.x + x, state_.y + y, width, height, x1,
                        y1, x2, y2, x3, y3, rounding, thickness + 1.0f));
    }

    bool tryDrawCollinearQuadratic(float a_x, float a_y, float b_x, float b_y, float c_x, float c_y,
                                   float thickness, float pixel_width = 1.0f) {
      static constexpr float kLinearThreshold = 0.01f;

      float collinear_distance_x = a_x - 2.0f * b_x + c_x;
      float collinear_distance_y = a_y - 2.0f * b_y + c_y;
      if (collinear_distance_x > kLinearThreshold || collinear_distance_x < -kLinearThreshold ||
          collinear_distance_y > kLinearThreshold || collinear_distance_y < -kLinearThreshold) {
        return false;
      }

      addSegment(a_x, a_y, c_x, c_y, thickness, true, pixel_width);
      return true;
    }

    void addSvg(const Svg& svg, float x, float y) {
      addShape(ImageWrapper(state_.clamp, state_.brush, state_.x + x, state_.y + y, svg.width,
                            svg.height, svg, imageAtlas()));
    }

    void addImage(const Image& image, float x, float y) {
      addShape(ImageWrapper(state_.clamp, state_.brush, state_.x + x, state_.y + y, image.width,
                            image.height, image, imageAtlas()));
    }

    Palette* palette_ = nullptr;
    float dpi_scale_ = 1.0f;
    double render_time_ = 0.0;
    double delta_time_ = 0.0;
    int render_frame_ = 0;
    int last_skipped_frame_ = 0;

    std::vector<State> state_memory_;
    State state_;

    GradientAtlas gradient_atlas_;
    ImageAtlas image_atlas_;

    Region window_region_;
    Region default_region_;
    Layer composite_layer_;
    std::vector<std::unique_ptr<Layer>> intermediate_layers_;
    std::vector<Layer*> layers_;

    float refresh_rate_ = 0.0f;

    VISAGE_LEAK_CHECKER(Canvas)
  };
}
