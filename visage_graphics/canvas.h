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
#include "path.h"
#include "region.h"
#include "screenshot.h"
#include "shape_batcher.h"
#include "svg.h"
#include "text.h"
#include "theme.h"
#include "visage_utils/dimension.h"
#include "visage_utils/space.h"
#include "visage_utils/time_utils.h"

#include <limits>

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
      Brush set_brush;
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

    const Screenshot& takeScreenshot();
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

    void setWindowless(int width, int height);
    void removeFromWindow() { composite_layer_.removeFromWindow(); }

    int width() const { return composite_layer_.width(); }
    int height() const { return composite_layer_.height(); }
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

    const Brush& brush() { return state_.set_brush; }
    void setBrush(const Brush& brush) {
      state_.set_brush = brush;
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

    void fill() {
      addShape(Fill(state_.clamp, state_.brush, state_.x, state_.y,
                    state_.clamp.right - state_.clamp.left, state_.clamp.bottom - state_.clamp.top));
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

    template<typename T1, typename T2, typename T3, typename T4>
    void squircleBorder(const T1& x, const T2& y, const T3& width, float power, const T4& thickness) {
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
      float shadow = std::max(1.0f, pixels(shadow_width));
      float full_width = pixels(width) + shadow;
      RoundedArc arc(state_.clamp, state_.brush, state_.x + pixels(x) - 0.5f * shadow,
                     state_.y + pixels(y) - 0.5f * shadow, full_width, full_width,
                     pixels(thickness) + shadow, center_radians, radians);
      arc.pixel_width = shadow;
      addShape(arc);
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void flatArcShadow(const T1& x, const T2& y, const T3& width, const T4& thickness,
                       float center_radians, float radians, const T5& shadow_width) {
      float shadow = std::max(1.0f, pixels(shadow_width));
      float raw_width = pixels(width);
      float full_width = raw_width + shadow;
      FlatArc arc(state_.clamp, state_.brush, state_.x + pixels(x) - 0.5f * shadow,
                  state_.y + pixels(y) - 0.5f * shadow, full_width, full_width, pixels(thickness) + shadow,
                  center_radians, radians + 0.25f * kPi * shadow / raw_width);
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
    void rectangleShadow(const T1& x, const T2& y, const T3& width, const T4& height, const T5& shadow_width) {
      roundedRectangleShadow(x, y, width, height, 0.0f, shadow_width);
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6>
    void roundedRectangleShadow(const T1& x, const T2& y, const T3& width, const T4& height,
                                const T5& rounding, const T6& shadow_width) {
      float pixel_width = std::max(1.0f, pixels(shadow_width));
      addShape(RoundedRectangle(state_.clamp, state_.brush, state_.x + pixels(x) - 0.5f * pixel_width,
                                state_.y + pixels(y) - 0.5f * pixel_width,
                                pixels(width) + pixel_width, pixels(height) + pixel_width,
                                std::max(1.0f, pixels(rounding) + 0.5f * pixel_width), pixel_width));
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
      addSvg(svg, pixels(x), pixels(y), state_.scale * svg.width(), state_.scale * svg.height());
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void svg(const Svg& svg, const T1& x, const T2& y, const T3& width, const T4& height) {
      addSvg(svg, pixels(x), pixels(y), pixels(width), pixels(height));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void svg(const unsigned char* svg_data, int svg_size, const T1& x, const T2& y, const T3& width,
             const T4& height) {
      Svg new_svg(svg_data, svg_size);
      new_svg.setDimensions(pixels(width) / state_.scale, pixels(height) / state_.scale, state_.scale);
      new_svg.setFillBrush(state_.set_brush);
      new_svg.setStrokeBrush(state_.set_brush);
      svg(new_svg, x, y, width, height);
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void svg(const EmbeddedFile& file, const T1& x, const T2& y, const T3& width, const T4& height) {
      svg(file.data, file.size, x, y, width, height);
    }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void graphLine(const GraphData& data, const T1& x, const T2& y, const T3& width,
                   const T4& height, const T5& thickness) {
      addGraphLine(data, pixels(x), pixels(y), pixels(width), pixels(height), pixels(thickness));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void graphFill(const GraphData& data, const T1& x, const T2& y, const T3& width,
                   const T4& height, float fill_center) {
      addGraphFill(data, pixels(x), pixels(y), pixels(width), pixels(height), fill_center);
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void heatMap(const HeatMapData& data, const T1& x, const T2& y, const T3& width, const T4& height) {
      if (data.width() == 0 || data.height() == 0)
        return;

      addHeatMap(data, pixels(x), pixels(y), pixels(width), pixels(height));
    }

    template<typename T1, typename T2>
    void image(const Image& image, const T1& x, const T2& y) {
      int w = std::round(pixels(image.width));
      int h = std::round(pixels(image.height));
      addImage({ image.data, image.data_size, w, h }, pixels(x), pixels(y));
    }

    template<typename T1, typename T2, typename T3, typename T4>
    void image(const unsigned char* image_data, int image_size, const T1& x, const T2& y,
               const T3& width, const T4& height) {
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

    template<typename T1, typename T2, typename T3, typename T4>
    void fill(const Path& path, const T1& x, const T2& y, const T3& width, const T4& height) {
      if (path.numPoints() == 0)
        return;

      addShape(PathFillWrapper(state_.clamp, state_.brush, state_.x + pixels(x), state_.y + pixels(y),
                               pixels(width), pixels(height), path, pathAtlas(), state_.scale));
    }

    template<typename T1, typename T2>
    void fill(const Path& path, const T1& x, const T2& y) {
      if (path.numPoints() == 0)
        return;

      auto bounding_box = path.boundingBox();
      addShape(PathFillWrapper(state_.clamp, state_.brush, state_.x + pixels(x),
                               state_.y + pixels(y), bounding_box.right() * state_.scale + 1.0f,
                               bounding_box.bottom() * state_.scale + 1.0f, path, pathAtlas(),
                               state_.scale));
    }

    void fill(const Path& path) { fill(path, 0, 0); }

    template<typename T1, typename T2, typename T3, typename T4, typename T5>
    void stroke(const Path& path, const T1& x, const T2& y, const T3& width, const T4& height,
                const T5& stroke_width, Path::Join join = Path::Join::Round,
                Path::EndCap end_cap = Path::EndCap::Round, std::vector<float> dash_array = {},
                float dash_offset = 0.0f, float miter_limit = Path::kDefaultMiterLimit) {
      if (path.numPoints() == 0)
        return;

      addShape(PathFillWrapper(state_.clamp, state_.brush, state_.x + pixels(x),
                               state_.y + pixels(y), pixels(width), pixels(height),
                               path.stroke(pixels(stroke_width), join, end_cap, dash_array,
                                           dash_offset, miter_limit),
                               state_.scale));
    }

    void saveState() { state_memory_.push_back(state_); }

    void restoreState() {
      VISAGE_ASSERT(!state_memory_.empty());
      if (!state_memory_.empty()) {
        state_ = state_memory_.back();
        state_memory_.pop_back();
      }
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

    void trimClampBounds(float x, float y, float width, float height) {
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

    PathAtlas* pathAtlas() { return &path_atlas_; }
    ImageAtlas* imageAtlas() { return &image_atlas_; }
    ImageAtlas* dataAtlas() { return &data_atlas_; }
    GradientAtlas* gradientAtlas() { return &gradient_atlas_; }

    State* state() { return &state_; }

  private:
    void setClampBounds(const ClampBounds& bounds) { state_.clamp = bounds; }

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
      if (thickness <= 0.0f)
        return;

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
      if (thickness <= 0.0f)
        return;

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

    void addRoundedRectangleBorder(float x, float y, float width, float height, float rounding,
                                   float thickness) {
      if (rounding > width * 0.5f - 1.0f || rounding > height * 0.5f - 1.0f) {
        fullRoundedRectangleBorder(x, y, width, height, rounding, thickness);
        return;
      }

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
      if (perimeter < std::numeric_limits<float>::epsilon())
        return;

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

      if (width < std::numeric_limits<float>::epsilon() || height < std::numeric_limits<float>::epsilon())
        return;

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

    void addSvg(const Svg& svg, float x, float y, float width, float height) {
      SvgDrawable::ColorContext context;
      if (state_.brush) {
        Brush current = state_.set_brush;
        context.current_color = &current;
      }
      svg.drawable()->drawAll(*this, &context, x, y, width, height);
    }

    void addImage(const Image& image, float x, float y) {
      addShape(ImageWrapper(state_.clamp, state_.brush, state_.x + x, state_.y + y, image.width,
                            image.height, image, imageAtlas()));
    }

    void addGraphLine(const GraphData& data, float x, float y, float width, float height, float thickness) {
      addShape(GraphLineWrapper(state_.clamp, state_.brush, state_.x + x, state_.y + y, width,
                                height, thickness, data, dataAtlas()));
    }

    void addGraphFill(const GraphData& data, float x, float y, float width, float height, float center) {
      addShape(GraphFillWrapper(state_.clamp, state_.brush, state_.x + x, state_.y + y, width,
                                height, center, data, dataAtlas()));
    }

    void addHeatMap(const HeatMapData& data, float x, float y, float width, float height) {
      addShape(HeatMapWrapper(state_.clamp, state_.brush, state_.x + x, state_.y + y, width, height,
                              data, dataAtlas()));
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
    PathAtlas path_atlas_;
    ImageAtlas image_atlas_;
    ImageAtlas data_atlas_;

    Region window_region_;
    Region default_region_;
    Layer composite_layer_;
    std::vector<std::unique_ptr<Layer>> intermediate_layers_;
    std::vector<Layer*> layers_;

    float refresh_time_ = 0.0f;

    VISAGE_LEAK_CHECKER(Canvas)
  };
}
