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

#pragma once

#include "font.h"
#include "graphics_utils.h"
#include "layer.h"
#include "region.h"
#include "shape_batcher.h"
#include "text.h"
#include "visage_utils/space.h"
#include "visage_utils/time_utils.h"

namespace visage {
  class Palette;
  class Shader;

  class Canvas {
  public:
    struct State {
      int x = 0;
      int y = 0;
      int palette_override = 0;
      QuadColor color;
      ClampBounds clamp;
      BlendMode blend_mode = BlendMode::Alpha;
      Region* current_region = nullptr;
    };

    Canvas();
    Canvas(const Canvas& other) = delete;
    Canvas& operator=(const Canvas&) = delete;

    void clearDrawnShapes();
    int submit(int submit_pass = 0);

    void ensureLayerExists(int layer);
    Layer* layer(int index) {
      ensureLayerExists(index);
      return layers_[index];
    }

    void invalidateRectInRegion(Bounds rect, Region* region, int layer);
    void addToPackedLayer(Region* region, int layer_index);
    void removeFromPackedLayer(Region* region, int layer_index);
    void changePackedLayer(Region* region, int from, int to);

    void pairToWindow(void* window_handle, int width, int height) {
      composite_layer_.pairToWindow(window_handle, width, height);
      setDimensions(width, height);
    }

    void removeFromWindow() { composite_layer_.removeFromWindow(); }

    void setDimensions(int width, int height);
    void setWidthScale(float width_scale) { width_scale_ = width_scale; }
    void setHeightScale(float height_scale) { height_scale_ = height_scale; }
    void setDpiScale(float scale) { dpi_scale_ = scale; }

    float widthScale() const { return width_scale_; }
    float heightScale() const { return height_scale_; }
    float dpiScale() const { return dpi_scale_; }
    void updateTime(double time);
    double time() const { return render_time_; }
    double deltaTime() const { return delta_time_; }
    int frameCount() const { return render_frame_; }

    void setBlendMode(BlendMode blend_mode) { state_.blend_mode = blend_mode; }
    void setColor(unsigned int color) { state_.color = color; }
    void setColor(const QuadColor& color) { state_.color = color; }
    void setPaletteColor(int color_id) { state_.color = color(color_id); }

    void setBlendedPaletteColor(int color_from, int color_to, float t) {
      state_.color = blendedColor(color_from, color_to, t);
    }

    void fill(float x, float y, float width, float height) {
      addShape(Fill(state_.clamp.clamp(x, y, width, height), state_.color, state_.x + x,
                    state_.y + y, width, height));
    }

    void circle(float x, float y, float width) {
      addShape(Circle(state_.clamp, state_.color, state_.x + x, state_.y + y, width));
    }

    void fadeCircle(float x, float y, float width, float fade) {
      Circle circle(state_.clamp, state_.color, state_.x + x, state_.y + y, width);
      circle.pixel_width = fade;
      addShape(circle);
    }

    void ring(float x, float y, float width, float thickness) {
      Circle circle(state_.clamp, state_.color, state_.x + x, state_.y + y, width);
      circle.thickness = thickness;
      addShape(circle);
    }

    void squircle(float x, float y, float width, float power) {
      addShape(Squircle(state_.clamp, state_.color, state_.x + x, state_.y + y, width, width, power));
    }

    void squircleBorder(float x, float y, float width, float power, float thickness) {
      Squircle squircle(state_.clamp, state_.color, state_.x + x, state_.y + y, width, width, power);
      squircle.thickness = thickness;
      addShape(squircle);
    }

    void superEllipse(float x, float y, float width, float height, float power) {
      addShape(Squircle(state_.clamp, state_.color, state_.x + x, state_.y + y, width, height, power));
    }

    void roundedArc(float x, float y, float width, float thickness, float center_radians,
                    float radians, float pixel_width = 1.0f) {
      addShape(RoundedArc(state_.clamp, state_.color, state_.x + x, state_.y + y, width, width,
                          thickness + 1.0f, center_radians, radians));
    }

    void flatArc(float x, float y, float width, float thickness, float center_radians,
                 float radians, float pixel_width = 1.0f) {
      addShape(FlatArc(state_.clamp, state_.color, state_.x + x, state_.y + y, width, width,
                       thickness + 1.0f, center_radians, radians));
    }

    void arc(float x, float y, float width, float thickness, float center_radians, float radians,
             bool rounded = false, float pixel_width = 1.0f) {
      if (rounded)
        roundedArc(x, y, width, thickness, center_radians, radians, pixel_width);
      else
        flatArc(x, y, width, thickness, center_radians, radians, pixel_width);
    }

    void roundedArcShadow(float x, float y, float width, float thickness, float center_radians,
                          float radians, float shadow_width, bool rounded = false) {
      float full_width = width + 2.0f * shadow_width;
      RoundedArc arc(state_.clamp, state_.color, state_.x + x - shadow_width,
                     state_.y + y - shadow_width, full_width, full_width,
                     thickness + 1.0f + 2.0f * shadow_width, center_radians, radians);
      arc.pixel_width = shadow_width;
      addShape(arc);
    }

    void flatArcShadow(float x, float y, float width, float thickness, float center_radians,
                       float radians, float shadow_width, bool rounded = false) {
      float full_width = width + 2.0f * shadow_width;
      FlatArc arc(state_.clamp, state_.color, state_.x + x - shadow_width,
                  state_.y + y - shadow_width, full_width, full_width,
                  thickness + 1.0f + 2.0f * shadow_width, center_radians, radians);
      arc.pixel_width = shadow_width;
      addShape(arc);
    }

    void segment(float a_x, float a_y, float b_x, float b_y, float thickness, bool rounded = false,
                 float pixel_width = 1.0f) {
      float x = std::min(a_x, b_x) - thickness;
      float width = std::max(a_x, b_x) + thickness - x;
      float y = std::min(a_y, b_y) - thickness;
      float height = std::max(a_y, b_y) + thickness - y;

      float x1 = 2.0f * (a_x - x) / width - 1.0f;
      float y1 = 2.0f * (a_y - y) / height - 1.0f;
      float x2 = 2.0f * (b_x - x) / width - 1.0f;
      float y2 = 2.0f * (b_y - y) / height - 1.0f;

      if (rounded) {
        addShape(RoundedSegment(state_.clamp, state_.color, state_.x + x, state_.y + y, width,
                                height, x1, y1, x2, y2, thickness + 1.0f, pixel_width));
      }
      else {
        addShape(FlatSegment(state_.clamp, state_.color, state_.x + x, state_.y + y, width, height,
                             x1, y1, x2, y2, thickness + 1.0f, pixel_width));
      }
    }

    void rotary(float x, float y, float width, float value, float hover_amount, float arc_thickness,
                const QuadColor& back_color, const QuadColor& thumb_color, bool bipolar = false) {
      addShape(Rotary(state_.clamp, state_.color, back_color, thumb_color, state_.x + x,
                      state_.y + y, width, value, bipolar, hover_amount, arc_thickness));
    }

    void rectangle(float x, float y, float width, float height) {
      addShape(Rectangle(state_.clamp, state_.color, state_.x + x, state_.y + y, width, height));
    }

    void rectangleBorder(float x, float y, float width, float height, float thickness) {
      Rectangle border(state_.clamp, state_.color, state_.x + x, state_.y + y, width, height);
      border.thickness = thickness + 1.0f;
      addShape(border);
    }

    void roundedRectangle(float x, float y, float width, float height, float rounding) {
      addShape(RoundedRectangle(state_.clamp, state_.color, state_.x + x, state_.y + y, width,
                                height, std::max(1.0f, rounding)));
    }

    void diamond(float x, float y, float width, float rounding) {
      addShape(Diamond(state_.clamp, state_.color, state_.x + x, state_.y + y, width, width,
                       std::max(1.0f, rounding)));
    }

    void leftRoundedRectangle(float x, float y, float width, float height, float rounding) {
      ClampBounds clamp = state_.clamp;
      clamp.right = std::min(clamp.right, state_.x + x + width);
      addShape(RoundedRectangle(clamp, state_.color, state_.x + x, state_.y + y,
                                width + rounding + 1.0f, height, std::max(1.0f, rounding)));
    }

    void rightRoundedRectangle(float x, float y, float width, float height, float rounding) {
      ClampBounds clamp = state_.clamp;
      clamp.left = std::max(clamp.left, state_.x + x);
      float growth = rounding + 1.0f;
      addShape(RoundedRectangle(clamp, state_.color, state_.x + x - growth, state_.y + y,
                                width + growth, height, std::max(1.0f, rounding)));
    }

    void topRoundedRectangle(float x, float y, float width, float height, float rounding) {
      ClampBounds clamp = state_.clamp;
      clamp.bottom = std::min(clamp.bottom, state_.y + y + height);
      addShape(RoundedRectangle(clamp, state_.color, state_.x + x, state_.y + y, width,
                                height + rounding + 1.0f, std::max(1.0f, rounding)));
    }

    void bottomRoundedRectangle(float x, float y, float width, float height, float rounding) {
      ClampBounds clamp = state_.clamp;
      clamp.top = std::max(clamp.top, state_.y + y);
      float growth = rounding + 1.0f;
      addShape(RoundedRectangle(clamp, state_.color, state_.x + x, state_.y + y - growth, width,
                                height + growth, std::max(1.0f, rounding)));
    }

    void rectangleShadow(float x, float y, float width, float height, float blur_radius) {
      if (blur_radius > 0.0f) {
        Rectangle rectangle(state_.clamp, state_.color, state_.x + x, state_.y + y, width, height);
        rectangle.pixel_width = blur_radius;
        addShape(rectangle);
      }
    }

    void roundedRectangleShadow(float x, float y, float width, float height, float rounding,
                                float blur_radius) {
      if (blur_radius <= 0.0f)
        return;

      float offset = -blur_radius * 0.5f;
      if (rounding <= 1.0f)
        rectangleShadow(state_.x + x + offset, state_.y + y + offset, width + blur_radius,
                        height + blur_radius, blur_radius);
      else {
        RoundedRectangle shadow(state_.clamp, state_.color, state_.x + x + offset, state_.y + y + offset,
                                width + blur_radius, height + blur_radius, rounding);
        shadow.pixel_width = blur_radius;
        addShape(shadow);
      }
    }

    void fullRoundedRectangleBorder(float x, float y, float width, float height, float rounding,
                                    float thickness) {
      RoundedRectangle border(state_.clamp, state_.color, state_.x + x, state_.y + y, width, height,
                              rounding);
      border.thickness = thickness;
      addShape(border);
    }

    void roundedRectangleBorder(float x, float y, float width, float height, float rounding, float thickness) {
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

    void triangleLeft(float x, float y, float width) {
      addShape(Triangle(state_.clamp, state_.color, state_.x + x, state_.y + y, width, width * 2.0f,
                        Direction::Left));
    }

    void triangleRight(float x, float y, float width) {
      addShape(Triangle(state_.clamp, state_.color, state_.x + x, state_.y + y, width, width * 2.0f,
                        Direction::Right));
    }

    void triangleUp(float x, float y, float width) {
      addShape(Triangle(state_.clamp, state_.color, state_.x + x, state_.y + y, width * 2.0f, width,
                        Direction::Up));
    }

    void triangleDown(float x, float y, float width) {
      addShape(Triangle(state_.clamp, state_.color, state_.x + x, state_.y + y, width * 2.0f, width,
                        Direction::Down));
    }

    void text(Text* text, float x, float y, float width, float height, Direction dir = Direction::Up) {
      VISAGE_ASSERT(text->font().packedFont());
      TextBlock text_block(state_.clamp, state_.color, state_.x + x, state_.y + y, width, height,
                           text, dir);
      addShape(std::move(text_block));
    }

    void text(const String& string, const Font& font, Font::Justification justification, float x,
              float y, float width, float height, Direction dir = Direction::Up) {
      if (!string.isEmpty()) {
        Text* stored_text = state_.current_region->addText(string, font, justification);
        text(stored_text, x, y, width, height, dir);
      }
    }

    void svg(const Svg& svg, float x, float y) {
      addShape(ImageWrapper(state_.clamp, state_.color, state_.x + x, state_.y + y, svg.width,
                            svg.height, svg, imageGroup()));
    }

    void svg(const char* svg_data, int svg_size, float x, float y, int width, int height,
             int blur_radius = 0) {
      svg({ svg_data, svg_size, width, height, blur_radius }, x, y);
    }

    void svg(const EmbeddedFile& file, float x, float y, int width, int height, int blur_radius = 0) {
      svg(file.data, file.size, x, y, width, height, blur_radius);
    }

    void image(const Image& image, float x, float y) {
      addShape(ImageWrapper(state_.clamp, state_.color, state_.x + x, state_.y + y, image.width,
                            image.height, image, imageGroup()));
    }

    void image(const char* image_data, int image_size, float x, float y, int width, int height) {
      image({ image_data, image_size, width, height }, x, y);
    }

    void image(const EmbeddedFile& image_file, float x, float y, int width, int height) {
      image(image_file.data, image_file.size, x, y, width, height);
    }

    void shader(Shader* shader, float x, float y, float width, float height) {
      addShape(ShaderWrapper(state_.clamp, state_.color, state_.x + x, state_.y + y, width, height, shader));
    }

    void line(Line* line, float x, float y, float width, float height, float line_width) {
      addShape(LineWrapper(state_.clamp, state_.color, state_.x + x, state_.y + y, width, height,
                           line, line_width));
    }

    void lineFill(Line* line, float x, float y, float width, float height, float fill_position) {
      addShape(LineFillWrapper(state_.clamp, state_.color, state_.x + x, state_.y + y, width,
                               height, line, fill_position));
    }

    void saveState() { state_memory_.push_back(state_); }

    void restoreState() {
      state_ = state_memory_.back();
      state_memory_.pop_back();
    }

    void setPosition(int x, int y) {
      state_.x += x;
      state_.y += y;
    }

    void addRegion(Region* region) {
      default_region_.addRegion(region);
      region->setCanvas(this);
    }

    void clearRegion(Region* region) { region->clear(); }

    void beginRegion(Region* region) {
      region->clear();
      saveState();
      state_.x = 0;
      state_.y = 0;
      state_.blend_mode = BlendMode::Alpha;
      setClampBounds(0, 0, region->width(), region->height());
      state_.color = {};
      region->palette_override_ = state_.palette_override;
      state_.current_region = region;
    }

    void endRegion() { restoreState(); }

    void setPalette(Palette* palette) { palette_ = palette; }
    void setPaletteOverride(int override_id) { state_.palette_override = override_id; }

    void setClampBounds(int x, int y, int width, int height) {
      VISAGE_ASSERT(width >= 0);
      VISAGE_ASSERT(height >= 0);
      state_.clamp.left = state_.x + x;
      state_.clamp.top = state_.y + y;
      state_.clamp.right = state_.clamp.left + width;
      state_.clamp.bottom = state_.clamp.top + height;
    }

    void setClampBounds(const ClampBounds& bounds) { state_.clamp = bounds; }

    void trimClampBounds(int x, int y, int width, int height) {
      state_.clamp = state_.clamp.clamp(state_.x + x, state_.y + y, width, height);
    }

    void moveClampBounds(int x_offset, int y_offset) {
      state_.clamp.left += x_offset;
      state_.clamp.top += y_offset;
      state_.clamp.right += x_offset;
      state_.clamp.bottom += y_offset;
    }

    const ClampBounds& currentClampBounds() const { return state_.clamp; }
    bool totallyClamped() const { return state_.clamp.totallyClamped(); }

    int x() const { return state_.x; }
    int y() const { return state_.y; }

    QuadColor color(unsigned int color_id);
    QuadColor blendedColor(int color_from, int color_to, float t) {
      return color(color_from).interpolate(color(color_to), t);
    }

    float value(unsigned int value_id);
    std::vector<std::string> debugInfo() const;

    ImageGroup* imageGroup() {
      if (image_group_ == nullptr)
        image_group_ = std::make_unique<ImageGroup>();
      return image_group_.get();
    }

    State* state() { return &state_; }

  private:
    template<typename T>
    void addShape(T shape) {
      state_.current_region->shape_batcher_.addShape(std::move(shape), state_.blend_mode);
    }

    Palette* palette_ = nullptr;
    float width_scale_ = 1.0f;
    float height_scale_ = 1.0f;
    float dpi_scale_ = 1.0f;
    double render_time_ = 0.0;
    double delta_time_ = 0.0;
    int render_frame_ = 0;

    std::vector<State> state_memory_;
    State state_;

    Region window_region_;
    Region default_region_;
    Layer composite_layer_;
    std::vector<std::unique_ptr<Layer>> intermediate_layers_;
    std::vector<Layer*> layers_;

    std::unique_ptr<ImageGroup> image_group_;

    float refresh_rate_ = 0.0f;

    VISAGE_LEAK_CHECKER(Canvas)
  };
}