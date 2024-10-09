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
#include "image.h"
#include "post_effects.h"
#include "shape_batcher.h"
#include "text.h"
#include "visage_utils/space.h"
#include "visage_utils/time_utils.h"

namespace visage {
  class Palette;
  class Shader;

  class Canvas {
  public:
    class Region {
    public:
      friend class Canvas;

      Region() = default;

      SubmitBatch* submitBatchAtPosition(int position) const { return shape_batcher_.batchAtIndex(position); }
      int numSubmitBatches() const { return shape_batcher_.numBatches(); }
      bool isEmpty() const { return shape_batcher_.isEmpty(); }
      const std::vector<Region*>& subRegions() const { return sub_regions_; }
      int numRegions() const { return sub_regions_.size(); }

      void addRegion(Region* region) { sub_regions_.push_back(region); }
      void removeRegion(Region* region) {
        sub_regions_.erase(std::find(sub_regions_.begin(), sub_regions_.end(), region));
      }

      void setBounds(int x, int y, int width, int height) {
        invalidate();
        x_ = x;
        y_ = y;
        width_ = width;
        height_ = height;
        invalidate();
      }

      void setVisible(bool visible) { visible_ = visible; }
      bool isVisible() const { return visible_; }
      bool overlaps(const Region* other) const {
        return x_ < other->x_ + other->width_ && x_ + width_ > other->x_ &&
               y_ < other->y_ + other->height_ && y_ + height_ > other->y_;
      }

      int x() const { return x_; }
      int y() const { return y_; }
      int width() const { return width_; }
      int height() const { return height_; }

      void invalidateRect(Bounds rect) {
        return;
        Region* region = this;
        while (region->parent_) {
          rect = rect + Point(region->x_, region->y_);
          region = region->parent_;
        }

        VISAGE_ASSERT(region->canvas_);
        if (region->canvas_)
          region->canvas_->invalidateRect(rect);
      }

      void invalidate() {
        if (width_ > 0 && height_ > 0)
          invalidateRect({ 0, 0, width_, height_ });
      }

    private:
      Text* addText(const String& string, const Font& font, Font::Justification justification) {
        text_store_.push_back(std::make_unique<Text>(string, font, justification));
        return text_store_.back().get();
      }

      void clear() {
        shape_batcher_.clear();
        text_store_.clear();
        external_canvases_.clear();
      }

      void clearSubRegions() { sub_regions_.clear(); }

      void clearAll() {
        clear();
        clearSubRegions();
      }

      int x_ = 0;
      int y_ = 0;
      int width_ = 0;
      int height_ = 0;
      int palette_override_ = 0;
      bool visible_ = true;
      Canvas* canvas_ = nullptr;
      Region* parent_ = nullptr;
      ShapeBatcher shape_batcher_;
      std::vector<std::unique_ptr<Text>> text_store_;
      std::vector<std::pair<Canvas*, PostEffect*>> external_canvases_;
      std::vector<Region*> sub_regions_;
    };

    struct State {
      int x = 0;
      int y = 0;
      int palette_override = 0;
      QuadColor color;
      ClampBounds clamp;
      Region* current_region = nullptr;
    };

    Canvas();
    Canvas(const Canvas& other) = delete;
    Canvas& operator=(const Canvas&) = delete;

    ~Canvas();

    void clearDrawnShapes();
    int submit(int submit_pass = 0);
    void render();

    void pairToWindow(void* window_handle, int width, int height);
    void clearWindowHandle();

    void setHdr(bool hdr);
    void setDimensions(int width, int height);
    void setWidthScale(float width_scale) { width_scale_ = width_scale; }
    void setHeightScale(float height_scale) { height_scale_ = height_scale; }
    void setDpiScale(float scale) { dpi_scale_ = scale; }

    void invalidateRect(Bounds rect);
    void invalidate() {
      invalid_rects_.clear();
      invalid_rects_.emplace_back(0, 0, width_, height_);
    }

    bool hdr() const { return hdr_; }
    int width() const { return width_; }
    int height() const { return height_; }
    float widthScale() const { return width_scale_; }
    float heightScale() const { return height_scale_; }
    float dpiScale() const { return dpi_scale_; }
    bool bottomLeftOrigin() const { return bottom_left_origin_; }
    void updateTime(double time);
    double time() const { return render_time_; }
    int frameCount() const { return render_frame_; }

    bgfx::FrameBufferHandle& frameBuffer() const;
    int frameBufferFormat() const;

    void setColor(unsigned int color) { state_.color = color; }
    void setColor(const QuadColor& color) { state_.color = color; }
    void setPaletteColor(int color_id) { state_.color = color(color_id); }

    void setBlendedPaletteColor(int color_from, int color_to, float t) {
      state_.color = blendedColor(color_from, color_to, t);
    }

    void fill(float x, float y, float width, float height) {
      addShape(Fill(state_.clamp, state_.color, state_.x + x, state_.y + y, width, height));
    }

    void clearArea(float x, float y, float width, float height) {
      addShape(Clear(state_.clamp, state_.color, state_.x + x, state_.y + y, width, height));
    }

    void circle(float x, float y, float width) {
      addShape(Circle(state_.clamp, state_.color, state_.x + x, state_.y + y, width, 1.0f));
    }

    void fadeCircle(float x, float y, float width, float fade) {
      addShape(Circle(state_.clamp, state_.color, state_.x + x, state_.y + y, width, fade));
    }

    void ring(float x, float y, float width, float thickness) {
      addShape(Ring(state_.clamp, state_.color, state_.x + x, state_.y + y, width, thickness + 1.0f));
    }

    void roundedArc(float x, float y, float width, float thickness, float center_radians,
                    float radians, float pixel_width = 1.0f) {
      addShape(RoundedArc(state_.clamp, state_.color, state_.x + x, state_.y + y, width, width,
                          thickness + 1.0f, center_radians, radians, 1.0f));
    }

    void flatArc(float x, float y, float width, float thickness, float center_radians,
                 float radians, float pixel_width = 1.0f) {
      addShape(FlatArc(state_.clamp, state_.color, state_.x + x, state_.y + y, width, width,
                       thickness + 1.0f, center_radians, radians, 1.0f));
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
      addShape(RoundedArc(state_.clamp, state_.color, state_.x + x - shadow_width, state_.y + y - shadow_width,
                          full_width, full_width, thickness + 1.0f + 2.0f * shadow_width,
                          center_radians, radians, 1.0f / shadow_width));
    }

    void flatArcShadow(float x, float y, float width, float thickness, float center_radians,
                       float radians, float shadow_width, bool rounded = false) {
      float full_width = width + 2.0f * shadow_width;
      addShape(FlatArc(state_.clamp, state_.color, state_.x + x - shadow_width, state_.y + y - shadow_width,
                       full_width, full_width, thickness + 1.0f + 2.0f * shadow_width,
                       center_radians, radians, 1.0f / shadow_width));
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
      addShape(RectangleBorder(state_.clamp, state_.color, state_.x + x, state_.y + y, width,
                               height, thickness + 1.0f));
    }

    void roundedRectangle(float x, float y, float width, float height, float rounding) {
      addShape(RoundedRectangle(state_.clamp, state_.color, state_.x + x, state_.y + y, width,
                                height, std::max(1.0f, rounding), 1.0f));
    }

    void diamond(float x, float y, float width, float rounding) {
      addShape(Diamond(state_.clamp, state_.color, state_.x + x, state_.y + y, width, width,
                       std::max(1.0f, rounding), 1.0f));
    }

    void leftRoundedRectangle(float x, float y, float width, float height, float rounding) {
      ClampBounds clamp = state_.clamp;
      clamp.right = std::min(clamp.right, state_.x + x + width);
      addShape(RoundedRectangle(clamp, state_.color, state_.x + x, state_.y + y,
                                width + rounding + 1.0f, height, std::max(1.0f, rounding), 1.0f));
    }

    void rightRoundedRectangle(float x, float y, float width, float height, float rounding) {
      ClampBounds clamp = state_.clamp;
      clamp.left = std::max(clamp.left, state_.x + x);
      float growth = rounding + 1.0f;
      addShape(RoundedRectangle(clamp, state_.color, state_.x + x - growth, state_.y + y,
                                width + growth, height, std::max(1.0f, rounding), 1.0f));
    }

    void topRoundedRectangle(float x, float y, float width, float height, float rounding) {
      ClampBounds clamp = state_.clamp;
      clamp.bottom = std::min(clamp.bottom, state_.y + y + height);
      addShape(RoundedRectangle(clamp, state_.color, state_.x + x, state_.y + y, width,
                                height + rounding + 1.0f, std::max(1.0f, rounding), 1.0f));
    }

    void bottomRoundedRectangle(float x, float y, float width, float height, float rounding) {
      ClampBounds clamp = state_.clamp;
      clamp.top = std::max(clamp.top, state_.y + y);
      float growth = rounding + 1.0f;
      addShape(RoundedRectangle(clamp, state_.color, state_.x + x, state_.y + y - growth, width,
                                height + growth, std::max(1.0f, rounding), 1.0f));
    }

    void rectangleShadow(float x, float y, float width, float height, float blur_radius) {
      if (blur_radius > 0.0f) {
        addShape(RectangleShadow(state_.clamp, state_.color, state_.x + x, state_.y + y, width,
                                 height, blur_radius));
      }
    }

    void roundedRectangleShadow(float x, float y, float width, float height, float rounding,
                                float blur_radius) {
      if (blur_radius <= 0.0f)
        return;

      if (rounding <= 1.0f)
        rectangleShadow(x, y, width, height, blur_radius);
      else {
        addShape(RoundedRectangleShadow(state_.clamp, state_.color, state_.x + x, state_.y + y,
                                        width, height, rounding, blur_radius));
      }
    }

    void fullRoundedRectangleBorder(float x, float y, float width, float height, float rounding,
                                    float thickness) {
      addShape(RoundedRectangleBorder(state_.clamp, state_.color, state_.x + x, state_.y + y, width,
                                      height, rounding, thickness + 1.0f));
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

    void topLeftCorner(float x, float y, float width) {
      roundedCorner(x, y, width, Corner::TopLeft);
    }

    void topRightCorner(float x, float y, float width) {
      roundedCorner(x - width, y, width, Corner::TopRight);
    }

    void bottomLeftCorner(float x, float y, float width) {
      roundedCorner(x, y - width, width, Corner::BottomLeft);
    }

    void bottomRightCorner(float x, float y, float width) {
      roundedCorner(x - width, y - width, width, Corner::BottomRight);
    }

    void bottomCorners(float x, float y, float rectangle_width, float corner_width) {
      bottomLeftCorner(x, y, corner_width);
      bottomRightCorner(x + rectangle_width, y, corner_width);
    }

    void topCorners(float x, float y, float rectangle_width, float corner_width) {
      topLeftCorner(x, y, corner_width);
      topRightCorner(x + rectangle_width, y, corner_width);
    }

    void allCorners(float x, float y, float rectangle_width, float rectangle_height, float corner_width) {
      topLeftCorner(x, y, corner_width);
      topRightCorner(x + rectangle_width, y, corner_width);
      bottomLeftCorner(x, y + rectangle_height, corner_width);
      bottomRightCorner(x + rectangle_width, y + rectangle_height, corner_width);
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

    void icon(const Icon& icon, float x, float y) {
      addShape(IconWrapper(state_.clamp, state_.color, state_.x + x, state_.y + y,
                           icon.width * 1.0f, icon.height * 1.0f, icon));
    }

    void icon(const char* svg_data, int svg_size, float x, float y, int width, int height,
              int blur_radius = 0) {
      Icon icon(svg_data, svg_size, width, height, blur_radius);
      addShape(IconWrapper(state_.clamp, state_.color, state_.x + x, state_.y + y, width * 1.0f,
                           height * 1.0f, icon));
    }

    void icon(const EmbeddedFile& svg, float x, float y, int width, int height, int blur_radius = 0) {
      icon(svg.data, svg.size, x, y, width, height, blur_radius);
    }

    void image(Image* image, float x, float y) {
      images_.insert(image);
      addShape(ImageWrapper(state_.clamp, state_.color, state_.x + x, state_.y + y,
                            image->width() * 1.0f, image->height() * 1.0f, image));
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

    void subcanvas(Canvas* canvas, float x, float y, float width, float height,
                   PostEffect* post_effect = nullptr) {
      state_.current_region->external_canvases_.emplace_back(canvas, post_effect);
      addShape(CanvasWrapper(state_.clamp, 0xffffffff, state_.x + x, state_.y + y, width, height,
                             canvas, post_effect));
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

    void addRegion(Region* region) { base_region_.addRegion(region); }

    void beginRegion(Region* region, int x, int y, int width, int height) {
      region->clear();
      saveState();
      state_.x = 0;
      state_.y = 0;
      setClampBounds(0, 0, width, height);
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
      state_.clamp.left = std::max<int>(state_.clamp.left, state_.x + x);
      state_.clamp.top = std::max<int>(state_.clamp.top, state_.y + y);
      state_.clamp.right = std::max<int>(state_.clamp.left,
                                         std::min<int>(state_.clamp.right, state_.x + x + width));
      state_.clamp.bottom = std::max<int>(state_.clamp.top,
                                          std::min<int>(state_.clamp.bottom, state_.y + y + height));
    }

    void moveClampBounds(int x_offset, int y_offset) {
      state_.clamp.left += x_offset;
      state_.clamp.top += y_offset;
      state_.clamp.right += x_offset;
      state_.clamp.bottom += y_offset;
    }

    const ClampBounds& currentClampBounds() const { return state_.clamp; }

    bool totallyClamped() const {
      return state_.clamp.bottom <= state_.clamp.top || state_.clamp.right <= state_.clamp.left;
    }

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

    IconGroup* iconGroup() {
      if (icon_group_ == nullptr)
        icon_group_ = std::make_unique<IconGroup>();
      return icon_group_.get();
    }

    State* state() { return &state_; }

  private:
    void roundedCorner(float x, float y, float width, Corner corner) {
      addShape(RoundedCorner(state_.clamp, state_.color, state_.x + x, state_.y + y, width, corner));
    }

    template<class T>
    void addShape(T shape) {
      state_.current_region->shape_batcher_.addShape(std::move(shape));
    }

    int redrawImages(int submit_pass);
    int submitExternalCanvases(int submit_pass, Region* region);
    int submitExternalCanvases(int submit_pass);
    void checkFrameBuffer();
    void destroyFrameBuffer() const;

    void* window_handle_ = nullptr;
    std::unique_ptr<FrameBufferData> frame_buffer_data_;
    Palette* palette_ = nullptr;

    int width_ = 0;
    int height_ = 0;
    float width_scale_ = 1.0f;
    float height_scale_ = 1.0f;
    float dpi_scale_ = 1.0f;
    long long start_ms_ = 0;
    double render_time_ = 0.0;
    int render_frame_ = 0;

    std::vector<State> state_memory_;
    State state_;

    std::unique_ptr<IconGroup> icon_group_;
    std::unique_ptr<ImageGroup> image_group_;
    Region base_region_;
    std::set<Image*> images_;
    std::vector<Bounds> invalid_rects_;
    std::vector<Bounds> invalid_rect_pieces_;

    bool bottom_left_origin_ = false;
    bool hdr_ = false;
    float refresh_rate_ = 0.0f;

    VISAGE_LEAK_CHECKER(Canvas)
  };
}