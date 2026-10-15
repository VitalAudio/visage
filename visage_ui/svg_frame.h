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

#include "frame.h"
#include "visage_file_embed/embedded_file.h"

namespace visage {
  class SvgFrame : public Frame {
  public:
    SvgFrame() { setIgnoresMouseEvents(true, false); }

    SvgFrame(const EmbeddedFile& file) { load(file); }
    SvgFrame(const uint8_t* data, size_t size) { load(data, size); }

    void load(const Svg& svg) {
      svg_ = svg;
      loadSubFrames();
      redraw();
    }

    void load(const EmbeddedFile& file) {
      svg_ = Svg(file);
      loadSubFrames();
      redraw();
    }

    void load(const uint8_t* data, size_t size) {
      svg_ = Svg(data, size);
      loadSubFrames();
      redraw();
    }

    void setMargin(const Dimension& margin) {
      margin_ = margin;
      setDimensions();
    }

    void setFillBrush(const Brush& brush) {
      svg_.setFillBrush(brush);
      redrawAll();
    }

    void resetFillBrush() {
      svg_.resetFillBrush();
      redrawAll();
    }

    void setStrokeBrush(const Brush& brush) {
      svg_.setStrokeBrush(brush);
      redrawAll();
    }

    void resetStrokeBrush() {
      svg_.resetStrokeBrush();
      redrawAll();
    }

    void setCurrentColor(const Brush& brush) {
      svg_.setCurrentColor(brush);
      redrawAll();
    }

  private:
    class SubFrame : public Frame {
    public:
      SubFrame(SvgDrawable* drawable, SvgDrawable::ColorContext* context, bool content_only = false,
               int clip_level = 0) :
          drawable_(drawable), context_(context), content_only_(content_only) {
        if (content_only)
          return;

        if (clip_level == 0)
          setAlphaTransparency(drawable_->opacity);

        const SvgDrawable* mask = drawable_->maskContent();
        int num_clips = drawable_->clipping_paths.empty() ? 0 : drawable_->clip_intersections.size() + 1;
        bool clipped = clip_level < num_clips;
        if (clipped)
          setClipMask(clip_level);

        if (clipped && (mask || clip_level + 1 < num_clips)) {
          content_only_ = true;
          addSubFrame(std::make_unique<SubFrame>(drawable_, context_, false, clip_level + 1));
          return;
        }

        addSubFrames(drawable);
        if (mask)
          setContentMask(mask);
      }

      void setClipMask(int clip_level) {
        onDrawMask() = [this, clip_level](Canvas& canvas) {
          if (clip_level > drawable_->clip_intersections.size())
            return;

          const auto& clipping_paths = clip_level ? drawable_->clip_intersections[clip_level - 1] :
                                                    drawable_->clipping_paths;
          canvas.setColor(0xffffffff);
          for (const auto& clipping_path : clipping_paths)
            canvas.fill(clipping_path, -svgOrigin().x, -svgOrigin().y);
        };
      }

      void setContentMask(const SvgDrawable* mask) {
        setMaskType(mask->mask_settings->alpha ? MaskType::Alpha : MaskType::Luminance);
        onDrawMask() = [this, mask](Canvas& canvas) {
          Point origin = svgOrigin();
          for (const auto& child : mask->children)
            child->drawAll(canvas, context_, -origin.x, -origin.y, width(), height());

          canvas.setBlendMode(BlendMode::Composite);
          canvas.setColor(0xff000000);
          for (const auto& region : mask->clipping_paths) {
            Path outside = region;
            outside.setFillRule(Path::FillRule::EvenOdd);
            outside.addRectangle(origin.x, origin.y, width(), height());
            canvas.fill(outside, -origin.x, -origin.y);
          }
        };
      }

      void addSubFrame(std::unique_ptr<SubFrame> child) {
        addChild(child.get());
        children_.push_back(std::move(child));
      }

      void addSubFrames(SvgDrawable* drawable) {
        for (auto& child : drawable->children) {
          if (child->is_defines || child->is_clip_path || child->mask_settings || !child->display ||
              child->opacity <= 0.0f)
            continue;

          if (child->opacity != 1.0f || !child->clipping_paths.empty() || child->maskContent()) {
            addSubFrame(std::make_unique<SubFrame>(child.get(), context_));
            content_ = nullptr;
            continue;
          }

          if (content_ == nullptr) {
            addSubFrame(std::make_unique<SubFrame>(drawable_, context_, true));
            content_ = children_.back().get();
          }
          content_->child_drawables_.push_back(child.get());
          addSubFrames(child.get());
        }
      }

      IBounds nativeBoundingBox() const {
        const Bounds& box = drawable_->post_bounding_box;
        int x = std::floor(box.x() * dpiScale());
        int y = std::floor(box.y() * dpiScale());
        return { x, y, static_cast<int>(std::ceil(box.right() * dpiScale())) - x,
                 static_cast<int>(std::ceil(box.bottom() * dpiScale())) - y };
      }

      Point svgOrigin() const {
        IBounds box = nativeBoundingBox();
        return { box.x() / dpiScale(), box.y() / dpiScale() };
      }

      void draw(Canvas& canvas) override {
        float offset_x = -svgOrigin().x;
        float offset_y = -svgOrigin().y;

        if (!content_only_)
          drawable_->draw(canvas, context_, offset_x, offset_y, width(), height());
        for (auto& child_drawable : child_drawables_)
          child_drawable->draw(canvas, context_, offset_x, offset_y, width(), height());
      }

      void resized() override {
        IBounds box = nativeBoundingBox();
        for (auto& child : children_)
          child->setNativeBounds(child->nativeBoundingBox() + IPoint(-box.x(), -box.y()));
      }

    private:
      SvgDrawable* drawable_ = nullptr;
      std::vector<std::unique_ptr<SubFrame>> children_;
      std::vector<SvgDrawable*> child_drawables_;
      SubFrame* content_ = this;
      SvgDrawable::ColorContext* context_ = nullptr;
      bool content_only_ = false;
    };

    void setDimensions();
    void loadSubFrames();

    void resized() override {
      context_ = {};
      setDimensions();
    }

    Svg svg_;
    SvgDrawable::ColorContext context_;
    std::unique_ptr<SubFrame> sub_frame_;
    Dimension margin_;
  };
}