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
      SubFrame(SvgDrawable* drawable, SvgDrawable::ColorContext* context, bool content_only = false) :
          drawable_(drawable), context_(context), content_only_(content_only) {
        if (content_only)
          return;

        setAlphaTransparency(drawable_->opacity);
        addSubFrames(drawable);

        if (!drawable_->clipping_paths.empty()) {
          setMasked(true);
          clipping_frame_ = std::make_unique<Frame>();
          clipping_frame_->onDraw() = [this](Canvas& canvas) {
            float offset_x = -svgOrigin().x;
            float offset_y = -svgOrigin().y;

            for (int i = 0; i < drawable_->clipping_paths.size(); ++i) {
              canvas.setBlendMode(BlendMode::Mult);
              canvas.setColor(0xffffffff);
              canvas.fill(drawable_->clipping_paths[i], offset_x, offset_y);
              auto bounding_box = drawable_->clipping_paths[i].boundingBox();

              canvas.setColor(0x00ffffff);
              float min_x = bounding_box.x() + offset_x;
              float max_x = bounding_box.right() + offset_x;
              canvas.fill(0, 0, min_x, height());
              canvas.fill(max_x, 0, width() - max_x, height());
              float min_y = bounding_box.y() + offset_y;
              float max_y = bounding_box.bottom() + offset_y;
              canvas.fill(min_x, 0, max_x - min_x, min_y);
              canvas.fill(min_x, max_y, max_x - min_x, height() - max_y);
            }
          };
          addChild(clipping_frame_.get());
          clipping_frame_->setOnTop(true);
        }
      }

      void addSubFrame(std::unique_ptr<SubFrame> child) {
        addChild(child.get());
        children_.push_back(std::move(child));
      }

      void addSubFrames(SvgDrawable* drawable) {
        for (auto& child : drawable->children) {
          if (child->is_defines || child->opacity <= 0.0f)
            continue;

          if (child->opacity != 1.0f || !child->clipping_paths.empty()) {
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

        if (clipping_frame_)
          clipping_frame_->setBounds(localBounds());
      }

    private:
      std::unique_ptr<Frame> clipping_frame_;
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