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
    }

    void load(const EmbeddedFile& file) {
      svg_ = Svg(file);
      loadSubFrames();
    }

    void load(const uint8_t* data, size_t size) {
      svg_ = Svg(data, size);
      loadSubFrames();
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
      SubFrame(SvgDrawable* drawable, SvgDrawable::ColorContext* context) :
          drawable_(drawable), context_(context) {
        setAlphaTransparency(drawable_->opacity);
      }

      void draw(Canvas& canvas) override {
        setAlphaTransparency(drawable_->opacity);
        drawable_->draw(canvas, context_, 0, 0, width(), height());
      }

      void resized() override {
        for (auto& child : children())
          child->setBounds(localBounds());
      }

    private:
      SvgDrawable* drawable_ = nullptr;
      SvgDrawable::ColorContext* context_ = nullptr;
    };

    void setDimensions();
    void loadSubFrames(SubFrame* frame, SvgDrawable* drawable);
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