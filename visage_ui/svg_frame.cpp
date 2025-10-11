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

#include "svg_frame.h"

namespace visage {
  void SvgFrame::setDimensions() {
    if (sub_frame_ == nullptr)
      return;

    int m = margin_.compute(dpiScale(), nativeWidth(), nativeHeight(), 0.0f);
    svg_.setDimensions(width() - 2 * m / dpiScale(), height() - 2 * m / dpiScale());
    sub_frame_->setNativeBounds(m, m, nativeWidth() - 2 * m, nativeHeight() - 2 * m);
  }

  void SvgFrame::loadSubFrames(SubFrame* frame, SvgDrawable* drawable) {
    for (auto& child : drawable->children) {
      auto sub_frame = std::make_unique<SubFrame>(child.get(), &context_);
      loadSubFrames(sub_frame.get(), child.get());
      frame->addChild(std::move(sub_frame));
    }
  }

  void SvgFrame::loadSubFrames() {
    sub_frame_ = std::make_unique<SubFrame>(svg_.drawable(), &context_);
    loadSubFrames(sub_frame_.get(), svg_.drawable());
    addChild(sub_frame_.get());
    setDimensions();
  }
}