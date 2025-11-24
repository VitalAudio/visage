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
    int m = margin_.compute(dpiScale(), nativeWidth(), nativeHeight(), 0.0f);
    svg_.setDimensions(width() - 2 * m / dpiScale(), height() - 2 * m / dpiScale(), dpiScale());

    if (sub_frame_ == nullptr && svg_.width() && svg_.height()) {
      sub_frame_ = std::make_unique<SubFrame>(svg_.drawable(), &context_);
      addChild(sub_frame_.get());
    }

    if (sub_frame_) {
      sub_frame_->setNativeBounds(m + svg_.drawable()->post_bounding_box.x() * dpiScale(),
                                  m + svg_.drawable()->post_bounding_box.y() * dpiScale(),
                                  nativeWidth() - 2 * m, nativeHeight() - 2 * m);
    }
  }

  void SvgFrame::loadSubFrames() {
    sub_frame_ = nullptr;
    setDimensions();
  }
}