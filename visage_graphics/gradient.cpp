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

#include "gradient.h"

#include <bgfx/bgfx.h>

namespace visage {
  struct GradientAtlasTexture {
    bgfx::TextureHandle handle = { bgfx::kInvalidHandle };

    ~GradientAtlasTexture() {
      if (bgfx::isValid(handle))
        bgfx::destroy(handle);
    }
  };

  GradientAtlas::PackedGradientReference::~PackedGradientReference() {
    if (auto atlas_pointer = atlas.lock())
      (*atlas_pointer)->removeGradient(packed_gradient_rect);
  }

  GradientAtlas::GradientAtlas() {
    reference_ = std::make_shared<GradientAtlas*>(this);
  }

  GradientAtlas::~GradientAtlas() = default;

  void GradientAtlas::updateGradient(const PackedGradientRect* gradient) {
    if (texture_ == nullptr || !bgfx::isValid(texture_->handle))
      return;

    int resolution = gradient->gradient.resolution();
    bgfx::updateTexture2D(texture_->handle, 0, 0, gradient->x, gradient->y, resolution, 1,
                          bgfx::copy(gradient->gradient.colors().data(), resolution * sizeof(uint64_t)));
  }

  void GradientAtlas::checkInit() {
    if (texture_ == nullptr)
      texture_ = std::make_unique<GradientAtlasTexture>();

    if (!bgfx::isValid(texture_->handle)) {
      texture_->handle = bgfx::createTexture2D(atlas_map_.width(), atlas_map_.height(), false, 1,
                                               bgfx::TextureFormat::RGBA16, BGFX_SAMPLER_UVW_CLAMP);

      for (auto& gradient : gradients_)
        updateGradient(gradient.second.get());
    }
  }

  void GradientAtlas::resize() {
    texture_.reset();
    atlas_map_.pack();

    for (auto& gradient : gradients_) {
      const PackedRect& rect = atlas_map_.rectForId(gradient.second.get());
      gradient.second->x = rect.x;
      gradient.second->y = rect.y;
    }
  }

  const bgfx::TextureHandle& GradientAtlas::colorTextureHandle() {
    checkInit();
    return texture_->handle;
  }
}