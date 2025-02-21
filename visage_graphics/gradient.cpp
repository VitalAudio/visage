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
    bgfx::TextureHandle color_handle = { bgfx::kInvalidHandle };
    bgfx::TextureHandle hdr_handle = { bgfx::kInvalidHandle };
  };

  GradientAtlas::GradientAtlas() {
    addGradient(Gradient(Color(0)));
    addGradient(Gradient(Color(0xffffffff)));
    addGradient(Gradient(Color(0xff000000)));
  }

  GradientAtlas::~GradientAtlas() = default;

  void GradientAtlas::updateGradient(const PackedGradient* gradient) {
    if (texture_ == nullptr || !bgfx::isValid(texture_->color_handle))
      return;

    int resolution = gradient->gradient.resolution();
    bgfx::updateTexture2D(texture_->color_handle, 0, 0, gradient->x, gradient->y, resolution, 1,
                          bgfx::copy(gradient->gradient.colors().data(), resolution * 4));
    bgfx::updateTexture2D(texture_->hdr_handle, 0, 0, gradient->x, gradient->y, resolution, 1,
                          bgfx::copy(gradient->gradient.hdrs().data(), resolution));
  }

  void GradientAtlas::checkInit() {
    if (texture_ == nullptr)
      texture_ = std::make_unique<GradientAtlasTexture>();

    if (!bgfx::isValid(texture_->color_handle)) {
      texture_->color_handle = bgfx::createTexture2D(atlas_.width(), atlas_.height(), false, 1,
                                                     bgfx::TextureFormat::BGRA8, BGFX_SAMPLER_UVW_CLAMP);
      texture_->hdr_handle = bgfx::createTexture2D(atlas_.width(), atlas_.height(), false, 1,
                                                   bgfx::TextureFormat::R32F, BGFX_SAMPLER_UVW_CLAMP);

      for (auto& gradient : gradients_)
        updateGradient(gradient.second.get());
    }
  }

  void GradientAtlas::resize() {
    if (texture_ && bgfx::isValid(texture_->color_handle))
      bgfx::destroy(texture_->color_handle);
    if (texture_ && bgfx::isValid(texture_->hdr_handle))
      bgfx::destroy(texture_->hdr_handle);

    texture_.reset();
    atlas_.pack();

    for (auto& gradient : gradients_) {
      const PackedRect& rect = atlas_.rectForId(gradient.second.get());
      gradient.second->x = rect.x;
      gradient.second->y = rect.y;
    }
  }

  const bgfx::TextureHandle& GradientAtlas::colorTextureHandle() {
    checkInit();
    return texture_->color_handle;
  }

  const bgfx::TextureHandle& GradientAtlas::hdrTextureHandle() {
    checkInit();
    return texture_->hdr_handle;
  }
}