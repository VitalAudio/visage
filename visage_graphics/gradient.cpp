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
  std::string Gradient::encode() const {
    std::ostringstream stream;
    encode(stream);
    return stream.str();
  }

  void Gradient::encode(std::ostringstream& stream) const {
    stream << static_cast<int>(repeat_) << std::endl;
    stream << static_cast<int>(reflect_) << std::endl;
    stream << static_cast<int>(colors_.size()) << std::endl;
    for (float position : positions_)
      stream << position << " ";
    stream << std::endl;
    for (const Color& color : colors_)
      color.encode(stream);
  }

  void Gradient::decode(const std::string& data) {
    std::istringstream stream(data);
    decode(stream);
  }

  void Gradient::decode(std::istringstream& stream) {
    int size, repeat, reflect;
    stream >> repeat >> reflect >> size;
    repeat_ = repeat;
    reflect_ = reflect;

    positions_.resize(size);
    colors_.resize(size);
    for (auto& position : positions_)
      stream >> position;
    for (Color& color : colors_)
      color.decode(stream);
  }

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
    atlas_map_.fixWidth(Gradient::kMaxGradientResolution);
    atlas_map_.setPadding(0);
  }

  GradientAtlas::~GradientAtlas() = default;

  void GradientAtlas::updateGradient(const PackedGradientRect* gradient) {
    if (texture_ == nullptr || !bgfx::isValid(texture_->handle))
      return;

    int resolution = gradient->gradient.resolution();
    std::unique_ptr<uint64_t[]> color_data = std::make_unique<uint64_t[]>(resolution);
    float step = 1.0f / std::max(1, resolution - 1);
    for (int i = 0; i < resolution; ++i)
      color_data[i] = gradient->gradient.sample(i * step).toABGR16F();

    bgfx::updateTexture2D(texture_->handle, 0, 0, gradient->x, gradient->y, resolution, 1,
                          bgfx::copy(color_data.get(), resolution * sizeof(uint64_t)));
  }

  void GradientAtlas::checkInit() {
    if (texture_ == nullptr)
      texture_ = std::make_unique<GradientAtlasTexture>();

    if (!bgfx::isValid(texture_->handle)) {
      texture_->handle = bgfx::createTexture2D(atlas_map_.width(), atlas_map_.height(), false, 1,
                                               bgfx::TextureFormat::RGBA16F);

      for (auto& gradient : gradients_)
        updateGradient(gradient.second.get());
    }
  }

  void GradientAtlas::destroy() {
    texture_.reset();
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

  std::string GradientPosition::encode() const {
    std::ostringstream stream;
    encode(stream);
    return stream.str();
  }

  void GradientPosition::encode(std::ostringstream& stream) const {
    stream << static_cast<int>(shape) << std::endl;
    stream << from.x << std::endl;
    stream << from.y << std::endl;
    stream << to.x << std::endl;
    stream << to.y << std::endl;
  }

  void GradientPosition::decode(const std::string& data) {
    std::istringstream stream(data);
    decode(stream);
  }

  void GradientPosition::decode(std::istringstream& stream) {
    int shape_int = 0;
    stream >> shape_int;
    shape = static_cast<InterpolationShape>(shape_int);
    stream >> from.x;
    stream >> from.y;
    stream >> to.x;
    stream >> to.y;
  }

  std::string Brush::encode() const {
    std::ostringstream stream;
    encode(stream);
    return stream.str();
  }

  void Brush::encode(std::ostringstream& stream) const {
    gradient_.encode(stream);
    position_.encode(stream);
  }

  void Brush::decode(const std::string& data) {
    std::istringstream stream(data);
    decode(stream);
  }

  void Brush::decode(std::istringstream& stream) {
    gradient_.decode(stream);
    position_.decode(stream);
  }
}