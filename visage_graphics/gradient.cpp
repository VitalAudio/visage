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
    stream << static_cast<int>(colors_.size()) << std::endl;
    for (const Color& color : colors_)
      color.encode(stream);
  }

  void Gradient::decode(const std::string& data) {
    std::istringstream stream(data);
    decode(stream);
  }

  void Gradient::decode(std::istringstream& stream) {
    int size = 0;
    stream >> size;
    colors_.resize(size);
    for (int i = 0; i < size; ++i)
      colors_[i].decode(stream);
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
  }

  GradientAtlas::~GradientAtlas() = default;

  void GradientAtlas::updateGradient(const PackedGradientRect* gradient) {
    if (texture_ == nullptr || !bgfx::isValid(texture_->handle))
      return;

    const auto& colors = gradient->gradient.colors();
    std::unique_ptr<uint64_t[]> color_data = std::make_unique<uint64_t[]>(colors.size());
    for (int i = 0; i < colors.size(); ++i)
      color_data[i] = colors[i].toABGR16F();

    bgfx::updateTexture2D(texture_->handle, 0, 0, gradient->x, gradient->y, colors.size(), 1,
                          bgfx::copy(color_data.get(), colors.size() * sizeof(uint64_t)));
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
    stream << point_from.x << std::endl;
    stream << point_from.y << std::endl;
    stream << point_to.x << std::endl;
    stream << point_to.y << std::endl;
  }

  void GradientPosition::decode(const std::string& data) {
    std::istringstream stream(data);
    decode(stream);
  }

  void GradientPosition::decode(std::istringstream& stream) {
    int shape_int = 0;
    stream >> shape_int;
    shape = static_cast<InterpolationShape>(shape_int);
    stream >> point_from.x;
    stream >> point_from.y;
    stream >> point_to.x;
    stream >> point_to.y;
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