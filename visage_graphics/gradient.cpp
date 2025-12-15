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
  visage::Gradient Gradient::kViridis =
      Gradient(0xFF440154, 0xFF450457, 0xFF46075A, 0xFF460A5D, 0xFF470D60, 0xFF471063, 0xFF471365,
               0xFF481668, 0xFF48186A, 0xFF481B6D, 0xFF481E6F, 0xFF482072, 0xFF482374, 0xFF482576,
               0xFF482878, 0xFF472A7A, 0xFF472D7B, 0xFF472F7D, 0xFF46327F, 0xFF463480, 0xFF453781,
               0xFF443983, 0xFF443C84, 0xFF433E85, 0xFF424086, 0xFF414387, 0xFF404588, 0xFF3F4788,
               0xFF3E4A89, 0xFF3D4C8A, 0xFF3C4E8A, 0xFF3B508B, 0xFF3A528B, 0xFF39558C, 0xFF38578C,
               0xFF37598C, 0xFF375B8D, 0xFF365D8D, 0xFF355F8D, 0xFF34618D, 0xFF33638D, 0xFF32658E,
               0xFF31678E, 0xFF30698E, 0xFF2F6B8E, 0xFF2E6D8E, 0xFF2E6F8E, 0xFF2D718E, 0xFF2C738E,
               0xFF2B758E, 0xFF2A778E, 0xFF2A798E, 0xFF297A8E, 0xFF287C8E, 0xFF277E8E, 0xFF27808E,
               0xFF26828E, 0xFF25848E, 0xFF24868E, 0xFF24888E, 0xFF238A8D, 0xFF228B8D, 0xFF228D8D,
               0xFF218F8D, 0xFF20918C, 0xFF20938C, 0xFF1F958B, 0xFF1F978B, 0xFF1F998A, 0xFF1F9A8A,
               0xFF1E9C89, 0xFF1F9E89, 0xFF1FA088, 0xFF1FA287, 0xFF20A486, 0xFF21A685, 0xFF22A884,
               0xFF23A983, 0xFF25AB82, 0xFF27AD81, 0xFF29AF80, 0xFF2BB17E, 0xFF2EB37D, 0xFF30B47B,
               0xFF33B67A, 0xFF36B878, 0xFF39BA76, 0xFF3DBB74, 0xFF40BD73, 0xFF44BF71, 0xFF47C06F,
               0xFF4BC26C, 0xFF4FC46A, 0xFF53C568, 0xFF57C766, 0xFF5BC863, 0xFF60CA61, 0xFF64CB5E,
               0xFF69CD5B, 0xFF6DCE59, 0xFF72CF56, 0xFF77D153, 0xFF7CD250, 0xFF81D34D, 0xFF86D44A,
               0xFF8BD647, 0xFF90D743, 0xFF95D840, 0xFF9AD93D, 0xFF9FDA39, 0xFFA5DB36, 0xFFAADC32,
               0xFFAFDD2F, 0xFFB5DD2B, 0xFFBADE28, 0xFFBFDF25, 0xFFC5E022, 0xFFCAE11F, 0xFFD0E11C,
               0xFFD5E21A, 0xFFDAE319, 0xFFDFE318, 0xFFE4E419, 0xFFEAE41A, 0xFFEFE51C, 0xFFF4E61E,
               0xFFF8E621, 0xFFFDE725);

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
    if (resolution == 0)
      return;

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
    }

    if (repacked_) {
      repacked_ = false;
      for (auto& gradient : gradients_)
        updateGradient(gradient.second.get());
    }
  }

  void GradientAtlas::destroy() {
    texture_.reset();
  }

  void GradientAtlas::resize() {
    int prev_width = atlas_map_.width();
    int prev_height = atlas_map_.height();
    atlas_map_.pack();
    repacked_ = true;
    if (atlas_map_.width() != prev_width && atlas_map_.height() != prev_height)
      texture_.reset();

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
    stream << point1.x << std::endl;
    stream << point1.y << std::endl;
    stream << point2.x << std::endl;
    stream << point2.y << std::endl;
  }

  void GradientPosition::decode(const std::string& data) {
    std::istringstream stream(data);
    decode(stream);
  }

  void GradientPosition::decode(std::istringstream& stream) {
    int shape_int = 0;
    stream >> shape_int;
    shape = static_cast<InterpolationShape>(shape_int);
    stream >> point1.x;
    stream >> point1.y;
    stream >> point2.x;
    stream >> point2.y;
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