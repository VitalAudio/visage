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

#include "gpu/gpu.h"
#include <sstream>

namespace visage {

  visage::Gradient Gradient::kViridis =
      Gradient(0xff440154, 0xff450457, 0xff46075a, 0xff460a5d, 0xff470d60, 0xff471063, 0xff471365,
               0xff481668, 0xff48186a, 0xff481b6d, 0xff481e6f, 0xff482072, 0xff482374, 0xff482576,
               0xff482878, 0xff472a7a, 0xff472d7b, 0xff472f7d, 0xff46327f, 0xff463480, 0xff453781,
               0xff443983, 0xff443c84, 0xff433e85, 0xff424086, 0xff414387, 0xff404588, 0xff3f4788,
               0xff3e4a89, 0xff3d4c8a, 0xff3c4e8a, 0xff3b508b, 0xff3a528b, 0xff39558c, 0xff38578c,
               0xff37598c, 0xff375b8d, 0xff365d8d, 0xff355f8d, 0xff34618d, 0xff33638d, 0xff32658e,
               0xff31678e, 0xff30698e, 0xff2f6b8e, 0xff2e6d8e, 0xff2e6f8e, 0xff2d718e, 0xff2c738e,
               0xff2b758e, 0xff2a778e, 0xff2a798e, 0xff297a8e, 0xff287c8e, 0xff277e8e, 0xff27808e,
               0xff26828e, 0xff25848e, 0xff24868e, 0xff24888e, 0xff238a8d, 0xff228b8d, 0xff228d8d,
               0xff218f8d, 0xff20918c, 0xff20938c, 0xff1f958b, 0xff1f978b, 0xff1f998a, 0xff1f9a8a,
               0xff1e9c89, 0xff1f9e89, 0xff1fa088, 0xff1fa287, 0xff20a486, 0xff21a685, 0xff22a884,
               0xff23a983, 0xff25ab82, 0xff27ad81, 0xff29af80, 0xff2bb17e, 0xff2eb37d, 0xff30b47b,
               0xff33b67a, 0xff36b878, 0xff39ba76, 0xff3dbb74, 0xff40bd73, 0xff44bf71, 0xff47c06f,
               0xff4bc26c, 0xff4fc46a, 0xff53c568, 0xff57c766, 0xff5bc863, 0xff60ca61, 0xff64cb5e,
               0xff69cd5b, 0xff6dce59, 0xff72cf56, 0xff77d153, 0xff7cd250, 0xff81d34d, 0xff86d44a,
               0xff8bd647, 0xff90d743, 0xff95d840, 0xff9ad93d, 0xff9fda39, 0xffa5db36, 0xffaadc32,
               0xffafdd2f, 0xffb5dd2b, 0xffbade28, 0xffbfdf25, 0xffc5e022, 0xffcae11f, 0xffd0e11c,
               0xffd5e21a, 0xffdae319, 0xffdfe318, 0xffe4e419, 0xffeae41a, 0xffefe51c, 0xfff4e61e,
               0xfff8e621, 0xfffde725);

  visage::Gradient Gradient::kMagma = Gradient(0xff000004, 0xff08051a, 0xff140e36, 0xff241253,
                                               0xff331067, 0xff42106a, 0xff50106b, 0xff5f136e,
                                               0xff6d186e, 0xff7c1d6f, 0xff8a226f, 0xff99266e,
                                               0xffa82b6c, 0xffb73069, 0xffc63663, 0xffd43d5c,
                                               0xffe24452, 0xffec4c46, 0xfff65539, 0xfffb5f2c,
                                               0xfffd6a1e, 0xfffe7611, 0xfffd8405, 0xfff98e09,
                                               0xfff39a1a, 0xffeda62b, 0xffe7b83f, 0xffe1c84f,
                                               0xffdcd65f, 0xffe6e97a, 0xfff1f3a1, 0xfffcfdbf);

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
    int size = 0, repeat = 0, reflect = 0;
    stream >> repeat >> reflect >> size;
    repeat_ = repeat;
    reflect_ = reflect;

    size = std::max(0, size);
    positions_.resize(size);
    colors_.resize(size);
    for (auto& position : positions_)
      stream >> position;
    for (Color& color : colors_)
      color.decode(stream);
  }

  struct GradientAtlasTexture {
    gpu::TextureHandle handle = {};

    ~GradientAtlasTexture() {
      if (gpu::isValid(handle))
        gpu::destroy(handle);
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
    int width = atlas_map_.width();
    if (repacked_ || color_data_.size() != static_cast<size_t>(width) * atlas_map_.height())
      return;

    int resolution = gradient->gradient.resolution();
    if (resolution == 0)
      return;

    uint64_t* row = color_data_.data() + gradient->y * width + gradient->x;
    float step = 1.0f / std::max(1, resolution - 1);
    for (int i = 0; i < resolution; ++i)
      row[i] = gradient->gradient.sample(i * step).toABGR16F();

    if (dirty_top_ >= dirty_bottom_) {
      dirty_top_ = gradient->y;
      dirty_bottom_ = gradient->y + 1;
    }
    else {
      dirty_top_ = std::min(dirty_top_, gradient->y);
      dirty_bottom_ = std::max(dirty_bottom_, gradient->y + 1);
    }
  }

  void GradientAtlas::uploadDirtyRows() {
    if (dirty_top_ >= dirty_bottom_)
      return;

    int width = atlas_map_.width();
    int rows = dirty_bottom_ - dirty_top_;
    gpu::updateTexture2D(texture_->handle, 0, dirty_top_, width, rows,
                          gpu::copy(color_data_.data() + dirty_top_ * width,
                                     width * rows * sizeof(uint64_t)));
    dirty_top_ = 0;
    dirty_bottom_ = 0;
  }

  void GradientAtlas::checkInit() {
    if (texture_ == nullptr)
      texture_ = std::make_unique<GradientAtlasTexture>();

    int width = atlas_map_.width();
    int height = atlas_map_.height();
    if (!gpu::isValid(texture_->handle)) {
      texture_->handle = gpu::createTexture2D(width, height, gpu::TextureFormat::RGBA16F);
      repacked_ = true;
    }

    if (repacked_) {
      repacked_ = false;
      color_data_.assign(static_cast<size_t>(width) * height, 0);
      for (auto& gradient : gradients_)
        updateGradient(gradient.second.get());
      dirty_top_ = 0;
      dirty_bottom_ = height;
    }

    uploadDirtyRows();
  }

  void GradientAtlas::destroy() {
    texture_.reset();
  }

  void GradientAtlas::resize() {
    int prev_width = atlas_map_.width();
    int prev_height = atlas_map_.height();
    atlas_map_.pack();
    repacked_ = true;
    if (atlas_map_.width() != prev_width || atlas_map_.height() != prev_height)
      texture_.reset();

    for (auto& row : solid_rows_) {
      const PackedRect& rect = atlas_map_.rectForId(row.get());
      row->x = rect.x;
      row->y = rect.y;
    }

    for (auto& gradient : gradients_) {
      PackedGradientRect* packed_gradient_rect = gradient.second.get();
      if (packed_gradient_rect->solid_row >= 0) {
        const PackedGradientRect* row = solid_rows_[packed_gradient_rect->solid_row].get();
        packed_gradient_rect->x = row->x + packed_gradient_rect->solid_column;
        packed_gradient_rect->y = row->y;
      }
      else {
        const PackedRect& rect = atlas_map_.rectForId(packed_gradient_rect);
        packed_gradient_rect->x = rect.x;
        packed_gradient_rect->y = rect.y;
      }
    }
  }

  void GradientAtlas::addSolid(PackedGradientRect* packed_gradient_rect) {
    if (free_solids_.empty()) {
      int row_index = solid_rows_.size();
      solid_rows_.push_back(std::make_unique<PackedGradientRect>(Gradient()));
      PackedGradientRect* row = solid_rows_.back().get();
      if (!atlas_map_.addRect(row, Gradient::kMaxGradientResolution, 1))
        resize();

      const PackedRect& rect = atlas_map_.rectForId(row);
      row->x = rect.x;
      row->y = rect.y;
      for (int i = Gradient::kMaxGradientResolution - 1; i >= 0; --i)
        free_solids_.push_back({ row_index, i });
    }

    auto [row_index, column] = free_solids_.back();
    free_solids_.pop_back();
    const PackedGradientRect* row = solid_rows_[row_index].get();
    packed_gradient_rect->solid_row = row_index;
    packed_gradient_rect->solid_column = column;
    packed_gradient_rect->x = row->x + column;
    packed_gradient_rect->y = row->y;
  }

  const gpu::TextureHandle& GradientAtlas::colorTextureHandle() {
    checkInit();
    return texture_->handle;
  }

  std::string GradientPosition::encode() const {
    std::ostringstream stream;
    encode(stream);
    return stream.str();
  }

  static constexpr int kRadialWithCoefficients = -1;

  void GradientPosition::encode(std::ostringstream& stream) const {
    if (shape == InterpolationShape::Radial)
      stream << kRadialWithCoefficients << std::endl;
    else
      stream << static_cast<int>(shape) << std::endl;
    stream << point1.x << std::endl;
    stream << point1.y << std::endl;
    stream << point2.x << std::endl;
    stream << point2.y << std::endl;
    if (shape == InterpolationShape::Radial) {
      stream << focal_radius << std::endl;
      stream << coefficientx2 << std::endl;
      stream << coefficienty2 << std::endl;
      stream << coefficientxy << std::endl;
    }
  }

  void GradientPosition::decode(const std::string& data) {
    std::istringstream stream(data);
    decode(stream);
  }

  void GradientPosition::decode(std::istringstream& stream) {
    int shape_int = 0;
    stream >> shape_int;
    bool has_coefficients = shape_int == kRadialWithCoefficients;
    shape = has_coefficients ? InterpolationShape::Radial : static_cast<InterpolationShape>(shape_int);
    stream >> point1.x;
    stream >> point1.y;
    stream >> point2.x;
    stream >> point2.y;
    if (has_coefficients) {
      stream >> focal_radius;
      stream >> coefficientx2;
      stream >> coefficienty2;
      stream >> coefficientxy;
    }
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