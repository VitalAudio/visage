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

#include "gpu_backend.h"

#include <cstring>

namespace visage::gpu {
  static constexpr char kMagic[4] = { 'V', 'S', 'G', 'P' };
  static constexpr uint8_t kVersion = 1;

  namespace {
    class Writer {
    public:
      void u8(uint8_t value) { data_.push_back(value); }
      void u16(uint16_t value) {
        u8(value & 0xff);
        u8(value >> 8);
      }
      void u32(uint32_t value) {
        u16(value & 0xffff);
        u16(value >> 16);
      }
      void bytes(const void* data, size_t size) {
        auto bytes = static_cast<const uint8_t*>(data);
        data_.insert(data_.end(), bytes, bytes + size);
      }
      void string(const std::string& value) {
        u8(static_cast<uint8_t>(value.size()));
        bytes(value.data(), value.size());
      }

      std::vector<uint8_t> take() { return std::move(data_); }

    private:
      std::vector<uint8_t> data_;
    };

    class Reader {
    public:
      Reader(const uint8_t* data, uint32_t size) : data_(data), size_(size) { }

      bool u8(uint8_t& value) {
        if (position_ >= size_)
          return false;
        value = data_[position_++];
        return true;
      }
      bool u16(uint16_t& value) {
        uint8_t low = 0, high = 0;
        if (!u8(low) || !u8(high))
          return false;
        value = low | (high << 8);
        return true;
      }
      bool u32(uint32_t& value) {
        uint16_t low = 0, high = 0;
        if (!u16(low) || !u16(high))
          return false;
        value = low | (static_cast<uint32_t>(high) << 16);
        return true;
      }
      bool bytes(void* result, uint32_t size) {
        if (size > size_ - position_)
          return false;
        std::memcpy(result, data_ + position_, size);
        position_ += size;
        return true;
      }
      bool string(std::string& value) {
        uint8_t length = 0;
        if (!u8(length))
          return false;
        value.resize(length);
        return bytes(value.data(), length);
      }

      uint32_t remaining() const { return size_ - position_; }

    private:
      const uint8_t* data_ = nullptr;
      uint32_t size_ = 0;
      uint32_t position_ = 0;
    };
  }

  std::vector<uint8_t> writeShader(const ShaderInfo& info) {
    Writer writer;
    writer.bytes(kMagic, sizeof(kMagic));
    writer.u8(kVersion);
    writer.u8(static_cast<uint8_t>(info.stage));
    writer.u8(static_cast<uint8_t>(info.code_type));
    writer.u8(0);

    writer.u16(static_cast<uint16_t>(info.uniforms.size()));
    for (const ShaderUniform& uniform : info.uniforms) {
      writer.string(uniform.name);
      writer.u8(static_cast<uint8_t>(uniform.type));
      writer.u16(uniform.num);
      writer.u16(uniform.offset);
    }
    writer.u32(info.uniform_block_size);

    writer.u8(static_cast<uint8_t>(info.samplers.size()));
    for (const ShaderSampler& sampler : info.samplers) {
      writer.string(sampler.name);
      writer.u8(sampler.stage);
    }

    writer.u8(static_cast<uint8_t>(info.storage.size()));
    for (const ShaderStorage& storage : info.storage) {
      writer.string(storage.name);
      writer.u8(storage.stage);
      writer.u8(storage.image ? 1 : 0);
      writer.u8(static_cast<uint8_t>(storage.access));
    }

    writer.u8(static_cast<uint8_t>(info.attributes.size()));
    for (const ShaderAttribute& attribute : info.attributes) {
      writer.u8(static_cast<uint8_t>(attribute.attrib));
      writer.u8(attribute.num_floats);
    }

    for (uint16_t size : info.workgroup_size)
      writer.u16(size);

    writer.u32(static_cast<uint32_t>(info.code.size()));
    writer.bytes(info.code.data(), info.code.size());
    return writer.take();
  }

  bool parseShader(const uint8_t* data, uint32_t size, ShaderInfo& info) {
    Reader reader(data, size);
    char magic[4] {};
    uint8_t version = 0, stage = 0, code_type = 0, reserved = 0;
    if (!reader.bytes(magic, sizeof(magic)) || std::memcmp(magic, kMagic, sizeof(kMagic)) ||
        !reader.u8(version) || version != kVersion || !reader.u8(stage) || !reader.u8(code_type) ||
        !reader.u8(reserved)) {
      return false;
    }
    if (stage > static_cast<uint8_t>(ShaderStage::Compute) || code_type > static_cast<uint8_t>(ShaderCode::Essl))
      return false;

    info = {};
    info.stage = static_cast<ShaderStage>(stage);
    info.code_type = static_cast<ShaderCode>(code_type);

    uint16_t num_uniforms = 0;
    if (!reader.u16(num_uniforms))
      return false;
    info.uniforms.resize(num_uniforms);
    for (ShaderUniform& uniform : info.uniforms) {
      uint8_t type = 0;
      if (!reader.string(uniform.name) || !reader.u8(type) || !reader.u16(uniform.num) ||
          !reader.u16(uniform.offset) || type > static_cast<uint8_t>(UniformType::Mat4))
        return false;
      uniform.type = static_cast<UniformType>(type);
    }
    if (!reader.u32(info.uniform_block_size))
      return false;
    for (const ShaderUniform& uniform : info.uniforms) {
      uint32_t element_bytes = uniform.type == UniformType::Mat4 ? 64 : (uniform.type == UniformType::Mat3 ? 48 : 16);
      if (uniform.num == 0 || uniform.offset + element_bytes * uniform.num > info.uniform_block_size)
        return false;
    }

    uint8_t num_samplers = 0;
    if (!reader.u8(num_samplers))
      return false;
    info.samplers.resize(num_samplers);
    for (ShaderSampler& sampler : info.samplers) {
      if (!reader.string(sampler.name) || !reader.u8(sampler.stage) || sampler.stage >= kMaxTextureStages)
        return false;
    }

    uint8_t num_storage = 0;
    if (!reader.u8(num_storage))
      return false;
    info.storage.resize(num_storage);
    for (ShaderStorage& storage : info.storage) {
      uint8_t image = 0, access = 0;
      if (!reader.string(storage.name) || !reader.u8(storage.stage) || !reader.u8(image) ||
          !reader.u8(access) || storage.stage >= kMaxComputeBindings ||
          access > static_cast<uint8_t>(Access::ReadWrite))
        return false;
      storage.image = image;
      storage.access = static_cast<Access>(access);
    }

    uint8_t num_attributes = 0;
    if (!reader.u8(num_attributes))
      return false;
    info.attributes.resize(num_attributes);
    for (ShaderAttribute& attribute : info.attributes) {
      uint8_t attrib = 0;
      if (!reader.u8(attrib) || !reader.u8(attribute.num_floats) ||
          attrib >= static_cast<uint8_t>(Attrib::Count))
        return false;
      attribute.attrib = static_cast<Attrib>(attrib);
    }

    for (uint16_t& workgroup : info.workgroup_size) {
      if (!reader.u16(workgroup))
        return false;
    }

    uint32_t code_size = 0;
    if (!reader.u32(code_size) || code_size > reader.remaining())
      return false;
    info.code.resize(code_size);
    return reader.bytes(info.code.data(), code_size);
  }
}
