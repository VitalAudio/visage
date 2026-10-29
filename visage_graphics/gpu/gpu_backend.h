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

#include "gpu.h"

#include <memory>
#include <string>
#include <vector>

namespace visage::gpu {
  enum class ShaderStage : uint8_t {
    Vertex,
    Fragment,
    Compute,
  };

  enum class ShaderCode : uint8_t {
    Spirv,
    Msl,
    Hlsl,
    Dxbc,
    Essl,
  };

  struct ShaderUniform {
    std::string name;
    UniformType type = UniformType::Vec4;
    uint16_t num = 1;
    uint16_t offset = 0;
  };

  struct ShaderSampler {
    std::string name;
    uint8_t stage = 0;
  };

  struct ShaderStorage {
    std::string name;
    uint8_t stage = 0;
    bool image = false;
    Access access = Access::Read;
  };

  struct ShaderAttribute {
    Attrib attrib = Attrib::Position;
    uint8_t num_floats = 4;
  };

  struct ShaderInfo {
    ShaderStage stage = ShaderStage::Vertex;
    ShaderCode code_type = ShaderCode::Spirv;
    std::vector<ShaderUniform> uniforms;
    uint32_t uniform_block_size = 0;
    std::vector<ShaderSampler> samplers;
    std::vector<ShaderStorage> storage;
    std::vector<ShaderAttribute> attributes;
    uint16_t workgroup_size[3] = { 1, 1, 1 };
    std::vector<uint8_t> code;
  };

  bool parseShader(const uint8_t* data, uint32_t size, ShaderInfo& info);
  std::vector<uint8_t> writeShader(const ShaderInfo& info);

  struct TextureDesc {
    int width = 0;
    int height = 0;
    TextureFormat format = TextureFormat::RGBA8;
    uint32_t flags = 0;
  };

  struct BufferDesc {
    uint32_t size = 0;
    uint16_t flags = 0;
    bool index = false;
    bool dynamic = false;
    VertexLayout layout;
  };

  struct TextureBinding {
    uint16_t texture = kInvalidHandle;
    uint32_t sampler_flags = 0;
  };

  struct DrawItem {
    RenderState state;
    uint16_t program = kInvalidHandle;
    uint16_t vertex_buffer = kInvalidHandle;
    uint32_t vertex_start = 0;
    uint32_t vertex_count = 0;
    uint16_t index_buffer = kInvalidHandle;
    uint32_t index_start = 0;
    uint32_t index_count = 0;
    TextureBinding textures[kMaxTextureStages];
    uint32_t vertex_uniform_offset = 0;
    uint32_t vertex_uniform_size = 0;
    uint32_t fragment_uniform_offset = 0;
    uint32_t fragment_uniform_size = 0;
  };

  struct ComputeBinding {
    uint16_t handle = kInvalidHandle;
    bool image = false;
    Access access = Access::Read;
  };

  struct ComputeItem {
    uint16_t program = kInvalidHandle;
    ComputeBinding bindings[kMaxComputeBindings];
    uint32_t groups[3] = { 1, 1, 1 };
    uint32_t uniform_offset = 0;
    uint32_t uniform_size = 0;
  };

  struct BlitItem {
    uint16_t destination = kInvalidHandle;
    int dest_x = 0;
    int dest_y = 0;
    uint16_t source = kInvalidHandle;
    int source_x = 0;
    int source_y = 0;
    int width = 0;
    int height = 0;
  };

  struct ViewItem {
    enum Type : uint8_t {
      Draw,
      Compute,
      Blit,
    };
    Type type = Draw;
    uint32_t index = 0;
  };

  struct ViewData {
    uint16_t id = 0;
    uint16_t frame_buffer = kInvalidHandle;
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    std::vector<ViewItem> items;
  };

  struct ReadRequest {
    uint16_t texture = kInvalidHandle;
    void* data = nullptr;
  };

  struct FrameData {
    uint32_t frame_number = 0;
    std::vector<ViewData> views;
    std::vector<DrawItem> draws;
    std::vector<ComputeItem> computes;
    std::vector<BlitItem> blits;
    std::vector<uint8_t> uniform_data;
    std::vector<ReadRequest> reads;
    std::vector<uint16_t> screenshots;
  };

  class Backend {
  public:
    virtual ~Backend() = default;

    virtual bool init(const Init& init, Caps& caps) = 0;
    virtual void shutdown() = 0;

    virtual bool createTexture(uint16_t idx, const TextureDesc& desc, const uint8_t* data, int pitch) = 0;
    virtual void updateTexture(uint16_t idx, int x, int y, int width, int height, const uint8_t* data,
                               int pitch) = 0;
    virtual void destroyTexture(uint16_t idx) = 0;

    virtual bool createBuffer(uint16_t idx, const BufferDesc& desc, const uint8_t* data) = 0;
    virtual void updateBuffer(uint16_t idx, uint32_t offset, const uint8_t* data, uint32_t size) = 0;
    virtual void destroyBuffer(uint16_t idx) = 0;

    virtual bool createShader(uint16_t idx, const ShaderInfo& info) = 0;
    virtual void destroyShader(uint16_t idx) = 0;
    virtual bool createProgram(uint16_t idx, uint16_t vertex, uint16_t fragment) = 0;
    virtual bool createComputeProgram(uint16_t idx, uint16_t compute) = 0;
    virtual void destroyProgram(uint16_t idx) = 0;

    virtual bool createFrameBuffer(uint16_t idx, uint16_t texture) = 0;
    virtual bool createSwapChain(uint16_t idx, void* window_handle, void* display, int width,
                                 int height, TextureFormat format) = 0;
    virtual void resizeSwapChain(uint16_t idx, int width, int height) = 0;
    virtual void destroyFrameBuffer(uint16_t idx) = 0;
    virtual bool waitForFrame(uint16_t idx, int timeout_ms) = 0;

    virtual void beginFrame(uint32_t frame_number) = 0;
    virtual void submitFrame(const FrameData& frame, Stats& stats) = 0;
  };

  std::unique_ptr<Backend> createVulkanBackend();
  std::unique_ptr<Backend> createMetalBackend();
  std::unique_ptr<Backend> createD3D11Backend();
  std::unique_ptr<Backend> createGlesBackend();
  std::unique_ptr<Backend> createNoopBackend();
}
