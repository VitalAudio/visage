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

#include <cstdint>
#include <functional>

namespace visage::gpu {
  static constexpr uint16_t kInvalidHandle = UINT16_MAX;

  struct TextureHandle {
    uint16_t idx = kInvalidHandle;
  };
  struct FrameBufferHandle {
    uint16_t idx = kInvalidHandle;
  };
  struct BufferHandle {
    uint16_t idx = kInvalidHandle;
  };
  struct ShaderHandle {
    uint16_t idx = kInvalidHandle;
  };
  struct ProgramHandle {
    uint16_t idx = kInvalidHandle;
  };
  struct UniformHandle {
    uint16_t idx = kInvalidHandle;
  };

  template<typename T>
  inline bool isValid(T handle) {
    return handle.idx != kInvalidHandle;
  }

  enum class RendererType : uint8_t {
    Noop,
    Vulkan,
    Metal,
    Direct3D11,
    OpenGLES,
  };

  enum class TextureFormat : uint8_t {
    RGBA8,
    BGRA8,
    R8,
    R32F,
    RGBA16F,
    RGBA32F,
    Count
  };

  enum class Attrib : uint8_t {
    Position,
    Color0,
    Color1,
    Color2,
    Color3,
    TexCoord0,
    TexCoord1,
    TexCoord2,
    TexCoord3,
    TexCoord4,
    TexCoord5,
    TexCoord6,
    TexCoord7,
    Count
  };

  enum class UniformType : uint8_t {
    Sampler,
    Vec4,
    Mat3,
    Mat4,
  };

  enum class Access : uint8_t {
    Read,
    Write,
    ReadWrite,
  };

  enum class BlendFactor : uint8_t {
    Zero,
    One,
    SrcColor,
    InvSrcColor,
    SrcAlpha,
    InvSrcAlpha,
    DstColor,
    InvDstColor,
    DstAlpha,
    InvDstAlpha,
  };

  enum class BlendOp : uint8_t {
    Add,
    Subtract,
    ReverseSubtract,
    Min,
    Max,
  };

  static constexpr uint8_t kWriteR = 1 << 0;
  static constexpr uint8_t kWriteG = 1 << 1;
  static constexpr uint8_t kWriteB = 1 << 2;
  static constexpr uint8_t kWriteA = 1 << 3;
  static constexpr uint8_t kWriteRGB = kWriteR | kWriteG | kWriteB;
  static constexpr uint8_t kWriteRGBA = kWriteRGB | kWriteA;

  struct RenderState {
    uint8_t write_mask = kWriteRGBA;
    bool blend = false;
    BlendFactor src_rgb = BlendFactor::One;
    BlendFactor dst_rgb = BlendFactor::Zero;
    BlendFactor src_alpha = BlendFactor::One;
    BlendFactor dst_alpha = BlendFactor::Zero;
    BlendOp op_rgb = BlendOp::Add;
    BlendOp op_alpha = BlendOp::Add;

    static constexpr RenderState write(uint8_t mask) {
      RenderState state;
      state.write_mask = mask;
      return state;
    }

    constexpr RenderState blendFunc(BlendFactor src, BlendFactor dst) const {
      return blendFuncSeparate(src, dst, src, dst);
    }

    constexpr RenderState blendFuncSeparate(BlendFactor rgb_src, BlendFactor rgb_dst,
                                            BlendFactor a_src, BlendFactor a_dst) const {
      RenderState state = *this;
      state.blend = true;
      state.src_rgb = rgb_src;
      state.dst_rgb = rgb_dst;
      state.src_alpha = a_src;
      state.dst_alpha = a_dst;
      return state;
    }

    constexpr RenderState blendEquation(BlendOp op) const { return blendEquationSeparate(op, op); }

    constexpr RenderState blendEquationSeparate(BlendOp rgb, BlendOp alpha) const {
      RenderState state = *this;
      state.blend = true;
      state.op_rgb = rgb;
      state.op_alpha = alpha;
      return state;
    }

    constexpr uint32_t key() const {
      return write_mask | (blend ? 1u << 4 : 0u) | (uint32_t(src_rgb) << 5) | (uint32_t(dst_rgb) << 9) |
             (uint32_t(src_alpha) << 13) | (uint32_t(dst_alpha) << 17) | (uint32_t(op_rgb) << 21) |
             (uint32_t(op_alpha) << 24);
    }
  };

  static constexpr uint32_t kSamplerUClamp = 1 << 0;
  static constexpr uint32_t kSamplerVClamp = 1 << 1;
  static constexpr uint32_t kSamplerUMirror = 1 << 2;
  static constexpr uint32_t kSamplerVMirror = 1 << 3;
  static constexpr uint32_t kSamplerMinPoint = 1 << 4;
  static constexpr uint32_t kSamplerMagPoint = 1 << 5;
  static constexpr uint32_t kSamplerClamp = kSamplerUClamp | kSamplerVClamp;
  static constexpr uint32_t kSamplerMirror = kSamplerUMirror | kSamplerVMirror;
  static constexpr uint32_t kSamplerPoint = kSamplerMinPoint | kSamplerMagPoint;
  static constexpr uint32_t kSamplerMask = 0xff;
  static constexpr uint32_t kSamplerUseTexture = UINT32_MAX;

  static constexpr uint32_t kTextureRenderTarget = 1 << 8;
  static constexpr uint32_t kTextureStorage = 1 << 9;
  static constexpr uint32_t kTextureReadBack = 1 << 10;
  static constexpr uint32_t kTextureBlitDst = 1 << 11;

  static constexpr uint16_t kBufferIndex32 = 1 << 0;
  static constexpr uint16_t kBufferComputeRead = 1 << 1;
  static constexpr uint16_t kBufferComputeWrite = 1 << 2;
  static constexpr uint16_t kBufferComputeReadWrite = kBufferComputeRead | kBufferComputeWrite;

  static constexpr uint32_t kFormatTexture = 1 << 0;
  static constexpr uint32_t kFormatRenderTarget = 1 << 1;
  static constexpr uint32_t kFormatStorage = 1 << 2;
  static constexpr uint32_t kFormatBlend = 1 << 3;

  static constexpr int kMaxViews = 4096;
  static constexpr int kMaxTextureStages = 8;
  static constexpr int kMaxComputeBindings = 8;

  struct Caps {
    RendererType renderer = RendererType::Noop;
    bool origin_bottom_left = false;
    bool compute = false;
    bool swap_chain = false;
    int max_texture_size = 0;
    uint32_t formats[static_cast<int>(TextureFormat::Count)] {};
  };

  struct Stats {
    int num_textures = 0;
    int num_frame_buffers = 0;
    int num_buffers = 0;
    int num_shaders = 0;
    int num_programs = 0;
    int num_draws = 0;
    int num_computes = 0;
    int num_views = 0;
    int width = 0;
    int height = 0;
    double cpu_frame_ms = 0.0;
    double gpu_frame_ms = 0.0;
  };

  struct Memory {
    uint8_t* data = nullptr;
    uint32_t size = 0;
  };

  const Memory* alloc(uint32_t size);
  const Memory* copy(const void* data, uint32_t size);
  const Memory* makeRef(const void* data, uint32_t size);

  class VertexLayout {
  public:
    VertexLayout& begin();
    VertexLayout& add(Attrib attrib, int num_floats);
    VertexLayout& end();

    uint16_t stride() const { return stride_; }
    uint16_t offset(Attrib attrib) const { return offsets_[static_cast<int>(attrib)]; }
    uint8_t count(Attrib attrib) const { return counts_[static_cast<int>(attrib)]; }
    bool has(Attrib attrib) const { return counts_[static_cast<int>(attrib)] > 0; }
    uint32_t hash() const { return hash_; }

  private:
    uint16_t stride_ = 0;
    uint16_t offsets_[static_cast<int>(Attrib::Count)] {};
    uint8_t counts_[static_cast<int>(Attrib::Count)] {};
    uint32_t hash_ = 0;
  };

  using ScreenshotCallback = std::function<void(FrameBufferHandle frame_buffer, const uint8_t* data,
                                                int width, int height, int pitch, bool bgra)>;
  using LogCallback = std::function<void(const char* message)>;

  struct Init {
    RendererType type = RendererType::Noop;
    void* display = nullptr;
    bool vsync = true;
    bool color_managed = true;
    bool debug = false;
    ScreenshotCallback screenshot_callback;
    LogCallback log_callback;
  };

  RendererType defaultRendererType();
  bool rendererSupported(RendererType type);
  const char* rendererName(RendererType type);

  bool init(const Init& init);
  void shutdown();
  bool initialized();
  const Caps* getCaps();
  const Stats* getStats();

  TextureHandle createTexture2D(int width, int height, TextureFormat format, uint32_t flags = 0,
                                const Memory* memory = nullptr);
  void updateTexture2D(TextureHandle handle, int x, int y, int width, int height,
                       const Memory* memory, int pitch = 0);
  uint32_t readTexture(TextureHandle handle, void* data);
  void destroy(TextureHandle handle);

  FrameBufferHandle createFrameBuffer(int width, int height, TextureFormat format, uint32_t flags = 0);
  FrameBufferHandle createFrameBuffer(void* window_handle, int width, int height,
                                      TextureFormat format = TextureFormat::RGBA8);
  void resizeFrameBuffer(FrameBufferHandle handle, int width, int height);
  TextureHandle getTexture(FrameBufferHandle handle);
  bool waitForFrame(FrameBufferHandle handle, int timeout_ms);
  void requestScreenShot(FrameBufferHandle handle);
  void destroy(FrameBufferHandle handle);

  BufferHandle createVertexBuffer(const Memory* memory, const VertexLayout& layout, uint16_t flags = 0);
  BufferHandle createIndexBuffer(const Memory* memory, uint16_t flags = 0);
  BufferHandle createDynamicVertexBuffer(uint32_t num_vertices, const VertexLayout& layout,
                                         uint16_t flags = 0);
  BufferHandle createDynamicIndexBuffer(uint32_t num_indices, uint16_t flags = 0);
  void update(BufferHandle handle, uint32_t start, const Memory* memory);
  void destroy(BufferHandle handle);

  ShaderHandle createShader(const Memory* memory);
  void destroy(ShaderHandle handle);
  ProgramHandle createProgram(ShaderHandle vertex, ShaderHandle fragment, bool destroy_shaders = false);
  ProgramHandle createProgram(ShaderHandle compute, bool destroy_shader = false);
  void destroy(ProgramHandle handle);

  UniformHandle createUniform(const char* name, UniformType type, int num = 1);
  void destroy(UniformHandle handle);

  void setViewRect(uint16_t view, int x, int y, int width, int height);
  void setViewFrameBuffer(uint16_t view, FrameBufferHandle handle);

  void setState(const RenderState& state);
  void setUniform(UniformHandle handle, const void* value, int num = 1);
  void setTexture(int stage, UniformHandle sampler, TextureHandle handle,
                  uint32_t flags = kSamplerUseTexture);
  void setVertexBuffer(int stream, BufferHandle handle, uint32_t start = 0, uint32_t count = UINT32_MAX);
  void setIndexBuffer(BufferHandle handle, uint32_t start = 0, uint32_t count = UINT32_MAX);
  void setVertexCount(uint32_t count);
  void submit(uint16_t view, ProgramHandle program, bool preserve_state = false);
  void discard();

  void setBuffer(int stage, BufferHandle handle, Access access);
  void setImage(int stage, TextureHandle handle, Access access);
  void dispatch(uint16_t view, ProgramHandle program, uint32_t groups_x, uint32_t groups_y = 1,
                uint32_t groups_z = 1);

  void blit(uint16_t view, TextureHandle destination, int dest_x, int dest_y, TextureHandle source,
            int source_x = 0, int source_y = 0, int width = -1, int height = -1);

  uint32_t frame();
  uint32_t frameNumber();
}
