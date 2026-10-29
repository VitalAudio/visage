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

#include <algorithm>
#include <chrono>
#include <cstring>
#include <map>
#include <unordered_map>

namespace visage::gpu {
  namespace {
    struct MemoryBlock : Memory {
      bool owned = false;
    };

    void release(const Memory* memory) {
      auto block = static_cast<const MemoryBlock*>(memory);
      if (block == nullptr)
        return;
      if (block->owned)
        delete[] block->data;
      delete block;
    }

    template<typename T>
    class HandlePool {
    public:
      uint16_t allocate() {
        uint16_t idx = 0;
        if (!free_.empty()) {
          idx = free_.back();
          free_.pop_back();
        }
        else {
          if (entries_.size() >= kInvalidHandle)
            return kInvalidHandle;
          idx = static_cast<uint16_t>(entries_.size());
          entries_.emplace_back();
        }
        entries_[idx] = T();
        entries_[idx].used = true;
        num_used_++;
        return idx;
      }

      void free(uint16_t idx) {
        if (!valid(idx))
          return;
        entries_[idx] = T();
        free_.push_back(idx);
        num_used_--;
      }

      bool valid(uint16_t idx) const { return idx < entries_.size() && entries_[idx].used; }
      T& operator[](uint16_t idx) { return entries_[idx]; }
      int numUsed() const { return num_used_; }

      template<typename F>
      void forEach(F function) {
        for (uint16_t i = 0; i < entries_.size(); ++i) {
          if (entries_[i].used)
            function(i);
        }
      }

    private:
      std::vector<T> entries_;
      std::vector<uint16_t> free_;
      int num_used_ = 0;
    };

    struct TextureEntry {
      bool used = false;
      bool owned_by_frame_buffer = false;
      TextureDesc desc;
    };

    struct FrameBufferEntry {
      bool used = false;
      bool swap_chain = false;
      uint16_t texture = kInvalidHandle;
      int width = 0;
      int height = 0;
    };

    struct BufferEntry {
      bool used = false;
      BufferDesc desc;
      uint32_t stride = 0;
    };

    struct ShaderEntry {
      bool used = false;
      ShaderInfo info;
      std::vector<uint16_t> uniform_slots;
      int programs = 0;
      bool destroy_requested = false;
    };

    struct ProgramEntry {
      bool used = false;
      bool compute = false;
      uint16_t vertex = kInvalidHandle;
      uint16_t fragment = kInvalidHandle;
      bool destroy_shaders = false;
    };

    struct UniformSlot {
      std::string name;
      uint32_t element_floats = 4;
      std::vector<float> values;
    };

    struct ViewSettings {
      uint16_t frame_buffer = kInvalidHandle;
      int x = 0;
      int y = 0;
      int width = 0;
      int height = 0;
    };

    struct PendingUpdate {
      bool texture = false;
      uint16_t idx = kInvalidHandle;
      int x = 0;
      int y = 0;
      int width = 0;
      int height = 0;
      int pitch = 0;
      uint32_t offset = 0;
      const Memory* memory = nullptr;
    };

    struct PendingDestroy {
      enum Type {
        Texture,
        FrameBuffer,
        Buffer,
        Shader,
        Program,
      };
      Type type = Texture;
      uint16_t idx = kInvalidHandle;
    };

    class NoopBackend : public Backend {
    public:
      bool init(const Init&, Caps& caps) override {
        caps.renderer = RendererType::Noop;
        caps.max_texture_size = 16384;
        caps.swap_chain = true;
        for (uint32_t& format : caps.formats)
          format = kFormatTexture | kFormatRenderTarget | kFormatBlend;
        return true;
      }
      void shutdown() override { }
      bool createTexture(uint16_t, const TextureDesc&, const uint8_t*, int) override { return true; }
      void updateTexture(uint16_t, int, int, int, int, const uint8_t*, int) override { }
      void destroyTexture(uint16_t) override { }
      bool createBuffer(uint16_t, const BufferDesc&, const uint8_t*) override { return true; }
      void updateBuffer(uint16_t, uint32_t, const uint8_t*, uint32_t) override { }
      void destroyBuffer(uint16_t) override { }
      bool createShader(uint16_t, const ShaderInfo&) override { return true; }
      void destroyShader(uint16_t) override { }
      bool createProgram(uint16_t, uint16_t, uint16_t) override { return true; }
      bool createComputeProgram(uint16_t, uint16_t) override { return true; }
      void destroyProgram(uint16_t) override { }
      bool createFrameBuffer(uint16_t, uint16_t) override { return true; }
      bool createSwapChain(uint16_t, void*, void*, int, int, TextureFormat) override { return true; }
      void resizeSwapChain(uint16_t, int, int) override { }
      void destroyFrameBuffer(uint16_t) override { }
      bool waitForFrame(uint16_t, int) override { return true; }
      void beginFrame(uint32_t) override { }
      void submitFrame(const FrameData&, Stats&) override { }
    };

    struct Context {
      Init init;
      Caps caps;
      Stats stats;
      std::unique_ptr<Backend> backend;
      uint32_t frame_number = 0;

      HandlePool<TextureEntry> textures;
      HandlePool<FrameBufferEntry> frame_buffers;
      HandlePool<BufferEntry> buffers;
      HandlePool<ShaderEntry> shaders;
      HandlePool<ProgramEntry> programs;

      std::vector<UniformSlot> uniforms;
      std::unordered_map<std::string, uint16_t> uniform_lookup;

      std::map<uint16_t, ViewSettings> view_settings;
      std::map<uint16_t, std::vector<ViewItem>> view_items;

      DrawItem draw;
      bool draw_vertex_count = false;
      ComputeBinding compute_bindings[kMaxComputeBindings];

      FrameData frame;
      bool submitting = false;
      std::vector<ReadRequest> pending_reads;
      std::vector<uint16_t> pending_screenshots;
      std::vector<PendingUpdate> updates;
      std::vector<PendingDestroy> destroys;
    };

    Context* context = nullptr;

    uint32_t formatBytes(TextureFormat format) {
      switch (format) {
      case TextureFormat::R8: return 1;
      case TextureFormat::RGBA16F: return 8;
      case TextureFormat::RGBA32F: return 16;
      default: return 4;
      }
    }

    uint32_t uniformTypeFloats(UniformType type) {
      switch (type) {
      case UniformType::Mat3: return 9;
      case UniformType::Mat4: return 16;
      default: return 4;
      }
    }

    uint16_t uniformSlot(const std::string& name) {
      auto found = context->uniform_lookup.find(name);
      if (found != context->uniform_lookup.end())
        return found->second;

      uint16_t slot = static_cast<uint16_t>(context->uniforms.size());
      context->uniforms.push_back({ name, 4, {} });
      context->uniform_lookup[name] = slot;
      return slot;
    }

    uint32_t packUniforms(const ShaderEntry& shader, uint32_t& size) {
      size = shader.info.uniform_block_size;
      if (size == 0)
        return 0;

      std::vector<uint8_t>& data = context->frame.uniform_data;
      uint32_t offset = static_cast<uint32_t>(data.size());
      data.resize(offset + size, 0);
      uint8_t* block = data.data() + offset;

      for (size_t i = 0; i < shader.info.uniforms.size(); ++i) {
        const ShaderUniform& uniform = shader.info.uniforms[i];
        const std::vector<float>& values = context->uniforms[shader.uniform_slots[i]].values;
        uint8_t* destination = block + uniform.offset;
        if (uniform.type == UniformType::Mat3) {
          for (int n = 0; n < uniform.num; ++n) {
            for (int column = 0; column < 3; ++column) {
              size_t source = n * 9 + column * 3;
              if (source + 3 <= values.size())
                std::memcpy(destination + (n * 3 + column) * 16, values.data() + source, 3 * sizeof(float));
            }
          }
        }
        else {
          uint32_t floats = uniformTypeFloats(uniform.type) * uniform.num;
          uint32_t available = static_cast<uint32_t>(std::min<size_t>(values.size(), floats));
          uint32_t limit = (size - uniform.offset) / sizeof(float);
          std::memcpy(destination, values.data(), std::min(available, limit) * sizeof(float));
        }
      }
      return offset;
    }

    bool computeBindingsMatch(const ShaderEntry& shader) {
      for (const ShaderStorage& storage : shader.info.storage) {
        const ComputeBinding& binding = context->compute_bindings[storage.stage];
        if (binding.image != storage.image)
          return false;
        if (storage.image) {
          if (!context->textures.valid(binding.handle) ||
              !(context->textures[binding.handle].desc.flags & kTextureStorage))
            return false;
        }
        else if (!context->buffers.valid(binding.handle) ||
                 !(context->buffers[binding.handle].desc.flags & kBufferComputeReadWrite))
          return false;
      }
      return true;
    }

    void resetDraw() {
      context->draw = DrawItem();
      context->draw_vertex_count = false;
    }

    void resetCompute() {
      for (ComputeBinding& binding : context->compute_bindings)
        binding = ComputeBinding();
    }

    void destroyNow(const PendingDestroy& destroy) {
      Backend* backend = context->backend.get();
      switch (destroy.type) {
      case PendingDestroy::Texture:
        backend->destroyTexture(destroy.idx);
        context->textures.free(destroy.idx);
        break;
      case PendingDestroy::FrameBuffer: {
        FrameBufferEntry& entry = context->frame_buffers[destroy.idx];
        uint16_t texture = entry.texture;
        backend->destroyFrameBuffer(destroy.idx);
        context->frame_buffers.free(destroy.idx);
        if (context->textures.valid(texture)) {
          backend->destroyTexture(texture);
          context->textures.free(texture);
        }
        break;
      }
      case PendingDestroy::Buffer:
        backend->destroyBuffer(destroy.idx);
        context->buffers.free(destroy.idx);
        break;
      case PendingDestroy::Shader:
        backend->destroyShader(destroy.idx);
        context->shaders.free(destroy.idx);
        break;
      case PendingDestroy::Program: {
        ProgramEntry entry = context->programs[destroy.idx];
        backend->destroyProgram(destroy.idx);
        context->programs.free(destroy.idx);
        for (uint16_t shader : { entry.vertex, entry.fragment }) {
          if (!context->shaders.valid(shader))
            continue;
          ShaderEntry& shader_entry = context->shaders[shader];
          shader_entry.programs--;
          if (shader_entry.programs <= 0 && (entry.destroy_shaders || shader_entry.destroy_requested))
            destroyNow({ PendingDestroy::Shader, shader });
        }
        break;
      }
      }
    }

    uint32_t bufferElements(const BufferEntry& entry) {
      return entry.stride ? entry.desc.size / entry.stride : 0;
    }

    BufferHandle createBuffer(const BufferDesc& desc, uint32_t stride, const Memory* memory) {
      if (context == nullptr || desc.size == 0) {
        release(memory);
        return {};
      }

      uint16_t idx = context->buffers.allocate();
      if (idx == kInvalidHandle) {
        release(memory);
        return {};
      }

      BufferEntry& entry = context->buffers[idx];
      entry.desc = desc;
      entry.stride = stride;
      bool created = context->backend->createBuffer(idx, desc, memory ? memory->data : nullptr);
      release(memory);
      if (!created) {
        context->buffers.free(idx);
        return {};
      }
      return { idx };
    }
  }

  const Memory* alloc(uint32_t size) {
    auto block = new MemoryBlock();
    block->data = new uint8_t[std::max<uint32_t>(1, size)];
    block->size = size;
    block->owned = true;
    return block;
  }

  const Memory* copy(const void* data, uint32_t size) {
    const Memory* memory = alloc(size);
    if (size)
      std::memcpy(memory->data, data, size);
    return memory;
  }

  const Memory* makeRef(const void* data, uint32_t size) {
    auto block = new MemoryBlock();
    block->data = static_cast<uint8_t*>(const_cast<void*>(data));
    block->size = size;
    return block;
  }

  VertexLayout& VertexLayout::begin() {
    *this = VertexLayout();
    return *this;
  }

  VertexLayout& VertexLayout::add(Attrib attrib, int num_floats) {
    int index = static_cast<int>(attrib);
    offsets_[index] = stride_;
    counts_[index] = static_cast<uint8_t>(num_floats);
    stride_ += static_cast<uint16_t>(num_floats * sizeof(float));
    return *this;
  }

  VertexLayout& VertexLayout::end() {
    uint32_t hash = 2166136261u;
    auto mix = [&hash](uint32_t value) {
      hash ^= value;
      hash *= 16777619u;
    };
    for (int i = 0; i < static_cast<int>(Attrib::Count); ++i) {
      mix(offsets_[i]);
      mix(counts_[i]);
    }
    mix(stride_);
    hash_ = hash;
    return *this;
  }

  RendererType defaultRendererType() {
#if VISAGE_WINDOWS
    return RendererType::Direct3D11;
#elif VISAGE_MAC
    return RendererType::Metal;
#elif VISAGE_LINUX
    return RendererType::Vulkan;
#elif VISAGE_EMSCRIPTEN
    return RendererType::OpenGLES;
#else
    return RendererType::Noop;
#endif
  }

  static std::unique_ptr<Backend> createBackend(RendererType type) {
    switch (type) {
#if VISAGE_LINUX
    case RendererType::Vulkan: return createVulkanBackend();
#endif
#if VISAGE_MAC
    case RendererType::Metal: return createMetalBackend();
#endif
#if VISAGE_WINDOWS
    case RendererType::Direct3D11: return createD3D11Backend();
#endif
#if VISAGE_EMSCRIPTEN
    case RendererType::OpenGLES: return createGlesBackend();
#endif
    case RendererType::Noop: return createNoopBackend();
    default: return nullptr;
    }
  }

  std::unique_ptr<Backend> createNoopBackend() {
    return std::make_unique<NoopBackend>();
  }

  bool rendererSupported(RendererType type) {
    return type == RendererType::Noop || type == defaultRendererType();
  }

  const char* rendererName(RendererType type) {
    switch (type) {
    case RendererType::Vulkan: return "Vulkan";
    case RendererType::Metal: return "Metal";
    case RendererType::Direct3D11: return "Direct3D 11";
    case RendererType::OpenGLES: return "OpenGL ES";
    default: return "Noop";
    }
  }

  bool init(const Init& init) {
    if (context)
      return true;

    std::unique_ptr<Backend> backend = createBackend(init.type);
    if (backend == nullptr)
      return false;

    auto new_context = std::make_unique<Context>();
    new_context->init = init;
    if (!backend->init(init, new_context->caps)) {
      backend->shutdown();
      return false;
    }

    new_context->backend = std::move(backend);
    context = new_context.release();
    return true;
  }

  bool initialized() {
    return context != nullptr;
  }

  void shutdown() {
    if (context == nullptr)
      return;

    for (const PendingUpdate& update : context->updates)
      release(update.memory);
    context->updates.clear();

    for (const PendingDestroy& destroy : context->destroys) {
      if (destroy.type == PendingDestroy::Program && context->programs.valid(destroy.idx))
        destroyNow(destroy);
    }
    std::vector<PendingDestroy> remaining;
    context->programs.forEach([&](uint16_t idx) { remaining.push_back({ PendingDestroy::Program, idx }); });
    for (const PendingDestroy& destroy : remaining)
      destroyNow(destroy);
    remaining.clear();

    context->shaders.forEach([&](uint16_t idx) { remaining.push_back({ PendingDestroy::Shader, idx }); });
    context->frame_buffers.forEach([&](uint16_t idx) {
      remaining.push_back({ PendingDestroy::FrameBuffer, idx });
    });
    for (const PendingDestroy& destroy : remaining)
      destroyNow(destroy);
    remaining.clear();

    context->textures.forEach([&](uint16_t idx) { remaining.push_back({ PendingDestroy::Texture, idx }); });
    context->buffers.forEach([&](uint16_t idx) { remaining.push_back({ PendingDestroy::Buffer, idx }); });
    for (const PendingDestroy& destroy : remaining)
      destroyNow(destroy);

    context->backend->shutdown();
    delete context;
    context = nullptr;
  }

  const Caps* getCaps() {
    static const Caps kEmptyCaps;
    return context ? &context->caps : &kEmptyCaps;
  }

  const Stats* getStats() {
    static const Stats kEmptyStats;
    if (context == nullptr)
      return &kEmptyStats;

    Stats& stats = context->stats;
    stats.num_textures = context->textures.numUsed();
    stats.num_frame_buffers = context->frame_buffers.numUsed();
    stats.num_buffers = context->buffers.numUsed();
    stats.num_shaders = context->shaders.numUsed();
    stats.num_programs = context->programs.numUsed();
    return &stats;
  }

  TextureHandle createTexture2D(int width, int height, TextureFormat format, uint32_t flags,
                                const Memory* memory) {
    if (context == nullptr || width <= 0 || height <= 0 || width > context->caps.max_texture_size ||
        height > context->caps.max_texture_size ||
        (memory && memory->size < static_cast<uint64_t>(width) * height * formatBytes(format))) {
      release(memory);
      return {};
    }

    uint16_t idx = context->textures.allocate();
    if (idx == kInvalidHandle) {
      release(memory);
      return {};
    }

    TextureEntry& entry = context->textures[idx];
    entry.desc = { width, height, format, flags };
    bool created = context->backend->createTexture(idx, entry.desc, memory ? memory->data : nullptr, 0);
    release(memory);
    if (!created) {
      context->textures.free(idx);
      return {};
    }
    return { idx };
  }

  void updateTexture2D(TextureHandle handle, int x, int y, int width, int height,
                       const Memory* memory, int pitch) {
    if (context == nullptr || !context->textures.valid(handle.idx) || memory == nullptr ||
        width <= 0 || height <= 0) {
      release(memory);
      return;
    }

    const TextureDesc& desc = context->textures[handle.idx].desc;
    uint64_t row_bytes = static_cast<uint64_t>(width) * formatBytes(desc.format);
    uint64_t source_pitch = pitch ? static_cast<uint64_t>(pitch) : row_bytes;
    if (x < 0 || y < 0 || x + width > desc.width || y + height > desc.height || source_pitch < row_bytes ||
        memory->size < source_pitch * (height - 1) + row_bytes) {
      release(memory);
      return;
    }

    PendingUpdate update;
    update.texture = true;
    update.idx = handle.idx;
    update.x = x;
    update.y = y;
    update.width = width;
    update.height = height;
    update.pitch = pitch;
    update.memory = memory;
    context->updates.push_back(update);
  }

  uint32_t readTexture(TextureHandle handle, void* data) {
    if (context == nullptr || !context->textures.valid(handle.idx))
      return 0;

    context->pending_reads.push_back({ handle.idx, data });
    return context->frame_number + (context->submitting ? 2 : 1);
  }

  void destroy(TextureHandle handle) {
    if (context && context->textures.valid(handle.idx) && !context->textures[handle.idx].owned_by_frame_buffer)
      context->destroys.push_back({ PendingDestroy::Texture, handle.idx });
  }

  FrameBufferHandle createFrameBuffer(int width, int height, TextureFormat format, uint32_t flags) {
    if (context == nullptr)
      return {};

    TextureHandle texture = createTexture2D(width, height, format, flags | kTextureRenderTarget);
    if (!isValid(texture))
      return {};

    uint16_t idx = context->frame_buffers.allocate();
    if (idx == kInvalidHandle || !context->backend->createFrameBuffer(idx, texture.idx)) {
      context->frame_buffers.free(idx);
      context->backend->destroyTexture(texture.idx);
      context->textures.free(texture.idx);
      return {};
    }

    context->textures[texture.idx].owned_by_frame_buffer = true;
    FrameBufferEntry& entry = context->frame_buffers[idx];
    entry.texture = texture.idx;
    entry.width = width;
    entry.height = height;
    return { idx };
  }

  FrameBufferHandle createFrameBuffer(void* window_handle, int width, int height, TextureFormat format) {
    if (context == nullptr || window_handle == nullptr)
      return {};

    uint16_t idx = context->frame_buffers.allocate();
    if (idx == kInvalidHandle)
      return {};

    if (!context->backend->createSwapChain(idx, window_handle, context->init.display, width, height, format)) {
      context->frame_buffers.free(idx);
      return {};
    }

    FrameBufferEntry& entry = context->frame_buffers[idx];
    entry.swap_chain = true;
    entry.width = width;
    entry.height = height;
    return { idx };
  }

  void resizeFrameBuffer(FrameBufferHandle handle, int width, int height) {
    if (context == nullptr || !context->frame_buffers.valid(handle.idx))
      return;

    FrameBufferEntry& entry = context->frame_buffers[handle.idx];
    if (!entry.swap_chain || (entry.width == width && entry.height == height))
      return;

    entry.width = width;
    entry.height = height;
    context->backend->resizeSwapChain(handle.idx, width, height);
  }

  TextureHandle getTexture(FrameBufferHandle handle) {
    if (context == nullptr || !context->frame_buffers.valid(handle.idx))
      return {};
    return { context->frame_buffers[handle.idx].texture };
  }

  bool waitForFrame(FrameBufferHandle handle, int timeout_ms) {
    Context* current = context;
    if (current == nullptr || !isValid(handle))
      return true;
    return current->backend->waitForFrame(handle.idx, timeout_ms);
  }

  void requestScreenShot(FrameBufferHandle handle) {
    if (context && context->frame_buffers.valid(handle.idx))
      context->pending_screenshots.push_back(handle.idx);
  }

  void destroy(FrameBufferHandle handle) {
    if (context && context->frame_buffers.valid(handle.idx))
      context->destroys.push_back({ PendingDestroy::FrameBuffer, handle.idx });
  }

  BufferHandle createVertexBuffer(const Memory* memory, const VertexLayout& layout, uint16_t flags) {
    BufferDesc desc;
    desc.size = memory ? memory->size : 0;
    desc.flags = flags;
    desc.layout = layout;
    return createBuffer(desc, layout.stride(), memory);
  }

  BufferHandle createIndexBuffer(const Memory* memory, uint16_t flags) {
    BufferDesc desc;
    desc.size = memory ? memory->size : 0;
    desc.flags = flags;
    desc.index = true;
    return createBuffer(desc, (flags & kBufferIndex32) ? 4 : 2, memory);
  }

  BufferHandle createDynamicVertexBuffer(uint32_t num_vertices, const VertexLayout& layout, uint16_t flags) {
    BufferDesc desc;
    desc.size = num_vertices * layout.stride();
    desc.flags = flags;
    desc.dynamic = true;
    desc.layout = layout;
    return createBuffer(desc, layout.stride(), nullptr);
  }

  BufferHandle createDynamicIndexBuffer(uint32_t num_indices, uint16_t flags) {
    uint32_t stride = (flags & kBufferIndex32) ? 4 : 2;
    BufferDesc desc;
    desc.size = num_indices * stride;
    desc.flags = flags;
    desc.index = true;
    desc.dynamic = true;
    return createBuffer(desc, stride, nullptr);
  }

  void update(BufferHandle handle, uint32_t start, const Memory* memory) {
    if (context == nullptr || !context->buffers.valid(handle.idx) || memory == nullptr) {
      release(memory);
      return;
    }

    BufferEntry& entry = context->buffers[handle.idx];
    uint32_t offset = start * entry.stride;
    if (offset >= entry.desc.size || memory->size == 0) {
      release(memory);
      return;
    }

    PendingUpdate update;
    update.idx = handle.idx;
    update.offset = offset;
    update.memory = memory;
    context->updates.push_back(update);
  }

  void destroy(BufferHandle handle) {
    if (context && context->buffers.valid(handle.idx))
      context->destroys.push_back({ PendingDestroy::Buffer, handle.idx });
  }

  ShaderHandle createShader(const Memory* memory) {
    if (context == nullptr || memory == nullptr) {
      release(memory);
      return {};
    }

    ShaderInfo info;
    bool parsed = parseShader(memory->data, memory->size, info);
    release(memory);
    if (!parsed)
      return {};

    uint16_t idx = context->shaders.allocate();
    if (idx == kInvalidHandle)
      return {};

    if (!context->backend->createShader(idx, info)) {
      context->shaders.free(idx);
      return {};
    }

    ShaderEntry& entry = context->shaders[idx];
    for (const ShaderUniform& uniform : info.uniforms)
      entry.uniform_slots.push_back(uniformSlot(uniform.name));
    info.code.clear();
    info.code.shrink_to_fit();
    entry.info = std::move(info);
    return { idx };
  }

  void destroy(ShaderHandle handle) {
    if (context == nullptr || !context->shaders.valid(handle.idx))
      return;

    ShaderEntry& entry = context->shaders[handle.idx];
    if (entry.programs > 0)
      entry.destroy_requested = true;
    else
      context->destroys.push_back({ PendingDestroy::Shader, handle.idx });
  }

  ProgramHandle createProgram(ShaderHandle vertex, ShaderHandle fragment, bool destroy_shaders) {
    if (context == nullptr || !context->shaders.valid(vertex.idx) || !context->shaders.valid(fragment.idx))
      return {};
    if (context->shaders[vertex.idx].info.stage != ShaderStage::Vertex ||
        context->shaders[fragment.idx].info.stage != ShaderStage::Fragment)
      return {};

    uint16_t idx = context->programs.allocate();
    if (idx == kInvalidHandle)
      return {};

    if (!context->backend->createProgram(idx, vertex.idx, fragment.idx)) {
      context->programs.free(idx);
      return {};
    }

    ProgramEntry& entry = context->programs[idx];
    entry.vertex = vertex.idx;
    entry.fragment = fragment.idx;
    entry.destroy_shaders = destroy_shaders;
    context->shaders[vertex.idx].programs++;
    context->shaders[fragment.idx].programs++;
    return { idx };
  }

  ProgramHandle createProgram(ShaderHandle compute, bool destroy_shader) {
    if (context == nullptr || !context->caps.compute || !context->shaders.valid(compute.idx) ||
        context->shaders[compute.idx].info.stage != ShaderStage::Compute)
      return {};

    uint16_t idx = context->programs.allocate();
    if (idx == kInvalidHandle)
      return {};

    if (!context->backend->createComputeProgram(idx, compute.idx)) {
      context->programs.free(idx);
      return {};
    }

    ProgramEntry& entry = context->programs[idx];
    entry.compute = true;
    entry.vertex = compute.idx;
    entry.destroy_shaders = destroy_shader;
    context->shaders[compute.idx].programs++;
    return { idx };
  }

  void destroy(ProgramHandle handle) {
    if (context && context->programs.valid(handle.idx))
      context->destroys.push_back({ PendingDestroy::Program, handle.idx });
  }

  UniformHandle createUniform(const char* name, UniformType type, int num) {
    if (context == nullptr || type == UniformType::Sampler)
      return { context ? uniformSlot(name) : kInvalidHandle };

    uint16_t slot = uniformSlot(name);
    context->uniforms[slot].element_floats = uniformTypeFloats(type);
    std::vector<float>& values = context->uniforms[slot].values;
    size_t size = uniformTypeFloats(type) * std::max(1, num);
    if (values.size() < size)
      values.resize(size, 0.0f);
    return { slot };
  }

  void destroy(UniformHandle) { }

  void setViewRect(uint16_t view, int x, int y, int width, int height) {
    if (context == nullptr)
      return;

    ViewSettings& settings = context->view_settings[view];
    settings.x = x;
    settings.y = y;
    settings.width = width;
    settings.height = height;
  }

  void setViewFrameBuffer(uint16_t view, FrameBufferHandle handle) {
    if (context)
      context->view_settings[view].frame_buffer = handle.idx;
  }

  void setState(const RenderState& state) {
    if (context)
      context->draw.state = state;
  }

  void setUniform(UniformHandle handle, const void* value, int num) {
    if (context == nullptr || handle.idx >= context->uniforms.size())
      return;

    UniformSlot& slot = context->uniforms[handle.idx];
    std::vector<float>& values = slot.values;
    size_t floats = slot.element_floats * std::max(1, num);
    if (values.size() < floats)
      values.resize(floats, 0.0f);
    std::memcpy(values.data(), value, floats * sizeof(float));
  }

  void setTexture(int stage, UniformHandle, TextureHandle handle, uint32_t flags) {
    if (context == nullptr || stage < 0 || stage >= kMaxTextureStages)
      return;

    TextureBinding& binding = context->draw.textures[stage];
    if (!context->textures.valid(handle.idx)) {
      binding = TextureBinding();
      return;
    }

    binding.texture = handle.idx;
    uint32_t texture_flags = context->textures[handle.idx].desc.flags;
    binding.sampler_flags = (flags == kSamplerUseTexture ? texture_flags : flags) & kSamplerMask;
  }

  void setVertexBuffer(int, BufferHandle handle, uint32_t start, uint32_t count) {
    if (context == nullptr || !context->buffers.valid(handle.idx))
      return;

    DrawItem& draw = context->draw;
    uint32_t elements = bufferElements(context->buffers[handle.idx]);
    draw.vertex_buffer = handle.idx;
    draw.vertex_start = std::min(start, elements);
    draw.vertex_count = std::min(count, elements - draw.vertex_start);
  }

  void setIndexBuffer(BufferHandle handle, uint32_t start, uint32_t count) {
    if (context == nullptr || !context->buffers.valid(handle.idx))
      return;

    DrawItem& draw = context->draw;
    uint32_t elements = bufferElements(context->buffers[handle.idx]);
    draw.index_buffer = handle.idx;
    draw.index_start = std::min(start, elements);
    draw.index_count = std::min(count, elements - draw.index_start);
  }

  void setVertexCount(uint32_t count) {
    if (context == nullptr)
      return;

    context->draw.vertex_buffer = kInvalidHandle;
    context->draw.vertex_start = 0;
    context->draw.vertex_count = count;
    context->draw_vertex_count = true;
  }

  void submit(uint16_t view, ProgramHandle program, bool preserve_state) {
    if (context == nullptr)
      return;

    if (!context->programs.valid(program.idx) || context->programs[program.idx].compute) {
      if (!preserve_state)
        resetDraw();
      return;
    }

    DrawItem draw = context->draw;
    bool has_vertices = draw.vertex_buffer != kInvalidHandle || context->draw_vertex_count;
    bool has_count = draw.index_buffer != kInvalidHandle ? draw.index_count > 0 : draw.vertex_count > 0;
    if (has_vertices && has_count) {
      const ProgramEntry& entry = context->programs[program.idx];
      draw.program = program.idx;
      draw.vertex_uniform_offset = packUniforms(context->shaders[entry.vertex], draw.vertex_uniform_size);
      draw.fragment_uniform_offset = packUniforms(context->shaders[entry.fragment],
                                                  draw.fragment_uniform_size);

      uint32_t index = static_cast<uint32_t>(context->frame.draws.size());
      context->frame.draws.push_back(draw);
      context->view_items[view].push_back({ ViewItem::Draw, index });
    }

    if (!preserve_state)
      resetDraw();
  }

  void discard() {
    if (context == nullptr)
      return;
    resetDraw();
    resetCompute();
  }

  void setBuffer(int stage, BufferHandle handle, Access access) {
    if (context == nullptr || stage < 0 || stage >= kMaxComputeBindings)
      return;
    context->compute_bindings[stage] = { handle.idx, false, access };
  }

  void setImage(int stage, TextureHandle handle, Access access) {
    if (context == nullptr || stage < 0 || stage >= kMaxComputeBindings)
      return;
    context->compute_bindings[stage] = { handle.idx, true, access };
  }

  void dispatch(uint16_t view, ProgramHandle program, uint32_t groups_x, uint32_t groups_y,
                uint32_t groups_z) {
    if (context == nullptr)
      return;

    if (context->programs.valid(program.idx) && context->programs[program.idx].compute && groups_x &&
        groups_y && groups_z && computeBindingsMatch(context->shaders[context->programs[program.idx].vertex])) {
      ComputeItem item;
      item.program = program.idx;
      std::copy(std::begin(context->compute_bindings), std::end(context->compute_bindings),
                std::begin(item.bindings));
      item.groups[0] = groups_x;
      item.groups[1] = groups_y;
      item.groups[2] = groups_z;
      item.uniform_offset = packUniforms(context->shaders[context->programs[program.idx].vertex],
                                         item.uniform_size);

      uint32_t index = static_cast<uint32_t>(context->frame.computes.size());
      context->frame.computes.push_back(item);
      context->view_items[view].push_back({ ViewItem::Compute, index });
    }

    resetCompute();
  }

  void blit(uint16_t view, TextureHandle destination, int dest_x, int dest_y, TextureHandle source,
            int source_x, int source_y, int width, int height) {
    if (context == nullptr || !context->textures.valid(destination.idx) ||
        !context->textures.valid(source.idx))
      return;

    const TextureDesc& source_desc = context->textures[source.idx].desc;
    const TextureDesc& dest_desc = context->textures[destination.idx].desc;
    if (source_x < 0 || source_y < 0 || dest_x < 0 || dest_y < 0 || source_desc.format != dest_desc.format)
      return;
    if (width < 0)
      width = source_desc.width - source_x;
    if (height < 0)
      height = source_desc.height - source_y;
    width = std::min({ width, source_desc.width - source_x, dest_desc.width - dest_x });
    height = std::min({ height, source_desc.height - source_y, dest_desc.height - dest_y });
    if (width <= 0 || height <= 0)
      return;

    BlitItem item { destination.idx, dest_x, dest_y, source.idx, source_x, source_y, width, height };
    uint32_t index = static_cast<uint32_t>(context->frame.blits.size());
    context->frame.blits.push_back(item);
    context->view_items[view].push_back({ ViewItem::Blit, index });
  }

  uint32_t frameNumber() {
    return context ? context->frame_number : 0;
  }

  uint32_t frame() {
    if (context == nullptr)
      return 0;

    auto start = std::chrono::steady_clock::now();
    Backend* backend = context->backend.get();
    FrameData& frame = context->frame;
    frame.frame_number = context->frame_number;
    backend->beginFrame(context->frame_number);

    for (const PendingUpdate& update : context->updates) {
      if (update.texture) {
        if (context->textures.valid(update.idx)) {
          backend->updateTexture(update.idx, update.x, update.y, update.width, update.height,
                                 update.memory->data, update.pitch);
        }
      }
      else if (context->buffers.valid(update.idx)) {
        const BufferEntry& entry = context->buffers[update.idx];
        uint32_t size = std::min(update.memory->size, entry.desc.size - update.offset);
        backend->updateBuffer(update.idx, update.offset, update.memory->data, size);
      }
      release(update.memory);
    }
    context->updates.clear();

    int width = 0;
    int height = 0;
    for (auto& view : context->view_items) {
      if (view.second.empty())
        continue;

      ViewSettings& settings = context->view_settings[view.first];
      ViewData data;
      data.id = view.first;
      if (context->frame_buffers.valid(settings.frame_buffer)) {
        const FrameBufferEntry& entry = context->frame_buffers[settings.frame_buffer];
        data.frame_buffer = settings.frame_buffer;
        if (entry.swap_chain) {
          width = std::max(width, entry.width);
          height = std::max(height, entry.height);
        }
        data.width = settings.width > 0 ? settings.width : entry.width;
        data.height = settings.height > 0 ? settings.height : entry.height;
      }
      data.x = settings.x;
      data.y = settings.y;
      data.items = std::move(view.second);
      frame.views.push_back(std::move(data));
    }

    Stats& stats = context->stats;
    stats.num_draws = static_cast<int>(frame.draws.size());
    stats.num_computes = static_cast<int>(frame.computes.size());
    stats.num_views = static_cast<int>(frame.views.size());
    stats.width = width;
    stats.height = height;
    frame.reads = std::move(context->pending_reads);
    frame.screenshots = std::move(context->pending_screenshots);
    context->pending_reads.clear();
    context->pending_screenshots.clear();
    context->submitting = true;
    backend->submitFrame(frame, stats);
    context->submitting = false;

    frame.views.clear();
    frame.draws.clear();
    frame.computes.clear();
    frame.blits.clear();
    frame.uniform_data.clear();
    frame.reads.clear();
    frame.screenshots.clear();
    context->view_items.clear();

    std::vector<PendingDestroy> destroys = std::move(context->destroys);
    context->destroys.clear();
    for (const PendingDestroy& destroy : destroys) {
      bool valid = false;
      switch (destroy.type) {
      case PendingDestroy::Texture: valid = context->textures.valid(destroy.idx); break;
      case PendingDestroy::FrameBuffer: valid = context->frame_buffers.valid(destroy.idx); break;
      case PendingDestroy::Buffer: valid = context->buffers.valid(destroy.idx); break;
      case PendingDestroy::Shader: valid = context->shaders.valid(destroy.idx); break;
      case PendingDestroy::Program: valid = context->programs.valid(destroy.idx); break;
      }
      if (valid)
        destroyNow(destroy);
    }

    auto end = std::chrono::steady_clock::now();
    stats.cpu_frame_ms = std::chrono::duration<double, std::milli>(end - start).count();
    return ++context->frame_number;
  }
}
