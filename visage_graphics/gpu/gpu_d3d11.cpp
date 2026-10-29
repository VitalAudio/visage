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

#if _WIN32

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <dxgi1_5.h>
#include <mutex>
#include <string>
#include <unordered_map>

namespace visage::gpu {
  namespace {
    template<typename T>
    void release(T*& object) {
      if (object)
        object->Release();
      object = nullptr;
    }

    struct FormatInfo {
      DXGI_FORMAT format;
      int bytes_per_pixel;
    };

    FormatInfo formatInfo(TextureFormat format) {
      switch (format) {
      case TextureFormat::RGBA8: return { DXGI_FORMAT_R8G8B8A8_UNORM, 4 };
      case TextureFormat::BGRA8: return { DXGI_FORMAT_B8G8R8A8_UNORM, 4 };
      case TextureFormat::R8: return { DXGI_FORMAT_R8_UNORM, 1 };
      case TextureFormat::R32F: return { DXGI_FORMAT_R32_FLOAT, 4 };
      case TextureFormat::RGBA16F: return { DXGI_FORMAT_R16G16B16A16_FLOAT, 8 };
      case TextureFormat::RGBA32F: return { DXGI_FORMAT_R32G32B32A32_FLOAT, 16 };
      default: return { DXGI_FORMAT_UNKNOWN, 0 };
      }
    }

    D3D11_BLEND blendFactor(BlendFactor factor) {
      switch (factor) {
      case BlendFactor::Zero: return D3D11_BLEND_ZERO;
      case BlendFactor::One: return D3D11_BLEND_ONE;
      case BlendFactor::SrcColor: return D3D11_BLEND_SRC_COLOR;
      case BlendFactor::InvSrcColor: return D3D11_BLEND_INV_SRC_COLOR;
      case BlendFactor::SrcAlpha: return D3D11_BLEND_SRC_ALPHA;
      case BlendFactor::InvSrcAlpha: return D3D11_BLEND_INV_SRC_ALPHA;
      case BlendFactor::DstColor: return D3D11_BLEND_DEST_COLOR;
      case BlendFactor::InvDstColor: return D3D11_BLEND_INV_DEST_COLOR;
      case BlendFactor::DstAlpha: return D3D11_BLEND_DEST_ALPHA;
      case BlendFactor::InvDstAlpha: return D3D11_BLEND_INV_DEST_ALPHA;
      }
      return D3D11_BLEND_ONE;
    }

    D3D11_BLEND alphaBlendFactor(BlendFactor factor) {
      switch (factor) {
      case BlendFactor::SrcColor: return D3D11_BLEND_SRC_ALPHA;
      case BlendFactor::InvSrcColor: return D3D11_BLEND_INV_SRC_ALPHA;
      case BlendFactor::DstColor: return D3D11_BLEND_DEST_ALPHA;
      case BlendFactor::InvDstColor: return D3D11_BLEND_INV_DEST_ALPHA;
      default: return blendFactor(factor);
      }
    }

    D3D11_BLEND_OP blendOp(BlendOp op) {
      switch (op) {
      case BlendOp::Add: return D3D11_BLEND_OP_ADD;
      case BlendOp::Subtract: return D3D11_BLEND_OP_SUBTRACT;
      case BlendOp::ReverseSubtract: return D3D11_BLEND_OP_REV_SUBTRACT;
      case BlendOp::Min: return D3D11_BLEND_OP_MIN;
      case BlendOp::Max: return D3D11_BLEND_OP_MAX;
      }
      return D3D11_BLEND_OP_ADD;
    }

    D3D11_TEXTURE_ADDRESS_MODE addressMode(uint32_t flags, uint32_t clamp, uint32_t mirror) {
      if (flags & clamp)
        return D3D11_TEXTURE_ADDRESS_CLAMP;
      if (flags & mirror)
        return D3D11_TEXTURE_ADDRESS_MIRROR;
      return D3D11_TEXTURE_ADDRESS_WRAP;
    }

    D3D11_FILTER filter(uint32_t flags) {
      bool min_point = flags & kSamplerMinPoint;
      bool mag_point = flags & kSamplerMagPoint;
      if (min_point && mag_point)
        return D3D11_FILTER_MIN_MAG_MIP_POINT;
      if (min_point)
        return D3D11_FILTER_MIN_POINT_MAG_LINEAR_MIP_POINT;
      if (mag_point)
        return D3D11_FILTER_MIN_LINEAR_MAG_MIP_POINT;
      return D3D11_FILTER_MIN_MAG_LINEAR_MIP_POINT;
    }

    DXGI_FORMAT attributeFormat(int num_floats) {
      switch (num_floats) {
      case 1: return DXGI_FORMAT_R32_FLOAT;
      case 2: return DXGI_FORMAT_R32G32_FLOAT;
      case 3: return DXGI_FORMAT_R32G32B32_FLOAT;
      default: return DXGI_FORMAT_R32G32B32A32_FLOAT;
      }
    }

    using D3DCompileFunction = decltype(&D3DCompile);

    struct Texture {
      ID3D11Texture2D* texture = nullptr;
      ID3D11ShaderResourceView* srv = nullptr;
      ID3D11UnorderedAccessView* uav = nullptr;
      ID3D11Texture2D* staging = nullptr;
      TextureDesc desc;
    };

    struct Buffer {
      ID3D11Buffer* buffer = nullptr;
      ID3D11ShaderResourceView* srv = nullptr;
      ID3D11UnorderedAccessView* uav = nullptr;
      BufferDesc desc;
    };

    struct Shader {
      ShaderInfo info;
      ID3D11VertexShader* vertex = nullptr;
      ID3D11PixelShader* pixel = nullptr;
      ID3D11ComputeShader* compute = nullptr;
      ID3D11Buffer* constants = nullptr;
      std::vector<uint8_t> constant_data;
      std::vector<uint8_t> bytecode;
    };

    struct Program {
      uint16_t vertex = kInvalidHandle;
      uint16_t fragment = kInvalidHandle;
      uint16_t compute = kInvalidHandle;
      std::unordered_map<uint32_t, ID3D11InputLayout*> input_layouts;
    };

    struct FrameBuffer {
      uint16_t texture = kInvalidHandle;
      ID3D11RenderTargetView* rtv = nullptr;
      IDXGISwapChain1* swap_chain = nullptr;
      ID3D11Texture2D* back_buffer = nullptr;
      HANDLE waitable = nullptr;
      HWND window = nullptr;
      UINT swap_chain_flags = 0;
      TextureFormat format = TextureFormat::BGRA8;
      int width = 0;
      int height = 0;
      bool needs_present = false;
    };
  }

  class D3D11Backend : public Backend {
  public:
    ~D3D11Backend() override { shutdown(); }

    bool init(const Init& init, Caps& caps) override {
      init_ = init;
      UINT flags = D3D11_CREATE_DEVICE_BGRA_SUPPORT;
      if (init.debug)
        flags |= D3D11_CREATE_DEVICE_DEBUG;

      static constexpr D3D_FEATURE_LEVEL kFeatureLevels[] = { D3D_FEATURE_LEVEL_11_1,
                                                              D3D_FEATURE_LEVEL_11_0 };
      HRESULT hr = createDevice(D3D_DRIVER_TYPE_HARDWARE, flags, kFeatureLevels, 2);
      if (FAILED(hr) && (flags & D3D11_CREATE_DEVICE_DEBUG)) {
        flags &= ~D3D11_CREATE_DEVICE_DEBUG;
        hr = createDevice(D3D_DRIVER_TYPE_HARDWARE, flags, kFeatureLevels, 2);
      }
      if (FAILED(hr))
        hr = createDevice(D3D_DRIVER_TYPE_HARDWARE, flags, kFeatureLevels + 1, 1);
      if (FAILED(hr)) {
        log("D3D11: no hardware device, falling back to WARP");
        hr = createDevice(D3D_DRIVER_TYPE_WARP, flags, kFeatureLevels, 2);
      }
      if (FAILED(hr)) {
        log("D3D11: failed to create device");
        return false;
      }

      IDXGIDevice* dxgi_device = nullptr;
      IDXGIAdapter* adapter = nullptr;
      if (SUCCEEDED(device_->QueryInterface(__uuidof(IDXGIDevice), (void**)&dxgi_device)) &&
          SUCCEEDED(dxgi_device->GetAdapter(&adapter))) {
        adapter->GetParent(__uuidof(IDXGIFactory2), (void**)&factory_);
        DXGI_ADAPTER_DESC adapter_desc;
        if (SUCCEEDED(adapter->GetDesc(&adapter_desc)))
          basic_render_adapter_ = adapter_desc.VendorId == 0x1414 && adapter_desc.DeviceId == 0x8c;
      }
      release(adapter);
      release(dxgi_device);
      if (factory_ == nullptr) {
        log("D3D11: failed to get DXGI factory");
        shutdown();
        return false;
      }

      IDXGIFactory5* factory5 = nullptr;
      if (SUCCEEDED(factory_->QueryInterface(__uuidof(IDXGIFactory5), (void**)&factory5))) {
        BOOL allow_tearing = FALSE;
        if (SUCCEEDED(factory5->CheckFeatureSupport(DXGI_FEATURE_PRESENT_ALLOW_TEARING,
                                                    &allow_tearing, sizeof(allow_tearing))))
          tearing_supported_ = allow_tearing;
        release(factory5);
      }

      D3D11_RASTERIZER_DESC raster_desc = {};
      raster_desc.FillMode = D3D11_FILL_SOLID;
      raster_desc.CullMode = D3D11_CULL_NONE;
      raster_desc.DepthClipEnable = TRUE;
      device_->CreateRasterizerState(&raster_desc, &rasterizer_state_);

      D3D11_DEPTH_STENCIL_DESC depth_desc = {};
      depth_desc.DepthEnable = FALSE;
      depth_desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
      depth_desc.DepthFunc = D3D11_COMPARISON_ALWAYS;
      device_->CreateDepthStencilState(&depth_desc, &depth_state_);

      caps.renderer = RendererType::Direct3D11;
      caps.origin_bottom_left = false;
      caps.compute = feature_level_ >= D3D_FEATURE_LEVEL_11_0;
      caps.swap_chain = true;
      caps.max_texture_size = D3D11_REQ_TEXTURE2D_U_OR_V_DIMENSION;
      for (int i = 0; i < static_cast<int>(TextureFormat::Count); ++i) {
        DXGI_FORMAT format = formatInfo(static_cast<TextureFormat>(i)).format;
        UINT support = 0;
        if (format == DXGI_FORMAT_UNKNOWN || FAILED(device_->CheckFormatSupport(format, &support)))
          continue;

        uint32_t result = 0;
        if (support & D3D11_FORMAT_SUPPORT_TEXTURE2D)
          result |= kFormatTexture;
        if (support & D3D11_FORMAT_SUPPORT_RENDER_TARGET)
          result |= kFormatRenderTarget;
        if (support & D3D11_FORMAT_SUPPORT_TYPED_UNORDERED_ACCESS_VIEW)
          result |= kFormatStorage;
        if (support & D3D11_FORMAT_SUPPORT_BLENDABLE)
          result |= kFormatBlend;
        caps.formats[i] = result;
      }
      return true;
    }

    void shutdown() override {
      if (context_)
        context_->ClearState();
      resetPipelineCache();
      current_rtv_ = nullptr;

      for (auto& texture : textures_)
        releaseTexture(texture.second);
      textures_.clear();
      for (auto& buffer : buffers_)
        releaseBuffer(buffer.second);
      buffers_.clear();
      for (auto& program : programs_)
        releaseProgram(program.second);
      programs_.clear();
      for (auto& shader : shaders_)
        releaseShader(shader.second);
      shaders_.clear();
      {
        std::lock_guard<std::mutex> lock(waitable_mutex_);
        for (auto& frame_buffer : frame_buffers_)
          releaseFrameBuffer(frame_buffer.second);
        frame_buffers_.clear();
      }
      for (auto& state : blend_states_)
        release(state.second);
      blend_states_.clear();
      for (auto& state : sampler_states_)
        release(state.second);
      sampler_states_.clear();

      release(rasterizer_state_);
      release(depth_state_);
      if (context_)
        context_->Flush();
      release(factory_);
      release(context_);
      release(device_);
      if (compiler_)
        FreeLibrary(compiler_);
      compiler_ = nullptr;
      compile_ = nullptr;
    }

    bool createTexture(uint16_t idx, const TextureDesc& desc, const uint8_t* data, int pitch) override {
      FormatInfo format = formatInfo(desc.format);
      D3D11_TEXTURE2D_DESC texture_desc = {};
      texture_desc.Width = std::max(1, desc.width);
      texture_desc.Height = std::max(1, desc.height);
      texture_desc.MipLevels = 1;
      texture_desc.ArraySize = 1;
      texture_desc.Format = format.format;
      texture_desc.SampleDesc.Count = 1;
      texture_desc.Usage = D3D11_USAGE_DEFAULT;
      texture_desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
      if (desc.flags & kTextureRenderTarget)
        texture_desc.BindFlags |= D3D11_BIND_RENDER_TARGET;
      if (desc.flags & kTextureStorage)
        texture_desc.BindFlags |= D3D11_BIND_UNORDERED_ACCESS;

      D3D11_SUBRESOURCE_DATA initial = {};
      initial.pSysMem = data;
      initial.SysMemPitch = pitch ? pitch : texture_desc.Width * format.bytes_per_pixel;

      Texture texture;
      texture.desc = desc;
      if (FAILED(device_->CreateTexture2D(&texture_desc, data ? &initial : nullptr, &texture.texture)) ||
          FAILED(device_->CreateShaderResourceView(texture.texture, nullptr, &texture.srv))) {
        releaseTexture(texture);
        return false;
      }
      if ((desc.flags & kTextureStorage) &&
          FAILED(device_->CreateUnorderedAccessView(texture.texture, nullptr, &texture.uav))) {
        releaseTexture(texture);
        return false;
      }

      releaseTexture(textures_[idx]);
      textures_[idx] = texture;
      return true;
    }

    void updateTexture(uint16_t idx, int x, int y, int width, int height, const uint8_t* data,
                       int pitch) override {
      Texture* texture = findTexture(idx);
      if (texture == nullptr || data == nullptr || width <= 0 || height <= 0)
        return;

      D3D11_BOX box = { UINT(x), UINT(y), 0, UINT(x + width), UINT(y + height), 1 };
      UINT row_pitch = pitch ? pitch : width * formatInfo(texture->desc.format).bytes_per_pixel;
      context_->UpdateSubresource(texture->texture, 0, &box, data, row_pitch, 0);
    }

    void destroyTexture(uint16_t idx) override {
      auto found = textures_.find(idx);
      if (found == textures_.end())
        return;
      unbindResource(found->second.srv, found->second.uav);
      releaseTexture(found->second);
      textures_.erase(found);
    }

    bool createBuffer(uint16_t idx, const BufferDesc& desc, const uint8_t* data) override {
      bool compute = desc.flags & kBufferComputeReadWrite;
      UINT size = std::max<UINT>(4, (desc.size + 3) & ~3u);

      D3D11_BUFFER_DESC buffer_desc = {};
      buffer_desc.ByteWidth = size;
      buffer_desc.Usage = D3D11_USAGE_DEFAULT;
      buffer_desc.BindFlags = desc.index ? D3D11_BIND_INDEX_BUFFER : D3D11_BIND_VERTEX_BUFFER;
      if (compute) {
        buffer_desc.BindFlags |= D3D11_BIND_SHADER_RESOURCE | D3D11_BIND_UNORDERED_ACCESS;
        buffer_desc.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
      }

      std::vector<uint8_t> padded;
      if (data && size != desc.size) {
        padded.resize(size, 0);
        std::memcpy(padded.data(), data, desc.size);
        data = padded.data();
      }

      D3D11_SUBRESOURCE_DATA initial = {};
      initial.pSysMem = data;

      Buffer buffer;
      buffer.desc = desc;
      if (FAILED(device_->CreateBuffer(&buffer_desc, data ? &initial : nullptr, &buffer.buffer)))
        return false;

      if (compute) {
        D3D11_SHADER_RESOURCE_VIEW_DESC srv_desc = {};
        srv_desc.Format = DXGI_FORMAT_R32_TYPELESS;
        srv_desc.ViewDimension = D3D11_SRV_DIMENSION_BUFFEREX;
        srv_desc.BufferEx.NumElements = size / 4;
        srv_desc.BufferEx.Flags = D3D11_BUFFEREX_SRV_FLAG_RAW;

        D3D11_UNORDERED_ACCESS_VIEW_DESC uav_desc = {};
        uav_desc.Format = DXGI_FORMAT_R32_TYPELESS;
        uav_desc.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
        uav_desc.Buffer.NumElements = size / 4;
        uav_desc.Buffer.Flags = D3D11_BUFFER_UAV_FLAG_RAW;

        if (FAILED(device_->CreateShaderResourceView(buffer.buffer, &srv_desc, &buffer.srv)) ||
            FAILED(device_->CreateUnorderedAccessView(buffer.buffer, &uav_desc, &buffer.uav))) {
          releaseBuffer(buffer);
          return false;
        }
      }

      releaseBuffer(buffers_[idx]);
      buffers_[idx] = buffer;
      return true;
    }

    void updateBuffer(uint16_t idx, uint32_t offset, const uint8_t* data, uint32_t size) override {
      auto found = buffers_.find(idx);
      if (found == buffers_.end() || data == nullptr || size == 0)
        return;

      D3D11_BOX box = { offset, 0, 0, offset + size, 1, 1 };
      context_->UpdateSubresource(found->second.buffer, 0, &box, data, 0, 0);
    }

    void destroyBuffer(uint16_t idx) override {
      auto found = buffers_.find(idx);
      if (found == buffers_.end())
        return;
      unbindResource(found->second.srv, found->second.uav);
      if (current_vertex_buffer_ == found->second.buffer || current_index_buffer_ == found->second.buffer) {
        context_->IASetVertexBuffers(0, 0, nullptr, nullptr, nullptr);
        context_->IASetIndexBuffer(nullptr, DXGI_FORMAT_R16_UINT, 0);
        current_vertex_buffer_ = nullptr;
        current_index_buffer_ = nullptr;
      }
      releaseBuffer(found->second);
      buffers_.erase(found);
    }

    bool createShader(uint16_t idx, const ShaderInfo& info) override {
      Shader shader;
      shader.info = info;
      shader.info.code.clear();

      if (info.code_type == ShaderCode::Dxbc)
        shader.bytecode = info.code;
      else if (info.code_type != ShaderCode::Hlsl || !compileHlsl(info, shader.bytecode))
        return false;

      const void* code = shader.bytecode.data();
      SIZE_T code_size = shader.bytecode.size();
      HRESULT hr = E_FAIL;
      if (info.stage == ShaderStage::Vertex)
        hr = device_->CreateVertexShader(code, code_size, nullptr, &shader.vertex);
      else if (info.stage == ShaderStage::Fragment)
        hr = device_->CreatePixelShader(code, code_size, nullptr, &shader.pixel);
      else if (info.stage == ShaderStage::Compute)
        hr = device_->CreateComputeShader(code, code_size, nullptr, &shader.compute);
      if (FAILED(hr)) {
        log("D3D11: failed to create shader");
        releaseShader(shader);
        return false;
      }

      if (info.uniform_block_size) {
        D3D11_BUFFER_DESC constants_desc = {};
        constants_desc.ByteWidth = (info.uniform_block_size + 15) & ~15u;
        constants_desc.Usage = D3D11_USAGE_DEFAULT;
        constants_desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        shader.constant_data.assign(constants_desc.ByteWidth, 0);
        D3D11_SUBRESOURCE_DATA initial = {};
        initial.pSysMem = shader.constant_data.data();
        if (FAILED(device_->CreateBuffer(&constants_desc, &initial, &shader.constants))) {
          releaseShader(shader);
          return false;
        }
      }

      if (info.stage != ShaderStage::Vertex)
        shader.bytecode.clear();

      releaseShader(shaders_[idx]);
      shaders_[idx] = std::move(shader);
      return true;
    }

    void destroyShader(uint16_t idx) override {
      auto found = shaders_.find(idx);
      if (found == shaders_.end())
        return;
      releaseShader(found->second);
      shaders_.erase(found);
      resetPipelineCache();
    }

    bool createProgram(uint16_t idx, uint16_t vertex, uint16_t fragment) override {
      if (findShader(vertex) == nullptr || findShader(fragment) == nullptr)
        return false;
      releaseProgram(programs_[idx]);
      programs_[idx] = Program { vertex, fragment, kInvalidHandle, {} };
      return true;
    }

    bool createComputeProgram(uint16_t idx, uint16_t compute) override {
      Shader* shader = findShader(compute);
      if (shader == nullptr || shader->compute == nullptr)
        return false;
      releaseProgram(programs_[idx]);
      programs_[idx] = Program { kInvalidHandle, kInvalidHandle, compute, {} };
      return true;
    }

    void destroyProgram(uint16_t idx) override {
      auto found = programs_.find(idx);
      if (found == programs_.end())
        return;
      releaseProgram(found->second);
      programs_.erase(found);
      resetPipelineCache();
    }

    bool createFrameBuffer(uint16_t idx, uint16_t texture) override {
      Texture* target = findTexture(texture);
      if (target == nullptr)
        return false;

      FrameBuffer frame_buffer;
      frame_buffer.texture = texture;
      frame_buffer.format = target->desc.format;
      frame_buffer.width = target->desc.width;
      frame_buffer.height = target->desc.height;
      if (FAILED(device_->CreateRenderTargetView(target->texture, nullptr, &frame_buffer.rtv)))
        return false;

      std::lock_guard<std::mutex> lock(waitable_mutex_);
      releaseFrameBuffer(frame_buffers_[idx]);
      frame_buffers_[idx] = frame_buffer;
      return true;
    }

    bool createSwapChain(uint16_t idx, void* window_handle, void*, int width, int height,
                         TextureFormat format) override {
      FrameBuffer frame_buffer;
      frame_buffer.window = static_cast<HWND>(window_handle);
      frame_buffer.format = format;
      frame_buffer.width = std::max(1, width);
      frame_buffer.height = std::max(1, height);
      frame_buffer.swap_chain_flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH |
                                      DXGI_SWAP_CHAIN_FLAG_FRAME_LATENCY_WAITABLE_OBJECT;
      if (tearing_supported_)
        frame_buffer.swap_chain_flags |= DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING;

      DXGI_SWAP_CHAIN_DESC1 desc = {};
      desc.Width = frame_buffer.width;
      desc.Height = frame_buffer.height;
      desc.Format = formatInfo(format).format;
      desc.SampleDesc.Count = 1;
      desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
      desc.BufferCount = 2;
      desc.Scaling = DXGI_SCALING_NONE;
      desc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
      desc.AlphaMode = DXGI_ALPHA_MODE_IGNORE;
      desc.Flags = frame_buffer.swap_chain_flags;

      DXGI_SWAP_CHAIN_FULLSCREEN_DESC fullscreen_desc = {};
      fullscreen_desc.Windowed = TRUE;

      HRESULT hr = factory_->CreateSwapChainForHwnd(device_, frame_buffer.window, &desc, &fullscreen_desc,
                                                    nullptr, &frame_buffer.swap_chain);
      if (FAILED(hr)) {
        log("D3D11: failed to create swap chain");
        return false;
      }

      IDXGISwapChain2* swap_chain2 = nullptr;
      if (SUCCEEDED(frame_buffer.swap_chain->QueryInterface(__uuidof(IDXGISwapChain2), (void**)&swap_chain2))) {
        swap_chain2->SetMaximumFrameLatency(1);
        frame_buffer.waitable = swap_chain2->GetFrameLatencyWaitableObject();
        release(swap_chain2);
      }

      factory_->MakeWindowAssociation(frame_buffer.window, DXGI_MWA_NO_WINDOW_CHANGES | DXGI_MWA_NO_ALT_ENTER);

      if (!createSwapChainViews(frame_buffer)) {
        releaseFrameBuffer(frame_buffer);
        return false;
      }

      std::lock_guard<std::mutex> lock(waitable_mutex_);
      releaseFrameBuffer(frame_buffers_[idx]);
      frame_buffers_[idx] = frame_buffer;
      return true;
    }

    void resizeSwapChain(uint16_t idx, int width, int height) override {
      auto found = frame_buffers_.find(idx);
      if (found == frame_buffers_.end() || found->second.swap_chain == nullptr)
        return;

      FrameBuffer& frame_buffer = found->second;
      width = std::max(1, width);
      height = std::max(1, height);
      if (width == frame_buffer.width && height == frame_buffer.height)
        return;

      context_->OMSetRenderTargets(0, nullptr, nullptr);
      current_rtv_ = nullptr;
      release(frame_buffer.rtv);
      release(frame_buffer.back_buffer);
      context_->Flush();

      HRESULT hr = frame_buffer.swap_chain->ResizeBuffers(0, width, height, DXGI_FORMAT_UNKNOWN,
                                                          frame_buffer.swap_chain_flags);
      if (SUCCEEDED(hr)) {
        frame_buffer.width = width;
        frame_buffer.height = height;
      }
      else if (!checkDeviceLost(hr))
        log("D3D11: failed to resize swap chain");
      createSwapChainViews(frame_buffer);
    }

    void destroyFrameBuffer(uint16_t idx) override {
      auto found = frame_buffers_.find(idx);
      if (found == frame_buffers_.end())
        return;

      if (found->second.rtv == current_rtv_) {
        context_->OMSetRenderTargets(0, nullptr, nullptr);
        current_rtv_ = nullptr;
      }

      std::lock_guard<std::mutex> lock(waitable_mutex_);
      releaseFrameBuffer(found->second);
      frame_buffers_.erase(found);
    }

    bool waitForFrame(uint16_t idx, int timeout_ms) override {
      if (device_lost_)
        return true;

      HANDLE waitable = nullptr;
      {
        std::lock_guard<std::mutex> lock(waitable_mutex_);
        auto found = frame_buffers_.find(idx);
        if (found == frame_buffers_.end() || found->second.waitable == nullptr)
          return true;

        HANDLE process = GetCurrentProcess();
        if (!DuplicateHandle(process, found->second.waitable, process, &waitable, SYNCHRONIZE, FALSE, 0))
          return true;
      }

      DWORD result = WaitForSingleObjectEx(waitable, timeout_ms < 0 ? INFINITE : DWORD(timeout_ms), TRUE);
      CloseHandle(waitable);
      return result == WAIT_OBJECT_0;
    }

    void beginFrame(uint32_t) override { cpu_start_ = std::chrono::steady_clock::now(); }

    bool deviceLost() override {
      if (device_lost_ || (device_ && checkDeviceLost(device_->GetDeviceRemovedReason())))
        return true;

      static constexpr auto kAdapterCheckInterval = std::chrono::milliseconds(250);
      if (!basic_render_adapter_ || factory_ == nullptr)
        return false;

      auto now = std::chrono::steady_clock::now();
      if (now - last_adapter_check_ < kAdapterCheckInterval)
        return false;

      last_adapter_check_ = now;
      if (factory_->IsCurrent() || device_lost_.exchange(true))
        return device_lost_;

      log("D3D11: adapters changed while on the Microsoft Basic Render Driver, rebuilding");
      return true;
    }

    void submitFrame(const FrameData& frame, Stats& stats) override {
      resetPipelineCache();
      context_->RSSetState(rasterizer_state_);
      context_->OMSetDepthStencilState(depth_state_, 0);
      context_->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

      for (const ViewData& view : frame.views) {
        auto found = frame_buffers_.find(view.frame_buffer);
        FrameBuffer* frame_buffer = found == frame_buffers_.end() ? nullptr : &found->second;
        if (frame_buffer && frame_buffer->swap_chain)
          frame_buffer->needs_present = true;

        D3D11_VIEWPORT viewport = { float(view.x), float(view.y), float(view.width), float(view.height),
                                    0.0f, 1.0f };
        bool targets_bound = false;

        for (const ViewItem& item : view.items) {
          if (item.type == ViewItem::Draw) {
            if (frame_buffer == nullptr || frame_buffer->rtv == nullptr)
              continue;
            if (!targets_bound) {
              bindRenderTarget(frame_buffer->rtv);
              context_->RSSetViewports(1, &viewport);
              targets_bound = true;
            }
            draw(frame.draws[item.index], frame.uniform_data);
          }
          else if (item.type == ViewItem::Compute) {
            unbindRenderTarget();
            targets_bound = false;
            dispatch(frame.computes[item.index], frame.uniform_data);
          }
          else if (item.type == ViewItem::Blit) {
            blit(frame.blits[item.index]);
          }
        }
      }

      clearShaderResources();
      unbindRenderTarget();

      for (const ReadRequest& read : frame.reads)
        readTexture(read);

      for (uint16_t screenshot : frame.screenshots)
        takeScreenshot(screenshot);

      UINT sync_interval = init_.vsync ? 1 : 0;
      UINT present_flags = !init_.vsync && tearing_supported_ ? DXGI_PRESENT_ALLOW_TEARING : 0;
      for (auto& frame_buffer : frame_buffers_) {
        if (!frame_buffer.second.needs_present)
          continue;
        frame_buffer.second.needs_present = false;
        if (checkDeviceLost(frame_buffer.second.swap_chain->Present(sync_interval, present_flags)))
          break;
      }

      stats.num_textures = int(textures_.size());
      stats.num_frame_buffers = int(frame_buffers_.size());
      stats.num_buffers = int(buffers_.size());
      stats.num_shaders = int(shaders_.size());
      stats.num_programs = int(programs_.size());
      stats.num_draws = int(frame.draws.size());
      stats.num_computes = int(frame.computes.size());
      stats.num_views = int(frame.views.size());
      stats.cpu_frame_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                                     cpu_start_)
                               .count();
    }

  private:
    HRESULT createDevice(D3D_DRIVER_TYPE type, UINT flags, const D3D_FEATURE_LEVEL* levels, UINT num_levels) {
      release(context_);
      release(device_);
      HRESULT hr = D3D11CreateDevice(nullptr, type, nullptr, flags, levels, num_levels, D3D11_SDK_VERSION,
                                     &device_, &feature_level_, &context_);
      return hr;
    }

    bool checkDeviceLost(HRESULT hr) {
      if (hr != DXGI_ERROR_DEVICE_REMOVED && hr != DXGI_ERROR_DEVICE_RESET && hr != DXGI_ERROR_DEVICE_HUNG &&
          hr != DXGI_ERROR_DRIVER_INTERNAL_ERROR)
        return false;

      if (!device_lost_.exchange(true)) {
        char message[96];
        std::snprintf(message, sizeof(message), "D3D11: device lost (0x%08lx, reason 0x%08lx)",
                      static_cast<unsigned long>(hr),
                      static_cast<unsigned long>(device_->GetDeviceRemovedReason()));
        log(message);
      }
      return true;
    }

    void log(const char* message) const {
      if (init_.log_callback)
        init_.log_callback(message);
    }

    bool compileHlsl(const ShaderInfo& info, std::vector<uint8_t>& bytecode) {
      if (compile_ == nullptr) {
        if (compiler_ == nullptr)
          compiler_ = LoadLibraryA("d3dcompiler_47.dll");
        if (compiler_)
          compile_ = reinterpret_cast<D3DCompileFunction>(GetProcAddress(compiler_, "D3DCompile"));
      }
      if (compile_ == nullptr) {
        log("D3D11: d3dcompiler_47.dll is unavailable for HLSL shaders");
        return false;
      }

      const char* target = info.stage == ShaderStage::Vertex   ? "vs_5_0" :
                           info.stage == ShaderStage::Fragment ? "ps_5_0" :
                                                                 "cs_5_0";
      ID3DBlob* code = nullptr;
      ID3DBlob* errors = nullptr;
      HRESULT hr = compile_(info.code.data(), info.code.size(), nullptr, nullptr, nullptr, "main", target,
                            D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &code, &errors);
      if (errors) {
        log(static_cast<const char*>(errors->GetBufferPointer()));
        release(errors);
      }
      if (FAILED(hr) || code == nullptr) {
        release(code);
        return false;
      }

      const uint8_t* data = static_cast<const uint8_t*>(code->GetBufferPointer());
      bytecode.assign(data, data + code->GetBufferSize());
      release(code);
      return true;
    }

    Texture* findTexture(uint16_t idx) {
      auto found = textures_.find(idx);
      return found == textures_.end() ? nullptr : &found->second;
    }

    Shader* findShader(uint16_t idx) {
      auto found = shaders_.find(idx);
      return found == shaders_.end() ? nullptr : &found->second;
    }

    static void releaseTexture(Texture& texture) {
      release(texture.staging);
      release(texture.uav);
      release(texture.srv);
      release(texture.texture);
    }

    static void releaseBuffer(Buffer& buffer) {
      release(buffer.uav);
      release(buffer.srv);
      release(buffer.buffer);
    }

    static void releaseShader(Shader& shader) {
      release(shader.constants);
      release(shader.compute);
      release(shader.pixel);
      release(shader.vertex);
    }

    static void releaseProgram(Program& program) {
      for (auto& layout : program.input_layouts)
        release(layout.second);
      program.input_layouts.clear();
    }

    static void releaseFrameBuffer(FrameBuffer& frame_buffer) {
      release(frame_buffer.rtv);
      release(frame_buffer.back_buffer);
      if (frame_buffer.waitable)
        CloseHandle(frame_buffer.waitable);
      frame_buffer.waitable = nullptr;
      release(frame_buffer.swap_chain);
    }

    bool createSwapChainViews(FrameBuffer& frame_buffer) {
      if (FAILED(frame_buffer.swap_chain->GetBuffer(0, __uuidof(ID3D11Texture2D),
                                                    (void**)&frame_buffer.back_buffer)))
        return false;
      return SUCCEEDED(device_->CreateRenderTargetView(frame_buffer.back_buffer, nullptr, &frame_buffer.rtv));
    }

    ID3D11BlendState* blendState(const RenderState& state) {
      uint32_t key = state.key();
      auto found = blend_states_.find(key);
      if (found != blend_states_.end())
        return found->second;

      D3D11_BLEND_DESC desc = {};
      D3D11_RENDER_TARGET_BLEND_DESC& target = desc.RenderTarget[0];
      target.BlendEnable = state.blend;
      target.SrcBlend = blendFactor(state.src_rgb);
      target.DestBlend = blendFactor(state.dst_rgb);
      target.BlendOp = blendOp(state.op_rgb);
      target.SrcBlendAlpha = alphaBlendFactor(state.src_alpha);
      target.DestBlendAlpha = alphaBlendFactor(state.dst_alpha);
      target.BlendOpAlpha = blendOp(state.op_alpha);
      target.RenderTargetWriteMask = (state.write_mask & kWriteR ? D3D11_COLOR_WRITE_ENABLE_RED : 0) |
                                     (state.write_mask & kWriteG ? D3D11_COLOR_WRITE_ENABLE_GREEN : 0) |
                                     (state.write_mask & kWriteB ? D3D11_COLOR_WRITE_ENABLE_BLUE : 0) |
                                     (state.write_mask & kWriteA ? D3D11_COLOR_WRITE_ENABLE_ALPHA : 0);

      ID3D11BlendState* blend_state = nullptr;
      device_->CreateBlendState(&desc, &blend_state);
      blend_states_[key] = blend_state;
      return blend_state;
    }

    ID3D11SamplerState* samplerState(uint32_t flags) {
      flags &= kSamplerMask;
      auto found = sampler_states_.find(flags);
      if (found != sampler_states_.end())
        return found->second;

      D3D11_SAMPLER_DESC desc = {};
      desc.Filter = filter(flags);
      desc.AddressU = addressMode(flags, kSamplerUClamp, kSamplerUMirror);
      desc.AddressV = addressMode(flags, kSamplerVClamp, kSamplerVMirror);
      desc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
      desc.MaxAnisotropy = 1;
      desc.ComparisonFunc = D3D11_COMPARISON_NEVER;
      desc.MaxLOD = D3D11_FLOAT32_MAX;

      ID3D11SamplerState* sampler_state = nullptr;
      device_->CreateSamplerState(&desc, &sampler_state);
      sampler_states_[flags] = sampler_state;
      return sampler_state;
    }

    ID3D11InputLayout* inputLayout(Program& program, const Shader& vertex, const Buffer& buffer) {
      uint32_t key = buffer.desc.layout.hash();
      auto found = program.input_layouts.find(key);
      if (found != program.input_layouts.end())
        return found->second;

      const VertexLayout& layout = buffer.desc.layout;
      D3D11_INPUT_ELEMENT_DESC elements[static_cast<int>(Attrib::Count)] = {};
      UINT num_elements = 0;
      for (const ShaderAttribute& attribute : vertex.info.attributes) {
        if (!layout.has(attribute.attrib))
          continue;
        D3D11_INPUT_ELEMENT_DESC& element = elements[num_elements++];
        element.SemanticName = "TEXCOORD";
        element.SemanticIndex = static_cast<UINT>(attribute.attrib);
        element.Format = attributeFormat(layout.count(attribute.attrib));
        element.AlignedByteOffset = layout.offset(attribute.attrib);
        element.InputSlotClass = D3D11_INPUT_PER_VERTEX_DATA;
      }

      ID3D11InputLayout* input_layout = nullptr;
      if (num_elements)
        device_->CreateInputLayout(elements, num_elements, vertex.bytecode.data(), vertex.bytecode.size(),
                                   &input_layout);
      program.input_layouts[key] = input_layout;
      return input_layout;
    }

    ID3D11Buffer* updateConstants(Shader& shader, const std::vector<uint8_t>& uniform_data,
                                  uint32_t offset, uint32_t size) {
      if (shader.constants == nullptr)
        return nullptr;

      size = std::min<uint32_t>(size, uint32_t(shader.constant_data.size()));
      if (offset + size > uniform_data.size())
        size = offset < uniform_data.size() ? uint32_t(uniform_data.size() - offset) : 0;
      if (size && std::memcmp(shader.constant_data.data(), uniform_data.data() + offset, size)) {
        std::memcpy(shader.constant_data.data(), uniform_data.data() + offset, size);
        context_->UpdateSubresource(shader.constants, 0, nullptr, shader.constant_data.data(), 0, 0);
      }
      return shader.constants;
    }

    void resetPipelineCache() {
      current_program_ = kInvalidHandle;
      current_layout_ = nullptr;
      current_blend_ = nullptr;
      current_vertex_buffer_ = nullptr;
      current_vertex_stride_ = 0;
      current_index_buffer_ = nullptr;
      current_index_format_ = DXGI_FORMAT_UNKNOWN;
      bound_vertex_srvs_ = 0;
      bound_pixel_srvs_ = 0;
    }

    void bindRenderTarget(ID3D11RenderTargetView* rtv) {
      if (current_rtv_ == rtv)
        return;
      clearShaderResources();
      context_->OMSetRenderTargets(1, &rtv, nullptr);
      current_rtv_ = rtv;
    }

    void unbindRenderTarget() {
      if (current_rtv_ == nullptr)
        return;
      context_->OMSetRenderTargets(0, nullptr, nullptr);
      current_rtv_ = nullptr;
    }

    void clearShaderResources() {
      ID3D11ShaderResourceView* null_srvs[kMaxTextureStages] = {};
      if (bound_pixel_srvs_)
        context_->PSSetShaderResources(0, kMaxTextureStages, null_srvs);
      if (bound_vertex_srvs_)
        context_->VSSetShaderResources(0, kMaxTextureStages, null_srvs);
      bound_pixel_srvs_ = 0;
      bound_vertex_srvs_ = 0;
    }

    void unbindResource(ID3D11ShaderResourceView* srv, ID3D11UnorderedAccessView* uav) {
      if (context_ == nullptr || (srv == nullptr && uav == nullptr))
        return;
      clearShaderResources();
      ID3D11ShaderResourceView* null_srvs[kMaxComputeBindings] = {};
      ID3D11UnorderedAccessView* null_uavs[kMaxComputeBindings] = {};
      context_->CSSetShaderResources(0, kMaxComputeBindings, null_srvs);
      context_->CSSetUnorderedAccessViews(0, kMaxComputeBindings, null_uavs, nullptr);
    }

    void bindTextures(const DrawItem& draw, const Shader& shader, bool pixel) {
      uint32_t& bound = pixel ? bound_pixel_srvs_ : bound_vertex_srvs_;
      for (const ShaderSampler& sampler : shader.info.samplers) {
        if (sampler.stage >= kMaxTextureStages)
          continue;

        const TextureBinding& binding = draw.textures[sampler.stage];
        Texture* texture = findTexture(binding.texture);
        ID3D11ShaderResourceView* srv = texture ? texture->srv : nullptr;
        ID3D11SamplerState* sampler_state = samplerState(binding.sampler_flags);
        if (pixel) {
          context_->PSSetShaderResources(sampler.stage, 1, &srv);
          context_->PSSetSamplers(sampler.stage, 1, &sampler_state);
        }
        else {
          context_->VSSetShaderResources(sampler.stage, 1, &srv);
          context_->VSSetSamplers(sampler.stage, 1, &sampler_state);
        }
        bound |= 1u << sampler.stage;
      }
    }

    void draw(const DrawItem& draw, const std::vector<uint8_t>& uniform_data) {
      auto program_found = programs_.find(draw.program);
      if (program_found == programs_.end())
        return;

      Program& program = program_found->second;
      Shader* vertex = findShader(program.vertex);
      Shader* fragment = findShader(program.fragment);
      if (vertex == nullptr || fragment == nullptr)
        return;

      if (current_program_ != draw.program) {
        context_->VSSetShader(vertex->vertex, nullptr, 0);
        context_->PSSetShader(fragment->pixel, nullptr, 0);
        current_program_ = draw.program;
      }

      ID3D11Buffer* vertex_constants = updateConstants(*vertex, uniform_data, draw.vertex_uniform_offset,
                                                       draw.vertex_uniform_size);
      ID3D11Buffer* fragment_constants = updateConstants(*fragment, uniform_data,
                                                         draw.fragment_uniform_offset,
                                                         draw.fragment_uniform_size);
      context_->VSSetConstantBuffers(0, 1, &vertex_constants);
      context_->PSSetConstantBuffers(0, 1, &fragment_constants);

      bindTextures(draw, *vertex, false);
      bindTextures(draw, *fragment, true);

      ID3D11BlendState* blend = blendState(draw.state);
      if (blend != current_blend_) {
        static constexpr float kBlendFactor[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        context_->OMSetBlendState(blend, kBlendFactor, 0xffffffff);
        current_blend_ = blend;
      }

      auto vertex_found = buffers_.find(draw.vertex_buffer);
      ID3D11InputLayout* layout = nullptr;
      if (vertex_found != buffers_.end()) {
        const Buffer& buffer = vertex_found->second;
        layout = inputLayout(program, *vertex, buffer);
        UINT stride = buffer.desc.layout.stride();
        UINT offset = 0;
        if (current_vertex_buffer_ != buffer.buffer || current_vertex_stride_ != stride) {
          context_->IASetVertexBuffers(0, 1, &buffer.buffer, &stride, &offset);
          current_vertex_buffer_ = buffer.buffer;
          current_vertex_stride_ = stride;
        }
      }
      if (layout != current_layout_ || current_layout_ == nullptr) {
        context_->IASetInputLayout(layout);
        current_layout_ = layout;
      }

      auto index_found = buffers_.find(draw.index_buffer);
      if (index_found != buffers_.end()) {
        const Buffer& buffer = index_found->second;
        DXGI_FORMAT format = buffer.desc.flags & kBufferIndex32 ? DXGI_FORMAT_R32_UINT : DXGI_FORMAT_R16_UINT;
        if (current_index_buffer_ != buffer.buffer || current_index_format_ != format) {
          context_->IASetIndexBuffer(buffer.buffer, format, 0);
          current_index_buffer_ = buffer.buffer;
          current_index_format_ = format;
        }
        context_->DrawIndexed(draw.index_count, draw.index_start, INT(draw.vertex_start));
      }
      else
        context_->Draw(draw.vertex_count, draw.vertex_start);
    }

    void dispatch(const ComputeItem& item, const std::vector<uint8_t>& uniform_data) {
      auto program_found = programs_.find(item.program);
      if (program_found == programs_.end())
        return;
      Shader* shader = findShader(program_found->second.compute);
      if (shader == nullptr)
        return;

      clearShaderResources();
      ID3D11ShaderResourceView* srvs[kMaxComputeBindings] = {};
      ID3D11UnorderedAccessView* uavs[kMaxComputeBindings] = {};
      for (int i = 0; i < kMaxComputeBindings; ++i) {
        const ComputeBinding& binding = item.bindings[i];
        if (binding.handle == kInvalidHandle)
          continue;

        ID3D11ShaderResourceView* srv = nullptr;
        ID3D11UnorderedAccessView* uav = nullptr;
        if (binding.image) {
          if (Texture* texture = findTexture(binding.handle)) {
            srv = texture->srv;
            uav = texture->uav;
          }
        }
        else {
          auto found = buffers_.find(binding.handle);
          if (found != buffers_.end()) {
            srv = found->second.srv;
            uav = found->second.uav;
          }
        }

        if (binding.access == Access::Read)
          srvs[i] = srv;
        else
          uavs[i] = uav;
      }

      ID3D11Buffer* constants = updateConstants(*shader, uniform_data, item.uniform_offset, item.uniform_size);
      context_->CSSetShader(shader->compute, nullptr, 0);
      context_->CSSetConstantBuffers(0, 1, &constants);
      context_->CSSetShaderResources(0, kMaxComputeBindings, srvs);
      context_->CSSetUnorderedAccessViews(0, kMaxComputeBindings, uavs, nullptr);
      context_->Dispatch(item.groups[0], item.groups[1], item.groups[2]);

      ID3D11ShaderResourceView* null_srvs[kMaxComputeBindings] = {};
      ID3D11UnorderedAccessView* null_uavs[kMaxComputeBindings] = {};
      context_->CSSetShaderResources(0, kMaxComputeBindings, null_srvs);
      context_->CSSetUnorderedAccessViews(0, kMaxComputeBindings, null_uavs, nullptr);
      current_vertex_buffer_ = nullptr;
      current_index_buffer_ = nullptr;
    }

    void blit(const BlitItem& item) {
      Texture* destination = findTexture(item.destination);
      Texture* source = findTexture(item.source);
      if (destination == nullptr || source == nullptr || item.width <= 0 || item.height <= 0)
        return;

      D3D11_BOX box = { UINT(item.source_x), UINT(item.source_y), 0, UINT(item.source_x + item.width),
                        UINT(item.source_y + item.height), 1 };
      context_->CopySubresourceRegion(destination->texture, 0, item.dest_x, item.dest_y, 0, source->texture,
                                      0, &box);
    }

    bool copyToStaging(ID3D11Texture2D* source, ID3D11Texture2D*& staging) {
      D3D11_TEXTURE2D_DESC desc;
      source->GetDesc(&desc);
      if (staging) {
        D3D11_TEXTURE2D_DESC staging_desc;
        staging->GetDesc(&staging_desc);
        if (staging_desc.Width != desc.Width || staging_desc.Height != desc.Height ||
            staging_desc.Format != desc.Format)
          release(staging);
      }

      if (staging == nullptr) {
        desc.Usage = D3D11_USAGE_STAGING;
        desc.BindFlags = 0;
        desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        desc.MiscFlags = 0;
        if (FAILED(device_->CreateTexture2D(&desc, nullptr, &staging)))
          return false;
      }

      context_->CopyResource(staging, source);
      return true;
    }

    void readTexture(const ReadRequest& read) {
      Texture* texture = findTexture(read.texture);
      if (texture == nullptr || read.data == nullptr || !copyToStaging(texture->texture, texture->staging))
        return;

      D3D11_MAPPED_SUBRESOURCE mapped;
      if (FAILED(context_->Map(texture->staging, 0, D3D11_MAP_READ, 0, &mapped)))
        return;

      int row_bytes = texture->desc.width * formatInfo(texture->desc.format).bytes_per_pixel;
      uint8_t* destination = static_cast<uint8_t*>(read.data);
      const uint8_t* source = static_cast<const uint8_t*>(mapped.pData);
      for (int y = 0; y < texture->desc.height; ++y)
        std::memcpy(destination + y * row_bytes, source + y * mapped.RowPitch, row_bytes);
      context_->Unmap(texture->staging, 0);

      if ((texture->desc.flags & kTextureReadBack) == 0)
        release(texture->staging);
    }

    void takeScreenshot(uint16_t idx) {
      auto found = frame_buffers_.find(idx);
      if (found == frame_buffers_.end() || !init_.screenshot_callback)
        return;

      FrameBuffer& frame_buffer = found->second;
      ID3D11Texture2D* source = frame_buffer.back_buffer;
      if (source == nullptr) {
        Texture* texture = findTexture(frame_buffer.texture);
        source = texture ? texture->texture : nullptr;
      }

      ID3D11Texture2D* staging = nullptr;
      if (source == nullptr || !copyToStaging(source, staging))
        return;

      D3D11_MAPPED_SUBRESOURCE mapped;
      if (SUCCEEDED(context_->Map(staging, 0, D3D11_MAP_READ, 0, &mapped))) {
        init_.screenshot_callback(FrameBufferHandle { idx }, static_cast<const uint8_t*>(mapped.pData),
                                  frame_buffer.width, frame_buffer.height, int(mapped.RowPitch),
                                  frame_buffer.format == TextureFormat::BGRA8);
        context_->Unmap(staging, 0);
      }
      release(staging);
    }

    Init init_;
    ID3D11Device* device_ = nullptr;
    ID3D11DeviceContext* context_ = nullptr;
    IDXGIFactory2* factory_ = nullptr;
    D3D_FEATURE_LEVEL feature_level_ = D3D_FEATURE_LEVEL_11_0;
    bool tearing_supported_ = false;
    std::atomic<bool> device_lost_ = false;
    bool basic_render_adapter_ = false;
    std::chrono::steady_clock::time_point last_adapter_check_;
    HMODULE compiler_ = nullptr;
    D3DCompileFunction compile_ = nullptr;

    ID3D11RasterizerState* rasterizer_state_ = nullptr;
    ID3D11DepthStencilState* depth_state_ = nullptr;
    std::unordered_map<uint32_t, ID3D11BlendState*> blend_states_;
    std::unordered_map<uint32_t, ID3D11SamplerState*> sampler_states_;

    std::unordered_map<uint16_t, Texture> textures_;
    std::unordered_map<uint16_t, Buffer> buffers_;
    std::unordered_map<uint16_t, Shader> shaders_;
    std::unordered_map<uint16_t, Program> programs_;
    std::unordered_map<uint16_t, FrameBuffer> frame_buffers_;
    std::mutex waitable_mutex_;

    uint16_t current_program_ = kInvalidHandle;
    ID3D11InputLayout* current_layout_ = nullptr;
    ID3D11BlendState* current_blend_ = nullptr;
    ID3D11Buffer* current_vertex_buffer_ = nullptr;
    UINT current_vertex_stride_ = 0;
    ID3D11Buffer* current_index_buffer_ = nullptr;
    DXGI_FORMAT current_index_format_ = DXGI_FORMAT_UNKNOWN;
    ID3D11RenderTargetView* current_rtv_ = nullptr;
    uint32_t bound_vertex_srvs_ = 0;
    uint32_t bound_pixel_srvs_ = 0;
    std::chrono::steady_clock::time_point cpu_start_;
  };

  std::unique_ptr<Backend> createD3D11Backend() {
    return std::make_unique<D3D11Backend>();
  }
}

#else

namespace visage::gpu {
  std::unique_ptr<Backend> createD3D11Backend() {
    return nullptr;
  }
}

#endif
