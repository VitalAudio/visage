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

#import <AppKit/AppKit.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <map>
#include <string>
#include <tuple>

namespace visage::gpu {
  namespace {
    constexpr int kMaxFramesInFlight = 3;
    constexpr int kMaxDrawables = 3;
    constexpr int kVertexBufferIndex = 30;
    constexpr uint32_t kMaxSetBytes = 4096;
    constexpr uint32_t kUniformAlignment = 256;
    constexpr uint32_t kStagingAlignment = 16;
    constexpr uint32_t kStagingChunkSize = 4 * 1024 * 1024;

    MTLPixelFormat pixelFormat(TextureFormat format) {
      switch (format) {
      case TextureFormat::RGBA8: return MTLPixelFormatRGBA8Unorm;
      case TextureFormat::BGRA8: return MTLPixelFormatBGRA8Unorm;
      case TextureFormat::R8: return MTLPixelFormatR8Unorm;
      case TextureFormat::R32F: return MTLPixelFormatR32Float;
      case TextureFormat::RGBA16F: return MTLPixelFormatRGBA16Float;
      case TextureFormat::RGBA32F: return MTLPixelFormatRGBA32Float;
      default: return MTLPixelFormatInvalid;
      }
    }

    uint32_t bytesPerPixel(TextureFormat format) {
      switch (format) {
      case TextureFormat::RGBA8:
      case TextureFormat::BGRA8:
      case TextureFormat::R32F: return 4;
      case TextureFormat::R8: return 1;
      case TextureFormat::RGBA16F: return 8;
      case TextureFormat::RGBA32F: return 16;
      default: return 0;
      }
    }

    MTLBlendFactor blendFactor(BlendFactor factor) {
      switch (factor) {
      case BlendFactor::Zero: return MTLBlendFactorZero;
      case BlendFactor::One: return MTLBlendFactorOne;
      case BlendFactor::SrcColor: return MTLBlendFactorSourceColor;
      case BlendFactor::InvSrcColor: return MTLBlendFactorOneMinusSourceColor;
      case BlendFactor::SrcAlpha: return MTLBlendFactorSourceAlpha;
      case BlendFactor::InvSrcAlpha: return MTLBlendFactorOneMinusSourceAlpha;
      case BlendFactor::DstColor: return MTLBlendFactorDestinationColor;
      case BlendFactor::InvDstColor: return MTLBlendFactorOneMinusDestinationColor;
      case BlendFactor::DstAlpha: return MTLBlendFactorDestinationAlpha;
      case BlendFactor::InvDstAlpha: return MTLBlendFactorOneMinusDestinationAlpha;
      }
      return MTLBlendFactorOne;
    }

    MTLBlendOperation blendOperation(BlendOp op) {
      switch (op) {
      case BlendOp::Add: return MTLBlendOperationAdd;
      case BlendOp::Subtract: return MTLBlendOperationSubtract;
      case BlendOp::ReverseSubtract: return MTLBlendOperationReverseSubtract;
      case BlendOp::Min: return MTLBlendOperationMin;
      case BlendOp::Max: return MTLBlendOperationMax;
      }
      return MTLBlendOperationAdd;
    }

    MTLVertexFormat vertexFormat(int num_floats) {
      switch (num_floats) {
      case 1: return MTLVertexFormatFloat;
      case 2: return MTLVertexFormatFloat2;
      case 3: return MTLVertexFormatFloat3;
      default: return MTLVertexFormatFloat4;
      }
    }

    MTLColorWriteMask writeMask(uint8_t mask) {
      MTLColorWriteMask result = MTLColorWriteMaskNone;
      if (mask & kWriteR)
        result |= MTLColorWriteMaskRed;
      if (mask & kWriteG)
        result |= MTLColorWriteMaskGreen;
      if (mask & kWriteB)
        result |= MTLColorWriteMaskBlue;
      if (mask & kWriteA)
        result |= MTLColorWriteMaskAlpha;
      return result;
    }

    MTLSamplerAddressMode addressMode(uint32_t flags, uint32_t clamp, uint32_t mirror) {
      if (flags & mirror)
        return MTLSamplerAddressModeMirrorRepeat;
      if (flags & clamp)
        return MTLSamplerAddressModeClampToEdge;
      return MTLSamplerAddressModeRepeat;
    }

    uint32_t alignUp(uint32_t value, uint32_t alignment) {
      return (value + alignment - 1) / alignment * alignment;
    }

    template<typename T>
    T& slot(std::vector<T>& list, uint16_t idx) {
      if (idx >= list.size())
        list.resize(idx + 1);
      return list[idx];
    }

    struct Texture {
      id<MTLTexture> texture = nil;
      TextureDesc desc;
    };

    struct Buffer {
      id<MTLBuffer> buffer = nil;
      BufferDesc desc;
    };

    struct Shader {
      id<MTLFunction> function = nil;
      std::vector<ShaderAttribute> attributes;
      MTLSize workgroup_size = MTLSizeMake(1, 1, 1);
    };

    using PipelineKey = std::tuple<uint32_t, uint32_t, uint32_t, bool>;

    struct Program {
      id<MTLFunction> vertex = nil;
      id<MTLFunction> fragment = nil;
      std::vector<ShaderAttribute> attributes;
      id<MTLComputePipelineState> compute = nil;
      MTLSize workgroup_size = MTLSizeMake(1, 1, 1);
      std::map<PipelineKey, id<MTLRenderPipelineState>> pipelines;
    };

    struct FrameBuffer {
      bool valid = false;
      uint16_t texture = kInvalidHandle;
      CAMetalLayer* layer = nil;
      NSView* __weak view = nil;
      id<CAMetalDrawable> drawable = nil;
      id<MTLTexture> current = nil;
      id<MTLTexture> fallback = nil;
      int width = 0;
      int height = 0;
    };

    struct StagingChunk {
      id<MTLBuffer> buffer = nil;
      uint32_t used = 0;
    };

    struct StagingAllocation {
      id<MTLBuffer> buffer = nil;
      uint32_t offset = 0;
      uint8_t* data = nullptr;
    };

    struct PendingRead {
      id<MTLBuffer> buffer = nil;
      void* data = nullptr;
      uint32_t size = 0;
    };

    struct PendingScreenshot {
      uint16_t frame_buffer = kInvalidHandle;
      id<MTLBuffer> buffer = nil;
      int width = 0;
      int height = 0;
    };
  }

  class MetalBackend : public Backend {
  public:
    bool init(const Init& init, Caps& caps) override {
      init_ = init;
      device_ = MTLCreateSystemDefaultDevice();
      if (device_ == nil)
        return false;

      queue_ = [device_ newCommandQueue];
      if (queue_ == nil)
        return false;

      frames_semaphore_ = dispatch_semaphore_create(kMaxFramesInFlight);

      caps.renderer = RendererType::Metal;
      caps.origin_bottom_left = false;
      caps.compute = true;
      caps.swap_chain = true;
      caps.max_texture_size = 16384;

      bool float_blend = [device_ supportsFamily:MTLGPUFamilyMac2] ||
                         [device_ supportsFamily:MTLGPUFamilyApple7];
      for (int i = 0; i < static_cast<int>(TextureFormat::Count); ++i) {
        auto format = static_cast<TextureFormat>(i);
        uint32_t support = kFormatTexture | kFormatRenderTarget;
        bool is_float32 = format == TextureFormat::R32F || format == TextureFormat::RGBA32F;
        if (!is_float32 || float_blend)
          support |= kFormatBlend;
        if (format != TextureFormat::BGRA8)
          support |= kFormatStorage;
        caps.formats[i] = support;
      }
      return true;
    }

    void shutdown() override {
      endUpdateEncoder();
      if (command_buffer_) {
        [command_buffer_ commit];
        command_buffer_ = nil;
      }

      if (frames_semaphore_) {
        for (int i = 0; i < kMaxFramesInFlight; ++i)
          dispatch_semaphore_wait(frames_semaphore_, DISPATCH_TIME_FOREVER);
        for (int i = 0; i < kMaxFramesInFlight; ++i)
          dispatch_semaphore_signal(frames_semaphore_);
      }

      textures_.clear();
      buffers_.clear();
      shaders_.clear();
      programs_.clear();
      frame_buffers_.clear();
      for (auto& chunks : staging_)
        chunks.clear();
      for (auto& sampler : samplers_)
        sampler = nil;
      queue_ = nil;
      device_ = nil;
    }

    bool createTexture(uint16_t idx, const TextureDesc& desc, const uint8_t* data, int pitch) override {
      MTLPixelFormat format = pixelFormat(desc.format);
      if (format == MTLPixelFormatInvalid || desc.width <= 0 || desc.height <= 0)
        return false;

      MTLTextureDescriptor* descriptor = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:format
                                                                                            width:desc.width
                                                                                           height:desc.height
                                                                                        mipmapped:NO];
      descriptor.storageMode = MTLStorageModePrivate;
      MTLTextureUsage usage = MTLTextureUsageShaderRead;
      if (desc.flags & kTextureRenderTarget)
        usage |= MTLTextureUsageRenderTarget;
      if (desc.flags & kTextureStorage)
        usage |= MTLTextureUsageShaderWrite;
      descriptor.usage = usage;

      id<MTLTexture> texture = [device_ newTextureWithDescriptor:descriptor];
      if (texture == nil)
        return false;

      Texture& entry = slot(textures_, idx);
      entry.texture = texture;
      entry.desc = desc;

      if (data) {
        uint32_t bpp = bytesPerPixel(desc.format);
        uint32_t row_size = desc.width * bpp;
        uint32_t source_pitch = pitch > 0 ? pitch : row_size;
        id<MTLBuffer> staging = [device_ newBufferWithLength:row_size * desc.height
                                                     options:MTLResourceStorageModeShared];
        copyRows(static_cast<uint8_t*>(staging.contents), row_size, data, source_pitch, row_size,
                 desc.height);

        id<MTLCommandBuffer> upload = [queue_ commandBuffer];
        id<MTLBlitCommandEncoder> blit = [upload blitCommandEncoder];
        [blit copyFromBuffer:staging
                   sourceOffset:0
              sourceBytesPerRow:row_size
            sourceBytesPerImage:row_size * desc.height
                     sourceSize:MTLSizeMake(desc.width, desc.height, 1)
                      toTexture:texture
               destinationSlice:0
               destinationLevel:0
              destinationOrigin:MTLOriginMake(0, 0, 0)];
        [blit endEncoding];
        [upload commit];
      }
      return true;
    }

    void updateTexture(uint16_t idx, int x, int y, int width, int height, const uint8_t* data,
                       int pitch) override {
      if (idx >= textures_.size() || textures_[idx].texture == nil || width <= 0 || height <= 0)
        return;

      Texture& texture = textures_[idx];
      uint32_t bpp = bytesPerPixel(texture.desc.format);
      uint32_t row_size = width * bpp;
      uint32_t source_pitch = pitch > 0 ? pitch : row_size;
      StagingAllocation staging = allocateStaging(row_size * height, kStagingAlignment);
      copyRows(staging.data, row_size, data, source_pitch, row_size, height);

      [updateEncoder() copyFromBuffer:staging.buffer
                         sourceOffset:staging.offset
                    sourceBytesPerRow:row_size
                  sourceBytesPerImage:row_size * height
                           sourceSize:MTLSizeMake(width, height, 1)
                            toTexture:texture.texture
                     destinationSlice:0
                     destinationLevel:0
                    destinationOrigin:MTLOriginMake(x, y, 0)];
    }

    void destroyTexture(uint16_t idx) override {
      if (idx < textures_.size() && textures_[idx].texture)
        textures_[idx] = {};
    }

    bool createBuffer(uint16_t idx, const BufferDesc& desc, const uint8_t* data) override {
      if (desc.size == 0)
        return false;

      id<MTLBuffer> buffer = [device_ newBufferWithLength:desc.size options:MTLResourceStorageModePrivate];
      if (buffer == nil)
        return false;

      Buffer& entry = slot(buffers_, idx);
      entry.buffer = buffer;
      entry.desc = desc;

      id<MTLCommandBuffer> upload = [queue_ commandBuffer];
      id<MTLBlitCommandEncoder> blit = [upload blitCommandEncoder];
      if (data) {
        id<MTLBuffer> staging = [device_ newBufferWithBytes:data
                                                     length:desc.size
                                                    options:MTLResourceStorageModeShared];
        [blit copyFromBuffer:staging sourceOffset:0 toBuffer:buffer destinationOffset:0 size:desc.size];
      }
      else
        [blit fillBuffer:buffer range:NSMakeRange(0, desc.size) value:0];
      [blit endEncoding];
      [upload commit];
      return true;
    }

    void updateBuffer(uint16_t idx, uint32_t offset, const uint8_t* data, uint32_t size) override {
      if (idx >= buffers_.size() || buffers_[idx].buffer == nil || size == 0)
        return;

      Buffer& buffer = buffers_[idx];
      if (offset >= buffer.desc.size)
        return;
      size = std::min(size, buffer.desc.size - offset);

      StagingAllocation staging = allocateStaging(size, kStagingAlignment);
      std::memcpy(staging.data, data, size);
      [updateEncoder() copyFromBuffer:staging.buffer
                         sourceOffset:staging.offset
                             toBuffer:buffer.buffer
                    destinationOffset:offset
                                 size:size];
    }

    void destroyBuffer(uint16_t idx) override {
      if (idx < buffers_.size() && buffers_[idx].buffer)
        buffers_[idx] = {};
    }

    bool createShader(uint16_t idx, const ShaderInfo& info) override {
      if (info.code_type != ShaderCode::Msl || info.code.empty())
        return false;

      NSString* source = [[NSString alloc] initWithBytes:info.code.data()
                                                  length:info.code.size()
                                                encoding:NSUTF8StringEncoding];
      NSError* error = nil;
      id<MTLLibrary> library = [device_ newLibraryWithSource:source options:nil error:&error];
      if (library == nil) {
        log("Metal shader compile failed", error);
        return false;
      }

      NSArray<NSString*>* names = library.functionNames;
      NSString* name = [names containsObject:@"main0"] ? @"main0" : names.firstObject;
      id<MTLFunction> function = name ? [library newFunctionWithName:name] : nil;
      if (function == nil)
        return false;

      Shader& shader = slot(shaders_, idx);
      shader.function = function;
      shader.attributes = info.attributes;
      shader.workgroup_size = MTLSizeMake(info.workgroup_size[0], info.workgroup_size[1],
                                          info.workgroup_size[2]);
      return true;
    }

    void destroyShader(uint16_t idx) override {
      if (idx < shaders_.size() && shaders_[idx].function)
        shaders_[idx] = {};
    }

    bool createProgram(uint16_t idx, uint16_t vertex, uint16_t fragment) override {
      if (vertex >= shaders_.size() || fragment >= shaders_.size() ||
          shaders_[vertex].function == nil || shaders_[fragment].function == nil)
        return false;

      Program& program = slot(programs_, idx);
      program = {};
      program.vertex = shaders_[vertex].function;
      program.fragment = shaders_[fragment].function;
      program.attributes = shaders_[vertex].attributes;
      return true;
    }

    bool createComputeProgram(uint16_t idx, uint16_t compute) override {
      if (compute >= shaders_.size() || shaders_[compute].function == nil)
        return false;

      NSError* error = nil;
      id<MTLComputePipelineState> pipeline = [device_ newComputePipelineStateWithFunction:shaders_[compute].function
                                                                                     error:&error];
      if (pipeline == nil) {
        log("Metal compute pipeline failed", error);
        return false;
      }

      Program& program = slot(programs_, idx);
      program = {};
      program.compute = pipeline;
      program.workgroup_size = shaders_[compute].workgroup_size;
      return true;
    }

    void destroyProgram(uint16_t idx) override {
      if (idx < programs_.size() && (programs_[idx].vertex || programs_[idx].compute))
        programs_[idx] = {};
    }

    bool createFrameBuffer(uint16_t idx, uint16_t texture) override {
      if (texture >= textures_.size() || textures_[texture].texture == nil)
        return false;

      FrameBuffer& frame_buffer = slot(frame_buffers_, idx);
      frame_buffer = {};
      frame_buffer.valid = true;
      frame_buffer.texture = texture;
      frame_buffer.width = textures_[texture].desc.width;
      frame_buffer.height = textures_[texture].desc.height;
      return true;
    }

    bool createSwapChain(uint16_t idx, void* window_handle, void*, int width, int height,
                         TextureFormat) override {
      NSObject* handle = (__bridge NSObject*)window_handle;
      CAMetalLayer* layer = metalLayer(handle);
      if (layer == nil)
        return false;

      layer.device = device_;
      layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
      layer.magnificationFilter = kCAFilterNearest;
      layer.displaySyncEnabled = init_.vsync;
      layer.maximumDrawableCount = kMaxDrawables;
      if (init_.color_managed)
        layer.colorspace = srgbColorSpace();
      if (width > 0 && height > 0)
        layer.drawableSize = CGSizeMake(width, height);

      FrameBuffer& frame_buffer = slot(frame_buffers_, idx);
      frame_buffer = {};
      frame_buffer.valid = true;
      frame_buffer.layer = layer;
      frame_buffer.view = handleView(handle);
      frame_buffer.width = width;
      frame_buffer.height = height;
      return true;
    }

    void resizeSwapChain(uint16_t idx, int width, int height) override {
      if (idx >= frame_buffers_.size() || frame_buffers_[idx].layer == nil || width <= 0 || height <= 0)
        return;

      FrameBuffer& frame_buffer = frame_buffers_[idx];
      frame_buffer.width = width;
      frame_buffer.height = height;
      frame_buffer.layer.drawableSize = CGSizeMake(width, height);
    }

    void destroyFrameBuffer(uint16_t idx) override {
      if (idx < frame_buffers_.size() && frame_buffers_[idx].valid)
        frame_buffers_[idx] = {};
    }

    bool waitForFrame(uint16_t, int) override { return true; }

    void beginFrame(uint32_t) override {
      @autoreleasepool {
        startFrame();
      }
    }

    void submitFrame(const FrameData& frame, Stats& stats) override {
      @autoreleasepool {
        finishFrame(frame, stats);
      }
    }

  private:
    void startFrame() {
      dispatch_semaphore_wait(frames_semaphore_, DISPATCH_TIME_FOREVER);
      long error = command_buffer_error_.exchange(0);
      if (error) {
        std::string message = "Metal command buffer failed with error " + std::to_string(error);
        log(message.c_str(), nil);
      }

      frame_slot_ = (frame_slot_ + 1) % kMaxFramesInFlight;
      for (StagingChunk& chunk : staging_[frame_slot_])
        chunk.used = 0;

      command_buffer_ = [queue_ commandBuffer];
      dispatch_semaphore_t semaphore = frames_semaphore_;
      std::atomic<double>* gpu_ms = &gpu_frame_ms_;
      std::atomic<long>* command_buffer_error = &command_buffer_error_;
      [command_buffer_ addCompletedHandler:^(id<MTLCommandBuffer> buffer) {
        if (buffer.status == MTLCommandBufferStatusError)
          command_buffer_error->store(buffer.error ? buffer.error.code : -1);
        if (buffer.GPUEndTime > buffer.GPUStartTime)
          gpu_ms->store((buffer.GPUEndTime - buffer.GPUStartTime) * 1000.0);
        dispatch_semaphore_signal(semaphore);
      }];
    }

    void finishFrame(const FrameData& frame, Stats& stats) {
      endUpdateEncoder();

      std::vector<uint16_t> presented;
      for (const ViewData& view : frame.views)
        submitView(frame, view, presented);

      std::vector<PendingRead> reads;
      std::vector<PendingScreenshot> screenshots;
      encodeReads(frame, reads);
      encodeScreenshots(frame, screenshots);

      for (uint16_t idx : presented) {
        if (frame_buffers_[idx].drawable)
          [command_buffer_ presentDrawable:frame_buffers_[idx].drawable];
      }

      id<MTLCommandBuffer> command_buffer = command_buffer_;
      command_buffer_ = nil;
      [command_buffer commit];

      for (uint16_t idx : presented) {
        frame_buffers_[idx].drawable = nil;
        frame_buffers_[idx].current = nil;
      }

      if (!reads.empty() || !screenshots.empty()) {
        [command_buffer waitUntilCompleted];
        for (const PendingRead& read : reads)
          std::memcpy(read.data, read.buffer.contents, read.size);
        for (const PendingScreenshot& screenshot : screenshots) {
          if (init_.screenshot_callback) {
            init_.screenshot_callback({ screenshot.frame_buffer },
                                      static_cast<const uint8_t*>(screenshot.buffer.contents),
                                      screenshot.width, screenshot.height, screenshot.width * 4, true);
          }
        }
      }

      stats.gpu_frame_ms = gpu_frame_ms_.load();
    }

    static void copyRows(uint8_t* dest, uint32_t dest_pitch, const uint8_t* source,
                         uint32_t source_pitch, uint32_t row_size, int rows) {
      if (dest_pitch == source_pitch && dest_pitch == row_size) {
        std::memcpy(dest, source, static_cast<size_t>(row_size) * rows);
        return;
      }
      for (int y = 0; y < rows; ++y)
        std::memcpy(dest + y * dest_pitch, source + y * source_pitch, row_size);
    }

    void log(const char* message, NSError* error) {
      if (!init_.log_callback)
        return;
      std::string text = message;
      if (error)
        text += std::string(": ") + error.localizedDescription.UTF8String;
      init_.log_callback(text.c_str());
    }

    static CGColorSpaceRef srgbColorSpace() {
      static CGColorSpaceRef color_space = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
      return color_space;
    }

    static NSView* handleView(NSObject* handle) {
      if ([handle isKindOfClass:[NSView class]])
        return (NSView*)handle;
      if ([handle isKindOfClass:[NSWindow class]])
        return ((NSWindow*)handle).contentView;
      return nil;
    }

    static bool occluded(const FrameBuffer& frame_buffer) {
      NSWindow* window = frame_buffer.view.window;
      return window && (window.occlusionState & NSWindowOcclusionStateVisible) == 0;
    }

    static CAMetalLayer* metalLayer(NSObject* handle) {
      if (handle == nil)
        return nil;
      if ([handle isKindOfClass:[CAMetalLayer class]])
        return (CAMetalLayer*)handle;

      NSView* view = handleView(handle);
      if (view == nil)
        return nil;

      __block CAMetalLayer* result = nil;
      void (^set_layer)(void) = ^{
        if ([view.layer isKindOfClass:[CAMetalLayer class]])
          result = (CAMetalLayer*)view.layer;
        else {
          [view setWantsLayer:YES];
          result = [CAMetalLayer layer];
          [view setLayer:result];
        }
      };

      if ([NSThread isMainThread])
        set_layer();
      else
        dispatch_sync(dispatch_get_main_queue(), set_layer);
      return result;
    }

    StagingAllocation allocateStaging(uint32_t size, uint32_t alignment) {
      std::vector<StagingChunk>& chunks = staging_[frame_slot_];
      for (StagingChunk& chunk : chunks) {
        uint32_t offset = alignUp(chunk.used, alignment);
        if (offset + size <= chunk.buffer.length) {
          chunk.used = offset + size;
          return { chunk.buffer, offset, static_cast<uint8_t*>(chunk.buffer.contents) + offset };
        }
      }

      StagingChunk chunk;
      chunk.buffer = [device_ newBufferWithLength:std::max(size, kStagingChunkSize)
                                          options:MTLResourceStorageModeShared |
                                                  MTLResourceCPUCacheModeWriteCombined];
      chunk.used = size;
      chunks.push_back(chunk);
      return { chunk.buffer, 0, static_cast<uint8_t*>(chunk.buffer.contents) };
    }

    id<MTLBlitCommandEncoder> updateEncoder() {
      if (update_encoder_ == nil)
        update_encoder_ = [command_buffer_ blitCommandEncoder];
      return update_encoder_;
    }

    void endUpdateEncoder() {
      if (update_encoder_) {
        [update_encoder_ endEncoding];
        update_encoder_ = nil;
      }
    }

    id<MTLSamplerState> sampler(uint32_t flags) {
      flags &= kSamplerMask;
      if (samplers_[flags])
        return samplers_[flags];

      MTLSamplerDescriptor* descriptor = [[MTLSamplerDescriptor alloc] init];
      descriptor.minFilter = (flags & kSamplerMinPoint) ? MTLSamplerMinMagFilterNearest
                                                        : MTLSamplerMinMagFilterLinear;
      descriptor.magFilter = (flags & kSamplerMagPoint) ? MTLSamplerMinMagFilterNearest
                                                        : MTLSamplerMinMagFilterLinear;
      descriptor.mipFilter = MTLSamplerMipFilterNotMipmapped;
      descriptor.sAddressMode = addressMode(flags, kSamplerUClamp, kSamplerUMirror);
      descriptor.tAddressMode = addressMode(flags, kSamplerVClamp, kSamplerVMirror);
      samplers_[flags] = [device_ newSamplerStateWithDescriptor:descriptor];
      return samplers_[flags];
    }

    id<MTLTexture> target(uint16_t frame_buffer_idx, std::vector<uint16_t>& presented) {
      if (frame_buffer_idx >= frame_buffers_.size() || !frame_buffers_[frame_buffer_idx].valid)
        return nil;

      FrameBuffer& frame_buffer = frame_buffers_[frame_buffer_idx];
      if (frame_buffer.layer == nil) {
        if (frame_buffer.texture >= textures_.size())
          return nil;
        return textures_[frame_buffer.texture].texture;
      }

      if (frame_buffer.current)
        return frame_buffer.current;

      if (!occluded(frame_buffer))
        frame_buffer.drawable = [frame_buffer.layer nextDrawable];

      if (frame_buffer.drawable)
        frame_buffer.current = frame_buffer.drawable.texture;
      else
        frame_buffer.current = fallbackTexture(frame_buffer);
      if (frame_buffer.current)
        presented.push_back(frame_buffer_idx);
      return frame_buffer.current;
    }

    id<MTLTexture> fallbackTexture(FrameBuffer& frame_buffer) {
      CGSize size = frame_buffer.layer.drawableSize;
      if (size.width < 1 || size.height < 1)
        return nil;

      id<MTLTexture> fallback = frame_buffer.fallback;
      if (fallback && fallback.width == size.width && fallback.height == size.height &&
          fallback.pixelFormat == frame_buffer.layer.pixelFormat)
        return fallback;

      MTLTextureDescriptor* descriptor =
          [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:frame_buffer.layer.pixelFormat
                                                             width:size.width
                                                            height:size.height
                                                         mipmapped:NO];
      descriptor.storageMode = MTLStorageModePrivate;
      descriptor.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
      frame_buffer.fallback = [device_ newTextureWithDescriptor:descriptor];
      return frame_buffer.fallback;
    }

    id<MTLRenderPipelineState> renderPipeline(Program& program, const RenderState& state,
                                              MTLPixelFormat format, const Buffer* vertex_buffer) {
      uint32_t layout_hash = vertex_buffer ? vertex_buffer->desc.layout.hash() : 0;
      PipelineKey key = { state.key(), static_cast<uint32_t>(format), layout_hash, vertex_buffer != nullptr };
      auto found = program.pipelines.find(key);
      if (found != program.pipelines.end())
        return found->second;

      MTLRenderPipelineDescriptor* descriptor = [[MTLRenderPipelineDescriptor alloc] init];
      descriptor.vertexFunction = program.vertex;
      descriptor.fragmentFunction = program.fragment;

      MTLRenderPipelineColorAttachmentDescriptor* color = descriptor.colorAttachments[0];
      color.pixelFormat = format;
      color.writeMask = writeMask(state.write_mask);
      color.blendingEnabled = state.blend;
      color.sourceRGBBlendFactor = blendFactor(state.src_rgb);
      color.destinationRGBBlendFactor = blendFactor(state.dst_rgb);
      color.sourceAlphaBlendFactor = blendFactor(state.src_alpha);
      color.destinationAlphaBlendFactor = blendFactor(state.dst_alpha);
      color.rgbBlendOperation = blendOperation(state.op_rgb);
      color.alphaBlendOperation = blendOperation(state.op_alpha);

      if (vertex_buffer && !program.attributes.empty()) {
        const VertexLayout& layout = vertex_buffer->desc.layout;
        MTLVertexDescriptor* vertex_descriptor = [MTLVertexDescriptor vertexDescriptor];
        for (const ShaderAttribute& attribute : program.attributes) {
          int location = static_cast<int>(attribute.attrib);
          MTLVertexAttributeDescriptor* attribute_descriptor = vertex_descriptor.attributes[location];
          attribute_descriptor.bufferIndex = kVertexBufferIndex;
          if (layout.has(attribute.attrib)) {
            attribute_descriptor.format = vertexFormat(layout.count(attribute.attrib));
            attribute_descriptor.offset = layout.offset(attribute.attrib);
          }
          else {
            attribute_descriptor.format = MTLVertexFormatFloat;
            attribute_descriptor.offset = 0;
          }
        }
        vertex_descriptor.layouts[kVertexBufferIndex].stride = layout.stride();
        vertex_descriptor.layouts[kVertexBufferIndex].stepFunction = MTLVertexStepFunctionPerVertex;
        descriptor.vertexDescriptor = vertex_descriptor;
      }

      NSError* error = nil;
      id<MTLRenderPipelineState> pipeline = [device_ newRenderPipelineStateWithDescriptor:descriptor
                                                                                    error:&error];
      if (pipeline == nil)
        log("Metal render pipeline failed", error);
      program.pipelines[key] = pipeline;
      return pipeline;
    }

    id<MTLRenderCommandEncoder> beginRenderPass(id<MTLTexture> texture, const ViewData& view) {
      MTLRenderPassDescriptor* pass = [MTLRenderPassDescriptor renderPassDescriptor];
      pass.colorAttachments[0].texture = texture;
      pass.colorAttachments[0].loadAction = MTLLoadActionLoad;
      pass.colorAttachments[0].storeAction = MTLStoreActionStore;

      id<MTLRenderCommandEncoder> encoder = [command_buffer_ renderCommandEncoderWithDescriptor:pass];
      [encoder setViewport:(MTLViewport) { static_cast<double>(view.x), static_cast<double>(view.y),
                                           static_cast<double>(view.width),
                                           static_cast<double>(view.height), 0.0, 1.0 }];
      return encoder;
    }

    void encodeDraw(id<MTLRenderCommandEncoder> encoder, const FrameData& frame, const DrawItem& draw,
                    MTLPixelFormat format) {
      if (draw.program >= programs_.size() || programs_[draw.program].vertex == nil)
        return;

      const Buffer* vertex_buffer = nullptr;
      if (draw.vertex_buffer < buffers_.size() && buffers_[draw.vertex_buffer].buffer)
        vertex_buffer = &buffers_[draw.vertex_buffer];

      id<MTLRenderPipelineState> pipeline = renderPipeline(programs_[draw.program], draw.state,
                                                           format, vertex_buffer);
      if (pipeline == nil)
        return;

      [encoder setRenderPipelineState:pipeline];
      if (vertex_buffer)
        [encoder setVertexBuffer:vertex_buffer->buffer offset:0 atIndex:kVertexBufferIndex];

      if (draw.vertex_uniform_size)
        setUniforms(encoder, frame, draw.vertex_uniform_offset, draw.vertex_uniform_size, true);
      if (draw.fragment_uniform_size)
        setUniforms(encoder, frame, draw.fragment_uniform_offset, draw.fragment_uniform_size, false);

      for (int stage = 0; stage < kMaxTextureStages; ++stage) {
        const TextureBinding& binding = draw.textures[stage];
        if (binding.texture >= textures_.size() || textures_[binding.texture].texture == nil)
          continue;

        id<MTLTexture> texture = textures_[binding.texture].texture;
        id<MTLSamplerState> sampler_state = sampler(binding.sampler_flags);
        [encoder setVertexTexture:texture atIndex:stage];
        [encoder setVertexSamplerState:sampler_state atIndex:stage];
        [encoder setFragmentTexture:texture atIndex:stage];
        [encoder setFragmentSamplerState:sampler_state atIndex:stage];
      }

      if (draw.index_buffer < buffers_.size() && buffers_[draw.index_buffer].buffer) {
        const Buffer& index_buffer = buffers_[draw.index_buffer];
        bool index32 = index_buffer.desc.flags & kBufferIndex32;
        [encoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                            indexCount:draw.index_count
                             indexType:index32 ? MTLIndexTypeUInt32 : MTLIndexTypeUInt16
                           indexBuffer:index_buffer.buffer
                     indexBufferOffset:draw.index_start * (index32 ? 4 : 2)
                         instanceCount:1
                            baseVertex:draw.vertex_start
                          baseInstance:0];
      }
      else if (draw.vertex_count) {
        [encoder drawPrimitives:MTLPrimitiveTypeTriangle
                    vertexStart:draw.vertex_start
                    vertexCount:draw.vertex_count];
      }
    }

    void setUniforms(id<MTLRenderCommandEncoder> encoder, const FrameData& frame, uint32_t offset,
                     uint32_t size, bool vertex) {
      const uint8_t* data = frame.uniform_data.data() + offset;
      if (size <= kMaxSetBytes) {
        if (vertex)
          [encoder setVertexBytes:data length:size atIndex:0];
        else
          [encoder setFragmentBytes:data length:size atIndex:0];
        return;
      }

      StagingAllocation staging = allocateStaging(size, kUniformAlignment);
      std::memcpy(staging.data, data, size);
      if (vertex)
        [encoder setVertexBuffer:staging.buffer offset:staging.offset atIndex:0];
      else
        [encoder setFragmentBuffer:staging.buffer offset:staging.offset atIndex:0];
    }

    void encodeCompute(const FrameData& frame, const ComputeItem& compute) {
      if (compute.program >= programs_.size() || programs_[compute.program].compute == nil)
        return;

      const Program& program = programs_[compute.program];
      id<MTLComputeCommandEncoder> encoder = [command_buffer_ computeCommandEncoder];
      [encoder setComputePipelineState:program.compute];

      if (compute.uniform_size) {
        const uint8_t* data = frame.uniform_data.data() + compute.uniform_offset;
        if (compute.uniform_size <= kMaxSetBytes)
          [encoder setBytes:data length:compute.uniform_size atIndex:0];
        else {
          StagingAllocation staging = allocateStaging(compute.uniform_size, kUniformAlignment);
          std::memcpy(staging.data, data, compute.uniform_size);
          [encoder setBuffer:staging.buffer offset:staging.offset atIndex:0];
        }
      }

      for (int stage = 0; stage < kMaxComputeBindings; ++stage) {
        const ComputeBinding& binding = compute.bindings[stage];
        if (binding.image) {
          if (binding.handle < textures_.size() && textures_[binding.handle].texture)
            [encoder setTexture:textures_[binding.handle].texture atIndex:stage];
        }
        else if (binding.handle < buffers_.size() && buffers_[binding.handle].buffer)
          [encoder setBuffer:buffers_[binding.handle].buffer offset:0 atIndex:1 + stage];
      }

      [encoder dispatchThreadgroups:MTLSizeMake(compute.groups[0], compute.groups[1], compute.groups[2])
              threadsPerThreadgroup:program.workgroup_size];
      [encoder endEncoding];
    }

    void encodeBlit(const BlitItem& blit) {
      if (blit.source >= textures_.size() || blit.destination >= textures_.size())
        return;
      id<MTLTexture> source = textures_[blit.source].texture;
      id<MTLTexture> destination = textures_[blit.destination].texture;
      if (source == nil || destination == nil || blit.width <= 0 || blit.height <= 0)
        return;

      id<MTLBlitCommandEncoder> encoder = [command_buffer_ blitCommandEncoder];
      [encoder copyFromTexture:source
                   sourceSlice:0
                   sourceLevel:0
                  sourceOrigin:MTLOriginMake(blit.source_x, blit.source_y, 0)
                    sourceSize:MTLSizeMake(blit.width, blit.height, 1)
                     toTexture:destination
              destinationSlice:0
              destinationLevel:0
             destinationOrigin:MTLOriginMake(blit.dest_x, blit.dest_y, 0)];
      [encoder endEncoding];
    }

    void submitView(const FrameData& frame, const ViewData& view, std::vector<uint16_t>& presented) {
      bool wants_screenshot = std::find(frame.screenshots.begin(), frame.screenshots.end(),
                                        view.frame_buffer) != frame.screenshots.end();
      if (wants_screenshot && view.frame_buffer < frame_buffers_.size() &&
          frame_buffers_[view.frame_buffer].layer && frame_buffers_[view.frame_buffer].current == nil)
        frame_buffers_[view.frame_buffer].layer.framebufferOnly = NO;

      id<MTLTexture> texture = nil;
      bool target_resolved = false;
      id<MTLRenderCommandEncoder> render = nil;
      for (const ViewItem& item : view.items) {
        if (item.type == ViewItem::Draw) {
          if (!target_resolved) {
            texture = target(view.frame_buffer, presented);
            target_resolved = true;
          }
          if (texture == nil)
            continue;
          if (render == nil)
            render = beginRenderPass(texture, view);
          encodeDraw(render, frame, frame.draws[item.index], texture.pixelFormat);
          continue;
        }

        if (render) {
          [render endEncoding];
          render = nil;
        }
        if (item.type == ViewItem::Compute)
          encodeCompute(frame, frame.computes[item.index]);
        else
          encodeBlit(frame.blits[item.index]);
      }

      if (render)
        [render endEncoding];
    }

    void encodeReads(const FrameData& frame, std::vector<PendingRead>& reads) {
      id<MTLBlitCommandEncoder> encoder = nil;
      for (const ReadRequest& request : frame.reads) {
        if (request.texture >= textures_.size() || textures_[request.texture].texture == nil ||
            request.data == nullptr)
          continue;

        const Texture& texture = textures_[request.texture];
        uint32_t row_size = texture.desc.width * bytesPerPixel(texture.desc.format);
        uint32_t size = row_size * texture.desc.height;
        id<MTLBuffer> buffer = [device_ newBufferWithLength:size options:MTLResourceStorageModeShared];
        if (encoder == nil)
          encoder = [command_buffer_ blitCommandEncoder];
        [encoder copyFromTexture:texture.texture
                         sourceSlice:0
                         sourceLevel:0
                        sourceOrigin:MTLOriginMake(0, 0, 0)
                          sourceSize:MTLSizeMake(texture.desc.width, texture.desc.height, 1)
                            toBuffer:buffer
                   destinationOffset:0
              destinationBytesPerRow:row_size
            destinationBytesPerImage:size];
        reads.push_back({ buffer, request.data, size });
      }
      if (encoder)
        [encoder endEncoding];
    }

    void encodeScreenshots(const FrameData& frame, std::vector<PendingScreenshot>& screenshots) {
      id<MTLBlitCommandEncoder> encoder = nil;
      for (uint16_t idx : frame.screenshots) {
        if (idx >= frame_buffers_.size() || frame_buffers_[idx].layer == nil)
          continue;

        FrameBuffer& frame_buffer = frame_buffers_[idx];
        frame_buffer.layer.framebufferOnly = YES;
        id<MTLTexture> texture = frame_buffer.current;
        if (texture == nil || texture.framebufferOnly) {
          log("Metal screenshot skipped: swap chain was not drawn this frame", nil);
          continue;
        }

        int width = static_cast<int>(texture.width);
        int height = static_cast<int>(texture.height);
        uint32_t row_size = width * 4;
        id<MTLBuffer> buffer = [device_ newBufferWithLength:row_size * height
                                                    options:MTLResourceStorageModeShared];
        if (encoder == nil)
          encoder = [command_buffer_ blitCommandEncoder];
        [encoder copyFromTexture:texture
                         sourceSlice:0
                         sourceLevel:0
                        sourceOrigin:MTLOriginMake(0, 0, 0)
                          sourceSize:MTLSizeMake(width, height, 1)
                            toBuffer:buffer
                   destinationOffset:0
              destinationBytesPerRow:row_size
            destinationBytesPerImage:row_size * height];
        screenshots.push_back({ idx, buffer, width, height });
      }
      if (encoder)
        [encoder endEncoding];
    }

    Init init_;
    id<MTLDevice> device_ = nil;
    id<MTLCommandQueue> queue_ = nil;
    id<MTLCommandBuffer> command_buffer_ = nil;
    id<MTLBlitCommandEncoder> update_encoder_ = nil;
    dispatch_semaphore_t frames_semaphore_ = nil;
    int frame_slot_ = 0;
    std::atomic<double> gpu_frame_ms_ = 0.0;
    std::atomic<long> command_buffer_error_ = 0;

    std::vector<Texture> textures_;
    std::vector<Buffer> buffers_;
    std::vector<Shader> shaders_;
    std::vector<Program> programs_;
    std::vector<FrameBuffer> frame_buffers_;
    std::vector<StagingChunk> staging_[kMaxFramesInFlight];
    id<MTLSamplerState> samplers_[kSamplerMask + 1] {};

  };

  std::unique_ptr<Backend> createMetalBackend() {
    return std::make_unique<MetalBackend>();
  }
}
