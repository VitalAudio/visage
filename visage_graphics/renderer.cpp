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

#include "renderer.h"

#include "visage_utils/string_utils.h"

#include <bgfx/bgfx.h>
#include <bimg/bimg.h>
#include <bx/allocator.h>

#if VISAGE_EMSCRIPTEN
#include <emscripten/html5_webgl.h>
#endif

namespace visage {
  class GraphicsCallbackHandler : public bgfx::CallbackI {
    void fatal(const char* file_path, uint16_t line, bgfx::Fatal::Enum _code, const char* error) override {
      VISAGE_LOG(String(file_path) + String(" (") + line + String(") "));
      VISAGE_LOG(error);
      VISAGE_LOG("Graphics fatal error");
      VISAGE_ASSERT(false);
    }

    void traceVargs(const char* file_path, uint16_t line, const char* format, va_list arg_list) override {
#if VISAGE_GRAPHICS_DEBUG_LOGGING
      visage::debugLogArgs(file_path, line, format, arg_list);
#endif
    }

    void profilerBegin(const char*, uint32_t, const char*, uint16_t) override { }
    void profilerBeginLiteral(const char*, uint32_t, const char*, uint16_t) override { }
    void profilerEnd() override { }
    uint32_t cacheReadSize(uint64_t) override { return 0; }
    bool cacheRead(uint64_t, void*, uint32_t) override { return false; }
    void cacheWrite(uint64_t, const void*, uint32_t) override { }

    void screenShot(const char* file_path, uint32_t width, uint32_t height, uint32_t pitch,
                    bgfx::TextureFormat::Enum format, const void* data, uint32_t size, bool y_flip) override {
      Renderer::instance().setScreenshotData(static_cast<const uint8_t*>(data), width, height,
                                             pitch, format, y_flip);
    }

    void captureBegin(uint32_t, uint32_t, uint32_t, bgfx::TextureFormat::Enum, bool) override { }
    void captureEnd() override { }
    void captureFrame(const void*, uint32_t) override { }
  };

  static constexpr uint32_t kUnusedTransientBufferSize = 16;

  static constexpr uint32_t resetFlags() {
#if VISAGE_WINDOWS
    return BGFX_RESET_FLIP_AFTER_RENDER;
#elif VISAGE_MAC
    return BGFX_RESET_FLIP_AFTER_RENDER | BGFX_RESET_VSYNC;
#elif VISAGE_LINUX
    return BGFX_RESET_VSYNC;
#else
    return 0;
#endif
  }

#if VISAGE_EMSCRIPTEN
  static bool enableWebGlExtension(const char* name) {
    return emscripten_webgl_enable_extension(emscripten_webgl_get_current_context(), name);
  }
#else
  static bool renderTargetSupported(bgfx::TextureFormat::Enum format) {
    static constexpr uint32_t kRequired = BGFX_CAPS_FORMAT_TEXTURE_2D | BGFX_CAPS_FORMAT_TEXTURE_FRAMEBUFFER;
    return (bgfx::getCaps()->formats[format] & kRequired) == kRequired;
  }
#endif

  Renderer& Renderer::instance() {
    static Renderer renderer;
    return renderer;
  }

  Renderer::Renderer() : Thread("Renderer Thread") { }

  Renderer::~Renderer() {
    stop();
  }

  void Renderer::startRenderThread() {
#if VISAGE_BACKGROUND_GRAPHICS_THREAD
    start();
    while (!render_thread_started_.load())
      yield();
#endif
  }

  void Renderer::run() {
    render();
  }

  void Renderer::render() {
    static constexpr int kRenderTimeout = 100;
    render_thread_started_ = bgfx::renderFrame() == bgfx::RenderFrame::NoContext;

    while (shouldRun())
      bgfx::renderFrame(kRenderTimeout);
  }

  void Renderer::initialize(void* model_window, void* display) {
    if (display_ == nullptr)
      display_ = display;

    if (initialized_)
      return;

    callback_handler_ = std::make_unique<GraphicsCallbackHandler>();
    initialized_ = true;
    startRenderThread();

    bgfx::Init bgfx_init;
    bgfx_init.swapChain.numBackBuffers = 1;
    bgfx_init.swapChain.width = 0;
    bgfx_init.swapChain.height = 0;
    bgfx_init.swapChain.formatColor = bgfx::TextureFormat::RGBA8;
    bgfx_init.callback = callback_handler_.get();

    bgfx_init.swapChain.ndt = display;
    bgfx_init.swapChain.nwh = model_window;
    bgfx_init.platformData.type = bgfx::NativeWindowHandleType::Default;

    bgfx::RendererType::Enum supported_renderers[bgfx::RendererType::Count];
    uint8_t num_supported = bgfx::getSupportedRenderers(bgfx::RendererType::Count, supported_renderers);

#if VISAGE_WINDOWS
    bgfx_init.type = bgfx::RendererType::Direct3D11;
#if USE_DIRECTX12
    for (int i = 0; i < num_supported; ++i) {
      if (supported_renderers[i] == bgfx::RendererType::Direct3D12)
        bgfx_init.type = bgfx::RendererType::Direct3D12;
    }
#endif
#elif VISAGE_MAC
    bgfx_init.type = bgfx::RendererType::Metal;
    bgfx_init.swapChain.width = 1;
    bgfx_init.swapChain.height = 1;
#elif VISAGE_LINUX
    bgfx_init.type = bgfx::RendererType::Vulkan;
#elif VISAGE_EMSCRIPTEN
    bgfx_init.type = bgfx::RendererType::OpenGLES;
#endif

    bgfx_init.reset = resetFlags();
    bgfx_init.limits.maxTransientVbSize = kUnusedTransientBufferSize;
    bgfx_init.limits.maxTransientIbSize = kUnusedTransientBufferSize;

    for (int i = 0; i < num_supported && !supported_; ++i)
      supported_ = supported_renderers[i] == bgfx_init.type;

    if (!supported_) {
      VISAGE_ASSERT(false);
      std::string renderer_name = bgfx::getRendererName(bgfx_init.type);
      error_message_ = renderer_name + " is required and not supported on this computer.";
    }

    bgfx::init(bgfx_init);
    VISAGE_ASSERT(bgfx::getRendererType() == bgfx_init.type);
    swap_chain_supported_ = bgfx::getCaps()->supported & BGFX_CAPS_SWAP_CHAIN;

#if VISAGE_EMSCRIPTEN
    bool color_buffer_float = enableWebGlExtension("EXT_color_buffer_float");
    float_render_targets_supported_ = color_buffer_float ||
                                      enableWebGlExtension("EXT_color_buffer_half_float");
    float32_blend_supported_ = color_buffer_float && enableWebGlExtension("EXT_float_blend");
#else
    float_render_targets_supported_ = renderTargetSupported(bgfx::TextureFormat::RGBA16F);
    float32_blend_supported_ = renderTargetSupported(bgfx::TextureFormat::R32F);
#endif
  }

  void Renderer::resetResolution(int width, int height) {
#if VISAGE_MAC
    bgfx::SwapChain swap_chain;
    swap_chain.width = width;
    swap_chain.height = height;
    bgfx::reset(resetFlags(), &swap_chain);
#endif
  }

  void Renderer::setScreenshotData(const uint8_t* data, int width, int height, int pitch,
                                   int format, bool y_flip) {
    if (format == bgfx::TextureFormat::RGBA8 || format == bgfx::TextureFormat::BGRA8) {
      screenshot_ = Screenshot(data, width, height, pitch, format == bgfx::TextureFormat::BGRA8, y_flip);
      return;
    }

    bx::DefaultAllocator allocator;
    std::vector<uint8_t> converted(width * height * 4);
    auto source_format = static_cast<bimg::TextureFormat::Enum>(format);
    VISAGE_ASSERT(pitch * 8 == width * bimg::getBitsPerPixel(source_format));
    if (bimg::imageConvert(&allocator, converted.data(), bimg::TextureFormat::RGBA8, data,
                           source_format, width, height, 1))
      screenshot_ = Screenshot(converted.data(), width, height, width * 4, false, y_flip);
    else
      screenshot_ = Screenshot();
  }
}
