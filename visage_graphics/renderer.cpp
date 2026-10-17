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
#include <bgfx/platform.h>

#if VISAGE_ANDROID || defined(__ANDROID__)
#include <android/native_window.h>
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
                    const void* data, uint32_t size, bool y_flip) override {
      Renderer::instance().setScreenshotData(static_cast<const uint8_t*>(data), width, height, pitch, true);
    }

    void captureBegin(uint32_t, uint32_t, uint32_t, bgfx::TextureFormat::Enum, bool) override { }
    void captureEnd() override { }
    void captureFrame(const void*, uint32_t) override { }
  };

  static bool vsync_enabled_ = true;

  static uint32_t resetFlags(bool vsync) {
#if VISAGE_WINDOWS
    return BGFX_RESET_FLIP_AFTER_RENDER;
#elif VISAGE_MAC
    return BGFX_RESET_FLIP_AFTER_RENDER | (vsync ? BGFX_RESET_VSYNC : 0);
#elif VISAGE_LINUX || VISAGE_ANDROID || defined(__ANDROID__)
    return vsync ? BGFX_RESET_VSYNC : 0;
#else
    return 0;
#endif
  }

  static uint32_t resetFlags() {
    return resetFlags(vsync_enabled_);
  }

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
    if (initialized_)
      return;

    callback_handler_ = std::make_unique<GraphicsCallbackHandler>();
    initialized_ = true;
    startRenderThread();

    bgfx::Init bgfx_init;
    bgfx_init.resolution.numBackBuffers = 1;
    bgfx_init.resolution.width = 0;
    bgfx_init.resolution.height = 0;
    bgfx_init.callback = callback_handler_.get();

    bgfx_init.platformData.ndt = display;
    bgfx_init.platformData.nwh = model_window;
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
    bgfx_init.resolution.width = 1;
    bgfx_init.resolution.height = 1;
#elif VISAGE_LINUX
    bgfx_init.type = bgfx::RendererType::Vulkan;
#elif VISAGE_ANDROID || defined(__ANDROID__)
    bgfx_init.type = bgfx::RendererType::OpenGLES;
    if (model_window != nullptr) {
      bgfx_init.resolution.width = ANativeWindow_getWidth(static_cast<ANativeWindow*>(model_window));
      bgfx_init.resolution.height = ANativeWindow_getHeight(static_cast<ANativeWindow*>(model_window));
    }
#elif VISAGE_EMSCRIPTEN
    bgfx_init.type = bgfx::RendererType::OpenGLES;
#endif

    bgfx_init.resolution.reset = resetFlags();

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
  }

  void Renderer::setVsync(bool vsync) {
    vsync_enabled_ = vsync;
  }

  bool Renderer::vsync() {
    return vsync_enabled_;
  }

  void Renderer::setNativeWindow(void* window, int width, int height, bool vsync) {
    setVsync(vsync);
#if VISAGE_ANDROID || defined(__ANDROID__)
    bgfx::PlatformData pd;
    pd.ndt = nullptr;
    pd.nwh = window;
    pd.context = nullptr;
    pd.backBuffer = nullptr;
    pd.backBufferDS = nullptr;
    bgfx::setPlatformData(pd);
    if (window != nullptr && width > 0 && height > 0) {
      static bool force_toggle = false;
      force_toggle = !force_toggle;
      uint32_t flags = resetFlags(vsync);
      if (force_toggle) {
        flags |= BGFX_RESET_FLUSH_AFTER_RENDER;
      }
      bgfx::reset(width, height, flags);
    }
#endif
  }

  void Renderer::resetResolution(int width, int height, bool vsync) {
    setVsync(vsync);
#if VISAGE_MAC
    bgfx::reset(width, height, resetFlags(vsync));
#elif VISAGE_ANDROID || defined(__ANDROID__)
    bgfx::reset(width, height, resetFlags(vsync));
#endif
  }

  void Renderer::setScreenshotData(const uint8_t* data, int width, int height, int pitch, bool blue_red) {
    screenshot_ = Screenshot(data, width, height, pitch, blue_red);
  }
}
