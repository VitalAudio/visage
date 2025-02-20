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
#include <bimg/bimg.h>
#include <bx/file.h>

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
      bx::FileWriter writer;
      bx::Error error;
      if (bx::open(&writer, file_path, false, &error)) {
        bimg::imageWritePng(&writer, width, height, pitch, data, bimg::TextureFormat::RGBA8, y_flip, &error);
        bx::close(&writer);
      }
    }

    void captureBegin(uint32_t, uint32_t, uint32_t, bgfx::TextureFormat::Enum, bool) override { }
    void captureEnd() override { }
    void captureFrame(const void*, uint32_t) override { }
  };

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

  void Renderer::checkInitialization(void* model_window, void* display) {
    if (initialized_)
      return;

    callback_handler_ = std::make_unique<GraphicsCallbackHandler>();
    initialized_ = true;
    startRenderThread();

    bgfx::Init bgfx_init;
    bgfx_init.resolution.numBackBuffers = 1;
    bgfx_init.resolution.width = 0;
    bgfx_init.resolution.height = 0;
    bgfx_init.resolution.reset = BGFX_RESET_FLIP_AFTER_RENDER;
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
    bgfx_init.resolution.reset |= BGFX_RESET_VSYNC;
#elif VISAGE_LINUX
    bgfx_init.type = bgfx::RendererType::Vulkan;
#elif VISAGE_EMSCRIPTEN
    bgfx_init.type = bgfx::RendererType::OpenGLES;
#endif

    bool backend_supported = false;
    for (int i = 0; i < num_supported && !backend_supported; ++i)
      backend_supported = supported_renderers[i] == bgfx_init.type;

    if (!backend_supported) {
      std::string renderer_name = bgfx::getRendererName(bgfx_init.type);
      error_message_ = renderer_name + " is required and not supported on this computer.";
    }

    bgfx::init(bgfx_init);

    bool swap_chain_supported = bgfx::getCaps()->supported & BGFX_CAPS_SWAP_CHAIN;
    if (!swap_chain_supported) {
      VISAGE_ASSERT(false);
      error_message_ = "Swap chain rendering is required.";
    }

    supported_ = backend_supported && swap_chain_supported;
  }
}
