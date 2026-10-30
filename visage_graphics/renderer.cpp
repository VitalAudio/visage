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

#include "font.h"
#include "gpu/gpu.h"
#include "graphics_caches.h"
#include "post_effects.h"
#include "shape_batcher.h"
#include "visage_utils/string_utils.h"

#if VISAGE_EMSCRIPTEN
#include <emscripten/html5_webgl.h>
#endif

namespace visage {
#if VISAGE_MAC
  static constexpr bool kVsync = false;
#else
  static constexpr bool kVsync = true;
#endif

#if VISAGE_EMSCRIPTEN
  static bool enableWebGlExtension(const char* name) {
    return emscripten_webgl_enable_extension(emscripten_webgl_get_current_context(), name);
  }
#else
  static bool renderTargetSupported(gpu::TextureFormat format) {
    static constexpr uint32_t kRequired = gpu::kFormatTexture | gpu::kFormatRenderTarget;
    return (gpu::getCaps()->formats[static_cast<int>(format)] & kRequired) == kRequired;
  }
#endif

  Renderer& Renderer::instance() {
    static Renderer renderer;
    return renderer;
  }

  void Renderer::initialize(void* model_window, void* display) {
    if (display_ == nullptr) {
      display_ = display;
      gpu::setDisplay(display_);
    }

    if (initialized_)
      return;

    initialized_ = true;
    model_window_ = model_window;

    gpu::Init init;
    init.type = gpu::defaultRendererType();
    init.display = display_;
    init.vsync = kVsync;
    init.color_managed = color_managed_;
#if VISAGE_GRAPHICS_DEBUG_LOGGING
    init.debug = true;
    init.log_callback = [](const char* message) { VISAGE_LOG(message); };
#endif
    init.screenshot_callback = [](gpu::FrameBufferHandle, const uint8_t* data, int width,
                                  int height, int pitch, bool bgra) {
      Renderer::instance().setScreenshotData(data, width, height, pitch, bgra,
                                             gpu::getCaps()->origin_bottom_left);
    };

    supported_ = gpu::rendererSupported(init.type) && gpu::init(init);
    if (!supported_) {
      VISAGE_ASSERT(false);
      error_message_ = std::string(gpu::rendererName(init.type)) +
                       " is required and not supported on this computer.";
      gpu::Init fallback;
      fallback.type = gpu::RendererType::Noop;
      gpu::init(fallback);
    }
    swap_chain_supported_ = gpu::getCaps()->swap_chain;

#if VISAGE_EMSCRIPTEN
    bool color_buffer_float = enableWebGlExtension("EXT_color_buffer_float");
    float_render_targets_supported_ = color_buffer_float ||
                                      enableWebGlExtension("EXT_color_buffer_half_float");
#else
    float_render_targets_supported_ = renderTargetSupported(gpu::TextureFormat::RGBA16F);
#endif
  }

  void Renderer::checkInitialization() {
    if (!initialized_ && shut_down_)
      initialize(model_window_, display_);
  }

  void Renderer::watchForLeaks() {
    struct LeakCheck {
      ~LeakCheck() { VISAGE_ASSERT(Renderer::instance().references_ == 0); }
    };
    static LeakCheck check;
  }

  void Renderer::removeReference() {
    if (--references_ > 0 || !initialized_)
      return;

    shutdown();
  }

  void Renderer::shutdown() {
    FontCache::releaseTextures();
    PostEffect::releaseScreenBuffers();
    ProgramCache::release();
    ShaderCache::release();
    UniformCache::release();
    releaseBufferPools();
    gpu::shutdown();
    releaseBufferPoolStaging();
    initialized_ = false;
    shut_down_ = true;
  }

  void Renderer::setScreenshotData(const uint8_t* data, int width, int height, int pitch, bool bgra,
                                   bool y_flip) {
    screenshot_ = Screenshot(data, width, height, pitch, bgra, y_flip);
  }
}
