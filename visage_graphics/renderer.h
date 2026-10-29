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

#include "screenshot.h"

#include <atomic>
#include <memory>

namespace visage {
  class Renderer {
  public:
    class Reference {
    public:
      Reference() { Renderer::instance().references_++; }
      Reference(const Reference&) = delete;
      Reference& operator=(const Reference&) = delete;
      ~Reference() { Renderer::instance().removeReference(); }
    };

    static Renderer& instance();

    void initializeWindowless() { initialize(nullptr, nullptr); }
    void initialize(void* model_window, void* display);
    void checkInitialization();
    void watchForLeaks();
    void setScreenshotData(const uint8_t* data, int width, int height, int pitch, bool bgra, bool y_flip);
    const Screenshot& screenshot() const { return screenshot_; }

    const std::string& errorMessage() const { return error_message_; }
    bool supported() const { return supported_; }
    bool swapChainSupported() const { return swap_chain_supported_; }
    bool floatRenderTargetsSupported() const { return float_render_targets_supported_; }
    void setColorManaged(bool color_managed) { color_managed_ = color_managed; }
    bool colorManaged() const { return color_managed_; }
    bool initialized() const { return initialized_; }
    void* display() const { return display_; }

  private:
    void removeReference();
    void shutdown();

    bool initialized_ = false;
    bool shut_down_ = false;
    std::atomic<int> references_ = 0;
    void* model_window_ = nullptr;
    void* display_ = nullptr;
    bool supported_ = false;
    bool swap_chain_supported_ = false;
    bool float_render_targets_supported_ = false;
    bool color_managed_ = true;

    Screenshot screenshot_;
    std::string error_message_;
  };
}
