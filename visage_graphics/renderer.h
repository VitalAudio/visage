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
#include "visage_utils/thread_utils.h"

namespace visage {
  class GraphicsCallbackHandler;

  class Renderer : public Thread {
  public:
    static Renderer& instance();

    Renderer();
    ~Renderer() override;

    static void resetResolution(int width, int height);

    void checkInitialization(void* model_window, void* display);
    void setScreenshotData(const uint8_t* data, int width, int height, int pitch, bool blue_red);
    const Screenshot& screenshot() const { return screenshot_; }

    const std::string& errorMessage() const { return error_message_; }
    bool supported() const { return supported_; }
    bool swapChainSupported() const { return swap_chain_supported_; }
    bool initialized() const { return initialized_; }

  private:
    void startRenderThread();
    void render();
    void run() override;

    bool initialized_ = false;
    bool supported_ = false;
    bool swap_chain_supported_ = false;

    Screenshot screenshot_;
    std::string error_message_;
    std::atomic<bool> render_thread_started_ = false;
    std::unique_ptr<GraphicsCallbackHandler> callback_handler_;
  };
}
