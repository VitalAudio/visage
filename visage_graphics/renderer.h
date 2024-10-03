/* Copyright Vital Audio, LLC
 *
 * visage is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * visage is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with visage.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "visage_utils/thread_utils.h"

namespace visage {
  class Renderer : public Thread {
  public:
    static Renderer& getInstance();

    Renderer() : Thread("Renderer Thread") { }
    ~Renderer() override;

    void checkInitialization(void* window_handle, void* display, int width, int height);

    const std::string& getErrorMessage() const { return error_message_; }
    bool supported() const { return supported_; }
    bool initialized() const { return initialized_; }

  private:
    void startRenderThread();
    void render();
    void run() override;

    bool initialized_ = false;
    bool supported_ = false;
    std::string error_message_;
    std::atomic<bool> render_thread_started_ = false;
  };
}