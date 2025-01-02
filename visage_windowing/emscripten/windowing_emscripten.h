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

#if VISAGE_EMSCRIPTEN
#include "windowing.h"

namespace visage {
  class WindowEmscripten : public Window {
  public:
    static WindowEmscripten* running_instance_;
    static WindowEmscripten* runningInstance() { return running_instance_; }

    WindowEmscripten(int width, int height);

    void* initWindow() const override { return (void*)"#canvas"; }
    void* nativeHandle() const override { return (void*)"#canvas"; }

    void runEventLoop() override;
    void windowContentsResized(int width, int height) override;
    void show() override { }
    void showMaximized() override;
    bool maximized() { return false; }
    void hide() override { }
    void setWindowTitle(const std::string& title) override;
    Point maxWindowDimensions() const override;
    Point minWindowDimensions() const override;
    void setMousePosition(int x, int y) {
      mouse_x_ = x;
      mouse_y_ = y;
    }
    int mouseX() const { return mouse_x_; }
    int mouseY() const { return mouse_x_; }
    int initialWidth() const { return initial_width_; }
    int initialHeight() const { return initial_height_; }
    bool mouseRelativeMode() const override { return false; }

    void handleWindowResize(int window_width, int window_height);
    void runLoopCallback();

  private:
    int initial_width_ = 0;
    int initial_height_ = 0;
    float display_scale_ = 1.0f;
    bool maximized_ = false;
    int mouse_x_ = 0;
    int mouse_y_ = 0;
    long long start_microseconds_ = 0;
  };
}

#endif
