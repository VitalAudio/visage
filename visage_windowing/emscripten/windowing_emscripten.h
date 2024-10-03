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

#if VA_EMSCRIPTEN
#include "windowing.h"

namespace va {
  class WindowEmscripten : public Window {
  public:
    static WindowEmscripten* running_instance_;
    static WindowEmscripten* runningInstance() { return running_instance_; }

    WindowEmscripten(int width, int height);

    void* getNativeHandle() const override { return (void*)"#canvas"; }

    void runEventThread() override;
    void windowContentsResized(int width, int height) override;
    void startTimer(float frequency) override { }
    void show() override { }
    void hide() override { }
    void setWindowTitle(const std::string& title) override;
    Point getMaxWindowDimensions() const override;
    Point getMinWindowDimensions() const override;
    void setMousePosition(int x, int y) {
      mouse_x_ = x;
      mouse_y_ = y;
    }
    int getMouseX() const { return mouse_x_; }
    int getMouseY() const { return mouse_x_; }
    bool getMouseRelativeMode() const override { return false; }

    void handleWindowResize(int window_width, int window_height);

  private:
    int initial_width_ = 0;
    int initial_height_ = 0;
    float display_scale_ = 1.0f;
    int mouse_x_ = 0;
    int mouse_y_ = 0;
  };
}

#endif
