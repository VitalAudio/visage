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

#include "visage_ui/frame.h"

namespace visage {
  class Canvas;
  class Window;
  class WindowEventHandler;

  class ApplicationEditor : public Frame {
  public:
    ApplicationEditor();
    ~ApplicationEditor() override;

    void setCanvasDetails();
    void resized() final;
    virtual void editorResized() { }

    void addToWindow(Window* handle);
    void removeFromWindow();
    void drawWindow();

    virtual int defaultWidth() const { return 800; }
    virtual int defaultHeight() const { return 600; }
    float defaultAspectRatio() const { return defaultWidth() * 1.0f / defaultHeight(); }

    bool isFixedAspectRatio() const { return fixed_aspect_ratio_; }
    void setFixedAspectRatio(bool fixed) { fixed_aspect_ratio_ = fixed; }

    Window* window() const { return window_; }

    void drawStaleChildren();

  private:
    Window* window_ = nullptr;
    Frame top_level_;
    FrameEventHandler event_handler_;
    std::unique_ptr<Canvas> canvas_;
    std::unique_ptr<WindowEventHandler> window_event_handler_;
    bool fixed_aspect_ratio_ = false;

    std::set<Frame*> stale_children_;
    std::set<Frame*> drawing_children_;

    VISAGE_LEAK_CHECKER(ApplicationEditor)
  };

  class WindowedEditor : public ApplicationEditor {
  public:
    ~WindowedEditor() override;

    void setTitle(std::string title) { title_ = std::move(title); }

    void setPopup(bool popup) { popup_ = popup; }

    void show(float window_scale);
    void show(int x, int y, int width, int height);
    void show(int width, int height);

    void showWithEventLoop(float window_scale);
    void showWithEventLoop(int x, int y, int width, int height);
    void showWithEventLoop(int width, int height);

  private:
    void showWindow();

    bool popup_ = false;
    std::string title_;
    std::unique_ptr<Window> window_;
  };
}
