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

#include <set>

namespace visage {
  class ApplicationEditor;
  class Canvas;
  class Window;
  class WindowEventHandler;

  class TopLevelFrame : public Frame {
  public:
    explicit TopLevelFrame(ApplicationEditor* editor) : editor_(editor) { }

    void resized() override;

  private:
    ApplicationEditor* editor_ = nullptr;
  };

  class ApplicationEditor : public Frame {
  public:
    ApplicationEditor();
    ~ApplicationEditor() override;

    void takeScreenshot(const std::string& filename);

    void setCanvasDetails();

    void addToWindow(Window* handle);
    void removeFromWindow();
    void drawWindow();

    void setReferenceDimensions(int width, int height) {
      reference_width_ = width;
      reference_height_ = height;
    }

    int referenceWidth() const { return reference_width_; }
    int referenceHeight() const { return reference_height_; }

    bool isFixedAspectRatio() const { return fixed_aspect_ratio_; }
    void setFixedAspectRatio(bool fixed) { fixed_aspect_ratio_ = fixed; }

    Window* window() const { return window_; }

    void drawStaleChildren();

  private:
    Window* window_ = nullptr;
    TopLevelFrame top_level_;
    FrameEventHandler event_handler_;
    std::unique_ptr<Canvas> canvas_;
    std::unique_ptr<WindowEventHandler> window_event_handler_;
    bool fixed_aspect_ratio_ = false;

    int reference_width_ = 0;
    int reference_height_ = 0;

    std::set<Frame*> stale_children_;
    std::set<Frame*> drawing_children_;

    VISAGE_LEAK_CHECKER(ApplicationEditor)
  };

  class WindowedEditor : public ApplicationEditor {
  public:
    ~WindowedEditor() override;

    void setTitle(std::string title) { title_ = std::move(title); }

    void show(Dimension width, Dimension height);
    void show(Dimension x, Dimension y, Dimension width, Dimension height);
    void showPopup(Dimension width, Dimension height);
    void showPopup(Dimension x, Dimension y, Dimension width, Dimension height);
    void showMaximized();

    void runEventLoop();
    Window* window() const { return window_.get(); }

  private:
    void show(Dimension x, Dimension y, Dimension width, Dimension height, bool popup);
    void showWindow(bool maximized);

    std::string title_;
    std::unique_ptr<Window> window_;
  };
}
