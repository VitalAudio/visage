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

#include "frame.h"
#include "visage_graphics/animation.h"
#include "visage_utils/time_utils.h"

namespace visage {
  class ScrollBar : public Frame {
  public:
    ScrollBar() :
        color_(Animation<float>::kRegularTime, Animation<float>::kEaseOut, Animation<float>::kEaseOut),
        width_(Animation<float>::kRegularTime, Animation<float>::kEaseOut, Animation<float>::kEaseOut) {
      color_.setTargetValue(1.0f);
    }

    void draw(Canvas& canvas) override;
    void resized() override {
      Frame::resized();
      width_.setSourceValue(width() / 2);
      width_.setTargetValue(width());
    }

    void mouseEnter(const MouseEvent& e) override;
    void mouseExit(const MouseEvent& e) override;
    void mouseDown(const MouseEvent& e) override;
    void mouseUp(const MouseEvent& e) override;
    void mouseDrag(const MouseEvent& e) override;

    void addScrollCallback(std::function<void(float)> callback) {
      callbacks_.push_back(std::move(callback));
    }

    void setRounding(float rounding) {
      rounding_ = rounding;
      redraw();
    }

    void setPosition(int position) {
      position_ = position;
      redraw();
    }

    void setViewPosition(int range, int view_height, int position) {
      range_ = range;
      view_height_ = view_height;
      position_ = position;

      active_ = view_height_ < range_;
      setIgnoresMouseEvents(!active_, true);
      redraw();
    }

    int viewRange() const { return range_; }
    int viewHeight() const { return view_height_; }

    void setLeftSide(bool left) { left_ = left; }

  private:
    std::vector<std::function<void(float)>> callbacks_;

    Animation<float> color_;
    Animation<float> width_;

    int last_drag_ = 0;

    bool active_ = false;
    bool left_ = false;
    int range_ = 0.0f;
    int view_height_ = 0.0f;
    float position_ = 0.0f;
    float rounding_ = 0.0f;

    VISAGE_LEAK_CHECKER(ScrollBar)
  };

  class ScrollableFrame : public Frame {
  public:
    static constexpr float kDefaultSmoothTime = 0.1f;
    static constexpr float kDefaultWheelSensitivity = 100.0f;

    explicit ScrollableFrame(const std::string& name = "") : Frame(name) {
      addChild(&container_);
      container_.setIgnoresMouseEvents(true, true);
      container_.setVisible(false);
      container_.onChildBoundsChanged() = [this](const Frame* child) {
        updateScrollableHeight(child);
      };
      container_.onChildAdded() = [this](const Frame* child) { updateScrollableHeight(child); };
      container_.onChildRemoved() = [this](const Frame* child) {
        if (child == bottom_most_child_)
          updateScrollableHeight();
      };

      addChild(&scroll_bar_);
      scroll_bar_.addScrollCallback([this](float position) {
        scrollPositionChanged(position);
        smooth_position_ = y_position_;
      });
      scroll_bar_.setOnTop(true);
    }

    void resized() override;

    void addScrolledChild(Frame* frame, bool make_visible = true) {
      container_.setVisible(true);
      container_.addChild(frame, make_visible);
    }

    void addScrolledChild(Frame& frame, bool make_visible = true) { addScrolledChild(&frame); }

    void addScrolledChild(std::unique_ptr<Frame> frame, bool make_visible = true) {
      container_.setVisible(true);
      container_.addChild(std::move(frame), make_visible);
    }

    void removeScrolledChild(Frame* frame) { container_.removeChild(frame); }

    bool scrollUp() {
      setYPosition(std::max(0.0f, y_position_ - height() / 8.0f));
      return true;
    }

    bool scrollDown() {
      setYPosition(y_position_ + height() / 8);
      return true;
    }

    void setScrollBarRounding(float rounding) { scroll_bar_.setRounding(rounding); }
    float scrollableHeight() const { return container_.height(); }
    void setScrollableHeight(float total_height, float view_height = 0) {
      if (view_height == 0)
        view_height = height();
      container_.setBounds(0, -y_position_, width(), total_height);
      setYPosition(std::max(0.0f, std::min(y_position_, total_height - view_height)));
      scroll_bar_.setViewPosition(total_height, view_height, y_position_);
    }

    void setScrollBarBounds(float x, float y, float width, float height) {
      scroll_bar_.setBounds(x, y, width, height);
    }

    void setYPosition(float position) {
      scrollPositionChanged(position);
      smooth_position_ = y_position_;
    }

    float yPosition() const { return y_position_; }

    bool mouseWheel(const MouseEvent& e) override {
      float delta = -e.precise_wheel_delta_y * sensitivity_;
      if (e.wheel_momentum) {
        float new_position = std::max(0.0f, std::min(maxScroll(), smooth_position_ + delta));
        if (new_position == smooth_position_)
          return false;

        smooth_position_ = new_position;
        scrollPositionChanged(smooth_position_);
        scroll_bar_.setViewPosition(scroll_bar_.viewRange(), scroll_bar_.viewHeight(), smooth_position_);
        return true;
      }
      else
        return smoothScroll(delta);
    }

    void setScrollBarLeft(bool left) {
      scroll_bar_left_ = left;
      scroll_bar_.setLeftSide(left);
    }

    Layout& scrollableLayout() { return container_.layout(); }

    auto& onScroll() { return on_scroll_; }
    ScrollBar& scrollBar() { return scroll_bar_; }

    void setSensitivity(float sensitivity) { sensitivity_ = sensitivity; }
    void setSmoothTime(float seconds) { smooth_time_ = seconds; }

  private:
    float bottomPadding();
    std::pair<const Frame*, float> findBottomMostChild();
    void updateScrollableHeight(const Frame* changed);
    void updateScrollableHeight();
    float maxScroll() const { return scroll_bar_.viewRange() - scroll_bar_.viewHeight(); }

    void scrollPositionChanged(float position) {
      y_position_ = std::round(dpiScale() * position) / dpiScale();
      container_.setTopLeft(container_.x(), -y_position_);
      scroll_bar_.setPosition(position);
      redraw();
      container_.redraw();
      on_scroll_.callback(this);
    }

    bool smoothScroll(float offset) {
      float max = maxScroll();
      if (max <= 0)
        return false;

      if (offset == 0.0f)
        return false;

      float t = (time::milliseconds() - smooth_start_time_) / (smooth_time_ * 1000.0f);
      if (t <= 1.0f && t >= 0.0f)
        smooth_start_position_ += (smooth_position_ - smooth_start_position_) * t;
      else
        smooth_start_position_ = smooth_position_;

      smooth_position_ += offset;
      smooth_position_ = std::min(max, smooth_position_);
      smooth_position_ = std::max(0.0f, smooth_position_);

      smooth_start_time_ = time::milliseconds();
      runOnEventThread([this] { smoothScrollUpdate(); });
      return true;
    }

    void smoothScrollUpdate() {
      float t = (time::milliseconds() - smooth_start_time_) / (smooth_time_ * 1000.0f);
      int position = smooth_start_position_ + (smooth_position_ - smooth_start_position_) * t;
      if (t >= 1.0f)
        position = smooth_position_;
      else if (t >= 0.0f)
        runOnEventThread([this] { smoothScrollUpdate(); });

      scrollPositionChanged(position);
      scroll_bar_.setViewPosition(scroll_bar_.viewRange(), scroll_bar_.viewHeight(), position);
    }

    CallbackList<void(ScrollableFrame*)> on_scroll_;
    float smooth_position_ = 0.0f;
    float y_position_ = 0;
    bool scroll_bar_left_ = false;
    Frame container_;
    const Frame* bottom_most_child_ = nullptr;
    ScrollBar scroll_bar_;
    float sensitivity_ = kDefaultWheelSensitivity;
    float smooth_time_ = kDefaultSmoothTime;

    float smooth_start_position_ = 0;
    long long smooth_start_time_ = 0;

    VISAGE_LEAK_CHECKER(ScrollableFrame)
  };
}