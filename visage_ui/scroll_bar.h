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

#include "drawable_component.h"
#include "visage_graphics/animation.h"

namespace visage {
  class ScrollBar : public DrawableComponent {
  public:
    ScrollBar() :
        color_(Animation<float>::kRegularTime, Animation<float>::kEaseOut, Animation<float>::kEaseOut),
        width_(Animation<float>::kRegularTime, Animation<float>::kEaseOut, Animation<float>::kEaseOut) {
      color_.setTargetValue(1.0f);
    }

    void draw(Canvas& canvas) override;
    void resized() override {
      DrawableComponent::resized();
      width_.setSourceValue(width() / 2);
      width_.setTargetValue(width());
    }

    void onMouseEnter(const MouseEvent& e) override;
    void onMouseExit(const MouseEvent& e) override;
    void onMouseDown(const MouseEvent& e) override;
    void onMouseUp(const MouseEvent& e) override;
    void onMouseDrag(const MouseEvent& e) override;

    void addScrollCallback(std::function<void(int)> callback) { callbacks_.push_back(callback); }

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
    std::vector<std::function<void(int)>> callbacks_;

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

  class ScrollableComponent : public DrawableComponent {
  public:
    explicit ScrollableComponent(const std::string& name = "") : DrawableComponent(name) {
      addDrawableComponent(&scroll_bar_);
      scroll_bar_.addScrollCallback([this](int position) { scrollPositionChanged(position); });
      scroll_bar_.setOnTop(true);

      addDrawableComponent(&container_);
      container_.setIgnoresMouseEvents(true, true);
      container_.setVisible(false);
    }

    void resized() override;

    void addScrolledComponent(DrawableComponent* component, bool make_visible = true) {
      container_.setVisible(true);
      container_.addDrawableComponent(component);
      component->setVisible(make_visible);
    }

    void scrollPositionChanged(int position) {
      setYPosition(position);
      for (auto& callback : callbacks_)
        callback(this);
    }

    bool scrollUp() {
      setYPosition(std::max(0, y_position_ - height() / 8));
      return true;
    }

    bool scrollDown() {
      setYPosition(y_position_ + height() / 8);
      return true;
    }

    void setScrollBarRounding(float rounding) { scroll_bar_.setRounding(rounding); }
    int scrollableHeight() const { return container_.height(); }
    void setScrollableHeight(int total_height, int view_height = 0) {
      if (view_height == 0)
        view_height = height();
      container_.setBounds(0, -y_position_, width(), total_height);
      setYPosition(std::max(0, std::min(y_position_, total_height - view_height)));
      scroll_bar_.setViewPosition(total_height, view_height, y_position_);
    }

    void setScrollBarBounds(int x, int y, int width, int height) {
      scroll_bar_.setBounds(x, y, width, height);
    }

    void setYPosition(float position) {
      float_position_ = position;
      y_position_ = position;
      container_.setTopLeft(container_.x(), -y_position_);
      scroll_bar_.setPosition(position);
      redraw();
    }
    int yPosition() const { return y_position_; }

    void onMouseWheel(const MouseEvent& e) override {
      static constexpr float kScale = 30.0f;

      float y = float_position_ - kScale * e.precise_wheel_delta_y * heightScale();
      float max = scroll_bar_.viewRange() - scroll_bar_.viewHeight();
      y = std::min(max, y);
      y = std::max(0.0f, y);
      scrollPositionChanged(y);
      scroll_bar_.setViewPosition(scroll_bar_.viewRange(), scroll_bar_.viewHeight(), y);
    }

    void setScrollBarLeft(bool left) {
      scroll_bar_left_ = left;
      scroll_bar_.setLeftSide(left);
    }

    void addScrollCallback(std::function<void(ScrollableComponent*)> callback) {
      callbacks_.push_back(callback);
    }

  private:
    std::vector<std::function<void(ScrollableComponent*)>> callbacks_;
    float float_position_ = 0.0f;
    int y_position_ = 0;
    bool scroll_bar_left_ = false;
    DrawableComponent container_;
    ScrollBar scroll_bar_;

    VISAGE_LEAK_CHECKER(ScrollableComponent)
  };
}