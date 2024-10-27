/* Copyright Matt Tytel
 *
 * test plugin is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * test plugin is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with test plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <visage_widgets/bar_component.h>
#include <visage_widgets/button.h>
#include <visage_widgets/line_component.h>
#include <visage_widgets/shader_quad.h>
#include <visage_widgets/text_editor.h>

class DragDropTarget;
class TextImage;
class AnimatedLines;

class TestDrawableComponent : public visage::Frame {
public:
  static constexpr int kNumLines = 2;
  static constexpr int kNumBars = 21;

  TestDrawableComponent();
  ~TestDrawableComponent() override;

  void resized() override;
  void draw(visage::Canvas& canvas) override;
  void setShadow(visage::Bounds bounds, float amount, float rounding) {
    shadow_bounds_ = bounds;
    shadow_amount_ = amount;
    shadow_rounding_ = rounding;
  }

  auto& onShowOverlay() { return on_show_overlay_; }

private:
  visage::CallbackList<void()> on_show_overlay_;
  std::unique_ptr<DragDropTarget> drag_drop_target_;
  std::unique_ptr<visage::BarComponent> bar_component_;
  std::unique_ptr<visage::ShaderQuad> shader_quad_;
  std::unique_ptr<visage::ToggleIconButton> icon_button_;
  std::unique_ptr<visage::ToggleTextButton> text_button_;
  std::unique_ptr<visage::UiButton> ui_button_;
  std::unique_ptr<visage::UiButton> action_button_;
  std::unique_ptr<TextImage> text_;
  std::unique_ptr<visage::TextEditor> text_editor_;
  std::unique_ptr<visage::TextEditor> left_text_editor_;
  std::unique_ptr<visage::TextEditor> number_editor_;
  std::unique_ptr<visage::TextEditor> right_text_editor_;
  std::unique_ptr<visage::Frame> shapes_;
  std::unique_ptr<AnimatedLines> animated_lines_;

  float shadow_amount_ = 0.0f;
  visage::Bounds shadow_bounds_;
  float shadow_rounding_ = 0.0f;
};
