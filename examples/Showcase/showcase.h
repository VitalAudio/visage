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

#include "examples_frame.h"

#include <visage_app/application_editor.h>
#include <visage_graphics/animation.h>
#include <visage_graphics/palette.h>
#include <visage_ui/popup_menu.h>
#include <visage_ui/undo_history.h>
#include <visage_widgets/palette_editor.h>
#include <visage_widgets/shader_editor.h>

class DebugInfo;

class Overlay : public visage::Frame {
public:
  Overlay();

  void resized() override;
  void draw(visage::Canvas& canvas) override;
  visage::Bounds getBodyBounds() const;
  float getBodyRounding();

  void mouseDown(const visage::MouseEvent& e) override {
    animation_.target(false);
    redraw();
  }
  void visibilityChanged() override { animation_.target(isVisible()); }
  auto& onAnimate() { return on_animate_; }

private:
  visage::Animation<float> animation_;
  visage::CallbackList<void(float)> on_animate_;
};

class Showcase : public visage::WindowedEditor,
                 public visage::UndoHistory {
public:
  static constexpr int kDefaultWidth = 700;
  static constexpr int kDefaultHeight = 600;

  Showcase();
  ~Showcase() override;

  int defaultWidth() const override { return kDefaultWidth; }
  int defaultHeight() const override { return kDefaultHeight; }
  void resized() override;

  void draw(visage::Canvas& canvas) override;

  void clearEditors();
  void showEditor(const Frame* editor, int default_width);
  bool keyPress(const visage::KeyEvent& key) override;

private:
  std::unique_ptr<visage::BlurBloomPostEffect> blur_bloom_;
  std::unique_ptr<visage::ShaderPostEffect> overlay_zoom_;
  std::unique_ptr<ExamplesFrame> examples_;
  std::unique_ptr<DebugInfo> debug_info_;

  visage::Palette palette_;
  visage::PaletteColorEditor color_editor_;
  visage::PaletteValueEditor value_editor_;
  visage::ShaderEditor shader_editor_;
  Overlay overlay_;

  VISAGE_LEAK_CHECKER(Showcase)
};
