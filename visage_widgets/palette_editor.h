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

#include "color_picker.h"
#include "embedded/fonts.h"
#include "shader_editor.h"
#include "text_editor.h"
#include "visage_ui/frame.h"
#include "visage_ui/scroll_bar.h"

namespace visage {
  class PaletteColorEditor : public ScrollableFrame {
  public:
    static constexpr float kPaletteWidthRatio = 0.25f;
    static constexpr int kColorSpacing = 2;
    static constexpr int kColorIdHeight = 70;
    static constexpr float kMinColorHeight = 16.0f;

    explicit PaletteColorEditor(Palette* palette) : palette_(palette) {
      setAcceptsKeystrokes(true);
      for (int i = 0; i < QuadColor::kNumCorners; ++i) {
        addChild(&color_pickers_[i]);
        color_pickers_[i].setVisible(i == 0);
      }

      color_list_.setIgnoresMouseEvents(true, true);
      color_list_.setScrollBarLeft(true);
      addChild(&color_list_);
    }

    void draw(Canvas& canvas) override;

    int listLength(const std::map<std::string, std::vector<unsigned int>>& color_ids) const;
    int colorIndex(const MouseEvent& e);
    int colorIdIndex(const MouseEvent& e) const;
    void toggleGroup(const MouseEvent& e);
    void mouseMove(const MouseEvent& e) override {
      checkScrollHeight();
      checkColorHover(e);
    }
    void mouseDown(const MouseEvent& e) override;
    void mouseDrag(const MouseEvent& e) override;
    void mouseUp(const MouseEvent& e) override;
    void mouseWheel(const MouseEvent& e) override;
    bool keyPress(const KeyEvent& key) override;

    void resized() override {
      setColorListHeight();
      setColorPickerBounds();
    }
    void setColorListHeight();
    void setColorPickerBounds();
    void checkColorHover(const MouseEvent& e);
    void checkScrollHeight();

    bool isExpanded(const std::string& group) const { return expanded_groups_.count(group); }

    void toggleExpandGroup(const std::string& group) {
      if (expanded_groups_.count(group))
        expanded_groups_.erase(group);
      else
        expanded_groups_.insert(group);

      checkScrollHeight();
    }

    void setEditedPalette(Palette* palette) {
      expanded_groups_.clear();
      palette_ = palette;
    }

    void setCurrentOverrideId(int override_id) { current_override_id_ = override_id; }
    int currentOverrideId() const { return current_override_id_; }
    float colorHeight();

  private:
    void setNumColorsEditing(int num);

    Palette* palette_ = nullptr;
    ScrollableFrame color_list_;
    ColorPicker color_pickers_[QuadColor::kNumCorners];
    std::set<std::string> expanded_groups_;
    int num_colors_editing_ = 1;

    int current_override_id_ = 0;
    int mouse_down_index_ = -1;
    int dragging_ = -1;
    int editing_ = -1;
    int highlight_ = -1;
    int temporary_set_ = -1;
    int previous_color_index_ = -1;
    int mouse_drag_x_ = 0;
    int mouse_drag_y_ = 0;

    VISAGE_LEAK_CHECKER(PaletteColorEditor)
  };

  class PaletteValueEditor : public ScrollableFrame {
  public:
    static constexpr int kValueIdHeight = 70;
    static constexpr int kMaxValues = 500;

    explicit PaletteValueEditor(Palette* palette);

    void draw(Canvas& canvas) override;
    void resized() override {
      ScrollableFrame::resized();
      setTextEditorBounds();
    }

    int listLength(const std::map<std::string, std::vector<unsigned int>>& value_ids) const;
    void toggleGroup(const MouseEvent& e);
    void mouseMove(const MouseEvent& e) override { checkScrollHeight(); }
    void mouseDown(const MouseEvent& e) override { toggleGroup(e); }
    void setTextEditorBounds();
    void checkScrollHeight();
    void visibilityChanged() override {
      ScrollableFrame::visibilityChanged();
      checkScrollHeight();
    }

    bool isExpanded(const std::string& group) const { return expanded_groups_.count(group); }

    void toggleExpandGroup(const std::string& group);

    void setEditedPalette(Palette* palette) {
      palette_ = palette;
      expanded_groups_.clear();
      setTextEditorBounds();
    }

    void setCurrentOverrideId(int override_id) {
      if (current_override_id_ == override_id)
        return;
      current_override_id_ = override_id;
      resized();
    }
    int currentOverrideId() const { return current_override_id_; }

  private:
    Palette* palette_ = nullptr;
    int current_override_id_ = 0;
    std::set<std::string> expanded_groups_;
    TextEditor text_editors_[kMaxValues];

    VISAGE_LEAK_CHECKER(PaletteValueEditor)
  };
}