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

#include "palette_editor.h"

#include "embedded/fonts.h"
#include "visage_graphics/palette.h"
#include "visage_graphics/theme.h"
#include "visage_utils/string_utils.h"

namespace visage {
  void PaletteColorEditor::draw(Canvas& canvas) {
    static constexpr int kSquareWidth = 15;

    int width = getWidth();
    int height = getHeight();
    canvas.setColor(0xffbbbbbb);
    canvas.rectangle(0, 0, width, height);

    canvas.setColor(0xff888888);
    for (int r = 0; r < height; r += kSquareWidth) {
      for (int c = (r % 2) * kSquareWidth; c < width; c += 2 * kSquareWidth)
        canvas.rectangle(c, r, kSquareWidth, kSquareWidth);
    }

    std::vector<Palette::EditColor> colors = palette_->getColorList();
    int palette_width = width * kPaletteWidthRatio;
    float color_height = getColorHeight();
    int color_position = color_list_.yPosition();
    canvas.saveState();
    canvas.trimClampBounds(0, 0, getWidth(), color_list_.getHeight());
    for (int i = 0; i < colors.size(); ++i) {
      int y = std::round(color_height * i) - color_position;
      int end_y = std::round(color_height * (i + 1) - kColorSpacing) - color_position;

      canvas.setColor(colors[i].toQuadColor());
      canvas.roundedRectangle(0, y, palette_width, end_y - y, 8);
      if (i == editing_) {
        canvas.setColor(0xffffffff);
        canvas.roundedRectangleBorder(0, y, palette_width, end_y - y, 8, 2);
        canvas.setColor(0xff000000);
        canvas.roundedRectangleBorder(2, y + 2, palette_width - 4, end_y - y - 4, 6, 2);
      }

      if (i == highlight_) {
        canvas.setColor(0xffff00ff);
        canvas.roundedRectangleBorder(0, y, palette_width, end_y - y, 8, 2);
        canvas.setColor(0xffffffff);
        canvas.roundedRectangleBorder(2, y + 2, palette_width - 4, end_y - y - 4, 6, 2);
      }
    }

    int additional_y = std::round(color_height * colors.size()) - color_position;
    int additional_end_y = std::round(color_height * (colors.size() + 1) - kColorSpacing) - color_position;
    canvas.setColor(0x88ffffff);
    canvas.roundedRectangle(0, additional_y, palette_width, additional_end_y - additional_y, 8);
    int plus_width = color_height / 3;
    int plus_x = (palette_width - plus_width) / 2;
    int plus_y = additional_y + (color_height - plus_width) / 2;

    canvas.setColor(0xff000000);
    canvas.rectangle(palette_width / 2 - 1, plus_y, 2, plus_width);
    canvas.rectangle(plus_x, additional_y + color_height / 2 - 1, plus_width, 2);
    canvas.restoreState();

    std::map<std::string, std::vector<unsigned int>> color_ids = palette_->getColorIdList(current_override_id_);

    int font_height = kColorIdHeight / 3.0f;
    int label_offset = kColorIdHeight / 4.0f;
    Font font(font_height, fonts::Lato_Regular_ttf);
    int id_width = getWidth() - palette_width;
    canvas.saveState();
    canvas.setPosition(0, -yPosition());
    canvas.setClampBounds(0, yPosition(), width, std::max(0, height - width));
    int index = 0;
    for (const auto& group : color_ids) {
      int y = kColorIdHeight * index;
      canvas.setColor(0xff111111);
      canvas.roundedRectangle(palette_width + label_offset, y + label_offset,
                              id_width - 2 * label_offset, kColorIdHeight - 2 * label_offset, 8);

      canvas.setColor(0xffffffff);
      canvas.text(group.first, font, Font::kCenter, palette_width, y, id_width, kColorIdHeight);
      index++;

      if (isExpanded(group.first)) {
        for (int color_id : group.second) {
          y = kColorIdHeight * index;

          QuadColor matched_color;
          if (palette_->getColor(current_override_id_, color_id, matched_color)) {
            canvas.setColor(matched_color);
            canvas.roundedRectangle(palette_width, y, id_width, kColorIdHeight, 8);
          }

          canvas.setColor(0xffffffff);
          canvas.roundedRectangle(palette_width + label_offset, y + label_offset,
                                  id_width - 2 * label_offset, kColorIdHeight - 2 * label_offset, 8);

          canvas.setColor(0xff000000);
          canvas.text(theme::ColorId::getColorName(color_id), font, Font::kCenter, palette_width, y,
                      id_width, kColorIdHeight);

          index++;
        }
      }
    }
    canvas.restoreState();

    int dragging = dragging_;
    if (dragging >= 0 && dragging < colors.size()) {
      canvas.setColor(colors[dragging].toQuadColor());
      canvas.circle(mouse_drag_x_ - 10, mouse_drag_y_ - 10, 20);
    }
  }

  void PaletteColorEditor::setColorListHeight() {
    int palette_width = getWidth() * kPaletteWidthRatio;
    int color_picker_height = getWidth();
    int spacing = 2;
    int total_height = getHeight() - color_picker_height + spacing;
    float color_height = getColorHeight();
    color_list_.setBounds(0, 0, palette_width, total_height);
    color_list_.setScrollableHeight(std::max<int>(total_height,
                                                  color_height * (palette_->numColors() + 1)));
  }

  void PaletteColorEditor::setColorPickerBounds() {
    int width = getWidth();
    int height = getHeight();
    for (int i = 1; i < QuadColor::kNumCorners; ++i)
      color_pickers_[i].setVisible(num_colors_editing_ > i);

    if (num_colors_editing_ > 2) {
      int picker_width = width / 2;
      color_pickers_[0].setBounds(0, height - width, picker_width, picker_width);
      color_pickers_[1].setBounds(picker_width, height - width, picker_width, picker_width);
      color_pickers_[2].setBounds(0, height - picker_width, picker_width, picker_width);
      color_pickers_[3].setBounds(picker_width, height - picker_width, picker_width, picker_width);
    }
    else if (num_colors_editing_ == 2) {
      int picker_height = width / 2;
      color_pickers_[0].setBounds(0, height - width, width, picker_height);
      color_pickers_[1].setBounds(0, height - picker_height, width, picker_height);
    }
    else
      color_pickers_[0].setBounds(0, height - width, width, width);

    setScrollBarBounds(getWidth() - 20, 0, 20, getHeight() - getWidth());
    color_list_.setScrollBarBounds(getWidth() - 20, 0, 20, getHeight() - getWidth());
  }

  void PaletteColorEditor::checkColorHover(const MouseEvent& e) {
    int color_id = getColorIdIndex(e);
    if (color_id >= 0)
      highlight_ = palette_->getColorMap(current_override_id_, color_id);
  }

  void PaletteColorEditor::checkScrollHeight() {
    int list_length = getListLength(palette_->getColorIdList(current_override_id_));
    setScrollableHeight(kColorIdHeight * list_length, getHeight() - getWidth());
  }

  void PaletteColorEditor::colorChanged(ColorPicker* picker, Color color) {
    if (editing_ < 0)
      return;

    for (int i = 0; i < QuadColor::kNumCorners; ++i) {
      if (picker == &color_pickers_[i])
        palette_->setColorIndex(editing_, i, color);
    }
  }

  int PaletteColorEditor::getColorIndex(const MouseEvent& e) {
    int num_colors = palette_->numColors();
    int palette_width = getWidth() * kPaletteWidthRatio;
    if (e.getPosition().x < 0 || e.getPosition().x > palette_width)
      return Palette::kInvalidId;

    float color_height = getColorHeight();
    int index = (e.getPosition().y + color_list_.yPosition()) / color_height;
    if (index < 0 || index > num_colors)
      return Palette::kInvalidId;
    return index;
  }

  int PaletteColorEditor::getListLength(const std::map<std::string, std::vector<unsigned int>>& color_ids) {
    int length = color_ids.size();
    for (const auto& group : color_ids) {
      if (isExpanded(group.first))
        length += group.second.size();
    }
    return length;
  }

  int PaletteColorEditor::getColorIdIndex(const MouseEvent& e) {
    std::map<std::string, std::vector<unsigned int>> color_ids = palette_->getColorIdList(current_override_id_);
    int width = getWidth();
    int palette_width = width * kPaletteWidthRatio;
    if (e.getPosition().x > width || e.getPosition().x < palette_width)
      return -1;

    int y_position = std::min(getHeight(), std::max(0, e.getPosition().y));
    int index = (y_position + yPosition()) / kColorIdHeight;

    for (const auto& group : color_ids) {
      index--;
      if (isExpanded(group.first)) {
        if (index >= 0 && index < group.second.size())
          return group.second[index];
        index -= group.second.size();
      }
    }

    return -1;
  }

  void PaletteColorEditor::toggleGroup(const MouseEvent& e) {
    std::map<std::string, std::vector<unsigned int>> color_ids = palette_->getColorIdList(current_override_id_);
    int width = getWidth();
    int palette_width = width * kPaletteWidthRatio;
    if (e.getPosition().x > width || e.getPosition().x < palette_width)
      return;

    int y_position = std::min(getHeight(), std::max(0, e.getPosition().y));
    int index = (y_position + yPosition()) / kColorIdHeight;

    for (const auto& group : color_ids) {
      if (index == 0) {
        toggleExpandGroup(group.first);
        return;
      }
      index--;
      if (isExpanded(group.first))
        index -= group.second.size();
    }
  }

  void PaletteColorEditor::onMouseDown(const MouseEvent& e) {
    redraw();
    mouse_down_index_ = getColorIndex(e);
    bool toggle = e.isMiddleButton() || e.isAltDown();
    if (toggle && mouse_down_index_ >= 0 && mouse_down_index_ < palette_->numColors()) {
      palette_->toggleColorIndexStyle(mouse_down_index_);
      setNumColorsEditing(palette_->getColorIndex(mouse_down_index_).numActiveCorners());
      mouse_down_index_ = -1;
      return;
    }

    if (mouse_down_index_ >= 0) {
      if (mouse_down_index_ == palette_->numColors()) {
        palette_->addColor();
        setColorListHeight();
      }

      const Palette::EditColor& color = palette_->getColorIndex(mouse_down_index_);
      for (int i = 0; i < QuadColor::kNumCorners; ++i)
        color_pickers_[i].setColor(color.colors[i]);

      setNumColorsEditing(color.numActiveCorners());
      editing_ = mouse_down_index_;
    }
    else
      toggleGroup(e);
    onMouseDrag(e);
  }

  void PaletteColorEditor::onMouseDrag(const MouseEvent& e) {
    dragging_ = mouse_down_index_;
    mouse_drag_x_ = e.getPosition().x;
    mouse_drag_y_ = e.getPosition().y;

    int color_id_index = getColorIdIndex(e);
    if (temporary_set_ != color_id_index) {
      if (temporary_set_ >= 0) {
        palette_->setColorMap(current_override_id_, temporary_set_, previous_color_index_);
        temporary_set_ = -1;
        previous_color_index_ = -1;
      }

      if (color_id_index >= 0) {
        temporary_set_ = color_id_index;
        previous_color_index_ = palette_->getColorMap(current_override_id_, temporary_set_);
        palette_->setColorMap(current_override_id_, temporary_set_, dragging_);
      }
    }

    redraw();
  }

  void PaletteColorEditor::onMouseUp(const MouseEvent& e) {
    if (dragging_ < 0 && temporary_set_ >= 0)
      palette_->setColorMap(current_override_id_, temporary_set_, previous_color_index_);

    mouse_down_index_ = -1;
    dragging_ = -1;
    temporary_set_ = -1;
    previous_color_index_ = -1;
    redraw();
  }

  void PaletteColorEditor::onMouseWheel(const MouseEvent& e) {
    if (e.getPosition().x < getWidth() * kPaletteWidthRatio)
      color_list_.onMouseWheel(e);
    else
      ScrollableComponent::onMouseWheel(e);
    redraw();
  }

  bool PaletteColorEditor::onKeyPress(const KeyEvent& key) {
    redraw();
    if (key.getKeyCode() == KeyCode::C && editing_ >= 0) {
      setClipboardText(palette_->getColorIndex(editing_).encode());
      return true;
    }
    else if (key.getKeyCode() == KeyCode::V && editing_ >= 0) {
      Palette::EditColor color;
      color.decode(getClipboardText());
      palette_->setEditColor(editing_, color);
      return true;
    }
    if ((key.getKeyCode() == KeyCode::Delete || key.getKeyCode() == KeyCode::KPBackspace) && editing_ >= 0) {
      palette_->removeColor(editing_);
      setColorListHeight();
      editing_ = -1;
      return true;
    }

    return false;
  }

  float PaletteColorEditor::getColorHeight() {
    int num_colors = palette_->numColors();
    float color_height = (getHeight() - getWidth() + kColorSpacing) * 1.0f / (num_colors + 1);
    color_height = std::min<float>(color_height, getWidth() * kPaletteWidthRatio + kColorSpacing);
    return std::max(color_height, kMinColorHeight * getHeightScale());
  }

  void PaletteColorEditor::setNumColorsEditing(int num) {
    num_colors_editing_ = num;
    setColorPickerBounds();
  }

  void PaletteValueEditor::draw(Canvas& canvas) {
    int width = getWidth();
    int height = getHeight();
    if (width <= 0 || height <= 0)
      return;

    canvas.setColor(0xff333639);
    canvas.rectangle(0, 0, width, height);

    std::map<std::string, std::vector<unsigned int>> value_ids = palette_->getValueIdList(current_override_id_);
    int font_height = kValueIdHeight / 3.0f;
    int label_offset = kValueIdHeight / 4.0f;
    Font font(font_height, fonts::Lato_Regular_ttf);
    int id_width = getWidth();
    canvas.saveState();
    canvas.setPosition(0, -yPosition());
    canvas.setClampBounds(0, yPosition(), 2 * width / 3, std::max(0, height - width));
    int index = 0;
    for (const auto& group : value_ids) {
      canvas.setClampBounds(0, yPosition(), width, height);
      int y = kValueIdHeight * index;

      canvas.setColor(0xff111111);
      canvas.roundedRectangle(label_offset, y + label_offset, id_width - 2 * label_offset,
                              kValueIdHeight - 2 * label_offset, 8);

      canvas.setColor(0xffffffff);
      canvas.text(group.first, font, Font::kCenter, 0, y, id_width, kValueIdHeight);
      index++;

      canvas.setClampBounds(0, yPosition(), 2 * width / 3 - kValueIdHeight / 4, height);
      if (isExpanded(group.first)) {
        for (int color_id : group.second) {
          y = kValueIdHeight * index;
          canvas.setColor(0xffffffff);
          canvas.text(theme::ValueId::getValueName(color_id), font, Font::kLeft, label_offset, y,
                      id_width, kValueIdHeight);

          index++;
        }
      }
    }
    canvas.restoreState();
  }

  void PaletteValueEditor::textEditorChanged(TextEditor* text_editor) {
    std::map<std::string, std::vector<unsigned int>> value_ids = palette_->getValueIdList(current_override_id_);
    int index = 0;
    for (const auto& group : value_ids) {
      for (int color_id : group.second) {
        if (text_editor == &text_editors_[index]) {
          String text = String(text_editors_[index].getText()).trim();
          if (text.isEmpty())
            palette_->removeValue(current_override_id_, color_id);
          else
            palette_->setValue(current_override_id_, color_id, text.toFloat());
          return;
        }

        index++;
      }
    }
  }

  int PaletteValueEditor::getListLength(const std::map<std::string, std::vector<unsigned int>>& value_ids) {
    int length = value_ids.size();
    for (const auto& group : value_ids) {
      if (isExpanded(group.first))
        length += group.second.size();
    }
    return length;
  }

  void PaletteValueEditor::toggleGroup(const MouseEvent& e) {
    std::map<std::string, std::vector<unsigned int>> color_ids = palette_->getValueIdList(current_override_id_);
    int y_position = std::min(getHeight(), std::max(0, e.getPosition().y));
    int index = (y_position + yPosition()) / kValueIdHeight;

    for (const auto& group : color_ids) {
      if (index == 0) {
        toggleExpandGroup(group.first);
        return;
      }
      index--;
      if (isExpanded(group.first))
        index -= group.second.size();
    }
  }

  void PaletteValueEditor::setTextEditorBounds() {
    std::map<std::string, std::vector<unsigned int>> value_ids = palette_->getValueIdList(current_override_id_);
    int index = 0;
    int width = getWidth();
    int edit_height = kValueIdHeight * 3.0f / 4.0f;
    int y_offset = (kValueIdHeight - edit_height) / 2;
    int edit_width = width / 3;
    int x = width - edit_width - kValueIdHeight / 4;
    int y = kValueIdHeight + y_offset;

    for (const auto& group : value_ids) {
      if (isExpanded(group.first)) {
        for (int value_id : group.second) {
          text_editors_[index].setBounds(x, y, edit_width, edit_height);
          text_editors_[index].setVisible(true);

          float matched_value = 0.0f;
          if (palette_->getValue(current_override_id_, value_id, matched_value))
            text_editors_[index].setText(String(matched_value).toUtf8());
          else
            text_editors_[index].setText("");

          text_editors_[index].setDefaultText(String(theme::ValueId::getDefaultValue(value_id)).toUtf8());

          index++;
          y += kValueIdHeight;
        }
      }
      else {
        for (int i = 0; i < group.second.size(); ++i)
          text_editors_[index++].setVisible(false);
      }

      y += kValueIdHeight;
    }
  }

  void PaletteValueEditor::checkScrollHeight() {
    int list_length = getListLength(palette_->getValueIdList(current_override_id_));
    setScrollableHeight(kValueIdHeight * list_length, getHeight());
  }

  void PaletteValueEditor::toggleExpandGroup(const std::string& group) {
    if (expanded_groups_.count(group))
      expanded_groups_.erase(group);
    else
      expanded_groups_.insert(group);

    setTextEditorBounds();
    checkScrollHeight();
  }
}