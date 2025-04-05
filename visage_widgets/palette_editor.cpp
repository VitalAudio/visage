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

#include "palette_editor.h"

#include "embedded/fonts.h"
#include "visage_graphics/palette.h"
#include "visage_graphics/theme.h"
#include "visage_utils/string_utils.h"

namespace visage {
  void PaletteColorEditor::draw(Canvas& canvas) {
    static constexpr int kSquareWidth = 15;

    int w = width();
    int h = height();
    canvas.setColor(0xffbbbbbb);
    canvas.rectangle(0, 0, w, h);

    canvas.setColor(0xff888888);
    for (int r = 0; r < h; r += kSquareWidth) {
      for (int c = (r % 2) * kSquareWidth; c < w; c += 2 * kSquareWidth)
        canvas.rectangle(c, r, kSquareWidth, kSquareWidth);
    }

    std::vector<Brush> colors = palette_->colorList();
    int palette_width = w * kPaletteWidthRatio;
    float color_height = colorHeight();
    int color_position = color_list_.yPosition();
    canvas.saveState();
    canvas.trimClampBounds(0, 0, width(), color_list_.height());
    for (int i = 0; i < colors.size(); ++i) {
      int y = std::round(color_height * i) - color_position;
      int end_y = std::round(color_height * (i + 1) - kColorSpacing) - color_position;

      canvas.setColor(colors[i]);
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

    std::map<std::string, std::vector<theme::ColorId>> color_ids = palette_->colorIdList(current_override_id_);

    int font_height = kColorIdHeight / 3.0f;
    int label_offset = kColorIdHeight / 4.0f;
    Font font(font_height, fonts::Lato_Regular_ttf);
    int id_width = width() - palette_width;
    canvas.saveState();
    canvas.setPosition(0, -yPosition());
    canvas.setClampBounds(0, yPosition(), w, std::max(0, h - w));
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
        for (theme::ColorId color_id : group.second) {
          y = kColorIdHeight * index;

          Brush matched_color;
          if (palette_->color(current_override_id_, color_id, matched_color)) {
            canvas.setColor(matched_color);
            canvas.roundedRectangle(palette_width, y, id_width, kColorIdHeight, 8);
          }

          canvas.setColor(0xffffffff);
          canvas.roundedRectangle(palette_width + label_offset, y + label_offset,
                                  id_width - 2 * label_offset, kColorIdHeight - 2 * label_offset, 8);

          canvas.setColor(0xff000000);
          canvas.text(theme::ColorId::name(color_id), font, Font::kCenter, palette_width, y,
                      id_width, kColorIdHeight);

          index++;
        }
      }
    }
    canvas.restoreState();

    int dragging = dragging_;
    if (dragging >= 0 && dragging < colors.size()) {
      canvas.setBrush(colors[dragging]);
      canvas.circle(mouse_drag_x_ - 10, mouse_drag_y_ - 10, 20);
    }
  }

  void PaletteColorEditor::setColorListHeight() {
    int palette_width = width() * kPaletteWidthRatio;
    int color_picker_height = width();
    int spacing = 2;
    int total_height = height() - color_picker_height + spacing;
    float color_height = colorHeight();
    color_list_.setBounds(0, 0, palette_width, total_height);
    color_list_.setScrollableHeight(std::max<int>(total_height,
                                                  color_height * (palette_->numColors() + 1)));
  }

  void PaletteColorEditor::setColorPickerBounds() {
    int w = width();
    int h = height();
    color_picker_from_.onColorChange() = [this](const Color& color) {
      if (editing_ >= 0)
        palette_->setColorIndexFrom(editing_, color);
      redraw();
    };
    color_picker_to_.onColorChange() = [this](const Color& color) {
      if (editing_ >= 0)
        palette_->setColorIndexTo(editing_, color);
      redraw();
    };
    color_picker_to_.setVisible(editing_gradient_);

    if (editing_gradient_) {
      int picker_height = w / 2;
      color_picker_from_.setBounds(0, h - w, w, picker_height);
      color_picker_to_.setBounds(0, h - picker_height, w, picker_height);
    }
    else
      color_picker_from_.setBounds(0, h - w, w, w);

    setScrollBarBounds(w - 20, 0, 20, h - w);
    color_list_.setScrollBarBounds(w - 20, 0, 20, h - w);
  }

  void PaletteColorEditor::checkColorHover(const MouseEvent& e) {
    theme::ColorId color_id = colorIdIndex(e);
    if (color_id.isValid())
      highlight_ = palette_->colorMap(current_override_id_, color_id);
  }

  void PaletteColorEditor::checkScrollHeight() {
    int list_length = listLength(palette_->colorIdList(current_override_id_));
    setScrollableHeight(kColorIdHeight * list_length, height() - width());
  }

  int PaletteColorEditor::colorIndex(const MouseEvent& e) {
    int num_colors = palette_->numColors();
    int palette_width = width() * kPaletteWidthRatio;
    if (e.position.x < 0 || e.position.x > palette_width)
      return Palette::kInvalidId;

    float color_height = colorHeight();
    int index = (e.position.y + color_list_.yPosition()) / color_height;
    if (index < 0 || index > num_colors)
      return Palette::kInvalidId;
    return index;
  }

  int PaletteColorEditor::listLength(const std::map<std::string, std::vector<theme::ColorId>>& color_ids) const {
    int length = color_ids.size();
    for (const auto& group : color_ids) {
      if (isExpanded(group.first))
        length += group.second.size();
    }
    return length;
  }

  theme::ColorId PaletteColorEditor::colorIdIndex(const MouseEvent& e) const {
    std::map<std::string, std::vector<theme::ColorId>> color_ids = palette_->colorIdList(current_override_id_);
    int w = width();
    int palette_width = w * kPaletteWidthRatio;
    if (e.position.x > w || e.position.x < palette_width)
      return {};

    float y_position = std::min(height(), std::max(0.0f, e.position.y));
    int index = (y_position + yPosition()) / kColorIdHeight;

    for (const auto& group : color_ids) {
      index--;
      if (isExpanded(group.first)) {
        if (index >= 0 && index < group.second.size())
          return group.second[index];
        index -= group.second.size();
      }
    }

    return {};
  }

  void PaletteColorEditor::toggleGroup(const MouseEvent& e) {
    std::map<std::string, std::vector<theme::ColorId>> color_ids = palette_->colorIdList(current_override_id_);
    int palette_width = width() * kPaletteWidthRatio;
    if (e.position.x > width() || e.position.x < palette_width)
      return;

    int y_position = std::min(height(), std::max(0.0f, e.position.y));
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

  void PaletteColorEditor::mouseDown(const MouseEvent& e) {
    redraw();
    mouse_down_index_ = colorIndex(e);
    bool toggle = e.isMiddleButton() || e.isAltDown();
    if (toggle && mouse_down_index_ >= 0 && mouse_down_index_ < palette_->numColors()) {
      palette_->toggleColorIndexStyle(mouse_down_index_);
      setEditingGradient(palette_->colorIndex(mouse_down_index_).gradient().resolution() > 1);
      mouse_down_index_ = -1;
      return;
    }

    if (mouse_down_index_ >= 0) {
      if (mouse_down_index_ == palette_->numColors()) {
        palette_->addColor();
        setColorListHeight();
      }

      const Brush& color = palette_->colorIndex(mouse_down_index_);
      color_picker_from_.setColor(color.gradient().sample(0.0f));
      color_picker_to_.setColor(color.gradient().sample(1.0f));
      setEditingGradient(color.gradient().resolution() > 1);
      editing_ = mouse_down_index_;
    }
    else
      toggleGroup(e);
    mouseDrag(e);
  }

  void PaletteColorEditor::mouseDrag(const MouseEvent& e) {
    dragging_ = mouse_down_index_;
    mouse_drag_x_ = e.position.x;
    mouse_drag_y_ = e.position.y;

    theme::ColorId color_id_index = colorIdIndex(e);
    if (temporary_set_.id != color_id_index.id) {
      if (temporary_set_.isValid()) {
        palette_->setColorMap(current_override_id_, temporary_set_, previous_color_index_);
        temporary_set_ = {};
        previous_color_index_ = -1;
      }

      if (color_id_index.isValid()) {
        temporary_set_ = color_id_index;
        previous_color_index_ = palette_->colorMap(current_override_id_, temporary_set_);
        palette_->setColorMap(current_override_id_, temporary_set_, dragging_);
      }
    }

    redraw();
  }

  void PaletteColorEditor::mouseUp(const MouseEvent& e) {
    if (dragging_ < 0 && temporary_set_.isValid())
      palette_->setColorMap(current_override_id_, temporary_set_, previous_color_index_);

    mouse_down_index_ = -1;
    dragging_ = -1;
    temporary_set_ = {};
    previous_color_index_ = -1;
    redraw();
  }

  bool PaletteColorEditor::mouseWheel(const MouseEvent& e) {
    redraw();

    if (e.position.x < width() * kPaletteWidthRatio)
      return color_list_.mouseWheel(e);

    return ScrollableFrame::mouseWheel(e);
  }

  bool PaletteColorEditor::keyPress(const KeyEvent& key) {
    redraw();
    if (key.keyCode() == KeyCode::C && editing_ >= 0) {
      setClipboardText(palette_->colorIndex(editing_).encode());
      return true;
    }
    else if (key.keyCode() == KeyCode::V && editing_ >= 0) {
      Brush color;
      color.decode(readClipboardText());
      palette_->setEditColor(editing_, color);
      return true;
    }
    if ((key.keyCode() == KeyCode::Delete || key.keyCode() == KeyCode::KPBackspace) && editing_ >= 0) {
      palette_->removeColor(editing_);
      setColorListHeight();
      editing_ = -1;
      return true;
    }

    return false;
  }

  float PaletteColorEditor::colorHeight() {
    int num_colors = palette_->numColors();
    float color_height = (height() - width() + kColorSpacing) * 1.0f / (num_colors + 1);
    color_height = std::min<float>(color_height, width() * kPaletteWidthRatio + kColorSpacing);
    return std::max(color_height, kMinColorHeight);
  }

  void PaletteColorEditor::setEditingGradient(bool gradient) {
    editing_gradient_ = gradient;
    setColorPickerBounds();
  }

  PaletteValueEditor::PaletteValueEditor(Palette* palette) : palette_(palette) {
    for (auto& text_editor : text_editors_) {
      text_editor.setTextFieldEntry();
      text_editor.setDefaultText("Not Set");
      text_editor.setMargin(8, 0);
      text_editor.setFont(Font(kValueIdHeight / 3, fonts::Lato_Regular_ttf));
      addScrolledChild(&text_editor, false);
    }
  }

  void PaletteValueEditor::draw(Canvas& canvas) {
    int w = width();
    int h = height();
    if (w <= 0 || h <= 0)
      return;

    canvas.setColor(0xff333639);
    canvas.rectangle(0, 0, w, h);

    std::map<std::string, std::vector<theme::ValueId>> value_ids = palette_->valueIdList(current_override_id_);
    int font_height = kValueIdHeight / 3.0f;
    int label_offset = kValueIdHeight / 4.0f;
    Font font(font_height, fonts::Lato_Regular_ttf);
    int id_width = width();
    canvas.saveState();
    canvas.setPosition(0, -yPosition());
    canvas.setClampBounds(0, yPosition(), 2 * w / 3, std::max(0, h - w));
    int index = 0;
    for (const auto& group : value_ids) {
      canvas.setClampBounds(0, yPosition(), w, h);
      int y = kValueIdHeight * index;

      canvas.setColor(0xff111111);
      canvas.roundedRectangle(label_offset, y + label_offset, id_width - 2 * label_offset,
                              kValueIdHeight - 2 * label_offset, 8);

      canvas.setColor(0xffffffff);
      canvas.text(group.first, font, Font::kCenter, 0, y, id_width, kValueIdHeight);
      index++;

      canvas.setClampBounds(0, yPosition(), std::max(0, 2 * w / 3 - kValueIdHeight / 4), h);
      if (isExpanded(group.first)) {
        for (theme::ValueId value_id : group.second) {
          y = kValueIdHeight * index;
          canvas.setColor(0xffffffff);
          canvas.text(theme::ValueId::name(value_id), font, Font::kLeft, label_offset, y, id_width,
                      kValueIdHeight);

          index++;
        }
      }
    }
    canvas.restoreState();
  }

  int PaletteValueEditor::listLength(const std::map<std::string, std::vector<theme::ValueId>>& value_ids) const {
    int length = value_ids.size();
    for (const auto& group : value_ids) {
      if (isExpanded(group.first))
        length += group.second.size();
    }
    return length;
  }

  void PaletteValueEditor::toggleGroup(const MouseEvent& e) {
    std::map<std::string, std::vector<theme::ValueId>> value_ids = palette_->valueIdList(current_override_id_);
    int y_position = std::min(height(), std::max(0.0f, e.position.y));
    int index = (y_position + yPosition()) / kValueIdHeight;

    for (const auto& group : value_ids) {
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
    std::map<std::string, std::vector<theme::ValueId>> value_ids = palette_->valueIdList(current_override_id_);
    int index = 0;
    int edit_height = kValueIdHeight * 3.0f / 4.0f;
    int y_offset = (kValueIdHeight - edit_height) / 2;
    int edit_width = width() / 3;
    int x = width() - edit_width - kValueIdHeight / 4;
    int y = kValueIdHeight + y_offset;

    for (const auto& group : value_ids) {
      if (isExpanded(group.first)) {
        for (theme::ValueId value_id : group.second) {
          text_editors_[index].onTextChange() = [this, index, value_id] {
            String text = String(text_editors_[index].text()).trim();
            if (text.isEmpty())
              palette_->removeValue(current_override_id_, value_id);
            else
              palette_->setValue(current_override_id_, value_id, text.toFloat());
            topParentFrame()->redrawAll();
          };

          text_editors_[index].setBounds(x, y, edit_width, edit_height);
          text_editors_[index].setVisible(true);

          float matched_value = 0.0f;
          if (palette_->value(current_override_id_, value_id, matched_value))
            text_editors_[index].setText(String(matched_value).toUtf8());
          else
            text_editors_[index].setText("");

          text_editors_[index].setDefaultText(String(theme::ValueId::defaultValue(value_id)).toUtf8());

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
    int list_length = listLength(palette_->valueIdList(current_override_id_));
    setScrollableHeight(kValueIdHeight * list_length, height());
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