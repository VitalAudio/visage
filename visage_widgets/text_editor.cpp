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

#include "text_editor.h"

#include "embedded/fonts.h"
#include "visage_graphics/text.h"
#include "visage_graphics/theme.h"
#include "visage_utils/string_utils.h"

namespace visage {
  VISAGE_THEME_IMPLEMENT_COLOR(TextEditor, TextEditorBackground, 0xff2c3033);
  VISAGE_THEME_IMPLEMENT_COLOR(TextEditor, TextEditorBorder, 0);
  VISAGE_THEME_IMPLEMENT_COLOR(TextEditor, TextEditorText, 0xffeeeeee);
  VISAGE_THEME_IMPLEMENT_COLOR(TextEditor, TextEditorDefaultText, 0xff848789);
  VISAGE_THEME_IMPLEMENT_COLOR(TextEditor, TextEditorCaret, 0xffffffff);
  VISAGE_THEME_IMPLEMENT_COLOR(TextEditor, TextEditorSelection, 0x22ffffff);

  VISAGE_THEME_IMPLEMENT_VALUE(TextEditor, TextEditorRounding, 5.0f);
  VISAGE_THEME_IMPLEMENT_VALUE(TextEditor, TextEditorMarginX, 9.0f);
  VISAGE_THEME_IMPLEMENT_VALUE(TextEditor, TextEditorMarginY, 9.0f);

  char32_t acuteAccentDeadKey(char32_t original) {
    static const std::map<char32_t, char32_t> lookup_map = {
      { U'a', U'\u00E1' }, { U'A', U'\u00C1' }, { U'c', U'\u0107' }, { U'C', U'\u0106' },
      { U'l', U'\u013A' }, { U'L', U'\u0139' }, { U'e', U'\u00E9' }, { U'E', U'\u00C9' },
      { U'i', U'\u00ED' }, { U'I', U'\u00CD' }, { U'o', U'\u00F3' }, { U'O', U'\u00D3' },
      { U'n', U'\u0144' }, { U'N', U'\u0143' }, { U'u', U'\u00FA' }, { U'U', U'\u00DA' },
    };

    if (lookup_map.count(original))
      return lookup_map.at(original);
    return original;
  }

  char32_t graveAccentDeadKey(char32_t original) {
    static const std::map<char32_t, char32_t> lookup_map = {
      { U'a', U'\u00E0' }, { U'A', U'\u00C0' }, { U'e', U'\u00E8' }, { U'E', U'\u00C8' },
      { U'i', U'\u00EC' }, { U'I', U'\u00CC' }, { U'o', U'\u00F2' }, { U'O', U'\u00D2' },
      { U'u', U'\u00F9' }, { U'U', U'\u00D9' },
    };

    if (lookup_map.count(original))
      return lookup_map.at(original);
    return original;
  }

  char32_t tildeDeadKey(char32_t original) {
    static const std::map<char32_t, char32_t> lookup_map = {
      { U'a', U'\u00E3' }, { U'A', U'\u00C3' }, { U'n', U'\u00F1' },
      { U'N', U'\u00D1' }, { U'o', U'\u00F5' }, { U'O', U'\u00D5' },
    };

    if (lookup_map.count(original))
      return lookup_map.at(original);
    return original;
  }

  char32_t umlautDeadKey(char32_t original) {
    static const std::map<char32_t, char32_t> lookup_map = {
      { U'a', U'\u00E4' }, { U'A', U'\u00C4' }, { U'e', U'\u00EB' }, { U'E', U'\u00CB' },
      { U'i', U'\u00EF' }, { U'I', U'\u00CF' }, { U'o', U'\u00F6' }, { U'O', U'\u00D6' },
      { U'u', U'\u00FC' }, { U'U', U'\u00DC' }, { U'y', U'\u00FF' }, { U'Y', U'\u0178' },
    };

    if (lookup_map.count(original))
      return lookup_map.at(original);
    return original;
  }

  char32_t circumflexDeadKey(char32_t original) {
    static const std::map<char32_t, char32_t> lookup_map = {
      { U'a', U'\u00E2' }, { U'A', U'\u00C2' }, { U'e', U'\u00EA' }, { U'E', U'\u00CA' },
      { U'i', U'\u00EE' }, { U'I', U'\u00CE' }, { U'o', U'\u00F4' }, { U'O', U'\u00D4' },
      { U'u', U'\u00FB' }, { U'U', U'\u00DB' },
    };

    if (lookup_map.count(original))
      return lookup_map.at(original);
    return original;
  }

  TextEditor::TextEditor(const std::string& name) : ScrollableFrame(name) {
    undo_history_.reserve(kMaxUndoHistory);
    undone_history_.reserve(kMaxUndoHistory);

    setAcceptsKeystrokes(true);
    text_.setFont(Font(10, fonts::Lato_Regular_ttf, 1.0f));
    default_text_.setFont(Font(10, fonts::Lato_Regular_ttf, 1.0f));
  }

  void TextEditor::drawBackground(Canvas& canvas) const {
    canvas.setColor(background_color_id_);
    canvas.roundedRectangle(0, 0, width(), height(), background_rounding_);
    canvas.setColor(TextEditorBorder);
    canvas.roundedRectangleBorder(0, 0, width(), height(), background_rounding_, 2.0f);
  }

  void TextEditor::selectionRectangle(Canvas& canvas, float x, float y, float w, float h) const {
    float left = std::max(0.0f, std::min(width(), x));
    float top = std::max(0.0f, std::min(height(), y));
    float right = std::max(0.0f, std::min(width(), x + w));
    float bottom = std::max(0.0f, std::min(height(), y + h));
    canvas.rectangle(left, top, right - left, bottom - top);
  }

  void TextEditor::drawSelection(Canvas& canvas) const {
    int selection_start = selectionStart();
    float line_height = font().lineHeight();

    std::pair<float, float> start_position = selection_start_point_;
    std::pair<float, float> end_position = selection_end_point_;

    float y_offset;
    if (justification() & Font::kTop)
      y_offset = -yPosition();
    else if (justification() & Font::kBottom)
      y_offset = yPosition();
    else {
      int num_lines = line_breaks_.size() + 1;
      y_offset = (height() - num_lines * line_height) * 0.5f - yPosition();
    }

    canvas.setColor(TextEditorCaret);
    if (caret_position_ == selection_start)
      selectionRectangle(canvas, start_position.first - x_position_,
                         start_position.second + y_offset, 1.0f, line_height);
    else
      selectionRectangle(canvas, end_position.first - x_position_, end_position.second + y_offset,
                         1.0f, line_height);

    canvas.setColor(TextEditorSelection);
    if (start_position.second == end_position.second) {
      float width = end_position.first - start_position.first;
      selectionRectangle(canvas, start_position.first - x_position_,
                         start_position.second + y_offset, width, line_height);
    }
    else {
      float x_margin = xMargin();
      float total_width = width();
      float start_width = total_width - start_position.first - x_margin;
      selectionRectangle(canvas, start_position.first - x_position_,
                         start_position.second + y_offset, start_width, line_height);
      selectionRectangle(canvas, x_margin - x_position_, end_position.second + y_offset,
                         end_position.first - x_margin, line_height);

      float block_height = end_position.second - start_position.second - line_height;
      if (block_height > 0)
        selectionRectangle(canvas, x_margin - x_position_, start_position.second + line_height + y_offset,
                           total_width - 2 * x_margin, block_height);
    }
  }

  void TextEditor::draw(Canvas& canvas) {
    drawBackground(canvas);
    float x_margin = xMargin();
    Bounds text_bounds(x_margin, 0.0f, width() - 2.0f * x_margin, std::max(height(), scrollableHeight()));

    if (hasKeyboardFocus())
      drawSelection(canvas);

    canvas.setPosition(0, yMargin());
    if (text_.text().isEmpty()) {
      bool center = (justification() & Font::kLeft) == 0 && (justification() & Font::kRight) == 0;
      if (!default_text_.text().isEmpty() && (!center || !hasKeyboardFocus())) {
        canvas.setColor(TextEditorDefaultText);
        canvas.text(&default_text_, x_margin - x_position_, -yPosition(),
                    x_position_ + text_bounds.width(), text_bounds.height());
      }
    }
    else {
      canvas.setColor(TextEditorText);
      if (justification() & Font::kLeft) {
        canvas.text(&text_, x_margin - x_position_, -yPosition(), x_position_ + text_bounds.width(),
                    text_bounds.height());
      }
      else if (justification() & Font::kRight) {
        canvas.text(&text_, 0, -yPosition(), x_margin + text_bounds.width() - x_position_,
                    text_bounds.height());
      }
      else {
        canvas.setPosition(-x_position_, 0.0f);
        float expansion = std::abs(x_position_);
        canvas.text(&text_, -expansion, -yPosition(), text_bounds.width() + 2 * expansion,
                    text_bounds.height());
      }
    }
  }

  std::pair<float, float> TextEditor::indexToPosition(int index) const {
    int line = 0;
    float line_height = font().lineHeight();

    while (line < line_breaks_.size() && index >= line_breaks_[line])
      line++;

    std::pair<int, int> range = lineRange(line);

    float pre_width = font().stringWidth(text_.text().c_str() + range.first, index - range.first,
                                         text_.characterOverride());
    float full_width = font().stringWidth(text_.text().c_str() + range.first,
                                          range.second - range.first, text_.characterOverride());

    float line_x = (width() - full_width) / 2.0f;
    float x_margin = xMargin();
    if (justification() & Font::kLeft)
      line_x = x_margin;
    else if (justification() & Font::kRight)
      line_x = width() - x_margin - full_width;

    return { line_x + pre_width, line_height * line + yMargin() };
  }

  std::pair<int, int> TextEditor::lineRange(int line) const {
    int start_index = 0;
    int end_index = textLength();
    line = std::max(line, 0);

    if (line > 0)
      start_index = line_breaks_[line - 1];
    if (line < line_breaks_.size()) {
      end_index = line_breaks_[line];

      if (Font::isNewLine(text_.text()[end_index - 1]))
        end_index--;
    }

    return { start_index, end_index };
  }

  int TextEditor::positionToIndex(const std::pair<float, float>& position) const {
    float line_height = font().lineHeight();
    int line = std::min<int>(line_breaks_.size(), (position.second - yMargin()) / line_height);
    std::pair<int, int> range = lineRange(line);

    float full_width = font().stringWidth(text_.text().c_str() + range.first,
                                          range.second - range.first, text_.characterOverride());

    float line_x = (width() - full_width) * 0.5f;
    float x_margin = xMargin();
    if (justification() & Font::kLeft)
      line_x = x_margin;
    else if (justification() & Font::kRight)
      line_x = width() - x_margin - full_width;

    int index = font().widthOverflowIndex(text_.text().c_str() + range.first, range.second - range.first,
                                          position.first - line_x, true, text_.characterOverride());
    return std::min(range.first + index, range.second);
  }

  bool TextEditor::enterPressed() {
    if (!active_)
      return false;

    if (text_.multiLine()) {
      addUndoPosition();
      action_state_ = kInserting;
      insertTextAtCaret(U"\n");
    }
    else
      on_enter_key_.callback();
    return true;
  }

  bool TextEditor::escapePressed() {
    deselect();
    cancel();
    return true;
  }

  void TextEditor::cancel() {
    on_escape_key_.callback();
  }

  void TextEditor::deselect() {
    selection_position_ = caret_position_;
    makeCaretVisible();
  }

  void TextEditor::clear() {
    selectAll();
    deleteSelected();
  }

  void TextEditor::deleteSelected() {
    undone_history_.clear();

    if (action_state_ != kDeleting)
      addUndoPosition();
    action_state_ = kDeleting;

    String before = text_.text().substring(0, selectionStart());
    String after = text_.text().substring(selectionEnd());
    text_.setText(before + after);
    setLineBreaks();
    caret_position_ = selectionStart();
    selection_position_ = caret_position_;
    makeCaretVisible();

    on_text_change_.callback();
  }

  void TextEditor::makeCaretVisible() {
    if (font().packedFont() == nullptr || width() == 0.0f || height() == 0.0f)
      return;

    if (text_.multiLine()) {
      float min_view = yMargin() + yPosition();
      float max_view = yPosition() + height() - font().lineHeight();

      std::pair<float, float> caret_location = indexToPosition(caret_position_);
      if (caret_location.second < min_view)
        setYPosition(caret_location.second);
      else if (caret_location.second > max_view)
        setYPosition(caret_location.second - height() + font().lineHeight());
    }
    else {
      float line_width = font().stringWidth(text_.text().c_str(), textLength(), text_.characterOverride());
      float x_margin = xMarginSize();
      float min_view = x_position_ + x_margin;
      float max_view = x_position_ + width() - x_margin;

      if (line_width <= width() - 2 * x_margin) {
        min_view = x_margin;
        max_view = width() - x_margin;
        x_position_ = 0.0f;
      }
      else {
        float max = (line_width - width()) / 2.0f + x_margin;
        float min = (width() - line_width) / 2.0f - x_margin;
        if (justification() & Font::kLeft) {
          max = line_width - width() + 2.0f * x_margin;
          min = 0;
        }
        else if (justification() & Font::kRight) {
          min = -line_width - 2 * x_margin;
          max = width();
        }
        x_position_ = std::min(x_position_, max);
        x_position_ = std::max(x_position_, min);
      }

      std::pair<float, float> caret_location = indexToPosition(caret_position_);
      if (caret_location.first < min_view)
        x_position_ = caret_location.first - x_margin;
      else if (caret_location.first > max_view)
        x_position_ = caret_location.first - width() + x_margin;
    }

    setViewBounds();
    selection_start_point_ = indexToPosition(selectionStart());
    selection_end_point_ = indexToPosition(selectionEnd());
    redraw();
  }

  void TextEditor::setViewBounds() {
    int num_lines = line_breaks_.size() + 1;
    float total_height = num_lines * font().lineHeight() + 2 * yMargin();
    setScrollableHeight(total_height);
  }

  String TextEditor::selection() const {
    int start = selectionStart();
    int end = selectionEnd();
    return text_.text().substring(start, end - start);
  }

  int TextEditor::beginningOfWord() const {
    int index = caret_position_ - 1;
    while (index > 0 && isVariableCharacter(text_.text()[index - 1]))
      --index;
    return std::max(0, index);
  }

  int TextEditor::endOfWord() const {
    int string_length = textLength();
    int index = caret_position_ + 1;
    while (index < string_length && isVariableCharacter(text_.text()[index]))
      ++index;
    return std::min(string_length, index);
  }

  void TextEditor::mouseEnter(const MouseEvent& e) {
    setCursorStyle(MouseCursor::IBeam);
  }

  void TextEditor::mouseExit(const MouseEvent& e) {
    setCursorStyle(MouseCursor::Arrow);
  }

  void TextEditor::mouseDown(const MouseEvent& e) {
    dead_key_entry_ = DeadKey::None;
    action_state_ = kNone;
    int click = e.repeatClickCount() % 3;
    if (click == 0)
      tripleClick(e);
    else if (click == 2)
      doubleClick(e);
    else if (!mouse_focus_) {
      caret_position_ = positionToIndex({ e.position.x + x_position_, e.position.y + yPosition() });
      if (!e.isShiftDown())
        selection_position_ = caret_position_;
      makeCaretVisible();
    }

    redraw();
  }

  void TextEditor::mouseUp(const MouseEvent& e) {
    mouse_focus_ = false;
  }

  void TextEditor::mouseDrag(const MouseEvent& e) {
    if (!mouse_focus_)
      caret_position_ = positionToIndex({ e.position.x + x_position_, e.position.y + yPosition() });
    makeCaretVisible();
    redraw();
  }

  void TextEditor::doubleClick(const MouseEvent& e) {
    if (mouse_focus_)
      return;

    if (text_.characterOverride()) {
      selectAll();
      return;
    }

    caret_position_ = positionToIndex({ e.position.x + x_position_, e.position.y + yPosition() });
    selection_position_ = beginningOfWord();
    caret_position_ = endOfWord();
    makeCaretVisible();
  }

  void TextEditor::tripleClick(const MouseEvent& e) {
    int line = std::min<int>(line_breaks_.size(),
                             (e.position.y - yMargin() + yPosition()) / font().lineHeight());
    std::pair<int, int> range = lineRange(line);
    selection_position_ = range.first;
    caret_position_ = range.second;
    makeCaretVisible();
  }

  bool TextEditor::handleDeadKey(const KeyEvent& key) {
    switch (key.key_code) {
    case KeyCode::E:
      insertTextAtCaret(kAcuteAccentCharacter);
      dead_key_entry_ = DeadKey::AcuteAccent;
      break;
    case KeyCode::Grave:
      insertTextAtCaret(kGraveAccentCharacter);
      dead_key_entry_ = DeadKey::GraveAccent;
      break;
    case KeyCode::N:
      insertTextAtCaret(kTildeCharacter);
      dead_key_entry_ = DeadKey::Tilde;
      break;
    case KeyCode::U:
      insertTextAtCaret(kUmlautCharacter);
      dead_key_entry_ = DeadKey::Umlaut;
      break;
    case KeyCode::I:
      insertTextAtCaret(kCircumflexCharacter);
      dead_key_entry_ = DeadKey::Circumflex;
      break;
    default: dead_key_entry_ = DeadKey::None; break;
    }
    if (dead_key_entry_ != DeadKey::None) {
      selection_position_ = caret_position_ - 1;
      makeCaretVisible();
    }

    return dead_key_entry_ != DeadKey::None;
  }

  bool TextEditor::keyPress(const KeyEvent& key) {
    redraw();

    bool modifier = key.isMainModifier();
    if (key.isAltDown()) {
      if (!modifier)
        handleDeadKey(key);
      return !modifier;
    }

    KeyCode code = key.keyCode();
    bool shift = key.isShiftDown();
    bool just_modifier = modifier && !shift;
    bool just_shift = shift && !modifier;

    if (just_modifier && code == KeyCode::A)
      return selectAll();
    if (just_modifier && code == KeyCode::C)
      return copyToClipboard();
    if (modifier && code == KeyCode::Z) {
      if (shift)
        return redo();
      return undo();
    }
    if (just_modifier && code == KeyCode::Y)
      return redo();
    if (just_modifier && code == KeyCode::Insert)
      return copyToClipboard();
    if (just_modifier && code == KeyCode::V)
      return pasteFromClipboard();
    if (just_shift && code == KeyCode::Insert)
      return pasteFromClipboard();
    if (just_modifier && code == KeyCode::X)
      return cutToClipboard();
    if (code == KeyCode::Delete) {
      if (just_shift)
        return cutToClipboard();
      return deleteForwards(modifier);
    }
    if (code == KeyCode::Backspace)
      return deleteBackwards(modifier);
    if (code == KeyCode::Tab) {
      if (shift)
        return focusPreviousTextReceiver();
      return focusNextTextReceiver();
    }
    if (code == KeyCode::Up) {
      if (modifier && shift)
        return false;
      if (modifier)
        return scrollUp();
      return moveCaretUp(shift);
    }
    if (code == KeyCode::Down) {
      if (modifier && shift)
        return false;
      if (modifier)
        return scrollDown();
      return moveCaretDown(shift);
    }
    if (code == KeyCode::PageUp) {
      if (modifier)
        return false;
      return pageUp(shift);
    }
    if (code == KeyCode::PageDown) {
      if (modifier)
        return false;
      return pageDown(shift);
    }
    if (code == KeyCode::Left)
      return moveCaretLeft(modifier, shift);
    if (code == KeyCode::Right)
      return moveCaretRight(modifier, shift);
    if (code == KeyCode::Home) {
      if (modifier)
        return moveCaretToTop(shift);
      return moveCaretToStartOfLine(shift);
    }
    if (code == KeyCode::End) {
      if (modifier)
        return moveCaretToEnd(shift);
      return moveCaretToEndOfLine(shift);
    }
    if (code == KeyCode::Return)
      return enterPressed();
    if (code == KeyCode::Escape)
      return escapePressed();

    return key.modifierMask() == 0 || key.modifierMask() == kModifierShift;
  }

  bool TextEditor::keyRelease(const KeyEvent& key) {
    return active_;
  }

  String TextEditor::translateDeadKeyText(const String& text) const {
    if (text.length() != 1 || dead_key_entry_ == DeadKey::None)
      return text;

    char32_t character = text[0];
    switch (dead_key_entry_) {
    case DeadKey::AcuteAccent: return acuteAccentDeadKey(character);
    case DeadKey::GraveAccent: return graveAccentDeadKey(character);
    case DeadKey::Tilde: return tildeDeadKey(character);
    case DeadKey::Umlaut: return umlautDeadKey(character);
    case DeadKey::Circumflex: return circumflexDeadKey(character);
    default: return text;
    }
  }

  void TextEditor::textInput(const std::string& text) {
    static constexpr unsigned char kMinChar = 32;

    unsigned char first = text[0];
    if (!active_ || first < kMinChar)
      return;

    insertTextAtCaret(text);
  }

  void TextEditor::focusChanged(bool is_focused, bool was_clicked) {
    redraw();
    if (!is_focused) {
      if (dead_key_entry_ != DeadKey::None) {
        dead_key_entry_ = DeadKey::None;
        selection_position_ = caret_position_;
      }
      return;
    }

    if (!was_clicked)
      makeCaretVisible();

    if (select_on_focus_) {
      mouse_focus_ = was_clicked;
      selectAll();
    }
  }

  bool TextEditor::moveCaretLeft(bool modifier, bool shift) {
    if (caret_position_ != selection_position_ && !shift)
      caret_position_ = selection_position_ = selectionStart();
    else if (modifier)
      caret_position_ = beginningOfWord();
    else
      caret_position_ = std::max(0, caret_position_ - 1);

    if (!shift)
      selection_position_ = caret_position_;

    makeCaretVisible();
    return true;
  }

  bool TextEditor::moveCaretRight(bool modifier, bool shift) {
    if (caret_position_ != selection_position_ && !shift)
      caret_position_ = selection_position_ = selectionEnd();
    else if (modifier)
      caret_position_ = endOfWord();
    else
      caret_position_ = std::min(textLength(), caret_position_ + 1);

    if (!shift)
      selection_position_ = caret_position_;

    makeCaretVisible();
    return true;
  }

  void TextEditor::moveCaretVertically(bool shift, float y_offset) {
    std::pair<float, float> position = indexToPosition(caret_position_);
    position.second += y_offset + font().lineHeight() * 0.5f;
    caret_position_ = positionToIndex(position);

    if (!shift)
      selection_position_ = caret_position_;

    makeCaretVisible();
  }

  bool TextEditor::moveCaretUp(bool shift) {
    moveCaretVertically(shift, -font().lineHeight());
    return true;
  }

  bool TextEditor::moveCaretDown(bool shift) {
    moveCaretVertically(shift, font().lineHeight());
    return true;
  }

  bool TextEditor::moveCaretToTop(bool shift) {
    caret_position_ = 0;

    if (!shift)
      selection_position_ = caret_position_;

    makeCaretVisible();
    return true;
  }

  bool TextEditor::moveCaretToStartOfLine(bool shift) {
    std::pair<float, float> position = indexToPosition(caret_position_);
    position.first = 0;
    caret_position_ = positionToIndex(position);

    if (!shift)
      selection_position_ = caret_position_;

    makeCaretVisible();
    return true;
  }

  bool TextEditor::moveCaretToEnd(bool shift) {
    caret_position_ = textLength();

    if (!shift)
      selection_position_ = caret_position_;

    makeCaretVisible();
    return true;
  }

  bool TextEditor::moveCaretToEndOfLine(bool shift) {
    std::pair<float, float> position = indexToPosition(caret_position_);
    position.first = width();
    caret_position_ = positionToIndex(position);

    if (!shift)
      selection_position_ = caret_position_;

    makeCaretVisible();
    return true;
  }

  bool TextEditor::pageUp(bool shift) {
    if (!text_.multiLine())
      return false;

    moveCaretVertically(shift, -height());
    return true;
  }

  bool TextEditor::pageDown(bool shift) {
    if (!text_.multiLine())
      return false;

    moveCaretVertically(shift, height());
    return true;
  }

  bool TextEditor::copyToClipboard() {
    if (text_.characterOverride())
      return false;

    String selected = selection();

    if (!selected.isEmpty())
      setClipboardText(selected.toUtf8());
    return true;
  }

  bool TextEditor::cutToClipboard() {
    if (text_.characterOverride() || !active_)
      return false;

    copyToClipboard();
    deleteSelected();
    return true;
  }

  bool TextEditor::pasteFromClipboard() {
    addUndoPosition();
    action_state_ = kInserting;
    insertTextAtCaret(readClipboardText());
    return true;
  }

  bool TextEditor::deleteBackwards(bool modifier) {
    if (caret_position_ == selection_position_) {
      if (modifier)
        selection_position_ = beginningOfWord();
      else
        selection_position_ = std::max(0, selectionEnd() - 1);
    }

    deleteSelected();
    return true;
  }

  bool TextEditor::deleteForwards(bool modifier) {
    if (caret_position_ == selection_position_) {
      if (modifier)
        selection_position_ = endOfWord();
      else
        selection_position_ = std::min<int>(textLength(), selectionStart() + 1);
    }

    deleteSelected();
    return true;
  }

  bool TextEditor::selectAll() {
    caret_position_ = 0;
    selection_position_ = textLength();
    makeCaretVisible();
    return true;
  }

  bool TextEditor::undo() {
    if (undo_history_.empty())
      return true;

    action_state_ = kNone;
    undone_history_.emplace_back(text_.text(), caret_position_);
    std::pair<String, int> state = undo_history_.back();
    undo_history_.pop_back();
    text_.setText(state.first);
    caret_position_ = state.second;
    selection_position_ = state.second;
    setLineBreaks();
    makeCaretVisible();
    on_text_change_.callback();
    return true;
  }

  bool TextEditor::redo() {
    if (undone_history_.empty())
      return false;

    addUndoPosition();
    std::pair<String, int> state = undone_history_.back();
    undone_history_.pop_back();
    text_.setText(state.first);
    caret_position_ = state.second;
    selection_position_ = state.second;
    setLineBreaks();
    makeCaretVisible();
    on_text_change_.callback();
    return true;
  }

  void TextEditor::insertTextAtCaret(const String& insert_text) {
    undone_history_.clear();
    String text = translateDeadKeyText(insert_text);
    if (dead_key_entry_ != DeadKey::None && text == insert_text)
      selection_position_ = caret_position_;

    dead_key_entry_ = DeadKey::None;
    if (!filtered_characters_.empty())
      text = text.removeCharacters(filtered_characters_);
    text = text.removeEmojiVariations();

    if (action_state_ != kInserting)
      addUndoPosition();
    action_state_ = kInserting;

    String before = text_.text().substring(0, selectionStart());
    String after = text_.text().substring(selectionEnd());
    int max_text = text.length();
    if (max_characters_)
      max_text = std::max(0, std::min<int>(max_text, max_characters_ - before.length() - after.length()));

    String trimmed = text.substring(0, max_text);
    text_.setText(before + trimmed + after);
    setLineBreaks();
    caret_position_ = before.length() + max_text;
    selection_position_ = caret_position_;
    makeCaretVisible();

    on_text_change_.callback();
    redraw();
  }

  void TextEditor::setNumberEntry() {
    setMultiLine(false);
    setSelectOnFocus(true);
    setJustification(Font::kCenter);
  }

  void TextEditor::setTextFieldEntry() {
    setMultiLine(false);
    setSelectOnFocus(true);
    setJustification(Font::kLeft);
  }
}