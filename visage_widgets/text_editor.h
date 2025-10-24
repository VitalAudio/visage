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

#include "visage_graphics/font.h"
#include "visage_ui/frame.h"
#include "visage_ui/scroll_bar.h"

namespace visage {
  class TextEditor : public ScrollableFrame {
  public:
    static constexpr int kDefaultPasswordCharacter = '*';
    static constexpr int kMaxUndoHistory = 1000;

    static constexpr char32_t kAcuteAccentCharacter = U'\u00B4';
    static constexpr char32_t kGraveAccentCharacter = U'\u0060';
    static constexpr char32_t kTildeCharacter = U'\u02DC';
    static constexpr char32_t kUmlautCharacter = U'\u00A8';
    static constexpr char32_t kCircumflexCharacter = U'\u02C6';

    static bool isAlphaNumeric(char character) {
      return (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z') ||
             (character >= '0' && character <= '9');
    }

    static bool isVariableCharacter(char character) {
      return isAlphaNumeric(character) || character == '_';
    }

    enum ActionState {
      kNone,
      kInserting,
      kDeleting,
    };

    enum class DeadKey {
      None,
      AcuteAccent,
      GraveAccent,
      Tilde,
      Umlaut,
      Circumflex
    };

    VISAGE_THEME_DEFINE_COLOR(TextEditorBackground);
    VISAGE_THEME_DEFINE_COLOR(TextEditorBorder);
    VISAGE_THEME_DEFINE_COLOR(TextEditorText);
    VISAGE_THEME_DEFINE_COLOR(TextEditorDefaultText);
    VISAGE_THEME_DEFINE_COLOR(TextEditorCaret);
    VISAGE_THEME_DEFINE_COLOR(TextEditorSelection);

    VISAGE_THEME_DEFINE_VALUE(TextEditorRounding);
    VISAGE_THEME_DEFINE_VALUE(TextEditorMarginX);
    VISAGE_THEME_DEFINE_VALUE(TextEditorMarginY);

    explicit TextEditor(const std::string& name = "");
    ~TextEditor() override = default;

    auto& onTextChange() { return on_text_change_; }
    auto& onEnterKey() { return on_enter_key_; }
    auto& onEscapeKey() { return on_escape_key_; }

    void drawBackground(Canvas& canvas) const;
    void selectionRectangle(Canvas& canvas, float x, float y, float w, float h) const;
    void drawSelection(Canvas& canvas) const;
    void draw(Canvas& canvas) override;

    std::pair<float, float> indexToPosition(int index) const;
    std::pair<int, int> lineRange(int line) const;
    int positionToIndex(const std::pair<float, float>& position) const;

    void cancel();
    void deselect();
    void clear();
    void deleteSelected();
    void makeCaretVisible();
    void setViewBounds();
    int selectionStart() const { return std::min(caret_position_, selection_position_); }
    int selectionEnd() const { return std::max(caret_position_, selection_position_); }
    String selection() const;

    int beginningOfWord() const;
    int endOfWord() const;

    void resized() override {
      ScrollableFrame::resized();
      setBackgroundRounding(paletteValue(TextEditorRounding));
      setLineBreaks();
      makeCaretVisible();
    }

    void dpiChanged() override {
      Font f = font().withDpiScale(dpiScale());
      text_.setFont(f);
      default_text_.setFont(f);
    }

    void mouseEnter(const MouseEvent& e) override;
    void mouseExit(const MouseEvent& e) override;
    void mouseDown(const MouseEvent& e) override;
    void mouseDrag(const MouseEvent& e) override;
    void mouseUp(const MouseEvent& e) override;
    void doubleClick(const MouseEvent& e);
    void tripleClick(const MouseEvent& e);
    bool handleDeadKey(const KeyEvent& key);
    bool keyPress(const KeyEvent& key) override;
    bool keyRelease(const KeyEvent& key) override;
    bool receivesTextInput() override { return active_; }
    String translateDeadKeyText(const String& text) const;
    void textInput(const std::string& text) override;
    void focusChanged(bool is_focused, bool was_clicked) override;

    bool moveCaretLeft(bool modifier, bool shift);
    bool moveCaretRight(bool modifier, bool shift);

    void moveCaretVertically(bool shift, float y_offset);
    bool enterPressed();
    bool escapePressed();
    bool moveCaretUp(bool shift);
    bool moveCaretDown(bool shift);
    bool moveCaretToTop(bool shift);
    bool moveCaretToStartOfLine(bool shift);
    bool moveCaretToEnd(bool shift);
    bool moveCaretToEndOfLine(bool shift);
    bool pageUp(bool shift);
    bool pageDown(bool shift);
    bool copyToClipboard();
    bool cutToClipboard();
    bool pasteFromClipboard();
    bool deleteBackwards(bool modifier);
    bool deleteForwards(bool modifier);
    bool selectAll();
    bool undo();
    bool redo();

    void insertTextAtCaret(const String& insert_text);

    void setBackgroundRounding(float rounding) {
      background_rounding_ = rounding;
      setScrollBarRounding(rounding);
    }
    void setMargin(float x, float y) {
      set_x_margin_ = x;
      set_y_margin_ = y;
    }

    float xMargin() const {
      if (text_.justification() & (Font::kLeft | Font::kRight))
        return xMarginSize();
      return 0;
    }
    float yMargin() const {
      if (text_.justification() & Font::kTop)
        return set_y_margin_ ? set_y_margin_ : paletteValue(TextEditorMarginY);
      return 0;
    }

    void setPassword(int character = kDefaultPasswordCharacter) {
      text_.setCharacterOverride(character);
      if (character) {
        setMultiLine(false);
        setJustification(Font::Justification::kLeft);
      }
    }

    void setLineBreaks() {
      if (text_.multiLine() && text_.font().packedFont()) {
        line_breaks_ = text_.font().lineBreaks(text_.text().c_str(), text_.text().length(),
                                               width() - 2 * xMargin());
      }
    }

    void setText(const String& text) {
      if (max_characters_)
        text_.setText(text.substring(0, max_characters_));
      else
        text_.setText(text);
      caret_position_ = text_.text().length();
      selection_position_ = caret_position_;
      setLineBreaks();
      makeCaretVisible();
    }

    void setFilteredCharacters(const std::string& characters) { filtered_characters_ = characters; }
    void setDefaultText(const String& default_text) { default_text_.setText(default_text); }
    void setMaxCharacters(int max) { max_characters_ = max; }
    void setMultiLine(bool multi_line) {
      text_.setMultiLine(multi_line);
      default_text_.setMultiLine(multi_line);
      if (multi_line)
        x_position_ = 0;
    }
    void setSelectOnFocus(bool select_on_focus) { select_on_focus_ = select_on_focus; }
    void setJustification(Font::Justification justification) {
      text_.setJustification(justification);
      default_text_.setJustification(justification);
    }
    void setFont(const Font& font) {
      Font f = font.withDpiScale(dpiScale());
      text_.setFont(f);
      default_text_.setFont(f);
      setLineBreaks();
      makeCaretVisible();
    }
    void setActive(bool active) { active_ = active; }
    void setNumberEntry();
    void setTextFieldEntry();

    const String& text() const { return text_.text(); }
    int textLength() const { return text_.text().length(); }
    const Font& font() const { return text_.font(); }
    Font::Justification justification() const { return text_.justification(); }
    void setBackgroundColorId(theme::ColorId color_id) { background_color_id_ = color_id; }

  private:
    float xMarginSize() const {
      return set_x_margin_ ? set_x_margin_ : paletteValue(TextEditorMarginX);
    }
    void addUndoPosition() { undo_history_.emplace_back(text_.text(), caret_position_); }

    CallbackList<void()> on_text_change_;
    CallbackList<void()> on_enter_key_;
    CallbackList<void()> on_escape_key_;

    DeadKey dead_key_entry_ = DeadKey::None;
    Text text_;
    Text default_text_;
    std::string filtered_characters_;
    std::vector<int> line_breaks_;
    int caret_position_ = 0;
    int selection_position_ = 0;
    std::pair<float, float> selection_start_point_;
    std::pair<float, float> selection_end_point_;
    int max_characters_ = 0;

    bool select_on_focus_ = false;
    bool mouse_focus_ = false;
    bool active_ = true;

    theme::ColorId background_color_id_ = TextEditorBackground;
    float background_rounding_ = 1.0f;
    float set_x_margin_ = 0.0f;
    float set_y_margin_ = 0.0f;
    float x_position_ = 0.0f;

    ActionState action_state_ = kNone;
    std::vector<std::pair<String, int>> undo_history_;
    std::vector<std::pair<String, int>> undone_history_;

    VISAGE_LEAK_CHECKER(TextEditor)
  };
}