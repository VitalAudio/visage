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

#include "visage_graphics/color.h"
#include "visage_graphics/font.h"
#include "visage_ui/frame.h"
#include "visage_ui/scroll_bar.h"

namespace visage {
  class TextEditor : public ScrollableComponent {
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

    THEME_DEFINE_COLOR(TextEditorBackground);
    THEME_DEFINE_COLOR(TextEditorBorder);
    THEME_DEFINE_COLOR(TextEditorText);
    THEME_DEFINE_COLOR(TextEditorDefaultText);
    THEME_DEFINE_COLOR(TextEditorCaret);
    THEME_DEFINE_COLOR(TextEditorSelection);

    THEME_DEFINE_VALUE(TextEditorRounding);
    THEME_DEFINE_VALUE(TextEditorMarginX);
    THEME_DEFINE_VALUE(TextEditorMarginY);

    class Listener {
    public:
      virtual ~Listener() = default;
      virtual void textEditorEnterPressed(TextEditor* editor) { }
      virtual void textEditorEscapePressed(TextEditor* editor) { }
      virtual void textEditorFocusLost(TextEditor* editor) { }
      virtual void textEditorFocusGained(TextEditor* editor) { }
      virtual void textEditorChanged(TextEditor* editor) { }
    };

    explicit TextEditor(const std::string& name = "");
    ~TextEditor() override = default;

    void drawBackground(Canvas& canvas) const;
    void selectionRectangle(Canvas& canvas, int x, int y, int w, int h) const;
    void drawSelection(Canvas& canvas) const;
    void draw(Canvas& canvas) override;

    std::pair<int, int> indexToPosition(int index) const;
    std::pair<int, int> lineRange(int line) const;
    int positionToIndex(const std::pair<int, int>& position) const;

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
      ScrollableComponent::resized();
      setBackgroundRounding(paletteValue(kTextEditorRounding));
      setMargin(paletteValue(kTextEditorMarginX), paletteValue(kTextEditorMarginY));
      setLineBreaks();
      makeCaretVisible();
    }

    void onMouseEnter(const MouseEvent& e) override;
    void onMouseExit(const MouseEvent& e) override;
    void onMouseDown(const MouseEvent& e) override;
    void onMouseDrag(const MouseEvent& e) override;
    void onMouseUp(const MouseEvent& e) override;
    void doubleClick(const MouseEvent& e);
    void tripleClick(const MouseEvent& e);
    bool handleDeadKey(const KeyEvent& key);
    bool onKeyPress(const KeyEvent& key) override;
    bool receivesTextInput() override { return active_; }
    visage::String translateDeadKeyText(const visage::String& text) const;
    void onTextInput(const std::string& text) override;
    void onFocusChange(bool is_focused, bool was_clicked) override;

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
    void setMargin(int x, int y) {
      if ((text_.justification() & Font::kLeft) || (text_.justification() & Font::kRight))
        x_margin_ = x;
      else
        x_margin_ = 0;
      if (text_.justification() & Font::kTop)
        y_margin_ = y;
      else
        y_margin_ = 0;
    }
    void setPassword(int character = kDefaultPasswordCharacter) {
      text_.setCharacterOverride(character);
      if (character) {
        setMultiLine(false);
        setJustification(Font::Justification::kLeft);
      }
    }

    void setLineBreaks() {
      if (text_.multiLine() && text_.font().packedFont())
        line_breaks_ = text_.font().lineBreaks(text_.text().c_str(), text_.text().length(),
                                               width() - 2 * x_margin_);
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
    }
    void setSelectOnFocus(bool select_on_focus) { select_on_focus_ = select_on_focus; }
    void setJustification(Font::Justification justification) {
      text_.setJustification(justification);
      default_text_.setJustification(justification);
    }
    void setFont(const Font& font) {
      text_.setFont(font);
      default_text_.setFont(font);
      setLineBreaks();
      makeCaretVisible();
    }
    void setActive(bool active) { active_ = active; }
    void addListener(Listener* listener) { listeners_.push_back(listener); }

    void setNumberEntry();
    void setTextFieldEntry();

    const String& text() const { return text_.text(); }
    int textLength() const { return text_.text().length(); }
    const Font& font() const { return text_.font(); }
    Font::Justification justification() const { return text_.justification(); }
    void setBackgroundColorId(int color_id) { background_color_id_ = color_id; }

    int xMargin() const { return x_margin_; }
    int yMargin() const { return y_margin_; }

  private:
    void addUndoPosition() { undo_history_.emplace_back(text_.text(), caret_position_); }

    DeadKey dead_key_entry_ = DeadKey::None;
    Text text_;
    Text default_text_;
    std::string filtered_characters_;
    std::vector<int> line_breaks_;
    std::vector<Listener*> listeners_;
    int caret_position_ = 0;
    int selection_position_ = 0;
    std::pair<int, int> selection_start_point_;
    std::pair<int, int> selection_end_point_;
    int max_characters_ = 0;

    bool select_on_focus_ = false;
    bool mouse_focus_ = false;
    bool active_ = true;

    int background_color_id_ = kTextEditorBackground;
    float background_rounding_ = 1.0f;
    int x_margin_ = 0;
    int y_margin_ = 0;
    int x_position_ = 0;

    ActionState action_state_ = kNone;
    std::vector<std::pair<String, int>> undo_history_;
    std::vector<std::pair<String, int>> undone_history_;

    VISAGE_LEAK_CHECKER(TextEditor)
  };
}