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

#include "visage_widgets/text_editor.h"

#include <catch2/catch_test_macros.hpp>

using namespace visage;

TEST_CASE("Text editor text input callbacks", "[widgets]") {
  TextEditor editor;
  int text_change_count = 0;
  std::string input_text;
  editor.onTextChange() += [&] { text_change_count++; };
  editor.onTextInput() += [&](const std::string& text) { input_text = text; };

  editor.processTextInput("a");
  REQUIRE(editor.text() == "a");
  REQUIRE(text_change_count == 1);
  REQUIRE(input_text == "a");
}

TEST_CASE("Text editor dead key at max characters", "[widgets]") {
  TextEditor editor;
  editor.setMaxCharacters(3);
  editor.setText("abc");

  editor.moveCaretToTop(false);
  editor.processKeyPress(KeyEvent(KeyCode::E, kModifierAlt, true));
  editor.processTextInput("e");
  REQUIRE(editor.text() == "abc");

  editor.moveCaretToEnd(false);
  editor.processKeyPress(KeyEvent(KeyCode::E, kModifierAlt, true));
  editor.processTextInput("e");
  REQUIRE(editor.text() == "abc");
}

TEST_CASE("Text editor dead key with filtered accent", "[widgets]") {
  TextEditor editor;
  editor.setFilteredCharacters("´");
  editor.setText("abc");

  editor.moveCaretToEnd(false);
  editor.processKeyPress(KeyEvent(KeyCode::E, kModifierAlt, true));
  editor.processTextInput("e");
  REQUIRE(editor.text() == "abce");
}

TEST_CASE("Text editor dead key", "[widgets]") {
  TextEditor editor;
  editor.setText("ab");

  editor.moveCaretToEnd(false);
  editor.processKeyPress(KeyEvent(KeyCode::E, kModifierAlt, true));
  editor.processTextInput("e");
  REQUIRE(editor.text() == "abé");
}

TEST_CASE("Text editor switching off multi-line clears line breaks", "[widgets]") {
  TextEditor editor;
  editor.setBounds(0, 0, 60, 200);
  editor.setMultiLine(true);
  editor.setText("one two three four five six seven eight");
  REQUIRE(editor.lineRange(1).first > 0);

  editor.setMultiLine(false);
  editor.setText("");
  REQUIRE(editor.lineRange(0) == std::pair<int, int>(0, 0));
}

TEST_CASE("Text editor line range past the last line", "[widgets]") {
  TextEditor editor;
  editor.setText("abc");
  REQUIRE(editor.lineRange(5) == std::pair<int, int>(0, 3));
}

TEST_CASE("Text editor inactive", "[widgets]") {
  TextEditor editor;
  editor.processTextInput("abc");
  editor.setActive(false);

  editor.processKeyPress(KeyEvent(KeyCode::Backspace, 0, true));
  editor.moveCaretToTop(false);
  editor.processKeyPress(KeyEvent(KeyCode::Delete, 0, true));
  editor.processKeyPress(KeyEvent(KeyCode::E, kModifierAlt, true));
  editor.processKeyPress(KeyEvent(KeyCode::Z, kModifierRegCtrl, true));
  REQUIRE(editor.text() == "abc");
}

TEST_CASE("Text editor single line input with newlines", "[widgets]") {
  TextEditor editor;
  editor.setMultiLine(false);
  editor.processTextInput("a\r\nb\nc");
  REQUIRE(editor.text() == "abc");

  editor.setMultiLine(true);
  editor.processTextInput("d\ne");
  REQUIRE(editor.text() == "abcd\ne");
}

TEST_CASE("Text editor end of a soft wrapped line keeps the caret on that line", "[widgets]") {
  TextEditor editor;
  editor.setBounds(0, 0, 60, 200);
  editor.setMultiLine(true);
  editor.setText("one two three four five six seven eight");
  std::pair<int, int> first_line = editor.lineRange(0);
  REQUIRE(first_line.second < editor.text().length());
  float first_line_y = editor.indexToPosition(0).second;

  editor.moveCaretToTop(false);
  editor.processKeyPress(KeyEvent(KeyCode::End, 0, true));
  int caret = editor.selectionStart();
  REQUIRE(caret == first_line.second - 1);
  REQUIRE(editor.indexToPosition(caret).second == first_line_y);

  REQUIRE(editor.positionToIndex({ 1000.0f, first_line_y }) == caret);
}

TEST_CASE("Text editor deleting nothing leaves the text and undo history alone", "[widgets]") {
  TextEditor editor;
  editor.setText("abc");
  int changes = 0;
  editor.onTextChange() += [&changes] { changes++; };

  editor.processTextInput("d");
  editor.undo();
  REQUIRE(editor.text() == "abc");
  REQUIRE(changes == 2);

  editor.moveCaretToEnd(false);
  editor.deleteForwards(false);
  editor.moveCaretToTop(false);
  editor.deleteBackwards(false);
  editor.cutToClipboard();
  REQUIRE(editor.text() == "abc");
  REQUIRE(changes == 2);

  editor.redo();
  REQUIRE(editor.text() == "abcd");
}

TEST_CASE("Text editor word deletion handles non-ASCII letters and runs of spaces", "[widgets]") {
  TextEditor editor;
  editor.setText("one naïve");
  editor.deleteBackwards(true);
  REQUIRE(editor.text() == "one ");

  editor.setText("one   two");
  editor.deleteBackwards(true);
  REQUIRE(editor.text() == "one   ");
  editor.deleteBackwards(true);
  REQUIRE(editor.text() == "");

  editor.setText("one   two");
  editor.moveCaretToTop(false);
  editor.deleteForwards(true);
  REQUIRE(editor.text() == "   two");
  editor.deleteForwards(true);
  REQUIRE(editor.text() == "");
}
