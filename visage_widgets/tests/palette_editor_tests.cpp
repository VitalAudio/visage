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

#include "visage_widgets/palette_editor.h"

#include <catch2/catch_test_macros.hpp>

using namespace visage;

TEST_CASE("PaletteColorEditor edited palette changed", "[widgets]") {
  Palette first_palette;
  first_palette.addColor();
  Palette second_palette;

  PaletteColorEditor editor(&first_palette);
  editor.setBounds(0, 0, 400, 800);
  MouseEvent mouse_event;
  mouse_event.position = { 1, 1 };
  editor.mouseDown(mouse_event);
  editor.mouseUp(mouse_event);

  editor.setEditedPalette(&second_palette);
  editor.keyPress(KeyEvent(KeyCode::Delete, 0, true));
  REQUIRE(first_palette.numColors() == 1);
  REQUIRE(second_palette.numColors() == 0);
}

static int visibleTextEditors(Frame* frame) {
  int count = 0;
  for (Frame* child : frame->children()) {
    if (!child->isVisible())
      continue;
    if (dynamic_cast<TextEditor*>(child))
      count++;
    count += visibleTextEditors(child);
  }
  return count;
}

TEST_CASE("PaletteValueEditor edits palettes with more than 500 values", "[widgets]") {
  Palette palette;
  for (int i = 0; i < 600; ++i) {
    theme::ValueId value_id = theme::ValueId::nextId("Value" + std::to_string(i),
                                                     "tests/many_values.cpp", 1.0f);
    palette.setValue(value_id, 2.0f);
  }

  auto editor = std::make_unique<PaletteValueEditor>(&palette);
  editor->setBounds(0, 0, 400, 800);
  editor->toggleExpandGroup("many_values");
  REQUIRE(visibleTextEditors(editor.get()) == 600);

  editor->toggleExpandGroup("many_values");
  REQUIRE(visibleTextEditors(editor.get()) == 0);
}
