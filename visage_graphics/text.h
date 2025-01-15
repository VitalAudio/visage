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

#include "font.h"
#include "visage_utils/string_utils.h"

namespace visage {
  class Canvas;

  class Text {
  public:
    Text() = default;
    explicit Text(const String& text, const Font& font,
                  Font::Justification justification = Font::kCenter, bool multi_line = false) :
        text_(text), font_(font), justification_(justification), multi_line_(multi_line) { }
    virtual ~Text() = default;

    void setText(const String& text) { text_ = text; }
    const String& text() const { return text_; }

    void setFont(const Font& font) { font_ = font; }
    const Font& font() const { return font_; }

    void setJustification(Font::Justification justification) { justification_ = justification; }
    Font::Justification justification() const { return justification_; }

    void setMultiLine(bool multi_line) { multi_line_ = multi_line; }
    bool multiLine() const { return multi_line_; }

    void setCharacterOverride(int character) { character_override_ = character; }
    int characterOverride() const { return character_override_; }

  private:
    String text_;
    Font font_;
    Font::Justification justification_ = Font::kCenter;
    bool multi_line_ = false;
    int character_override_ = 0;
  };
}