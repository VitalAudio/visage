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