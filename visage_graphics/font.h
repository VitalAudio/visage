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

#include "color.h"
#include "graphics_utils.h"
#include "visage_file_embed/embedded_file.h"

#include <map>
#include <vector>

namespace visage {
  struct FontAtlasVertex {
    float x;
    float y;
    float coordinate_x;
    float coordinate_y;
  };

  class PackedFont;

  class Font {
  public:
    enum Justification {
      kCenter = 0,
      kLeft = 0x1,
      kRight = 0x2,
      kTop = 0x10,
      kBottom = 0x20,
      kTopLeft = kTop | kLeft,
      kBottomLeft = kBottom | kLeft,
      kTopRight = kTop | kRight,
      kBottomRight = kBottom | kRight,
    };

    static inline bool isPrintable(char32_t character) {
      return character != ' ' && character != '\t' && character != '\n';
    }
    static inline bool isNewLine(char32_t character) { return character == '\n'; }
    static inline bool isIgnored(char32_t character) { return character == '\r'; }
    static bool hasNewLine(const char32_t* string, int length);

    Font() = default;
    Font(int size, const char* font_data);
    Font(int size, const EmbeddedFile& file);
    Font(const Font& other);
    Font operator=(const Font& other);
    ~Font();

    int getWidthOverflowIndex(const char32_t* string, int string_length, float width,
                              bool round = false, int character_override = 0) const;
    float getStringWidth(const char32_t* string, int length, int character_override = 0) const;
    float getStringWidth(const std::u32string& string, int character_override = 0) const {
      return getStringWidth(string.c_str(), string.size(), character_override);
    }
    float getStringTop(const char32_t* string, int length) const;
    float getStringBottom(const char32_t* string, int length) const;
    int getLineHeight() const;
    float getCapitalHeight() const;
    float getLowerDip() const;

    int atlasWidth() const;
    inline int size() const { return size_; }
    inline const char* fontData() const { return font_data_; }
    const bgfx::TextureHandle& textureHandle() const;

    void setVertexPositions(FontAtlasVertex* vertices, const char32_t* string, int length, float x,
                            float y, float width, float height,
                            Justification justification = Justification::kCenter,
                            int character_override = 0) const;

    std::vector<int> getLineBreaks(const char32_t* string, int length, float width) const;

    void setMultiLineVertexPositions(FontAtlasVertex* vertices, const char32_t* string, int length,
                                     float x, float y, float width, float height,
                                     Justification justification = Justification::kCenter) const;

    void checkInit() const;
    const PackedFont* packedFont() const { return packed_font_; }

  private:
    int size_ = 0;
    const char* font_data_ = nullptr;
    PackedFont* packed_font_ = nullptr;
  };

  class FontCache {
  public:
    friend class Font;

    ~FontCache();

  private:
    static FontCache* getInstance() {
      static FontCache cache;
      return &cache;
    }

    static PackedFont* loadPackedFont(int size, const EmbeddedFile& font) {
      return getInstance()->createOrLoadPackedFont(size, font.data);
    }

    static PackedFont* loadPackedFont(int size, const char* font_data) {
      return getInstance()->createOrLoadPackedFont(size, font_data);
    }

    static void returnPackedFont(PackedFont* packed_font) {
      getInstance()->decrementPackedFont(packed_font);
    }

    FontCache();

    PackedFont* createOrLoadPackedFont(int size, const char* font_data);
    void decrementPackedFont(PackedFont*);

    std::map<std::pair<int, unsigned const char*>, std::unique_ptr<PackedFont>> cache_;
    std::map<PackedFont*, int> ref_count_;
  };
}