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
  struct PackedGlyph {
    int atlas_left = -1;
    int atlas_top = -1;
    int width = -1;
    int height = -1;
    float x_offset = 0.0f;
    float y_offset = 0.0f;
    float x_advance = 0.0f;
  };

  struct FontAtlasQuad {
    const PackedGlyph* packed_glyph;
    float x;
    float y;
    float width;
    float height;
  };

  class PackedFont;

  class Font {
  public:
    static constexpr PackedGlyph kNullPackedGlyph = { 0, 0, 0, 0, 0.0f, 0.0f, 0.0f };
    
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

    static bool isVariationSelector(char32_t character) {
      return (character & 0xfffffff0) == 0xfe00;
    }
    static bool isPrintable(char32_t character) {
      return character != ' ' && character != '\t' && character != '\n';
    }
    static bool isNewLine(char32_t character) { return character == '\n'; }
    static bool isIgnored(char32_t character) {
      return character == '\r' || isVariationSelector(character);
    }
    static bool hasNewLine(const char32_t* string, int length);

    Font() = default;
    Font(int size, const char* font_data, int data_size);
    Font(int size, const EmbeddedFile& file);
    Font(const Font& other);
    Font& operator=(const Font& other);
    ~Font();

    int widthOverflowIndex(const char32_t* string, int string_length, float width,
                           bool round = false, int character_override = 0) const;
    float stringWidth(const char32_t* string, int length, int character_override = 0) const;
    float stringWidth(const std::u32string& string, int character_override = 0) const {
      return stringWidth(string.c_str(), string.size(), character_override);
    }
    int lineHeight() const;
    float capitalHeight() const;
    float lowerDipHeight() const;

    int atlasWidth() const;
    int size() const { return size_; }
    const char* fontData() const { return font_data_; }
    int dataSize() const { return data_size_; }
    const bgfx::TextureHandle& textureHandle() const;

    void setVertexPositions(FontAtlasQuad* quads, const char32_t* string, int length, float x, float y,
                            float width, float height, Justification justification = Justification::kCenter,
                            int character_override = 0) const;

    std::vector<int> lineBreaks(const char32_t* string, int length, float width) const;

    void setMultiLineVertexPositions(FontAtlasQuad* quads, const char32_t* string, int length,
                                     float x, float y, float width, float height,
                                     Justification justification = Justification::kCenter) const;

    const PackedFont* packedFont() const { return packed_font_; }

  private:
    int size_ = 0;
    const char* font_data_ = nullptr;
    int data_size_ = 0;
    PackedFont* packed_font_ = nullptr;
  };

  class FontCache {
  public:
    friend class Font;

    ~FontCache();

    static void clearStaleFonts() {
      if (instance()->has_stale_fonts_)
        instance()->removeStaleFonts();
    }

  private:
    static FontCache* instance() {
      static FontCache cache;
      return &cache;
    }

    static PackedFont* loadPackedFont(int size, const EmbeddedFile& font) {
      return instance()->createOrLoadPackedFont(size, font.data, font.size);
    }

    static PackedFont* loadPackedFont(int size, const char* font_data, int data_size) {
      return instance()->createOrLoadPackedFont(size, font_data, data_size);
    }

    static void returnPackedFont(PackedFont* packed_font) {
      instance()->decrementPackedFont(packed_font);
    }

    FontCache();

    PackedFont* createOrLoadPackedFont(int size, const char* font_data, int data_size);
    void decrementPackedFont(PackedFont* packed_font);
    void removeStaleFonts();

    std::map<std::pair<int, unsigned const char*>, std::unique_ptr<PackedFont>> cache_;
    std::map<PackedFont*, int> ref_count_;
    bool has_stale_fonts_ = false;
  };
}