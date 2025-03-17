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

#include "color.h"
#include "graphics_utils.h"
#include "visage_file_embed/embedded_file.h"

#include <map>
#include <vector>

namespace visage {
  class TypeFace;
  class PackedFont;

  struct PackedGlyph {
    int atlas_left = -1;
    int atlas_top = -1;
    int width = -1;
    int height = -1;
    float x_offset = 0.0f;
    float y_offset = 0.0f;
    float x_advance = 0.0f;
    const TypeFace* type_face = nullptr;
  };

  struct FontAtlasQuad {
    const PackedGlyph* packed_glyph;
    float x;
    float y;
    float width;
    float height;
  };

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
    Font(float size, const char* font_data, int data_size);
    Font(float size, const EmbeddedFile& file);
    Font(float size, const char* font_data, int data_size, float dpi_scale);
    Font(float size, const EmbeddedFile& file, float dpi_scale);
    Font(const Font& other);
    Font& operator=(const Font& other);
    ~Font();

    float dpiScale() const { return dpi_scale_ ? dpi_scale_ : 1.0f; }
    Font withDpiScale(float dpi_scale) const {
      return Font(size_, fontData(), dataSize(), dpi_scale);
    }

    int widthOverflowIndex(const char32_t* string, int string_length, float width,
                           bool round = false, int character_override = 0) const {
      return nativeWidthOverflowIndex(string, string_length, width * dpiScale(), round, character_override);
    }
    std::vector<int> lineBreaks(const char32_t* string, int length, float width) const {
      return nativeLineBreaks(string, length, width * dpiScale());
    }

    float stringWidth(const char32_t* string, int length, int character_override = 0) const {
      return nativeStringWidth(string, length, character_override) / dpiScale();
    }
    float stringWidth(const std::u32string& string, int character_override = 0) const {
      return stringWidth(string.c_str(), string.size(), character_override);
    }
    float lineHeight() const { return nativeLineHeight() / dpiScale(); }
    float capitalHeight() const { return nativeCapitalHeight() / dpiScale(); }
    float lowerDipHeight() const { return nativeLowerDipHeight() / dpiScale(); }

    int atlasWidth() const;
    int atlasHeight() const;
    int size() const { return size_; }
    const char* fontData() const { return font_data_; }
    int dataSize() const { return data_size_; }
    const bgfx::TextureHandle& textureHandle() const;

    void setVertexPositions(FontAtlasQuad* quads, const char32_t* string, int length, float x, float y,
                            float width, float height, Justification justification = Justification::kCenter,
                            int character_override = 0) const;

    void setMultiLineVertexPositions(FontAtlasQuad* quads, const char32_t* string, int length,
                                     float x, float y, float width, float height,
                                     Justification justification = Justification::kCenter) const;

    const PackedFont* packedFont() const { return packed_font_; }

  private:
    int nativeWidthOverflowIndex(const char32_t* string, int string_length, float width,
                                 bool round = false, int character_override = 0) const;
    float nativeStringWidth(const char32_t* string, int length, int character_override = 0) const;
    int nativeLineHeight() const;
    float nativeCapitalHeight() const;
    float nativeLowerDipHeight() const;
    std::vector<int> nativeLineBreaks(const char32_t* string, int length, float width) const;

    float size_ = 0.0f;
    int native_size_ = 0;
    float dpi_scale_ = 0.0f;
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