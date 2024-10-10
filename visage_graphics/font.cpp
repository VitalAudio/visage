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

#include "font.h"

#include "graphics_libs.h"
#include "visage_utils/thread_utils.h"

#include <vector>

#if EMOJI_FALLBACK
#include "noto_emoji.h"
#endif

namespace visage {

  struct FontRange {
    FontRange(const unsigned char* data, char32_t start, unsigned int num) :
        font_data(data), start_range(start), num_glyphs(num) {
      packed_chars = std::make_unique<stbtt_packedchar[]>(num_glyphs);
    }
    const unsigned char* font_data = nullptr;
    char32_t start_range = 0;
    unsigned int num_glyphs = 0;

    std::unique_ptr<stbtt_packedchar[]> packed_chars;
  };

  class PackedFont {
  public:
    static constexpr int kDefaultRange = 2048;
    static constexpr int kMinWidth = 512;
    static constexpr int kPadding = 1;
    static constexpr int kKerningRange = 128;

    PackedFont(int size, const unsigned char* data) : size_(size), data_(data) {
      stbtt_InitFont(&font_info_, data, 0);
      font_ranges_.emplace_back(data, 0, kDefaultRange);
      int glyphs = font_info_.numGlyphs + addFallbackFonts();
      atlas_width_ = std::max<int>((sqrt(glyphs) + 1) * size + 2 * kPadding, kMinWidth);
      texture_ = std::make_unique<unsigned char[]>(atlas_width_ * atlas_width_);

      stbtt_PackBegin(&pack_context_, texture_.get(), atlas_width_, atlas_width_, atlas_width_, 1, nullptr);

      for (FontRange& range : font_ranges_)
        stbtt_PackFontRange(&pack_context_, range.font_data, 0, size, range.start_range,
                            range.num_glyphs, range.packed_chars.get());

      stbtt_PackEnd(&pack_context_);

      float scale = stbtt_ScaleForMappingEmToPixels(&font_info_, size_);
      for (int f = 0; f < kKerningRange; ++f) {
        int from_index = f * kKerningRange;
        for (int t = 0; t < kKerningRange; ++t)
          kerning_lookup_[from_index + t] = scale * stbtt_GetCodepointKernAdvance(&font_info_, f, t);
      }
    }

    ~PackedFont() {
      if (bgfx::isValid(texture_handle_)) {
        bgfx::destroy(texture_handle_);
        bgfx::frame();
        bgfx::frame();
      }
    }

    int addFallbackFonts() {
#if EMOJI_FALLBACK
      int start_index = font_ranges_.size();
      font_ranges_.emplace_back((const unsigned char*)fallback_fonts::NotoEmoji_Medium_ttf, 0x1f314, 0xf00);

      int num_glyphs = 0;
      for (int i = start_index; i < font_ranges_.size(); ++i) {
        stbtt_fontinfo font_info;
        stbtt_InitFont(&font_info, font_ranges_[i].font_data, 0);
        num_glyphs += font_info.numGlyphs;
      }
      return num_glyphs;
#else
      return 0;
#endif
    }

    const FontRange* fontRange(char32_t character) const {
      const FontRange* result = &font_ranges_[0];
      for (const FontRange& range : font_ranges_) {
        if (range.start_range > character)
          break;
        result = &range;
      }

      if (character - result->start_range >= result->num_glyphs)
        return nullptr;
      return result;
    }

    const stbtt_packedchar* packedChar(char32_t character) const {
      const FontRange* range = fontRange(character);
      if (range == nullptr)
        return &font_ranges_[0].packed_chars['\n'];
      return &range->packed_chars[character - range->start_range];
    }

    int kerningAdvance(char32_t from, char32_t to) const {
      if (from >= kKerningRange || to >= kKerningRange)
        return 0;
      return kerning_lookup_[from * kKerningRange + to];
    }

    void checkInit() {
      if (!bgfx::isValid(texture_handle_)) {
        const bgfx::Memory* texture_ref = bgfx::makeRef(texture_.get(), atlas_width_ * atlas_width_ *
                                                                            sizeof(unsigned char));
        texture_handle_ = bgfx::createTexture2D(atlas_width_, atlas_width_, false, 1,
                                                bgfx::TextureFormat::A8,
                                                BGFX_TEXTURE_NONE | BGFX_SAMPLER_NONE, texture_ref);
      }
    }

    int atlasWidth() const { return atlas_width_; }
    bgfx::TextureHandle& textureHandle() { return texture_handle_; }
    stbtt_fontinfo* info() { return &font_info_; }
    int size() const { return size_; }
    const unsigned char* data() const { return data_; }

  private:
    stbtt_fontinfo font_info_ {};
    stbtt_pack_context pack_context_ {};
    std::vector<FontRange> font_ranges_;
    int atlas_width_ = 0;
    int size_ = 0;
    const unsigned char* data_ = nullptr;

    std::unique_ptr<unsigned char[]> texture_;
    bgfx::TextureHandle texture_handle_ = { bgfx::kInvalidHandle };
    float kerning_lookup_[kKerningRange * kKerningRange] {};
  };

  bool Font::hasNewLine(const char32_t* string, int length) {
    for (int i = 0; i < length; ++i) {
      if (isNewLine(string[i]))
        return true;
    }
    return false;
  }

  Font::Font(int size, const char* font_data) : size_(size), font_data_(font_data) {
    packed_font_ = FontCache::loadPackedFont(size, font_data);
  }

  Font::Font(int size, const EmbeddedFile& file) : size_(size), font_data_(file.data) {
    packed_font_ = FontCache::loadPackedFont(size, file.data);
  }

  Font::Font(const Font& other) {
    size_ = other.size_;
    font_data_ = other.font_data_;
    packed_font_ = FontCache::loadPackedFont(size_, font_data_);
  }

  Font& Font::operator=(const Font& other) {
    if (this == &other)
      return *this;

    if (packed_font_)
      FontCache::returnPackedFont(packed_font_);
    size_ = other.size_;
    font_data_ = other.font_data_;
    packed_font_ = FontCache::loadPackedFont(size_, font_data_);
    return *this;
  }

  Font::~Font() {
    if (packed_font_)
      FontCache::returnPackedFont(packed_font_);
  }

  int Font::widthOverflowIndex(const char32_t* string, int string_length, float width, bool round,
                               int character_override) const {
    static constexpr stbtt_packedchar kNullPackedChar = { 0, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f };

    char32_t last_character = 0;
    float string_width = 0;
    for (int i = 0; i < string_length; ++i) {
      char32_t character = string[i];
      if (character_override)
        character = character_override;
      const stbtt_packedchar* packed_char = &kNullPackedChar;
      if (!isIgnored(character))
        packed_char = packed_font_->packedChar(character);

      float advance = packed_char->xadvance + packed_font_->kerningAdvance(last_character, character);
      if (isNewLine(character))
        advance = 0.0f;

      float break_point = advance;
      if (round)
        break_point = advance * 0.5f;

      if (string_width + break_point > width)
        return i;

      string_width += advance;
      last_character = character;
    }

    return string_length;
  }

  float Font::stringWidth(const char32_t* string, int length, int character_override) const {
    if (length <= 0)
      return 0.0f;

    if (character_override) {
      float advance = packed_font_->packedChar(character_override)->xadvance;
      float kerning = packed_font_->kerningAdvance(character_override, character_override);
      return (advance + kerning) * length - kerning;
    }

    float width = 0.0f;
    char32_t last_character = 0;
    for (int i = 0; i < length; ++i) {
      if (!isNewLine(string[i]) && !isIgnored(string[i]))
        width += packed_font_->packedChar(string[i])->xadvance +
                 packed_font_->kerningAdvance(last_character, string[i]);
      last_character = string[i];
    }

    return width;
  }

  float Font::stringTop(const char32_t* string, int length) const {
    if (length <= 0)
      return 0.0f;

    float top = packed_font_->packedChar(string[0])->yoff;
    for (int i = 0; i < length; ++i)
      top = std::min(top, packed_font_->packedChar(string[i])->yoff);

    return top;
  }

  float Font::stringBottom(const char32_t* string, int length) const {
    if (length <= 0)
      return 0.0f;

    float bottom = packed_font_->packedChar(string[0])->yoff2;
    for (int i = 0; i < length; ++i)
      bottom = std::max(bottom, packed_font_->packedChar(string[i])->yoff2);

    return bottom;
  }

  void Font::setVertexPositions(FontAtlasQuad* quads, const char32_t* text, int length, float x,
                                float y, float width, float height, Justification justification,
                                int character_override) const {
    static constexpr stbtt_packedchar kNullPackedChar = { 0, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f };

    if (length <= 0)
      return;

    float string_width = stringWidth(text, length, character_override);
    float pen_x = x + (width - string_width) * 0.5f;
    float pen_y = y + static_cast<int>((capitalHeight() + height) * 0.5f);

    if (justification & kLeft)
      pen_x = x;
    else if (justification & kRight)
      pen_x = x + width - string_width;

    if (justification & kTop)
      pen_y = y + static_cast<int>((capitalHeight() + lineHeight()) * 0.5f);
    else if (justification & kBottom)
      pen_y = y + static_cast<int>(height - (capitalHeight() + lineHeight()) * 0.5f);

    pen_x = std::round(pen_x);
    pen_y = std::round(pen_y);

    float atlas_scale = 1.0f / atlasWidth();

    char32_t last_character = 0;
    for (int i = 0; i < length; ++i) {
      int character = text[i];
      if (character_override)
        character = character_override;

      pen_x += packed_font_->kerningAdvance(last_character, character);
      last_character = character;

      const stbtt_packedchar* packed_char = &kNullPackedChar;
      if (!isIgnored(character) && !isNewLine(character))
        packed_char = packed_font_->packedChar(character);

      quads[i].left_coordinate = packed_char->x0 * atlas_scale;
      quads[i].right_coordinate = packed_char->x1 * atlas_scale;
      quads[i].top_coordinate = packed_char->y0 * atlas_scale;
      quads[i].bottom_coordinate = packed_char->y1 * atlas_scale;

      quads[i].left_position = pen_x + packed_char->xoff;
      quads[i].right_position = pen_x + packed_char->xoff2;
      quads[i].top_position = pen_y + packed_char->yoff;
      quads[i].bottom_position = pen_y + packed_char->yoff2;

      pen_x += packed_char->xadvance;
    }
  }

  std::vector<int> Font::lineBreaks(const char32_t* string, int length, float width) const {
    std::vector<int> line_breaks;
    int break_index = 0;
    while (break_index < length) {
      int overflow_index = widthOverflowIndex(string + break_index, length - break_index, width) + break_index;
      if (overflow_index == length && !hasNewLine(string + break_index, overflow_index - break_index))
        break;

      int next_break_index = overflow_index;
      while (next_break_index < length && next_break_index > break_index &&
             isPrintable(string[next_break_index - 1]))
        next_break_index--;

      if (next_break_index == break_index)
        next_break_index = overflow_index;

      for (int i = break_index; i < next_break_index; ++i) {
        if (isNewLine(string[i]))
          next_break_index = i + 1;
      }

      next_break_index = std::max(next_break_index, break_index + 1);
      line_breaks.push_back(next_break_index);
      break_index = next_break_index;
    }

    return line_breaks;
  }

  void Font::setMultiLineVertexPositions(FontAtlasQuad* quads, const char32_t* text, int length,
                                         float x, float y, float width, float height,
                                         Justification justification) const {
    int line_height = lineHeight();
    std::vector<int> line_breaks = lineBreaks(text, length, width);
    line_breaks.push_back(length);

    Justification line_justification = kTop;
    if (justification & kLeft)
      line_justification = kTopLeft;
    else if (justification & kRight)
      line_justification = kTopRight;

    int text_height = line_height * line_breaks.size();
    int line_y = y + 0.5 * (height - text_height);
    if (justification & kTop)
      line_y = y;
    else if (justification & kBottom)
      line_y = y + height - text_height;

    int last_break = 0;
    for (int line_break : line_breaks) {
      int line_length = line_break - last_break;
      setVertexPositions(quads + last_break, text + last_break, line_length, x, line_y, width,
                         height, line_justification);
      last_break = line_break;
      line_y += line_height;
    }
  }

  int Font::lineHeight() const {
    int ascent, descent, line_gap;
    stbtt_GetFontVMetrics(packed_font_->info(), &ascent, &descent, &line_gap);
    float scale = stbtt_ScaleForMappingEmToPixels(packed_font_->info(), size_);
    return scale * (ascent - descent + line_gap);
  }

  float Font::capitalHeight() const {
    return -packed_font_->packedChar('T')->yoff;
  }

  float Font::lowerDipHeight() const {
    return packed_font_->packedChar('y')->yoff2;
  }

  int Font::atlasWidth() const {
    return packed_font_->atlasWidth();
  }

  const bgfx::TextureHandle& Font::textureHandle() const {
    if (!bgfx::isValid(packed_font_->textureHandle()))
      packed_font_->checkInit();
    return packed_font_->textureHandle();
  }

  void Font::checkInit() const {
    packed_font_->checkInit();
  }

  FontCache::FontCache() = default;
  FontCache::~FontCache() = default;

  PackedFont* FontCache::createOrLoadPackedFont(int size, const char* font_data) {
    VISAGE_ASSERT(Thread::isMainThread());

    const unsigned char* data = reinterpret_cast<const unsigned char*>(font_data);
    std::pair<int, unsigned const char*> font_info(size, data);
    if (cache_.count(font_info) == 0)
      cache_[font_info] = std::make_unique<PackedFont>(size, data);

    ref_count_[cache_[font_info].get()]++;
    return cache_[font_info].get();
  }

  void FontCache::decrementPackedFont(PackedFont* packed_font) {
    VISAGE_ASSERT(Thread::isMainThread());

    ref_count_[packed_font]--;
    VISAGE_ASSERT(ref_count_[packed_font] >= 0);

    if (ref_count_[packed_font] == 0) {
      ref_count_.erase(packed_font);
      cache_.erase(std::pair<int, unsigned const char*>(packed_font->size(), packed_font->data()));
    }
  }
}