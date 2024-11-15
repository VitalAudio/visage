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

#include "emoji.h"
#include "graphics_libs.h"
#include "visage_utils/thread_utils.h"

#include <freetype/freetype.h>
#include <vector>

namespace visage {

  class FreeTypeLibrary {
  public:
    static FreeTypeLibrary& instance() {
      static FreeTypeLibrary instance;
      return instance;
    }

    static FT_Face newMemoryFace(const unsigned char* data, int data_size) {
      FT_Face face = nullptr;
      FT_New_Memory_Face(instance().library_, data, data_size, 0, &face);
      instance().faces_.insert(face);
      return face;
    }

    static void doneFace(FT_Face face) {
      VISAGE_ASSERT(instance().faces_.count(face));
      if (instance().faces_.count(face) == 0)
        return;

      FT_Done_Face(face);
      instance().faces_.erase(face);
    }

  private:
    FreeTypeLibrary() { FT_Init_FreeType(&library_); }
    ~FreeTypeLibrary() {
      for (FT_Face face : faces_)
        FT_Done_Face(face);
      FT_Done_FreeType(library_);
    }

    std::set<FT_Face> faces_;
    FT_Library library_ = nullptr;
  };

  class FreeTypeFace {
  public:
    FreeTypeFace(int size, const unsigned char* data, int data_size) {
      face_ = FreeTypeLibrary::newMemoryFace(data, data_size);
      FT_Set_Pixel_Sizes(face_, 0, size);
    }

    ~FreeTypeFace() { FreeTypeLibrary::doneFace(face_); }

    int numGlyphs() const { return face_->num_glyphs; }
    std::string familyName() const { return face_->family_name; }
    std::string styleName() const { return face_->style_name; }

    int glyphIndex(char32_t character) const { return FT_Get_Char_Index(face_, character); }
    bool hasGlyph(char32_t character) const { return glyphIndex(character); }
    int lineHeight() { return face_->size->metrics.height >> 6; }

    const FT_GlyphSlot characterGlyph(char32_t character) {
      FT_Load_Char(face_, character, FT_LOAD_RENDER);
      return face_->glyph;
    }

    const FT_GlyphSlot indexGlyph(int index) {
      FT_Load_Glyph(face_, index, FT_LOAD_RENDER);
      return face_->glyph;
    }

    FT_Face face() { return face_; }

  private:
    FreeTypeFace(const FreeTypeFace&) = delete;
    FreeTypeFace& operator=(const FreeTypeFace&) = delete;

    FT_Face face_ = nullptr;
  };

  class PackedFont {
  public:
    static constexpr int kMinWidth = 512;
    static constexpr int kPadding = 2;

    struct PackedFace {
      std::unique_ptr<FreeTypeFace> face;
      std::unique_ptr<PackedGlyph[]> glyphs;

      PackedGlyph* packedGlyph(char32_t character) const {
        return &glyphs[face->glyphIndex(character)];
      }
    };

    PackedFont(int size, const unsigned char* data, int data_size) :
        size_(size), data_(data), atlas_width_(kMinWidth) {
      std::unique_ptr<FreeTypeFace> face = std::make_unique<FreeTypeFace>(size, data, data_size);
      std::unique_ptr<PackedGlyph[]> glyphs = std::make_unique<PackedGlyph[]>(face->numGlyphs());
      texture_ = std::make_unique<unsigned int[]>(atlas_width_ * atlas_width_);
      packed_faces_.push_back({ std::move(face), std::move(glyphs) });
    }

    ~PackedFont() {
      if (bgfx::isValid(texture_handle_))
        bgfx::destroy(texture_handle_);
    }

    void copyGlyph(PackedGlyph* packed_glyph, unsigned int* dest, int dest_width, int dest_x, int dest_y) {
      for (int y = 0; y < packed_glyph->height; ++y) {
        for (int x = 0; x < packed_glyph->width; ++x) {
          int index = (dest_y + y) * dest_width + dest_x + x;
          dest[index] = texture_[(packed_glyph->atlas_top + y) * atlas_width_ + packed_glyph->atlas_left + x];
        }
      }
      packed_glyph->atlas_left = dest_x;
      packed_glyph->atlas_top = dest_y;
    }

    void resize() {
      int new_width = atlas_width_ * 2;
      std::unique_ptr<unsigned int[]> new_texture = std::make_unique<unsigned int[]>(new_width * new_width);

      write_x_ = 0;
      write_y_ = 0;
      write_glyph_height_ = 0;

      for (const auto& packed_face : packed_faces_) {
        int num_glyphs = packed_face.face->numGlyphs();
        for (int i = 0; i < num_glyphs; ++i) {
          PackedGlyph* packed_glyph = &packed_face.glyphs[i];
          if (packed_glyph->atlas_left < 0)
            continue;

          if (write_x_ + packed_glyph->width + kPadding > atlas_width_) {
            write_x_ = 0;
            write_y_ += write_glyph_height_ + kPadding;
            write_glyph_height_ = packed_glyph->height;
          }
          else
            write_glyph_height_ = std::max<int>(write_glyph_height_, packed_glyph->height);

          if (write_y_ + write_glyph_height_ + kPadding > atlas_width_)
            break;

          copyGlyph(packed_glyph, new_texture.get(), new_width, write_x_, write_y_);
          write_x_ += packed_glyph->width + kPadding;
        }
      }

      atlas_width_ *= 2;
      texture_ = std::move(new_texture);
    }

    PackedGlyph* rasterizeGlyph(const PackedFace& packed_face, int index) {
      static constexpr float kAdvanceMult = 1.0f / (1 << 6);

      if (bgfx::isValid(texture_handle_)) {
        bgfx::destroy(texture_handle_);
        texture_handle_ = BGFX_INVALID_HANDLE;
      }

      const FT_GlyphSlot glyph = packed_face.face->indexGlyph(index);
      if (write_x_ + glyph->bitmap.width + kPadding > atlas_width_) {
        write_x_ = 0;
        write_y_ += write_glyph_height_ + kPadding;
        write_glyph_height_ = glyph->bitmap.rows;
      }
      else
        write_glyph_height_ = std::max<int>(write_glyph_height_, glyph->bitmap.rows);

      if (write_y_ + write_glyph_height_ + kPadding > atlas_width_)
        resize();

      PackedGlyph* packed_glyph = &packed_face.glyphs[index];
      packed_glyph->width = glyph->bitmap.width;
      packed_glyph->height = glyph->bitmap.rows;
      packed_glyph->x_offset = glyph->bitmap_left;
      packed_glyph->y_offset = glyph->bitmap_top;
      packed_glyph->x_advance = glyph->advance.x * kAdvanceMult;
      packed_glyph->atlas_left = write_x_;
      packed_glyph->atlas_top = write_y_;

      for (int y = 0; y < packed_glyph->height; ++y) {
        for (int x = 0; x < packed_glyph->width; ++x) {
          int i = (write_y_ + y) * atlas_width_ + write_x_ + x;
          texture_[i] = ((glyph->bitmap.buffer[y * packed_glyph->width + x]) << 24) + 0xffffff;
        }
      }
      write_x_ += packed_glyph->width + kPadding;

      return packed_glyph;
    }

    PackedGlyph* rasterizeEmojiGlyph(char32_t emoji) {
      if (bgfx::isValid(texture_handle_)) {
        bgfx::destroy(texture_handle_);
        texture_handle_ = BGFX_INVALID_HANDLE;
      }

      int raster_width = lineHeight();
      if (write_x_ + raster_width > atlas_width_) {
        write_x_ = 0;
        write_y_ += write_glyph_height_ + kPadding;
        write_glyph_height_ = raster_width;
      }
      else
        write_glyph_height_ = std::max<int>(write_glyph_height_, raster_width);

      if (write_y_ + write_glyph_height_ + kPadding > atlas_width_)
        resize();

      emoji_glyphs_.push_back(std::make_unique<PackedGlyph>());
      PackedGlyph* packed_glyph = emoji_glyphs_.back().get();
      packed_glyph->width = raster_width;
      packed_glyph->height = raster_width;
      packed_glyph->x_offset = 0;
      packed_glyph->y_offset = size_;
      packed_glyph->x_advance = raster_width;
      packed_glyph->atlas_left = write_x_;
      packed_glyph->atlas_top = write_y_;

      EmojiRasterizer::instance().drawIntoBuffer(emoji, size_, raster_width, texture_.get(),
                                                 atlas_width_, write_x_, write_y_);
      write_x_ += packed_glyph->width + kPadding;

      return packed_glyph;
    }

    const PackedGlyph* packedGlyph(char32_t character) {
      for (const auto& packed_face : packed_faces_) {
        if (packed_face.face->hasGlyph(character)) {
          PackedGlyph* glyph = packed_face.packedGlyph(character);
          if (glyph->atlas_left >= 0)
            return glyph;

          return rasterizeGlyph(packed_face, packed_face.face->glyphIndex(character));
        }
      }
      if (emoji_indices_.count(character))
        return emoji_glyphs_[emoji_indices_[character]].get();

      emoji_indices_[character] = emoji_glyphs_.size();
      return rasterizeEmojiGlyph(character);
    }

    void checkInit() {
      if (!bgfx::isValid(texture_handle_)) {
        int size = atlas_width_ * atlas_width_ * sizeof(unsigned int);
        const bgfx::Memory* texture_ref = bgfx::makeRef(texture_.get(), size);
        texture_handle_ = bgfx::createTexture2D(atlas_width_, atlas_width_, false, 1,
                                                bgfx::TextureFormat::BGRA8,
                                                BGFX_TEXTURE_NONE | BGFX_SAMPLER_NONE, texture_ref);
      }
    }

    int atlasWidth() const { return atlas_width_; }
    bgfx::TextureHandle& textureHandle() { return texture_handle_; }
    int size() const { return size_; }
    int lineHeight() { return packed_faces_[0].face->lineHeight(); }
    const unsigned char* data() const { return data_; }

  private:
    std::vector<PackedFace> packed_faces_;
    int atlas_width_ = 0;
    int size_ = 0;
    const unsigned char* data_ = nullptr;

    std::unique_ptr<unsigned int[]> texture_;
    int write_x_ = 0;
    int write_y_ = 0;
    int write_glyph_height_ = 0;

    std::map<char32_t, int> emoji_indices_;
    std::vector<std::unique_ptr<PackedGlyph>> emoji_glyphs_;

    bgfx::TextureHandle texture_handle_ = { bgfx::kInvalidHandle };
  };

  bool Font::hasNewLine(const char32_t* string, int length) {
    for (int i = 0; i < length; ++i) {
      if (isNewLine(string[i]))
        return true;
    }
    return false;
  }

  Font::Font(int size, const char* data, int data_size) :
      size_(size), font_data_(data), data_size_(data_size) {
    packed_font_ = FontCache::loadPackedFont(size, data, data_size);
  }

  Font::Font(int size, const EmbeddedFile& file) :
      size_(size), font_data_(file.data), data_size_(file.size) {
    packed_font_ = FontCache::loadPackedFont(size, file);
  }

  Font::Font(const Font& other) {
    size_ = other.size_;
    font_data_ = other.font_data_;
    data_size_ = other.data_size_;
    packed_font_ = FontCache::loadPackedFont(size_, font_data_, data_size_);
  }

  Font& Font::operator=(const Font& other) {
    if (this == &other)
      return *this;

    if (packed_font_)
      FontCache::returnPackedFont(packed_font_);
    size_ = other.size_;
    font_data_ = other.font_data_;
    data_size_ = other.data_size_;
    packed_font_ = FontCache::loadPackedFont(size_, font_data_, data_size_);
    return *this;
  }

  Font::~Font() {
    if (packed_font_)
      FontCache::returnPackedFont(packed_font_);
  }

  int Font::widthOverflowIndex(const char32_t* string, int string_length, float width, bool round,
                               int character_override) const {
    static constexpr PackedGlyph kNullPackedGlyph = { 0, 0, 0, 0, 0.0f, 0.0f, 0.0f };

    float string_width = 0;
    for (int i = 0; i < string_length; ++i) {
      char32_t character = string[i];
      if (character_override)
        character = character_override;
      const PackedGlyph* packed_char = &kNullPackedGlyph;
      if (!isIgnored(character))
        packed_char = packed_font_->packedGlyph(character);

      float advance = packed_char->x_advance;
      if (isNewLine(character))
        advance = 0.0f;

      float break_point = advance;
      if (round)
        break_point = advance * 0.5f;

      if (string_width + break_point > width)
        return i;

      string_width += advance;
    }

    return string_length;
  }

  float Font::stringWidth(const char32_t* string, int length, int character_override) const {
    if (length <= 0)
      return 0.0f;

    if (character_override) {
      float advance = packed_font_->packedGlyph(character_override)->x_advance;
      return advance * length;
    }

    float width = 0.0f;
    for (int i = 0; i < length; ++i) {
      if (!isNewLine(string[i]) && !isIgnored(string[i]))
        width += packed_font_->packedGlyph(string[i])->x_advance;
    }

    return width;
  }

  void Font::setVertexPositions(FontAtlasQuad* quads, const char32_t* text, int length, float x,
                                float y, float width, float height, Justification justification,
                                int character_override) const {
    if (length <= 0)
      return;

    float string_width = stringWidth(text, length, character_override);
    float pen_x = x + (width - string_width) * 0.5f;
    float pen_y = y + static_cast<int>((height + capitalHeight()) * 0.5f);

    if (justification & kLeft)
      pen_x = x;
    else if (justification & kRight)
      pen_x = x + width - string_width;

    if (justification & kTop)
      pen_y = y + static_cast<int>((capitalHeight() + lineHeight()) * 0.5f);
    else if (justification & kBottom)
      pen_y = y + static_cast<int>(height);

    pen_x = std::round(pen_x);
    pen_y = std::round(pen_y);

    for (int i = 0; i < length; ++i) {
      const PackedGlyph* packed_glyph = packed_font_->packedGlyph(text[i]);
      if (character_override)
        packed_glyph = packed_font_->packedGlyph(character_override);

      quads[i].packed_glyph = packed_glyph;
      quads[i].x = pen_x + packed_glyph->x_offset;
      quads[i].y = pen_y - packed_glyph->y_offset;
      quads[i].width = packed_glyph->width;
      quads[i].height = packed_glyph->height;

      pen_x += packed_glyph->x_advance;
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
    return packed_font_->lineHeight();
  }

  float Font::capitalHeight() const {
    return packed_font_->packedGlyph('T')->y_offset;
  }

  float Font::lowerDipHeight() const {
    const PackedGlyph* glyph = packed_font_->packedGlyph('y');
    return glyph->y_offset + glyph->height;
  }

  int Font::atlasWidth() const {
    return packed_font_->atlasWidth();
  }

  const bgfx::TextureHandle& Font::textureHandle() const {
    packed_font_->checkInit();
    return packed_font_->textureHandle();
  }

  FontCache::FontCache() {
    FreeTypeLibrary::instance();
  }

  FontCache::~FontCache() = default;

  PackedFont* FontCache::createOrLoadPackedFont(int size, const char* font_data, int data_size) {
    VISAGE_ASSERT(Thread::isMainThread());

    const unsigned char* data = reinterpret_cast<const unsigned char*>(font_data);
    std::pair<int, unsigned const char*> font_info(size, data);
    if (cache_.count(font_info) == 0)
      cache_[font_info] = std::make_unique<PackedFont>(size, data, data_size);

    ref_count_[cache_[font_info].get()]++;
    return cache_[font_info].get();
  }

  void FontCache::decrementPackedFont(PackedFont* packed_font) {
    VISAGE_ASSERT(Thread::isMainThread());
    ref_count_[packed_font]--;
    int count = ref_count_[packed_font];
    has_stale_fonts_ = has_stale_fonts_ || count == 0;
    VISAGE_ASSERT(ref_count_[packed_font] >= 0);
  }

  void FontCache::removeStaleFonts() {
    for (auto it = ref_count_.begin(); it != ref_count_.end();) {
      if (it->second)
        ++it;
      else {
        cache_.erase({ it->first->size(), it->first->data() });
        it = ref_count_.erase(it);
      }
    }
    has_stale_fonts_ = false;
  }
}