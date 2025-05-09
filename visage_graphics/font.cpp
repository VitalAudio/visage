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

#include "font.h"

#include "emoji.h"
#include "image.h"
#include "visage_utils/thread_utils.h"

#include <bgfx/bgfx.h>
#include <freetype/freetype.h>
#include <set>
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

    static std::string idForFont(const unsigned char* data, int data_size) {
      FT_Face face = newMemoryFace(data, data_size);
      std::string id = std::string(face->family_name) + "-" + std::string(face->style_name);
      doneFace(face);
      return id;
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

  class TypeFace {
  public:
    TypeFace(const TypeFace&) = delete;
    TypeFace& operator=(const TypeFace&) = delete;

    TypeFace(int size, const unsigned char* data, int data_size) {
      face_ = FreeTypeLibrary::newMemoryFace(data, data_size);
      FT_Set_Pixel_Sizes(face_, 0, std::max(0, size));
    }

    ~TypeFace() { FreeTypeLibrary::doneFace(face_); }

    int numGlyphs() const { return face_->num_glyphs; }
    std::string familyName() const { return face_->family_name; }
    std::string styleName() const { return face_->style_name; }

    int glyphIndex(char32_t character) const { return FT_Get_Char_Index(face_, character); }
    bool hasCharacter(char32_t character) const { return glyphIndex(character); }
    int lineHeight() const { return face_->size->metrics.height >> 6; }

    FT_GlyphSlot characterInfo(char32_t character) const {
      FT_Load_Char(face_, character, 0);
      return face_->glyph;
    }

    FT_GlyphSlot characterRasterData(char32_t character) const {
      FT_Load_Char(face_, character, FT_LOAD_RENDER);
      return face_->glyph;
    }

    FT_Face face() const { return face_; }

  private:
    FT_Face face_ = nullptr;
  };

  class PackedFont {
  public:
    PackedFont(const std::string& id, int size, const unsigned char* data, int data_size) :
        id_(id), size_(size), data_size_(data_size) {
      data_ = std::make_unique<unsigned char[]>(data_size);
      std::memcpy(data_.get(), data, data_size);
      type_face_ = std::make_unique<TypeFace>(size, data_.get(), data_size);
      std::unique_ptr<PackedGlyph[]> glyphs = std::make_unique<PackedGlyph[]>(type_face_->numGlyphs());

      packed_glyphs_['\n'] = Font::kNullPackedGlyph;
    }

    ~PackedFont() {
      if (bgfx::isValid(texture_handle_))
        bgfx::destroy(texture_handle_);
      type_face_ = nullptr;
    }

    void resize() {
      if (bgfx::isValid(texture_handle_)) {
        bgfx::destroy(texture_handle_);
        texture_handle_ = BGFX_INVALID_HANDLE;
      }

      atlas_map_.pack();
      for (auto& glyph : packed_glyphs_) {
        if (glyph.second.width == 0)
          continue;

        const PackedRect& rect = atlas_map_.rectForId(glyph.first);
        glyph.second.atlas_left = rect.x;
        glyph.second.atlas_top = rect.y;
      }
    }

    void rasterizeGlyph(char32_t character, const PackedGlyph* packed_glyph) {
      int size = packed_glyph->width * packed_glyph->height;
      if (size == 0)
        return;

      std::unique_ptr<unsigned int[]> texture = std::make_unique<unsigned int[]>(size);
      if (packed_glyph->type_face) {
        FT_GlyphSlot glyph = packed_glyph->type_face->characterRasterData(character);
        for (int y = 0; y < packed_glyph->height; ++y) {
          for (int x = 0; x < packed_glyph->width; ++x) {
            int i = y * packed_glyph->width + x;
            texture[i] = ((glyph->bitmap.buffer[y * packed_glyph->width + x]) << 24) + 0xffffff;
          }
        }
      }
      else {
        EmojiRasterizer::instance().drawIntoBuffer(character, size_, packed_glyph->width,
                                                   texture.get(), packed_glyph->width, 0, 0);
      }

      bgfx::updateTexture2D(texture_handle_, 0, 0, packed_glyph->atlas_left,
                            packed_glyph->atlas_top, packed_glyph->width, packed_glyph->height,
                            bgfx::copy(texture.get(), size * ImageAtlas::kChannels));
    }

    PackedGlyph* packCharacterGlyph(PackedGlyph* packed_glyph, const TypeFace* type_face, char32_t character) {
      static constexpr float kAdvanceMult = 1.0f / (1 << 6);

      FT_GlyphSlot glyph = type_face->characterInfo(character);
      packed_glyph->width = glyph->bitmap.width;
      packed_glyph->height = glyph->bitmap.rows;
      packed_glyph->x_offset = glyph->bitmap_left;
      packed_glyph->y_offset = glyph->bitmap_top;
      packed_glyph->x_advance = glyph->advance.x * kAdvanceMult;
      packed_glyph->type_face = type_face;

      packGlyph(packed_glyph, character);
      return packed_glyph;
    }

    PackedGlyph* packEmojiGlyph(PackedGlyph* packed_glyph, char32_t emoji) {
      int raster_width = lineHeight();
      packed_glyph->width = raster_width;
      packed_glyph->height = raster_width;
      packed_glyph->x_offset = 0;
      packed_glyph->y_offset = size_;
      packed_glyph->x_advance = raster_width;

      packGlyph(packed_glyph, emoji);
      return packed_glyph;
    }

    const PackedGlyph* packedGlyph(char32_t character) {
      PackedGlyph* packed_glyph = &packed_glyphs_[character];
      if (packed_glyph->atlas_left >= 0)
        return packed_glyph;

      if (type_face_->hasCharacter(character))
        return packCharacterGlyph(packed_glyph, type_face_.get(), character);

      return packEmojiGlyph(packed_glyph, character);
    }

    void checkInit() {
      if (!bgfx::isValid(texture_handle_)) {
        texture_handle_ = bgfx::createTexture2D(atlas_map_.width(), atlas_map_.height(), false, 1,
                                                bgfx::TextureFormat::BGRA8);
        int width = atlas_map_.width();
        int height = atlas_map_.height();
        std::unique_ptr<unsigned int[]> clear = std::make_unique<unsigned int[]>(width * height);
        bgfx::updateTexture2D(texture_handle_, 0, 0, 0, 0, width, height,
                              bgfx::copy(clear.get(), width * height * ImageAtlas::kChannels));

        for (auto& glyph : packed_glyphs_)
          rasterizeGlyph(glyph.first, &glyph.second);
      }
    }

    int atlasWidth() const { return atlas_map_.width(); }
    int atlasHeight() const { return atlas_map_.width(); }
    bgfx::TextureHandle& textureHandle() { return texture_handle_; }
    int lineHeight() const { return type_face_->lineHeight(); }
    int size() const { return size_; }
    const unsigned char* data() const { return data_.get(); }
    int dataSize() const { return data_size_; }
    const std::string& id() const { return id_; }

  private:
    void packGlyph(PackedGlyph* packed_glyph, char32_t character) {
      if (!atlas_map_.addRect(character, packed_glyph->width, packed_glyph->height))
        resize();

      const PackedRect& rect = atlas_map_.rectForId(character);
      packed_glyph->atlas_left = rect.x;
      packed_glyph->atlas_top = rect.y;

      if (bgfx::isValid(texture_handle_))
        rasterizeGlyph(character, packed_glyph);
    }

    PackedAtlasMap<char32_t> atlas_map_;
    std::unique_ptr<TypeFace> type_face_;
    std::string id_;
    int size_ = 0;
    std::unique_ptr<unsigned char[]> data_;
    int data_size_ = 0;

    std::map<char32_t, PackedGlyph> packed_glyphs_;
    bgfx::TextureHandle texture_handle_ = { bgfx::kInvalidHandle };
  };

  bool Font::hasNewLine(const char32_t* string, int length) {
    for (int i = 0; i < length; ++i) {
      if (isNewLine(string[i]))
        return true;
    }
    return false;
  }

  Font::Font(float size, const char* data, int data_size, float dpi_scale) :
      size_(size), dpi_scale_(dpi_scale) {
    native_size_ = std::round(size * (dpi_scale ? dpi_scale : 1.0f));
    packed_font_ = FontCache::loadPackedFont(native_size_, data, data_size);
  }

  Font::Font(float size, const EmbeddedFile& file, float dpi_scale) :
      size_(size), dpi_scale_(dpi_scale) {
    native_size_ = std::round(size * (dpi_scale ? dpi_scale : 1.0f));
    packed_font_ = FontCache::loadPackedFont(native_size_, file);
  }

  Font::Font(float size, const std::string& file_path, float dpi_scale) :
      size_(size), dpi_scale_(dpi_scale) {
    native_size_ = std::round(size * (dpi_scale ? dpi_scale : 1.0f));
    packed_font_ = FontCache::loadPackedFont(native_size_, file_path);
  }

  Font::Font(const Font& other) {
    size_ = other.size_;
    native_size_ = other.native_size_;
    dpi_scale_ = other.dpi_scale_;
    packed_font_ = FontCache::loadPackedFont(other.packed_font_);
  }

  Font& Font::operator=(const Font& other) {
    if (this == &other)
      return *this;

    if (packed_font_)
      FontCache::returnPackedFont(packed_font_);

    size_ = other.size_;
    native_size_ = other.native_size_;
    dpi_scale_ = other.dpi_scale_;
    packed_font_ = FontCache::loadPackedFont(other.packed_font_);
    return *this;
  }

  Font::~Font() {
    if (packed_font_)
      FontCache::returnPackedFont(packed_font_);
  }

  Font Font::withDpiScale(float dpi_scale) const {
    if (packed_font_ == nullptr)
      return { size_, nullptr, 0, dpi_scale };
    return { size_, (const char*)packed_font_->data(), packed_font_->dataSize(), dpi_scale };
  }

  Font Font::withSize(float size) const {
    if (packed_font_ == nullptr)
      return { size, nullptr, 0, dpi_scale_ };
    return { size, (const char*)packed_font_->data(), packed_font_->dataSize(), dpi_scale_ };
  }

  int Font::nativeWidthOverflowIndex(const char32_t* string, int string_length, float width,
                                     bool round, int character_override) const {
    float string_width = 0;
    for (int i = 0; i < string_length; ++i) {
      char32_t character = string[i];
      if (character_override)
        character = character_override;
      const PackedGlyph* packed_char = &kNullPackedGlyph;
      if (!isIgnored(character))
        packed_char = packed_font_->packedGlyph(character);

      float advance = packed_char->x_advance;
      float break_point = advance;
      if (round)
        break_point = advance * 0.5f;

      if (string_width + break_point > width)
        return i;

      string_width += advance;
    }

    return string_length;
  }

  float Font::nativeStringWidth(const char32_t* string, int length, int character_override) const {
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

    float string_width = nativeStringWidth(text, length, character_override);
    float pen_x = x + (width - string_width) * 0.5f;
    float pen_y = y + static_cast<int>((height + nativeCapitalHeight()) * 0.5f);

    if (justification & kLeft)
      pen_x = x;
    else if (justification & kRight)
      pen_x = x + width - string_width;

    if (justification & kTop)
      pen_y = y + static_cast<int>((nativeCapitalHeight() + nativeLineHeight()) * 0.5f);
    else if (justification & kBottom)
      pen_y = y + static_cast<int>(height);

    for (int i = 0; i < length; ++i) {
      char32_t character = character_override ? character_override : text[i];
      const PackedGlyph* packed_glyph = packed_font_->packedGlyph(character);

      quads[i].packed_glyph = packed_glyph;
      quads[i].x = pen_x + packed_glyph->x_offset;
      quads[i].y = pen_y - packed_glyph->y_offset;
      quads[i].width = packed_glyph->width;
      quads[i].height = packed_glyph->height;

      pen_x += packed_glyph->x_advance;
    }
  }

  std::vector<int> Font::nativeLineBreaks(const char32_t* string, int length, float width) const {
    std::vector<int> line_breaks;
    int break_index = 0;
    while (break_index < length) {
      int overflow_index = nativeWidthOverflowIndex(string + break_index, length - break_index, width) +
                           break_index;
      if (overflow_index == length && !hasNewLine(string + break_index, overflow_index - break_index))
        break;

      int next_break_index = overflow_index;
      while (next_break_index < length && next_break_index > break_index &&
             isPrintable(string[next_break_index - 1])) {
        next_break_index--;
      }

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
    int line_height = nativeLineHeight();
    std::vector<int> line_breaks = nativeLineBreaks(text, length, width);
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

  int Font::nativeLineHeight() const {
    return packed_font_->lineHeight();
  }

  float Font::nativeCapitalHeight() const {
    return packed_font_->packedGlyph('T')->y_offset;
  }

  float Font::nativeLowerDipHeight() const {
    const PackedGlyph* glyph = packed_font_->packedGlyph('y');
    return (glyph->y_offset + glyph->height);
  }

  int Font::atlasWidth() const {
    return packed_font_->atlasWidth();
  }

  int Font::atlasHeight() const {
    return packed_font_->atlasHeight();
  }

  const bgfx::TextureHandle& Font::textureHandle() const {
    packed_font_->checkInit();
    return packed_font_->textureHandle();
  }

  FontCache::FontCache() {
    FreeTypeLibrary::instance();
  }

  FontCache::~FontCache() = default;

  PackedFont* FontCache::loadPackedFont(const PackedFont* packed_font) {
    if (packed_font == nullptr)
      return nullptr;
    return instance()->incrementPackedFont(packed_font->id());
  }

  PackedFont* FontCache::loadPackedFont(int size, const char* font_data, int data_size) {
    if (font_data == nullptr)
      return nullptr;
    std::string id = FreeTypeLibrary::idForFont((const unsigned char*)font_data, data_size) +
                     " - " + std::to_string(size);
    return instance()->createOrLoadPackedFont(id, size, font_data, data_size);
  }

  PackedFont* FontCache::incrementPackedFont(const std::string& id) {
    ref_count_[cache_[id].get()]++;
    return cache_[id].get();
  }

  PackedFont* FontCache::createOrLoadPackedFont(const std::string& id, int size,
                                                const char* font_data, int data_size) {
    VISAGE_ASSERT(Thread::isMainThread());

    const unsigned char* data = reinterpret_cast<const unsigned char*>(font_data);
    if (cache_.count(id) == 0) {
      TypeFaceData type_face_data(font_data, data_size);
      if (type_face_data_lookup_.count(type_face_data) == 0) {
        auto saved_data = std::make_unique<char[]>(data_size);
        type_face_data.data = saved_data.get();
        type_face_data_lookup_[type_face_data] = std::move(saved_data);
      }

      type_face_data.data = type_face_data_lookup_[type_face_data].get();
      type_face_data_ref_count_[type_face_data]++;
      cache_[id] = std::make_unique<PackedFont>(id, size, data, data_size);
    }

    return incrementPackedFont(id);
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
        TypeFaceData type_face_data((const char*)it->first->data(), it->first->dataSize());
        type_face_data_ref_count_[type_face_data]--;
        if (type_face_data_ref_count_[type_face_data] == 0) {
          type_face_data_ref_count_.erase(type_face_data);
          type_face_data_lookup_.erase(type_face_data);
        }

        cache_.erase(it->first->id());
        it = ref_count_.erase(it);
      }
    }
    has_stale_fonts_ = false;
  }
}