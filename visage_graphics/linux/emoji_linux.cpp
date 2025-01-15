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

#include "embedded/fonts.h"
#include "emoji.h"

#include <freetype/freetype.h>

namespace visage {

  class EmojiRasterizerImpl {
  public:
    EmojiRasterizerImpl() {
      FT_Init_FreeType(&library_);
      FT_New_Memory_Face(library_, (const unsigned char*)(fonts::Twemoji_Mozilla_ttf.data),
                         fonts::Twemoji_Mozilla_ttf.size, 0, &face_);
    }

    void drawIntoBuffer(char32_t emoji, int font_size, int write_width, unsigned int* dest,
                        int dest_width, int dest_x, int dest_y) {
      FT_UInt glyph_index = FT_Get_Char_Index(face_, emoji);
      FT_Set_Pixel_Sizes(face_, 0, font_size);
      FT_Int32 flags = FT_LOAD_TARGET_NORMAL;
      if (FT_HAS_COLOR(face_))
        flags |= FT_LOAD_COLOR;
      else
        flags |= FT_LOAD_RENDER;

      if (FT_Load_Glyph(face_, glyph_index, flags))
        return;

      if (FT_Render_Glyph(face_->glyph, FT_RENDER_MODE_NORMAL))
        return;

      int height = face_->glyph->bitmap.rows;
      int width = face_->glyph->bitmap.width;
      unsigned int* source = (unsigned int*)face_->glyph->bitmap.buffer;
      int offset_x = std::max(0, write_width - width) / 2;
      int offset_y = std::max(0, write_width - height) / 2;
      for (int y = 0; y < height && y < write_width; ++y) {
        for (int x = 0; x < width && x < write_width; ++x) {
          int i = (dest_y + y + offset_y) * dest_width + dest_x + x + offset_x;
          dest[i] = source[y * width + x];
        }
      }
    }

    ~EmojiRasterizerImpl() {
      FT_Done_Face(face_);
      FT_Done_FreeType(library_);
    }

  private:
    FT_Library library_ = nullptr;
    FT_Face face_ = nullptr;
  };

  EmojiRasterizer::EmojiRasterizer() {
    impl_ = std::make_unique<EmojiRasterizerImpl>();
  }

  EmojiRasterizer::~EmojiRasterizer() = default;

  void EmojiRasterizer::drawIntoBuffer(char32_t emoji, int font_size, int write_width,
                                       unsigned int* dest, int dest_width, int x, int y) {
    impl_->drawIntoBuffer(emoji, font_size, write_width, dest, dest_width, x, y);
  }
}
