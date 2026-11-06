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

#include "emoji.h"

#include <emscripten/emscripten.h>

namespace visage {

  class EmojiRasterizerImpl {
  public:
    void drawIntoBuffer(char32_t emoji, int font_size, int write_width, unsigned int* dest,
                        int dest_width, int x, int y) {
      EM_ASM(
          {
            var emoji = String.fromCodePoint($0);
            var fontSize = $1;
            var writeWidth = $2;
            var dest = $3 >> 2;
            var destWidth = $4;
            var destX = $5;
            var destY = $6;

            var canvas = document.createElement('canvas');
            canvas.width = writeWidth;
            canvas.height = writeWidth;
            var context = canvas.getContext('2d', { willReadFrequently: true });
            var fontFamilies = 'px "Apple Color Emoji", "Segoe UI Emoji", "Noto Color Emoji", sans-serif';
            context.font = fontSize + fontFamilies;
            context.fillStyle = 'white';

            var metrics = context.measureText(emoji);
            var inkSize = Math.max(metrics.actualBoundingBoxLeft + metrics.actualBoundingBoxRight,
                                   metrics.actualBoundingBoxAscent + metrics.actualBoundingBoxDescent);
            if (inkSize > writeWidth) {
              context.font = fontSize * writeWidth / inkSize + fontFamilies;
              metrics = context.measureText(emoji);
            }

            var inkWidth = metrics.actualBoundingBoxLeft + metrics.actualBoundingBoxRight;
            var inkHeight = metrics.actualBoundingBoxAscent + metrics.actualBoundingBoxDescent;
            context.fillText(emoji, Math.round((writeWidth - inkWidth) / 2 + metrics.actualBoundingBoxLeft),
                             Math.round((writeWidth - inkHeight) / 2 + metrics.actualBoundingBoxAscent));

            var pixels = context.getImageData(0, 0, writeWidth, writeWidth).data;
            for (var row = 0; row < writeWidth; ++row) {
              var write = dest + (destY + row) * destWidth + destX;
              for (var column = 0; column < writeWidth; ++column) {
                var read = (row * writeWidth + column) * 4;
                var alpha = pixels[read + 3];
                var scale = alpha / 255;
                HEAPU32[write + column] = (alpha << 24) | (Math.round(pixels[read] * scale) << 16) |
                                          (Math.round(pixels[read + 1] * scale) << 8) |
                                          Math.round(pixels[read + 2] * scale);
              }
            }
          },
          static_cast<int>(emoji), font_size, write_width, dest, dest_width, x, y);
    }
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
