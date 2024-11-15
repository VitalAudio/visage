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

#include "emoji.h"
#include "visage_utils/string_utils.h"

#include <CoreFoundation/CoreFoundation.h>
#include <CoreGraphics/CoreGraphics.h>
#include <CoreText/CoreText.h>

namespace visage {

  class EmojiRasterizerImpl {
  public:
    EmojiRasterizerImpl() { color_space_ = CGColorSpaceCreateDeviceRGB(); }

    void drawIntoBuffer(char32_t emoji, int font_size, int write_width, unsigned int* dest,
                        int dest_width, int x, int y) {
      unsigned int* write_location = dest + (x + y * write_width);
      CGContextRef context = CGBitmapContextCreate(write_location, write_width, write_width, 8,
                                                   dest_width * 4, color_space_,
                                                   kCGImageAlphaPremultipliedLast);

      CGContextSetRGBFillColor(context, 1, 1, 1, 0);
      CGContextFillRect(context, CGRectMake(0, 0, write_width, write_width));
      CGContextSetTextMatrix(context, CGAffineTransformIdentity);
      CGContextTranslateCTM(context, 0, write_width);
      CGContextScaleCTM(context, 1.0, 1.0);

      String emoji_string = emoji;
      CFStringRef string = CFStringCreateWithCString(NULL, emoji_string.toUtf8().c_str(),
                                                     kCFStringEncodingUTF8);

      CTFontRef font = CTFontCreateWithName(CFSTR("Apple Color Emoji"), font_size, NULL);

      CFStringRef keys[] = { kCTFontAttributeName };
      CFTypeRef values[] = { font };
      CFDictionaryRef attributes = CFDictionaryCreate(NULL, (const void**)keys, (const void**)values,
                                                      sizeof(keys) / sizeof(keys[0]),
                                                      &kCFTypeDictionaryKeyCallBacks,
                                                      &kCFTypeDictionaryValueCallBacks);

      CFAttributedStringRef attributed_string = CFAttributedStringCreate(NULL, string, attributes);
      CTLineRef line = CTLineCreateWithAttributedString(attributed_string);
      double ascent, descent, leading;
      double text_width = CTLineGetTypographicBounds(line, &ascent, &descent, &leading);
      CGContextSetTextPosition(context, (write_width - text_width) / 2.0, -font_size);
      CTLineDraw(line, context);

      CFRelease(line);
      CFRelease(attributed_string);
      CFRelease(attributes);
      CFRelease(font);
      CFRelease(string);
      CGContextRelease(context);
      for (int i = 0; i < write_width * write_width; ++i) {
        int row = i / write_width;
        int col = i % write_width;
        int index = (y + row) * dest_width + x + col;
        int value = dest[index];
        int blue = value & 0x00ff0000;
        int red = value & 0x000000ff;
        dest[index] = (value & 0xff00ff00) + (red << 16) + (blue >> 16);
      }
    }

    ~EmojiRasterizerImpl() { CGColorSpaceRelease(color_space_); }

  private:
    CGColorSpaceRef color_space_;
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
