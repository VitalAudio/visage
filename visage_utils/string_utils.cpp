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

#include "string_utils.h"

#include <algorithm>

namespace visage {
  std::string encodeDataBase64(const unsigned char* data, size_t size) {
    static const char* kBase64Chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    std::string result;
    result.reserve((size + 2) / 3 * 4);

    for (size_t i = 0; i < size; i += 3) {
      unsigned char c1 = data[i];
      unsigned char c2 = i + 1 < size ? data[i + 1] : 0;
      unsigned char c3 = i + 2 < size ? data[i + 2] : 0;

      result.push_back(kBase64Chars[c1 >> 2]);
      result.push_back(kBase64Chars[((c1 & 0x3) << 4) | (c2 >> 4)]);
      result.push_back(i + 1 < size ? kBase64Chars[((c2 & 0xF) << 2) | (c3 >> 6)] : '=');
      result.push_back(i + 2 < size ? kBase64Chars[c3 & 0x3F] : '=');
    }

    return result;
  }

  std::unique_ptr<char[]> decodeBase64Data(const std::string& string, int& size) {
    int groups = string.size() / 4;
    int write_index = 0;

    std::unique_ptr<char[]> result = std::make_unique<char[]>(groups * 3);
    auto base64_value = [](char character) {
      if (character >= 'A' && character <= 'Z')
        return character - 'A';
      if (character >= 'a' && character <= 'z')
        return character - 'a' + 26;
      if (character >= '0' && character <= '9')
        return character - '0' + 52;
      if (character == '+')
        return 62;
      if (character == '/')
        return 63;
      if (character == '=')
        return 64;

      return 65;
    };

    for (int i = 0; i < string.length(); i += 4) {
      unsigned char value0 = base64_value(string[i]);
      unsigned char value1 = base64_value(string[i + 1]);
      unsigned char value2 = base64_value(string[i + 2]);
      unsigned char value3 = base64_value(string[i + 3]);

      if (value0 > 64 || value1 > 64 || value2 > 64 || value3 > 64)
        return nullptr;

      result[write_index++] = (value0 << 2) | (value1 >> 4);
      if (value2 != 64) {
        result[write_index++] = (value1 << 4) | (value2 >> 2);

        if (value3 != 64)
          result[write_index++] = (value2 << 6) | value3;
      }
    }

    size = write_index;
    return result;
  }

  String String::toLower() const {
    std::u32string result = string_;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
  }

  String String::toUpper() const {
    std::u32string result = string_;
    std::transform(result.begin(), result.end(), result.begin(), ::toupper);
    return result;
  }

  String String::removeCharacters(const std::string& characters) const {
    std::u32string result = string_;
    auto filter = [&characters](char32_t c) { return characters.find(c) != std::string::npos; };
    result.erase(std::remove_if(result.begin(), result.end(), filter), result.end());
    return result;
  }

  String String::removeEmojiVariations() const {
    std::u32string result = string_;
    auto filter = [](char32_t c) { return (c & 0xfffffff0) == 0xfe00; };
    result.erase(std::remove_if(result.begin(), result.end(), filter), result.end());
    return result;
  }
}