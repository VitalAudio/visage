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

#include <memory>
#include <string>

namespace visage {
  class String {
  public:
    static constexpr char32_t kReplacementCharacter = 0xFFFD;

    static bool isValidCharacter(char32_t character) {
      return character < 0xD800 || (character > 0xDFFF && character <= 0x10FFFF);
    }

    template<typename Utf32String>
    static Utf32String convertUtf8ToUtf32(const std::string& utf8_str) {
      Utf32String result;
      result.reserve(utf8_str.size());

      size_t i = 0;
      while (i < utf8_str.size()) {
        unsigned char ch = utf8_str[i++];
        if (ch < 0x80) {  // ASCII character
          result.push_back(ch);
          continue;
        }

        int continuation_bytes = 0;
        char32_t character = 0;
        char32_t min_character = 0;
        if (ch >= 0xC2 && ch < 0xE0) {  // 2 byte character
          continuation_bytes = 1;
          character = ch & 0x1F;
          min_character = 0x80;
        }
        else if (ch >= 0xE0 && ch < 0xF0) {  // 3 byte character
          continuation_bytes = 2;
          character = ch & 0x0F;
          min_character = 0x800;
        }
        else if (ch >= 0xF0 && ch < 0xF5) {  // 4 byte character
          continuation_bytes = 3;
          character = ch & 0x07;
          min_character = 0x10000;
        }

        int read = 0;
        while (read < continuation_bytes && i < utf8_str.size() &&
               (static_cast<unsigned char>(utf8_str[i]) & 0xC0) == 0x80) {
          character = (character << 6) | (utf8_str[i++] & 0x3F);
          read++;
        }

        if (continuation_bytes && read == continuation_bytes && character >= min_character &&
            isValidCharacter(character))
          result.push_back(character);
        else
          result.push_back(kReplacementCharacter);
      }

      return result;
    }

    template<typename Utf32String>
    static std::string convertUtf32ToUtf8(const Utf32String& utf32_str) {
      std::string result;
      result.reserve(utf32_str.size() * 4);

      for (auto value : utf32_str) {
        char32_t character = static_cast<char32_t>(value);
        if (!isValidCharacter(character))
          character = kReplacementCharacter;

        if (character < 0x80)  // ASCII character
          result.push_back(static_cast<char>(character));
        else if (character < 0x800) {  // 2 byte character
          result.push_back(static_cast<char>((character >> 6) | 0xC0));
          result.push_back(static_cast<char>((character & 0x3F) | 0x80));
        }
        else if (character < 0x10000) {  // 3 byte character
          result.push_back(static_cast<char>((character >> 12) | 0xE0));
          result.push_back(static_cast<char>(((character >> 6) & 0x3F) | 0x80));
          result.push_back(static_cast<char>((character & 0x3F) | 0x80));
        }
        else {  // 4 byte character
          result.push_back(static_cast<char>((character >> 18) | 0xF0));
          result.push_back(static_cast<char>(((character >> 12) & 0x3F) | 0x80));
          result.push_back(static_cast<char>(((character >> 6) & 0x3F) | 0x80));
          result.push_back(static_cast<char>((character & 0x3F) | 0x80));
        }
      }
      return result;
    }

    template<typename T>
    static T convertUtf32ToUtf16(const std::u32string& utf32_str) {
      T result;
      result.reserve(utf32_str.size());

      for (char32_t character : utf32_str) {
        if (!isValidCharacter(character))
          character = kReplacementCharacter;

        if (character < 0x10000)
          result.push_back(static_cast<char16_t>(character));
        else {
          character -= 0x10000;
          auto high_surrogate = static_cast<char16_t>((character >> 10) + 0xD800);
          auto low_surrogate = static_cast<char16_t>((character & 0x3FF) + 0xDC00);
          result.push_back(high_surrogate);
          result.push_back(low_surrogate);
        }
      }
      return result;
    }

    template<typename T>
    static std::u32string convertUtf16ToUtf32(const T& utf16_str) {
      std::u32string result;
      result.reserve(utf16_str.size());

      for (size_t i = 0; i < utf16_str.size(); ++i) {
        char16_t ch = utf16_str[i];
        bool high_surrogate = ch >= 0xD800 && ch <= 0xDBFF;
        char16_t low_surrogate = i + 1 < utf16_str.size() ? utf16_str[i + 1] : 0;

        if (high_surrogate && low_surrogate >= 0xDC00 && low_surrogate <= 0xDFFF) {  // 4 byte character
          result.push_back(0x10000 + ((ch - 0xD800) << 10) + (low_surrogate - 0xDC00));
          ++i;
        }
        else if (ch >= 0xD800 && ch <= 0xDFFF)
          result.push_back(kReplacementCharacter);
        else  // 2 byte character
          result.push_back(static_cast<char32_t>(ch));
      }

      return result;
    }

    static std::u32string convertToUtf32(const std::string& utf8_str) {
      return convertUtf8ToUtf32<std::u32string>(utf8_str);
    }

    static std::string convertToUtf8(const std::u32string& utf32_str) {
      return convertUtf32ToUtf8<std::u32string>(utf32_str);
    }

    static std::u32string convertToUtf32(const std::wstring& w_str) {
      return convertToUtf32(convertToUtf8(w_str));
    }

    static std::wstring convertToWide(const std::u32string& utf32_str) {
      if constexpr (sizeof(wchar_t) == 4)
        return { utf32_str.begin(), utf32_str.end() };

      return convertUtf32ToUtf16<std::wstring>(utf32_str);
    }

    static std::wstring convertToWide(const std::string& utf8_str) {
      if constexpr (sizeof(wchar_t) == 4)
        return convertUtf8ToUtf32<std::wstring>(utf8_str);

      return convertUtf32ToUtf16<std::wstring>(convertUtf8ToUtf32<std::u32string>(utf8_str));
    }

    static std::string convertToUtf8(const std::wstring& w_str) {
      if constexpr (sizeof(wchar_t) == 4)
        return convertUtf32ToUtf8<std::wstring>(w_str);

      return convertUtf32ToUtf8<std::u32string>(convertUtf16ToUtf32<std::wstring>(w_str));
    }

    static int naturalCompare(const String& a, const String& b);
    static int naturalCompare(const std::u32string& a, const std::u32string& b);
    static int naturalCompare(const std::string& a, const std::string& b);
    static std::string formatDecimal(double value);

    String() = default;

    String(std::u32string string) : string_(std::move(string)) { }
    String(const std::string& string) : string_(convertToUtf32(string)) { }
    String(const std::wstring& string) : string_(convertToUtf32(string)) { }
    String(const char32_t* string) : string_(string) { }
    String(const wchar_t* string) : string_(convertToUtf32(string)) { }
    String(const char* string) : string_(convertToUtf32(string)) { }

    explicit String(bool value) : String(value ? U"true" : U"false") { }
    String(char32_t character) { string_.push_back(character); }
    String(char character) {
      std::string tmp;
      tmp.push_back(character);
      string_ = convertToUtf32(tmp);
    }

    String(int value) : String(std::to_string(value)) { }
    String(unsigned int value) : String(std::to_string(value)) { }
    String(long value) : String(std::to_string(value)) { }
    String(unsigned long value) : String(std::to_string(value)) { }
    String(long long value) : String(std::to_string(value)) { }
    String(unsigned long long value) : String(std::to_string(value)) { }
    String(float value) : String(formatDecimal(value)) { removeTrailingZeros(); }
    String(float value, int precision) : String(formatDecimal(value)) {
      *this = withPrecision(precision);
    }

    String(double value) : String(formatDecimal(value)) { removeTrailingZeros(); }
    String(double value, int precision) : String(formatDecimal(value)) {
      *this = withPrecision(precision);
    }

    String withPrecision(int precision) const {
      size_t pos = find('.');

      if (pos == std::u32string::npos)
        return string_;

      pos += precision + 1;
      if (pos < string_.length()) {
        std::u32string result = string_.substr(0, string_[pos - 1] == '.' ? pos - 1 : pos);
        bool carry = string_[pos] >= '5';
        while (pos > 0 && carry) {
          --pos;
          if (string_[pos] == '9')
            result[pos] = '0';
          else if (string_[pos] != '.' && string_[pos] != '-') {
            ++result[pos];
            carry = false;
          }
        }
        if (carry && result[0] == '-')
          return U"-1" + result.substr(1);
        if (carry)
          return U"1" + result;

        return result;
      }

      return string_ + std::u32string(pos - string_.length(), '0');
    }

    std::wstring toWide() const { return convertToWide(string_); }
    std::string toUtf8() const { return convertToUtf8(string_); }
    const std::u32string& toUtf32() const { return string_; }

    void removeTrailingZeros() {
      size_t pos = find('.');

      if (pos != std::u32string::npos) {
        while (string_.back() == '0')
          string_.pop_back();

        if (string_.back() == '.')
          string_.pop_back();
      }
    }

    int compare(const String& other) const { return string_.compare(other.string_); }
    int naturalCompare(const String& other) const { return naturalCompare(string_, other.string_); }

    String toLower() const;
    String toUpper() const;
    String removeCharacters(const String& characters) const;
    String removeEmojiVariations() const;

    float toFloat() const;

    int toInt() const {
      try {
        return std::stoi(toUtf8());
      }
      catch (...) {
        return 0;
      }
    }

    bool endsWith(const std::u32string& suffix) const {
      return string_.size() >= suffix.size() &&
             0 == string_.compare(string_.size() - suffix.size(), suffix.size(), suffix);
    }

    bool endsWith(const std::string& suffix) const { return endsWith(convertToUtf32(suffix)); }
    bool endsWith(char suffix) const { return !string_.empty() && string_.back() == suffix; }

    bool contains(const std::u32string& substring) const {
      return string_.find(substring) != std::u32string::npos;
    }

    bool contains(const std::string& substring) const {
      return contains(convertToUtf32(substring));
    }

    std::u32string::iterator begin() { return string_.begin(); }
    std::u32string::iterator end() { return string_.end(); }
    std::u32string::const_iterator begin() const { return string_.begin(); }
    std::u32string::const_iterator end() const { return string_.end(); }

    String operator+(const String& other) const { return string_ + other.string_; }
    String operator+(const std::u32string& other) const { return string_ + other; }
    String operator+(const std::string& other) const { return string_ + convertToUtf32(other); }
    String operator+(const char32_t* other) const { return string_ + other; }
    String operator+(const char* other) const { return string_ + convertToUtf32(other); }

    String& operator+=(const String& other) {
      string_ += other.string_;
      return *this;
    }

    String& operator+=(const std::u32string& other) {
      string_ += other;
      return *this;
    }

    String& operator+=(const std::string& other) {
      string_ += convertToUtf32(other);
      return *this;
    }

    String& operator+=(const char32_t* other) {
      string_ += other;
      return *this;
    }

    String& operator+=(const char* other) {
      string_ += convertToUtf32(other);
      return *this;
    }

    bool operator==(const String& other) const { return string_ == other.string_; }
    bool operator==(const std::u32string& other) const { return string_ == other; }
    bool operator==(const std::string& other) const { return string_ == convertToUtf32(other); }
    bool operator==(const char32_t* other) const { return string_ == other; }
    bool operator==(const char* other) const { return string_ == convertToUtf32(other); }
    bool operator!=(const String& other) const { return string_ != other.string_; }
    bool operator!=(const std::u32string& other) const { return string_ != other; }
    bool operator!=(const std::string& other) const { return string_ != convertToUtf32(other); }
    bool operator!=(const char32_t* other) const { return string_ != other; }
    bool operator!=(const char* other) const { return string_ != convertToUtf32(other); }
    bool operator<(const String& other) const { return string_ < other.string_; }
    bool operator<(const std::u32string& other) const { return string_ < other; }
    bool operator<(const std::string& other) const { return string_ < convertToUtf32(other); }
    bool operator<(const char32_t* other) const { return string_ < other; }
    bool operator<(const char* other) const { return string_ < convertToUtf32(other); }
    bool operator<=(const String& other) const { return string_ <= other.string_; }
    bool operator<=(const std::u32string& other) const { return string_ <= other; }
    bool operator<=(const std::string& other) const { return string_ <= convertToUtf32(other); }
    bool operator<=(const char32_t* other) const { return string_ <= other; }
    bool operator<=(const char* other) const { return string_ <= convertToUtf32(other); }
    bool operator>(const String& other) const { return string_ > other.string_; }
    bool operator>(const std::u32string& other) const { return string_ > other; }
    bool operator>(const std::string& other) const { return string_ > convertToUtf32(other); }
    bool operator>(const char32_t* other) const { return string_ > other; }
    bool operator>(const char* other) const { return string_ > convertToUtf32(other); }
    bool operator>=(const String& other) const { return string_ >= other.string_; }
    bool operator>=(const std::u32string& other) const { return string_ >= other; }
    bool operator>=(const std::string& other) const { return string_ >= convertToUtf32(other); }
    bool operator>=(const char32_t* other) const { return string_ >= other; }
    bool operator>=(const char* other) const { return string_ >= convertToUtf32(other); }

    size_t find(char32_t character) const { return string_.find(character); }
    char32_t operator[](size_t index) const { return string_[index]; }
    const char32_t* c_str() const { return string_.c_str(); }
    size_t length() const { return string_.length(); }
    size_t size() const { return string_.size(); }
    void clear() { string_.clear(); }
    bool isEmpty() const { return string_.empty(); }

    String substring(size_t position = 0, size_t count = std::u32string::npos) const {
      return string_.substr(position, count);
    }

    String trim() const {
      size_t start = string_.find_first_not_of(U" \t\n\r");
      size_t end = string_.find_last_not_of(U" \t\n\r");

      if (start == std::u32string::npos || end == std::u32string::npos)
        return "";

      return string_.substr(start, end - start + 1);
    }

  private:
    std::u32string string_;
  };

  inline std::ostream& operator<<(std::ostream& os, const String& string) {
    return os << string.toUtf8();
  }

  std::string encodeDataBase64(const unsigned char* data, size_t size);
  std::unique_ptr<unsigned char[]> decodeBase64Data(const std::string& string, int& size);
}