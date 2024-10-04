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

#include <memory>
#include <string>

namespace visage {
  class String {
  public:
    template<class Utf32String>
    static Utf32String convertUtf8ToUtf32(const std::string& utf8_str) {
      Utf32String result;
      result.reserve(utf8_str.size());

      for (size_t i = 0; i < utf8_str.size(); ++i) {
        unsigned char ch = utf8_str[i];

        if (ch < 0x80)  // ASCII character
          result.push_back(ch);
        else if (ch < 0xC0)  // Error on continuation byte
          result.push_back('*');
        else if (ch < 0xE0) {  // 2 byte character
          if (i + 1 >= utf8_str.size())  // Error - unfinished character.
            return result;

          result.push_back(((ch & 0x1F) << 6) | (utf8_str[i + 1] & 0x3F));
          i += 1;
        }
        else if (ch < 0xF0) {  // 3 byte character
          if (i + 2 >= utf8_str.size())  // Error - unfinished character.
            return result;

          result.push_back(((ch & 0x0F) << 12) | ((utf8_str[i + 1] & 0x3F) << 6) |
                           (utf8_str[i + 2] & 0x3F));
          i += 2;
        }
        else if (ch < 0xF8) {  // 4 byte character
          if (i + 3 >= utf8_str.size())  // Error - unfinished character.
            return result;

          result.push_back(((ch & 0x07) << 18) | ((utf8_str[i + 1] & 0x3F) << 12) |
                           ((utf8_str[i + 2] & 0x3F) << 6) | (utf8_str[i + 3] & 0x3F));
          i += 3;
        }
        else  // Error
          return result;
      }

      return result;
    }

    template<class Utf32String>
    static std::string convertUtf32ToUtf8(const Utf32String& utf32_str) {
      std::string result;
      result.reserve(utf32_str.size() * 4);

      for (auto character : utf32_str) {
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
        else if (character < 0x110000) {  // 4 byte character
          result.push_back(static_cast<char>((character >> 18) | 0xF0));
          result.push_back(static_cast<char>(((character >> 12) & 0x3F) | 0x80));
          result.push_back(static_cast<char>(((character >> 6) & 0x3F) | 0x80));
          result.push_back(static_cast<char>((character & 0x3F) | 0x80));
        }
        else  // Error
          return result;
      }
      return result;
    }

    template<class T>
    static T convertUtf32ToUtf16(const std::u32string& utf32_str) {
      T result;
      result.reserve(utf32_str.size());

      for (char32_t character : utf32_str) {
        if (character <= 0xFFFF) {
          if (character >= 0xD800 && character <= 0xDFFF)  // Error
            return result;
          result.push_back(static_cast<char16_t>(character));
        }
        else if (character <= 0x10FFFF) {
          character -= 0x10000;
          auto high_surrogate = static_cast<char16_t>((character >> 10) + 0xD800);
          auto low_surrogate = static_cast<char16_t>((character & 0x3FF) + 0xDC00);
          result.push_back(high_surrogate);
          result.push_back(low_surrogate);
        }
        else  // Error
          return result;
      }
      return result;
    }

    template<class T>
    static std::u32string convertUtf16ToUtf32(const T& utf16_str) {
      std::u32string result;
      result.reserve(utf16_str.size());

      for (size_t i = 0; i < utf16_str.size(); ++i) {
        char16_t ch = utf16_str[i];

        if (ch >= 0xD800 && ch <= 0xDBFF) {  // 4 byte character
          if (i + 1 >= utf16_str.size())  // Error - unfinished character
            return result;

          char16_t low_surrogate = utf16_str[++i];

          if (low_surrogate < 0xDC00 || low_surrogate > 0xDFFF)  // Error - bad low surrogate
            return result;

          char32_t character = 0x10000 + ((ch - 0xD800) << 10) + (low_surrogate - 0xDC00);
          result.push_back(character);
        }
        else if (ch >= 0xDC00 && ch <= 0xDFFF)  // Error - missing high surrogate
          return result;
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

    String() = default;

    String(std::u32string string) : string_(std::move(string)) { }
    String(std::string string) : string_(convertToUtf32(string)) { }
    String(std::wstring string) : string_(convertToUtf32(string)) { }
    String(const char32_t* string) : string_(string) { }
    String(const wchar_t* string) : string_(convertToUtf32(string)) { }
    String(const char* string) : string_(convertToUtf32(string)) { }

    String(const visage::String& other) = default;

    explicit String(bool value) : String(value ? U"true" : U"false") { }
    String(char32_t character) { string_.push_back(character); }
    String(char character) {
      std::string tmp;
      tmp.push_back(character);
      string_ = convertToUtf32(tmp);
    }

    String(unsigned int value) : String(std::to_string(value)) { }
    String(int value) : String(std::to_string(value)) { }
    String(long value) : String(std::to_string(value)) { }
    String(long long value) : String(std::to_string(value)) { }
    String(unsigned long long value) : String(std::to_string(value)) { }
    String(float value) : String(std::to_string(value)) { removeTrailingZeros(); }
    String(float value, int precision) : String(std::to_string(value)) {
      trimToPrecision(precision);
    }

    String(double value) : String(std::to_string(value)) { removeTrailingZeros(); }
    String(double value, int precision) : String(std::to_string(value)) {
      trimToPrecision(precision);
    }

    void trimToPrecision(int precision) {
      size_t pos = find('.');

      if (pos != std::string::npos) {
        pos += precision + 1;

        if (pos < string_.length())
          string_.erase(pos);
      }
    }

    std::wstring toWide() const { return convertToWide(string_); }
    std::string toUtf8() const { return convertToUtf8(string_); }
    std::u32string toUtf32() const { return string_; }

    void removeTrailingZeros() {
      size_t pos = find('.');

      if (pos != std::string::npos) {
        while (string_.back() == '0')
          string_.pop_back();

        if (string_.back() == '.')
          string_.pop_back();
      }
    }

    String toLower() const;
    String toUpper() const;
    String removeCharacters(const std::string& characters) const;

    float toFloat() const {
      try {
        return std::stof(toUtf8());
      }
      catch (const std::exception&) {
        return 0.0f;
      }
    }

    int toInt() const {
      try {
        return std::stoi(toUtf8());
      }
      catch (const std::exception&) {
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
      return string_.find(substring) != std::string::npos;
    }

    bool contains(const std::string& substring) const {
      return contains(convertToUtf32(substring));
    }

    std::u32string::iterator begin() { return string_.begin(); }

    visage::String operator+(const visage::String& other) const { return string_ + other.string_; }
    visage::String operator+(const std::u32string& other) const { return string_ + other; }
    visage::String operator+(const std::string& other) const {
      return string_ + convertToUtf32(other);
    }
    visage::String operator+(const char32_t* other) const { return string_ + other; }
    visage::String operator+(const char* other) const { return string_ + convertToUtf32(other); }
    visage::String& operator+=(const visage::String& other) {
      string_ += other.string_;
      return *this;
    }

    visage::String& operator+=(const std::u32string& other) {
      string_ += other;
      return *this;
    }

    visage::String& operator+=(const std::string& other) {
      string_ += convertToUtf32(other);
      return *this;
    }

    visage::String& operator+=(const char32_t* other) {
      string_ += other;
      return *this;
    }

    visage::String& operator+=(const char* other) {
      string_ += convertToUtf32(other);
      return *this;
    }

    bool operator==(const visage::String& other) const { return string_ == other.string_; }
    bool operator==(const std::u32string& other) const { return string_ == other; }
    bool operator==(const std::string& other) const { return string_ == convertToUtf32(other); }
    bool operator==(const char32_t* other) const { return string_ == other; }
    bool operator==(const char* other) const { return string_ == convertToUtf32(other); }
    bool operator!=(const visage::String& other) const { return string_ != other.string_; }
    bool operator!=(const std::u32string& other) const { return string_ != other; }
    bool operator!=(const std::string& other) const { return string_ != convertToUtf32(other); }
    bool operator!=(const char32_t* other) const { return string_ != other; }
    bool operator!=(const char* other) const { return string_ != convertToUtf32(other); }
    bool operator<(const visage::String& other) const { return string_ < other.string_; }
    bool operator<(const std::u32string& other) const { return string_ < other; }
    bool operator<(const std::string& other) const { return string_ < convertToUtf32(other); }
    bool operator<(const char32_t* other) const { return string_ < other; }
    bool operator<(const char* other) const { return string_ < convertToUtf32(other); }
    bool operator<=(const visage::String& other) const { return string_ <= other.string_; }
    bool operator<=(const std::u32string& other) const { return string_ <= other; }
    bool operator<=(const std::string& other) const { return string_ <= convertToUtf32(other); }
    bool operator<=(const char32_t* other) const { return string_ <= other; }
    bool operator<=(const char* other) const { return string_ <= convertToUtf32(other); }
    bool operator>(const visage::String& other) const { return string_ > other.string_; }
    bool operator>(const std::u32string& other) const { return string_ > other; }
    bool operator>(const std::string& other) const { return string_ > convertToUtf32(other); }
    bool operator>(const char32_t* other) const { return string_ > other; }
    bool operator>(const char* other) const { return string_ > convertToUtf32(other); }
    bool operator>=(const visage::String& other) const { return string_ >= other.string_; }
    bool operator>=(const std::u32string& other) const { return string_ >= other; }
    bool operator>=(const std::string& other) const { return string_ >= convertToUtf32(other); }
    bool operator>=(const char32_t* other) const { return string_ >= other; }
    bool operator>=(const char* other) const { return string_ >= convertToUtf32(other); }

    int find(char32_t character) const { return string_.find(character); }
    char32_t operator[](size_t index) const { return string_[index]; }
    const char32_t* c_str() const { return string_.c_str(); }
    size_t length() const { return string_.length(); }
    size_t size() const { return string_.size(); }
    void clear() { string_.clear(); }
    bool isEmpty() const { return string_.empty(); }

    visage::String getSubstring(size_t position = 0, size_t count = std::string::npos) const {
      return string_.substr(position, count);
    }

    visage::String trim() const {
      size_t start = string_.find_first_not_of(U" \t\n\r");
      size_t end = string_.find_last_not_of(U" \t\n\r");

      if (start == std::string::npos || end == std::string::npos)
        return "";

      return string_.substr(start, end - start + 1);
    }

  private:
    std::u32string string_;
  };

  std::string encodeDataBase64(const char* data, size_t size);
  std::unique_ptr<char[]> decodeBase64Data(const std::string& string, int& size);
}