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

#include "visage_utils/string_utils.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace visage;

TEST_CASE("String conversion", "[utils]") {
  std::u32string original = U"Hello, \U0001F602 \u00E0\u00C0\u00E8!";
  String test = original;
  std::string std = test.toUtf8();
  std::wstring wide = test.toWide();
  REQUIRE(String(std).toUtf32() == original);
  REQUIRE(String(wide).toUtf32() == original);
}

TEST_CASE("Base 64 conversion", "[utils]") {
  static constexpr int kMaxSize = 10000;
  int size = 1 + (rand() % (kMaxSize - 1));

  std::unique_ptr<unsigned char[]> random_data = std::make_unique<unsigned char[]>(size);
  for (int i = 0; i < size; ++i)
    random_data[i] = static_cast<unsigned char>(rand() % 256);

  std::string encoded = visage::encodeDataBase64(random_data.get(), size);
  int decoded_size = 0;
  std::unique_ptr<unsigned char[]> decoded = visage::decodeBase64Data(encoded, decoded_size);
  REQUIRE(decoded_size == size);
  bool equal = true;
  for (int i = 0; i < size; ++i)
    equal = equal && (random_data[i] == decoded[i]);

  REQUIRE(equal);
}

TEST_CASE("String trim", "[utils]") {
  String test = "\n \t \r \nHello \n World \r Again\n \t \r \n";
  REQUIRE(test.trim().toUtf8() == "Hello \n World \r Again");

  String all_space = "\n \t \r \n\n\r\n \t \r \n";
  REQUIRE(all_space.trim().toUtf8() == "");
}

TEST_CASE("String remove characters", "[utils]") {
  String test = "\n \t \r \nHello \n World \r Again\n \t \r \n";
  REQUIRE(test.removeCharacters("\n ").toUtf8() == "\t\rHelloWorld\rAgain\t\r");
  REQUIRE(test.removeCharacters("\n HeloAgain").toUtf8() == "\t\rWrd\r\t\r");
}

TEST_CASE("String upper/lower case conversion", "[utils]") {
  String test = "Hello World 123!";
  REQUIRE(test.toUpper().toUtf8() == "HELLO WORLD 123!");
  REQUIRE(test.toLower().toUtf8() == "hello world 123!");
}

TEST_CASE("String comparison operators", "[utils]") {
  String a = "abc";
  String b = "abc";
  String c = "def";

  REQUIRE(bool(a == b));
  REQUIRE(bool(a != c));
  REQUIRE(bool(a < c));
  REQUIRE(bool(a <= b));
  REQUIRE(bool(a <= c));
  REQUIRE(bool(c > a));
  REQUIRE(bool(c >= a));
  REQUIRE(bool(b >= a));
}

TEST_CASE("String contains and endsWith", "[utils]") {
  String test = "Hello World";

  REQUIRE(test.contains("Hello"));
  REQUIRE(test.contains("World"));
  REQUIRE(test.contains(" "));
  REQUIRE_FALSE(test.contains("goodbye"));

  REQUIRE(test.endsWith("World"));
  REQUIRE(test.endsWith('d'));
  REQUIRE_FALSE(test.endsWith("Hello"));
}

TEST_CASE("String numerical precision", "[utils]") {
  String test1 = "0.123456";
  REQUIRE(test1.withPrecision(0).toUtf8() == "0");
  REQUIRE(test1.withPrecision(1).toUtf8() == "0.1");
  REQUIRE(test1.withPrecision(2).toUtf8() == "0.12");
  REQUIRE(test1.withPrecision(3).toUtf8() == "0.123");
  REQUIRE(test1.withPrecision(4).toUtf8() == "0.1235");
  REQUIRE(test1.withPrecision(5).toUtf8() == "0.12346");
  REQUIRE(test1.withPrecision(6).toUtf8() == "0.123456");
  REQUIRE(test1.withPrecision(7).toUtf8() == "0.1234560");
  REQUIRE(test1.withPrecision(8).toUtf8() == "0.12345600");

  String test2 = "9.9995493";
  REQUIRE(test2.withPrecision(0).toUtf8() == "10");
  REQUIRE(test2.withPrecision(1).toUtf8() == "10.0");
  REQUIRE(test2.withPrecision(2).toUtf8() == "10.00");
  REQUIRE(test2.withPrecision(3).toUtf8() == "10.000");
  REQUIRE(test2.withPrecision(4).toUtf8() == "9.9995");
  REQUIRE(test2.withPrecision(5).toUtf8() == "9.99955");
  REQUIRE(test2.withPrecision(6).toUtf8() == "9.999549");
  REQUIRE(test2.withPrecision(7).toUtf8() == "9.9995493");
  REQUIRE(test2.withPrecision(8).toUtf8() == "9.99954930");
}
