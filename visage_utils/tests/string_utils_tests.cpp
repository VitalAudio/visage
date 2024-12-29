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

  std::unique_ptr<char[]> random_data = std::make_unique<char[]>(size);
  for (int i = 0; i < size; ++i)
    random_data[i] = static_cast<char>(rand() % 256);

  std::string encoded = visage::encodeDataBase64(random_data.get(), size);
  int decoded_size = 0;
  std::unique_ptr<char[]> decoded = visage::decodeBase64Data(encoded, decoded_size);
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
