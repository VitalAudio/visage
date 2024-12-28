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