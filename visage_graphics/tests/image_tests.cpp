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

#include "visage_graphics/image.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <random>

using namespace visage;
using namespace Catch;

TEST_CASE("Image blur", "[graphics]") {
  static constexpr int kWidth = 128;
  static constexpr int kHeight = 64;
  auto image = std::make_unique<unsigned char[]>((kWidth * kHeight + 2) * ImageAtlas::kChannels);
  auto start_image = image.get() + ImageAtlas::kChannels;
  for (int i = 0; i < kWidth * kHeight * ImageAtlas::kChannels; i++) {
    start_image[i] = 255;
  }

  ImageAtlas::blurImage(start_image, kWidth, kHeight, 48);
  for (int i = 0; i < ImageAtlas::kChannels; i++) {
    REQUIRE(image[i] == 0);
    REQUIRE(image[(kWidth * kHeight + 1) * ImageAtlas::kChannels + i] == 0);
  }
}