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

#include "visage_utils/child_process.h"
#include "visage_utils/defines.h"
#include "visage_utils/string_utils.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace visage;

TEST_CASE("Echo child process", "[utils]") {
  std::string output;

#if VISAGE_WINDOWS
  REQUIRE(spawnChildProcess("cmd.exe", "/C echo Hello, World!", output, 1000));
  REQUIRE(String(output).trim().toUtf8() == "Hello, World!");
#else
#endif
}