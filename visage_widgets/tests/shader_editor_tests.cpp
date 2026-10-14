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

#include "visage_utils/file_system.h"
#include "visage_widgets/shader_editor.h"

#include <catch2/catch_test_macros.hpp>
#include <filesystem>

using namespace visage;

TEST_CASE("Shader compiler watches folders added while it runs", "[widgets]") {
  static constexpr int kFolders = 20;
  static constexpr int kShadersPerFolder = 20;

  std::filesystem::path root = std::filesystem::temp_directory_path() / "visage_shader_watch_test";
  for (int folder = 0; folder < kFolders; ++folder) {
    std::filesystem::create_directories(root / std::to_string(folder));
    for (int i = 0; i < kShadersPerFolder; ++i)
      (void)replaceFileWithText(root / std::to_string(folder) / ("fs_" + std::to_string(i) + ".sc"), "");
  }

  ShaderCompiler compiler;
  for (int folder = 0; folder < kFolders; ++folder) {
    compiler.watchShaderFolder((root / std::to_string(folder)).string());
    Thread::sleep(20);
  }

  REQUIRE(compiler.running());
  compiler.stop();
  std::filesystem::remove_all(root);
}
