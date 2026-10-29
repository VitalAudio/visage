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

#include "visage_ui/events.h"
#include "visage_utils/file_system.h"
#include "visage_widgets/shader_editor.h"

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <thread>

#if VISAGE_WINDOWS
#include <process.h>
#else
#include <unistd.h>
#endif

using namespace visage;

static std::filesystem::path uniqueTempPath(const std::string& name) {
#if VISAGE_WINDOWS
  int process_id = _getpid();
#else
  int process_id = getpid();
#endif
  return std::filesystem::temp_directory_path() / (std::to_string(process_id) + "_" + name);
}

TEST_CASE("Shader compiler watches folders added while it runs", "[widgets]") {
  static constexpr int kFolders = 20;
  static constexpr int kShadersPerFolder = 20;

  std::filesystem::path root = uniqueTempPath("visage_shader_watch_test");
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

TEST_CASE("Shader compiler compiles code sent while its thread is finishing", "[widgets]") {
  class SlowExitCompiler : public ShaderCompiler {
  public:
    void run() override {
      ShaderCompiler::run();
      finishing = true;
      while (!release && shouldRun())
        sleep(1);
    }

    std::atomic<bool> finishing = false;
    std::atomic<bool> release = false;
  };

  SlowExitCompiler compiler;
  int first = 0;
  int second = 0;
  compiler.compile("fs_first", "", [&first](const std::string&) { first++; });
  for (int i = 0; i < 2000 && !compiler.finishing; ++i)
    Thread::sleep(1);
  REQUIRE(compiler.finishing);

  compiler.compile("fs_second", "", [&second](const std::string&) { second++; });
  compiler.release = true;
  for (int i = 0; i < 2000 && second == 0; ++i) {
    EventManager::instance().checkEventTimers();
    Thread::sleep(1);
  }

  REQUIRE(first == 1);
  REQUIRE(second == 1);
  compiler.stop();
}

TEST_CASE("ShaderEditor compile results arriving after it's deleted are ignored", "[widgets]") {
  static constexpr char kShader[] = "void main() { }";
  EmbeddedFile shader = { "test_shader.sc", reinterpret_cast<const unsigned char*>(kShader),
                          static_cast<int>(sizeof(kShader) - 1) };

  auto editor = std::make_unique<ShaderEditor>();
  editor->setShader(shader, shader);
  auto* text_editor = dynamic_cast<TextEditor*>(editor->children()[0]);
  REQUIRE(text_editor);
  text_editor->processTextInput(" ");

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  editor.reset();
  EventManager::instance().checkEventTimers();
}

#if VISAGE_LIVE_SHADERS
TEST_CASE("Live shader compilation runs in process and reports errors", "[widgets]") {
  static constexpr char kValid[] = "$input v_coordinates\n\n#include <shader_include.sh>\n\n"
                                   "uniform vec4 u_color;\n\nvoid main() {\n  gl_FragColor = u_color;\n}\n";
  static constexpr char kInvalid[] = "$input v_coordinates\n\nvoid main() {\n  gl_FragColor = missing;\n}\n";

  ShaderCompiler compiler;
  auto compile = [&compiler](const std::string& code) {
    std::string result = "pending";
    compiler.compile("fs_live_test", code, [&result](const std::string& message) { result = message; });
    for (int i = 0; i < 5000 && result == "pending"; ++i) {
      EventManager::instance().checkEventTimers();
      Thread::sleep(1);
    }
    return result;
  };

  REQUIRE(compile(kValid).empty());
  std::string error = compile(kInvalid);
  REQUIRE(error != "pending");
  REQUIRE(error.find("missing") != std::string::npos);
  compiler.stop();
}
#endif
