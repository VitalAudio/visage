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

#include "shader_editor.h"

#include "embedded/fonts.h"
#include "embedded/icons.h"
#include "embedded/shaders.h"
#include "visage_utils/child_process.h"
#include "visage_utils/file_system.h"

#include <sstream>

namespace visage {
  namespace {
    std::string getShaderExecutable() {
#if VISAGE_WINDOWS
      return "shaderc.exe";
#else
      return "shaderc";
#endif
    }
  }

  ShaderCompiler::ShaderCompiler() : Thread("Shader Compiler") {
    compiler_path_ = (std::filesystem::current_path() / getShaderExecutable()).string();
#if VISAGE_MAC
    if (!std::filesystem::exists(File(compiler_path_))) {
      std::filesystem::path app_path = std::filesystem::current_path().parent_path().parent_path().parent_path() /
                                       getShaderExecutable();
      if (std::filesystem::exists(app_path))
        compiler_path_ = app_path;
    }
#endif
  }

  void ShaderCompiler::run() {
    while (new_code_.load())
      compileShader(true);
  }

  void ShaderCompiler::setCompilerPath(const std::string& string_path) {
    File path = File(string_path) / getShaderExecutable();
    compiler_path_ = path.string();
  }

  bool ShaderCompiler::compileShader(bool hot_swap) {
    new_code_ = false;

    File compile_path = std::filesystem::temp_directory_path() / "shader_compiler";
    File include_path = compile_path / "includes";
    std::filesystem::create_directories(include_path);

    File output_file = compile_path / "output.bin";
    replaceFileWithData(compile_path / "varying.def.sc", shaders::varying_def_sc.data,
                        shaders::varying_def_sc.size);
    replaceFileWithData(include_path / "bgfx_shader.sh", shaders::bgfx_shader_sh.data,
                        shaders::bgfx_shader_sh.size);
    replaceFileWithData(include_path / "shader_utils.sh", shaders::shader_utils_sh.data,
                        shaders::shader_utils_sh.size);

    EmbeddedFile vertex, fragment, original;
    std::string code;
    loadCode(vertex, fragment, original, code);

    std::string type = getTypeArgument(kFragment);
#if VISAGE_WINDOWS
    std::string platform = getPlatformArgument(kWindows);
    std::string profile = getProfileArgument(kDx11, kFragment);
#elif VISAGE_LINUX
    std::string platform = getPlatformArgument(kLinux);
    std::string profile = getProfileArgument(kGlsl, kFragment);
#elif VISAGE_MAC
    std::string platform = getPlatformArgument(kMac);
    std::string profile = getProfileArgument(kMetal, kFragment);
#elif VISAGE_EMSCRIPTEN
    std::string platform = getPlatformArgument(kLinux);
    std::string profile = getProfileArgument(kGlsl, kFragment);
#endif

    if (!std::filesystem::exists(File(compiler_path_))) {
      setError("Shader compiler not found");
      return false;
    }

    File temporary_shader = compile_path / original.name;
    replaceFileWithText(temporary_shader, code);

    std::string arguments = "-f " + temporary_shader.string() + " -i " + include_path.string() +
                            " -o " + output_file.string() + " --type " + type + " --platform " +
                            platform + " -p " + profile;

    std::string output;
    if (!spawnChildProcess(compiler_path_, arguments, output)) {
      if (output.empty())
        output = "Failed to compile shader";
      runOnEventThread([this, output]() { setError(output); });
      return false;
    }

    if (hot_swap) {
      int size = 0;
      shader_memory_ = loadFileData(output_file, size);
      ShaderCache::swapShader(fragment, shader_memory_.get(), size);
      ProgramCache::refreshProgram(vertex, fragment);
    }

    runOnEventThread([this, output]() { setError(output); });
    std::filesystem::remove_all(compile_path);
    return true;
  }

  ShaderEditor::ShaderEditor() {
    addDrawableComponent(&editor_);
    addDrawableComponent(&error_);
    addDrawableComponent(&status_);
    editor_.addListener(this);
    editor_.setMultiLine(true);
    editor_.setMargin(15, 10);
    editor_.setFont(Font(10, fonts::DroidSansMono_ttf));
    editor_.setJustification(Font::kTopLeft);
    editor_.setDefaultText("No shader set");

    error_.addListener(this);
    error_.setMultiLine(true);
    error_.setMargin(15, 10);
    error_.setFont(Font(10, fonts::DroidSansMono_ttf));
    error_.setJustification(Font::kTopLeft);
    error_.setActive(false);

    compiler_.addCompileCallback([this](const std::string& error) {
      error_.setText(error);
      status_.redraw();
      redraw();
    });

    status_.setDrawFunction([this](Canvas& canvas) {
      if (error_.getText().isEmpty()) {
        canvas.setColor(0xff66ff66);
        canvas.icon(icons::check_circle_svg, 0, 0, status_.getWidth(), status_.getHeight());
      }
      else {
        canvas.setColor(0xffff6666);
        canvas.icon(icons::x_circle_svg, 0, 0, status_.getWidth(), status_.getHeight());
      }
    });
  }

  void ShaderEditor::textEditorChanged(TextEditor* editor) {
    if (editor != &editor_)
      return;

    if (fragment_.data) {
      std::string text = editor->getText().toUtf8();
      compiler_.setCodeAndCompile(vertex_, fragment_, original_fragment_, text);
    }
  }

  void ShaderEditor::draw(Canvas& canvas) {
    canvas.setColor(0xff222222);
    canvas.fill(0, 0, getWidth(), getHeight());
  }

  void ShaderEditor::resized() {
    int info_height = getHeight() * kInfoHeightRatio;
    int padding = getHeight() * kPaddingHeightRatio;

    int editor_width = getWidth() - 2 * padding;
    error_.setBounds(padding, getHeight() - info_height - padding, editor_width, info_height);
    editor_.setBounds(padding, padding, editor_width, error_.getY() - 2 * padding);

    int font_size = getHeightScale() * 14.0f;
    status_.setBounds(error_.getRight() - font_size - padding, error_.getY() + padding, font_size, font_size);
    editor_.setFont(Font(font_size, fonts::DroidSansMono_ttf));
    error_.setFont(editor_.font());
  }
}