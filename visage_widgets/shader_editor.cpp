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

namespace visage {
  static std::string shaderExecutable() {
#if VISAGE_WINDOWS
    return "shaderc.exe";
#else
    return "shaderc";
#endif
  }

  inline int shaderEditTime(const std::string& file_path) {
    std::error_code error;
    auto edit_time = std::filesystem::last_write_time(file_path, error).time_since_epoch();
    return std::chrono::duration_cast<std::chrono::seconds>(edit_time).count();
  }

  ShaderCompiler::ShaderCompiler() : Thread("Shader Compiler") {
    compiler_path_ = (std::filesystem::current_path() / shaderExecutable()).string();
#if VISAGE_MAC
    if (!std::filesystem::exists(File(compiler_path_))) {
      std::filesystem::path app_path = std::filesystem::current_path().parent_path().parent_path().parent_path() /
                                       shaderExecutable();
      if (std::filesystem::exists(app_path))
        compiler_path_ = app_path;
    }
#endif
  }

  void ShaderCompiler::run() {
    compileWaitingShader();

    while (!watched_edit_times_.empty() && shouldRun()) {
      sleep(100);
      compileWaitingShader();

      for (auto& shader : watched_edit_times_)
        checkShaderForEdits(shader.first);
    }
  }

  void ShaderCompiler::watchShaderFolder(const std::string& folder_path) {
    std::vector<File> files = searchForFiles(folder_path, ".sc$");
    for (const auto& file : files)
      watched_edit_times_[file.string()] = shaderEditTime(file.string());

    if (!running())
      start();
  }

  void ShaderCompiler::compileWaitingShader() {
    while (new_code_.load())
      compileShader();
  }

  void ShaderCompiler::loadShaderEditTimes() {
    for (auto& shader : watched_edit_times_)
      shader.second = shaderEditTime(shader.first);
  }

  void ShaderCompiler::checkShaderForEdits(const std::string& file_path) {
    int seconds = shaderEditTime(file_path);
    if (watched_edit_times_[file_path] < seconds) {
      watched_edit_times_[file_path] = seconds;
      int size = 0;
      std::unique_ptr<char[]> shader_memory = loadFileData(file_path, size);
      std::string file_stem = fileStem(file_path);
      std::string code(shader_memory.get(), size);
      setCode(file_stem, code, [](std::string& error) { VISAGE_LOG(error); });
      compileShader();
    }
  }

  bool ShaderCompiler::compileShader() {
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

    std::string shader_name;
    std::string code;
    std::function<void(std::string)> callback;
    loadCode(shader_name, code, callback);

    ShaderType shader_type = shader_name[0] == 'v' ? ShaderType::Vertex : ShaderType::Fragment;
    std::string type = typeArgument(shader_type);

#if VISAGE_WINDOWS
    std::string platform = platformArgument(Platform::Windows);
    std::string profile = profileArgument(Backend::Dx11, shader_type);
#elif VISAGE_LINUX
    std::string platform = platformArgument(Platform::Linux);
    std::string profile = profileArgument(Backend::Glsl, shader_type);
#elif VISAGE_MAC
    std::string platform = platformArgument(Platform::Mac);
    std::string profile = profileArgument(Backend::Metal, shader_type);
#elif VISAGE_EMSCRIPTEN
    std::string platform = platformArgument(Platform::Linux);
    std::string profile = profileArgument(Backend::Glsl, shader_type);
#endif

    if (!std::filesystem::exists(File(compiler_path_))) {
      runOnEventThread([callback] { callback("Shader compiler not found"); });
      return false;
    }

    File temporary_shader = compile_path / shader_name;
    replaceFileWithText(temporary_shader, code);

    std::string arguments = "-f " + temporary_shader.string() + " -i " + include_path.string() +
                            " -o " + output_file.string() + " --type " + type + " --platform " +
                            platform + " -p " + profile;

    std::string output;
    if (!spawnChildProcess(compiler_path_, arguments, output)) {
      if (output.empty())
        output = "Failed to compile shader";
      runOnEventThread([callback, output]() { callback(output); });
      return false;
    }

    int size = 0;
    std::unique_ptr<char[]> shader_memory = loadFileData(output_file, size);
    std::string compile_shader(shader_memory.get(), size);
    runOnEventThread([compile_shader, shader_name, size]() {
      if (ShaderCache::swapShader(shader_name, compile_shader.c_str(), size))
        ProgramCache::refreshAllProgramsWithShader(shader_name);
    });

    runOnEventThread([callback, output]() { callback(output); });
    std::filesystem::remove_all(compile_path);
    return true;
  }

  ShaderEditor::ShaderEditor() {
    addChild(&editor_);
    addChild(&error_);
    addChild(&status_);
    editor_.setMultiLine(true);
    editor_.setMargin(15, 10);
    editor_.setFont(Font(10, fonts::DroidSansMono_ttf));
    editor_.setJustification(Font::kTopLeft);
    editor_.setDefaultText("No shader set");

    editor_.onTextChange() += [this] {
      if (shader_.data) {
        std::string text = editor_.text().toUtf8();
        auto callback = [this](const std::string& error) {
          error_.setText(error);
          status_.redraw();
          redraw();
        };
        compiler_.compile(shader_, text, callback);
      }
    };

    error_.setMultiLine(true);
    error_.setMargin(15, 10);
    error_.setFont(Font(10, fonts::DroidSansMono_ttf));
    error_.setJustification(Font::kTopLeft);
    error_.setActive(false);

    status_.onDraw() = [this](Canvas& canvas) {
      if (error_.text().isEmpty()) {
        canvas.setColor(0xff66ff66);
        canvas.svg(icons::check_circle_svg, 0, 0, status_.width(), status_.height());
      }
      else {
        canvas.setColor(0xffff6666);
        canvas.svg(icons::x_circle_svg, 0, 0, status_.width(), status_.height());
      }
    };
  }

  void ShaderEditor::draw(Canvas& canvas) {
    canvas.setColor(0xff222222);
    canvas.fill(0, 0, width(), height());
  }

  void ShaderEditor::resized() {
    int info_height = height() * kInfoHeightRatio;
    int padding = dpiScale() * kPaddingHeight;

    int editor_width = width() - 2 * padding;
    error_.setBounds(padding, height() - info_height - padding, editor_width, info_height);
    editor_.setBounds(padding, padding, editor_width, error_.y() - 2 * padding);

    int font_size = dpiScale() * 16.0f;
    status_.setBounds(error_.right() - font_size - padding, error_.y() + padding, font_size, font_size);
    editor_.setFont(Font(font_size, fonts::DroidSansMono_ttf));
    error_.setFont(editor_.font());
  }
}