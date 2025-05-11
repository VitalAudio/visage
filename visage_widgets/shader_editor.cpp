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

#include "shader_editor.h"

#include "embedded/fonts.h"
#include "embedded/icons.h"
#include "embedded/shaders.h"
#include "visage_graphics/graphics_utils.h"
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
    static constexpr int kMaxParentDirectories = 4;
    std::string executable = shaderExecutable();
    std::filesystem::path path = std::filesystem::current_path();
    for (int i = 0; i < kMaxParentDirectories && std::filesystem::exists(path); ++i) {
      std::filesystem::path compiler_path = path / executable;
      if (std::filesystem::exists(compiler_path)) {
        compiler_path_ = compiler_path.string();
        break;
      }
      path = path.parent_path();
    }
  }

  void ShaderCompiler::compileWebGlShader(const std::string& shader_name, const std::string& code,
                                          const std::function<void(std::string)>& callback) {
    auto varying_data = reinterpret_cast<const char*>(shaders::varying_def_sc.data);
    auto utils_data = reinterpret_cast<const char*>(shaders::shader_utils_sh.data);
    std::string varying(varying_data, shaders::varying_def_sc.size);
    std::string utils(utils_data, shaders::shader_utils_sh.size);
    std::string result;
    if (visage::preprocessWebGlShader(result, code, utils, varying)) {
      if (ShaderCache::swapShader(shader_name, result.c_str(), result.length()))
        ProgramCache::refreshAllProgramsWithShader(shader_name);
      callback("");
    }
    else
      callback(result);
  }

  void ShaderCompiler::run() {
    compileWaitingShader();

    while (!watched_edit_times_.empty() && shouldRun()) {
      sleep(100);
      compileWaitingShader();

      for (auto& shader : watched_edit_times_) {
        if (!shouldRun())
          break;
        checkShaderForEdits(shader.first);
      }
    }
  }

  void ShaderCompiler::watchShaderFolder(const std::string& folder_path) {
    std::vector<File> files = searchForFiles(folder_path, ".sc$");
    for (const auto& file : files)
      watched_edit_times_[file.string()] = shaderEditTime(file.string());

#ifndef VISAGE_EMSCRIPTEN
    if (!running())
      start();
#endif
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
      std::string code = loadFileAsString(file_path);
      std::string file_stem = fileStem(file_path);
      setCode(file_stem, code, [](const std::string& error) {
        if (!error.empty())
          VISAGE_LOG(error);
      });
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
    replaceFileWithData(include_path / "shader_include.sh", shaders::shader_include_sh.data,
                        shaders::shader_include_sh.size);
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
    std::string profile = profileArgument(Backend::Vulkan, shader_type);
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

    std::string compile_shader = loadFileAsString(output_file);
    runOnEventThread([compile_shader, shader_name]() {
      if (ShaderCache::swapShader(shader_name, compile_shader.c_str(), compile_shader.size()))
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
    editor_.setFont(Font(16, fonts::DroidSansMono_ttf));
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
    error_.setFont(Font(16, fonts::DroidSansMono_ttf));
    error_.setJustification(Font::kTopLeft);
    error_.setActive(false);

    status_.onDraw() = [this](Canvas& canvas) {
      if (error_.text().isEmpty()) {
        canvas.setColor(0xff66ff66);
        canvas.svg(icons::check_circle_svg, 0.0f, 0.0f, status_.width(), status_.height());
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

    int editor_width = width() - 2 * kPaddingHeight;
    error_.setBounds(kPaddingHeight, height() - info_height - kPaddingHeight, editor_width, info_height);
    editor_.setBounds(kPaddingHeight, kPaddingHeight, editor_width, error_.y() - 2 * kPaddingHeight);

    int font_size = editor_.font().size();
    status_.setBounds(error_.right() - font_size - kPaddingHeight, error_.y() + kPaddingHeight,
                      font_size, font_size);
  }
}