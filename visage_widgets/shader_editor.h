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

#pragma once

#include "text_editor.h"
#include "visage_graphics/graphics_caches.h"
#include "visage_ui/frame.h"
#include "visage_utils/thread_utils.h"
#include "visage_widgets/button.h"

#include <mutex>
#include <string>

// For shader development purposes only.
// Not for production use.

namespace visage {
  class ShaderCompiler : public Thread {
  public:
    enum class Platform {
      Linux,
      Mac,
      Windows,
      Emscripten,
    };

    enum class ShaderType {
      Vertex,
      Fragment,
    };

    enum class Backend {
      Glsl,
      Vulkan,
      Metal,
      Dx11,
      WebGl,
    };

    ShaderCompiler();
    ~ShaderCompiler() override { stop(); }

    static constexpr const char* platformArgument(Platform platform) {
      switch (platform) {
      case Platform::Linux: return "linux";
      case Platform::Mac: return "osx";
      case Platform::Windows: return "windows";
      case Platform::Emscripten: return "asm.js";
      default: return "";
      }
    }

    static constexpr const char* typeArgument(ShaderType type) {
      switch (type) {
      case ShaderType::Vertex: return "v";
      case ShaderType::Fragment: return "f";
      default: return "";
      }
    }

    static constexpr const char* profileArgument(Backend backend, ShaderType type) {
      switch (backend) {
      case Backend::Glsl: return "120";
      case Backend::Vulkan: return "spirv";
      case Backend::Metal: return "metal";
      case Backend::Dx11: return type == ShaderType::Vertex ? "vs_4_0 -O3" : "ps_4_0 -O3";
      default: return "";
      }
    }

    void compile(const std::string& shader_name, std::string code, std::function<void(std::string)> callback) {
#if !VISAGE_EMSCRIPTEN
      setCode(shader_name, std::move(code), std::move(callback));
      if (completed()) {
        stop();
        start();
      }
#endif
    }

    void compile(const EmbeddedFile& shader, std::string code, std::function<void(std::string)> callback) {
      compile(shader.name, std::move(code), std::move(callback));
    }

    void run() override;
    void watchShaderFolder(const std::string& folder_path);

    void watchShaders(const std::vector<std::string>& shaders) {
#if !VISAGE_EMSCRIPTEN
      for (const std::string& shader : shaders)
        watched_edit_times_[shader] = 0;

      start();
#endif
    }

  private:
    void compileWaitingShader();
    void loadShaderEditTimes();
    void checkShaderForEdits(const std::string& file_path);
    bool compileShader();
    bool compiling() const { return new_code_.load(); }

    void setCode(const std::string& shader_name, std::string code, std::function<void(std::string)> callback) {
      std::lock_guard lock(code_mutex_);
      shader_name_ = shader_name;
      shader_code_ = std::move(code);
      callback_ = std::move(callback);
      new_code_ = true;
    }

    void loadCode(std::string& shader_name, std::string& code, std::function<void(std::string)>& callback) {
      std::lock_guard lock(code_mutex_);
      shader_name = shader_name_;
      code = shader_code_;
      callback = callback_;
      new_code_ = false;
    }

    std::atomic<bool> new_code_ = false;
    std::string compiler_path_;
    std::mutex code_mutex_;
    std::string shader_name_;
    std::function<void(std::string)> callback_ = nullptr;
    std::string shader_code_;
    std::map<std::string, int> watched_edit_times_;
  };

  class ShaderEditor : public Frame {
  public:
    static constexpr float kPaddingHeight = 8.0f;
    static constexpr float kInfoHeightRatio = 0.3f;

    ShaderEditor();

    void setShader(const EmbeddedFile& shader, const EmbeddedFile& original_shader) {
      shader_ = shader;
      original_shader_ = original_shader;
      editor_.setText(std::string(original_shader_.data, original_shader_.size));
    }

    void draw(Canvas& canvas) override;
    void resized() override;

  private:
    ShaderCompiler compiler_;
    EmbeddedFile shader_;
    EmbeddedFile original_shader_;

    TextEditor error_;
    TextEditor editor_;
    Frame status_;
  };
}