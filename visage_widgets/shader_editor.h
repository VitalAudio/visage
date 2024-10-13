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

#include <iosfwd>
#include <mutex>
#include <string>

// For shader development purposes only
namespace visage {
  class ShaderCompiler : public Thread {
  public:
    enum Platform {
      kLinux,
      kMac,
      kWindows,
      kEmscripten,
    };

    enum ShaderType {
      kVertex,
      kFragment,
    };

    enum Backend {
      kGlsl,
      kVulkan,
      kMetal,
      kDx11,
      kWebGl,
    };

    static constexpr const char* platformArgument(Platform platform) {
      switch (platform) {
      case kLinux: return "linux";
      case kMac: return "osx";
      case kWindows: return "windows";
      case kEmscripten: return "asm.js";
      default: return "";
      }
    }

    static constexpr const char* typeArgument(ShaderType type) {
      switch (type) {
      case kVertex: return "v";
      case kFragment: return "f";
      default: return "";
      }
    }

    static constexpr const char* profileArgument(Backend backend, ShaderType type) {
      switch (backend) {
      case kGlsl: return "120";
      case kVulkan: return "spirv";
      case kMetal: return "metal";
      case kDx11: return type == kVertex ? "vs_4_0 -O3" : "ps_4_0 -O3";
      default: return "";
      }
    }

    ShaderCompiler();
    ~ShaderCompiler() override { callbacks_.clear(); }

    void setCodeAndCompile(const EmbeddedFile& vertex, const EmbeddedFile& fragment,
                           const EmbeddedFile& original, std::string code) {
      setCode(vertex, fragment, original, std::move(code));
      if (completed()) {
        stop();
        start();
      }
    }

    void setCode(const EmbeddedFile& vertex, const EmbeddedFile& fragment,
                 const EmbeddedFile& original, std::string code) {
      std::lock_guard lock(code_mutex_);
      vertex_ = vertex;
      fragment_ = fragment;
      original_ = original;
      shader_code_ = std::move(code);
      new_code_ = true;
    }

    void loadCode(EmbeddedFile& vertex, EmbeddedFile& fragment, EmbeddedFile& original, std::string& code) {
      std::lock_guard lock(code_mutex_);
      vertex = vertex_;
      fragment = fragment_;
      original = original_;
      code = shader_code_;
      new_code_ = false;
    }

    void setError(std::string error) {
      for (auto& callback : callbacks_)
        callback(error);

      error_ = std::move(error);
    }

    void addCompileCallback(std::function<void(const std::string&)> callback) {
      callbacks_.push_back(std::move(callback));
    }

    void run() override;
    void setCompilerPath(const std::string& path);
    bool compileShader(bool hot_swap);
    bool compiling() const { return new_code_.load(); }

  private:
    std::vector<std::function<void(const std::string&)>> callbacks_;

    std::atomic<bool> new_code_ = false;
    std::string compiler_path_;
    std::mutex code_mutex_;
    EmbeddedFile vertex_;
    EmbeddedFile fragment_;
    EmbeddedFile original_;
    std::string shader_code_;
    std::string error_;
    std::unique_ptr<char[]> shader_memory_;
  };

  class ShaderEditor : public Frame,
                       public TextEditor::Listener {
  public:
    static constexpr float kPaddingHeightRatio = 0.012f;
    static constexpr float kInfoHeightRatio = 0.3f;

    ShaderEditor();

    void textEditorChanged(TextEditor* editor) override;
    void setShader(const EmbeddedFile& vertex, const EmbeddedFile& fragment,
                   const EmbeddedFile& original_fragment) {
      vertex_ = vertex;
      fragment_ = fragment;
      original_fragment_ = original_fragment;
      editor_.setText(std::string(original_fragment_.data, original_fragment_.size));
    }

    void draw(Canvas& canvas) override;
    void resized() override;

    void setCompilerPath(const std::string& path) { compiler_.setCompilerPath(path); }

  private:
    EmbeddedFile vertex_;
    EmbeddedFile fragment_;
    EmbeddedFile original_fragment_;

    TextEditor error_;
    TextEditor editor_;
    ShaderCompiler compiler_;
    Frame status_;
  };
}