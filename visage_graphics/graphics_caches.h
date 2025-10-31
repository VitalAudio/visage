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

#pragma once

#include "graphics_utils.h"
#include "visage_file_embed/embedded_file.h"

namespace visage {
  struct ShaderCacheMap;
  struct ProgramCacheMap;
  struct UniformCacheMap;

  class ShaderCache {
  public:
    static ShaderCache* instance() {
      static ShaderCache cache;
      return &cache;
    }

    static bgfx::ShaderHandle& shaderHandle(const EmbeddedFile& file) {
      return instance()->handle(file);
    }

    static bgfx::ShaderHandle& shaderHandle(const char* data) { return instance()->handle(data); }

    static bool swapShader(const EmbeddedFile& file, const char* data, int size) {
      return instance()->swap(file, data, size);
    }

    static bool swapShader(const std::string& name, const char* data, int size) {
      return instance()->swap(name, data, size);
    }

    static void restoreShader(const EmbeddedFile& file) { instance()->restore(file); }

    static const char* originalData(const std::string& name) {
      return instance()->shaderData(name);
    }

  private:
    ShaderCache();
    ~ShaderCache();

    bgfx::ShaderHandle& handle(const EmbeddedFile& file) const;
    bgfx::ShaderHandle& handle(const char* data) const;
    bool swap(const char* original_data, const char* data, int size) const;
    bool swap(const EmbeddedFile& file, const char* data, int size) const;
    bool swap(const std::string& name, const char* data, int size) const;
    const char* shaderData(const std::string& name) const;
    void restore(const EmbeddedFile& file) const;

    std::unique_ptr<ShaderCacheMap> cache_;
  };

  class ProgramCache {
  public:
    struct ShaderPair {
      EmbeddedFile vertex;
      EmbeddedFile fragment;
    };

    static ProgramCache* instance() {
      static ProgramCache cache;
      return &cache;
    }

    static bgfx::ProgramHandle& programHandle(const EmbeddedFile& vertex, const EmbeddedFile& fragment) {
      return instance()->handle(vertex, fragment);
    }

    static void refreshAllProgramsWithShader(const std::string& shader_name) {
      instance()->reloadAll(shader_name);
    }

    static void refreshAllProgramsWithShader(const EmbeddedFile& shader) {
      instance()->reloadAll(shader);
    }

    static void refreshProgram(const EmbeddedFile& vertex, const EmbeddedFile& fragment) {
      instance()->reload(vertex, fragment);
    }

    static void restoreProgram(const EmbeddedFile& vertex, const EmbeddedFile& fragment) {
      instance()->restore(vertex, fragment);
    }

    static std::vector<ShaderPair> programList() { return instance()->listPrograms(); }

  private:
    ProgramCache();
    ~ProgramCache();

    std::vector<ShaderPair> listPrograms() const;

    bgfx::ProgramHandle& handle(const EmbeddedFile& vertex, const EmbeddedFile& fragment) const;
    void reload(const EmbeddedFile& vertex, const EmbeddedFile& fragment) const;
    void reloadAll(const char* shader_data) const;
    void reloadAll(const std::string& shader_name) const;
    void reloadAll(const EmbeddedFile& shader) const;
    void restore(const EmbeddedFile& vertex, const EmbeddedFile& fragment) const;

    std::unique_ptr<ProgramCacheMap> cache_;
  };

  class UniformCache {
  public:
    enum Type {
      Sampler,
      Vec4,
      Mat3,
      Mat4,
    };

    static UniformCache* instance() {
      static UniformCache cache;
      return &cache;
    }

    static bgfx::UniformHandle& uniformHandle(const char* name, Type type = Vec4) {
      return instance()->handle(name, type);
    }

  private:
    UniformCache();
    ~UniformCache();

    bgfx::UniformHandle& handle(const char* name, Type type, int size = 1) const;

    std::unique_ptr<UniformCacheMap> cache_;
  };
}