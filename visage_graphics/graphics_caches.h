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

#include "embedded_file.h"
#include "graphics_utils.h"

#include <string>

namespace visage {
  struct ShaderCacheMap;
  struct ProgramCacheMap;
  struct UniformCacheMap;
  struct EmbeddedFile;

  class ShaderCache {
  public:
    static ShaderCache* getInstance() {
      static ShaderCache cache;
      return &cache;
    }

    static bgfx::ShaderHandle& getShader(const EmbeddedFile& file) {
      return getInstance()->get(file);
    }

    static void swapShader(const EmbeddedFile& file, const char* data, int size) {
      return getInstance()->swap(file, data, size);
    }

    static void restoreShader(const EmbeddedFile& file) { return getInstance()->restore(file); }

  private:
    ShaderCache();
    ~ShaderCache();

    bgfx::ShaderHandle& get(const EmbeddedFile& file) const;
    void swap(const EmbeddedFile& file, const char* data, int size) const;
    void restore(const EmbeddedFile& file) const;

    std::unique_ptr<ShaderCacheMap> cache_;
  };

  class ProgramCache {
  public:
    struct ShaderPair {
      EmbeddedFile vertex;
      EmbeddedFile fragment;
    };

    static ProgramCache* getInstance() {
      static ProgramCache cache;
      return &cache;
    }

    static bgfx::ProgramHandle& getProgram(const EmbeddedFile& vertex, const EmbeddedFile& fragment) {
      return getInstance()->get(vertex, fragment);
    }

    static void refreshProgram(const EmbeddedFile& vertex, const EmbeddedFile& fragment) {
      getInstance()->reload(vertex, fragment);
    }

    static void restoreProgram(const EmbeddedFile& vertex, const EmbeddedFile& fragment) {
      getInstance()->restore(vertex, fragment);
    }

    static std::vector<ShaderPair> getProgramList() { return getInstance()->programList(); }

  private:
    ProgramCache();
    ~ProgramCache();

    std::vector<ShaderPair> programList() const;

    bgfx::ProgramHandle& get(const EmbeddedFile& vertex, const EmbeddedFile& fragment) const;
    bgfx::ProgramHandle& reload(const EmbeddedFile& vertex, const EmbeddedFile& fragment) const;
    bgfx::ProgramHandle& restore(const EmbeddedFile& vertex, const EmbeddedFile& fragment) const;

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

    static UniformCache* getInstance() {
      static UniformCache cache;
      return &cache;
    }

    static inline bgfx::UniformHandle& getUniform(const char* name, Type type = Vec4) {
      return getInstance()->get(name, type);
    }

  private:
    UniformCache();
    ~UniformCache();

    bgfx::UniformHandle& get(const char* name, Type type, int size = 1) const;

    std::unique_ptr<UniformCacheMap> cache_;
  };
}