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

#include "graphics_caches.h"

#include "embedded/shaders.h"

#include <bgfx/bgfx.h>
#include <map>

namespace visage {
  struct ShaderCacheMap {
    std::map<const char*, bgfx::ShaderHandle> cache;
    std::map<const char*, bgfx::ShaderHandle> originals;
  };

  ShaderCache::ShaderCache() {
    cache_ = std::make_unique<ShaderCacheMap>();
  }

  ShaderCache::~ShaderCache() {
    for (const auto& shader : cache_->cache)
      bgfx::destroy(shader.second);
  }

  bgfx::ShaderHandle& ShaderCache::handle(const EmbeddedFile& file) const {
    if (cache_->cache.count(file.data))
      return cache_->cache[file.data];

    const bgfx::Memory* shader_memory = bgfx::copy(file.data, file.size);
    cache_->cache[file.data] = bgfx::createShader(shader_memory);
    cache_->originals[file.data] = cache_->cache[file.data];

    return cache_->cache[file.data];
  }

  bool ShaderCache::swap(const EmbeddedFile& file, const char* data, int size) const {
    bgfx::ShaderHandle handle = bgfx::createShader(bgfx::makeRef(data, size));
    if (!bgfx::isValid(handle))
      return false;

    cache_->cache[file.data] = handle;
    return true;
  }

  void ShaderCache::restore(const EmbeddedFile& file) const {
    cache_->cache[file.data] = cache_->originals[file.data];
  }

  struct ProgramCacheMap {
    std::map<const char*, std::map<const char*, ProgramCache::ShaderPair>> shader_lookup;
    std::map<const char*, std::map<const char*, bgfx::ProgramHandle>> cache;
    std::map<const char*, std::map<const char*, bgfx::ProgramHandle>> originals;
  };

  ProgramCache::ProgramCache() {
    cache_ = std::make_unique<ProgramCacheMap>();
  }

  ProgramCache::~ProgramCache() {
    for (const auto& programs : cache_->cache) {
      for (const auto& program : programs.second) {
        if (bgfx::isValid(program.second))
          bgfx::destroy(program.second);
      }
    }
  }

  std::vector<ProgramCache::ShaderPair> ProgramCache::listPrograms() const {
    std::vector<ProgramCache::ShaderPair> results;
    for (const auto& vertex : cache_->shader_lookup) {
      for (const auto& fragment : vertex.second)
        results.push_back(fragment.second);
    }
    return results;
  }

  bgfx::ProgramHandle& ProgramCache::handle(const EmbeddedFile& vertex, const EmbeddedFile& fragment) const {
    if (cache_->cache.count(vertex.data) && cache_->cache[vertex.data].count(fragment.data))
      return cache_->cache[vertex.data][fragment.data];

    bgfx::ProgramHandle program = bgfx::createProgram(ShaderCache::shaderHandle(vertex),
                                                      ShaderCache::shaderHandle(fragment), false);
    cache_->cache[vertex.data][fragment.data] = program;
    cache_->shader_lookup[vertex.data][fragment.data] = { vertex, fragment };

    cache_->originals[vertex.data][fragment.data] = cache_->cache[vertex.data][fragment.data];
    return cache_->cache[vertex.data][fragment.data];
  }

  void ProgramCache::reload(const EmbeddedFile& vertex, const EmbeddedFile& fragment) const {
    bgfx::ProgramHandle handle = bgfx::createProgram(ShaderCache::shaderHandle(vertex),
                                                     ShaderCache::shaderHandle(fragment), false);
    if (bgfx::isValid(handle))
      cache_->cache[vertex.data][fragment.data] = handle;
  }

  void ProgramCache::restore(const EmbeddedFile& vertex, const EmbeddedFile& fragment) const {
    cache_->cache[vertex.data][fragment.data] = cache_->originals[vertex.data][fragment.data];
  }

  struct UniformCacheMap {
    std::map<std::string, bgfx::UniformHandle> cache;
  };

  UniformCache::UniformCache() {
    cache_ = std::make_unique<UniformCacheMap>();
  }

  UniformCache::~UniformCache() {
    for (const auto& uniform : cache_->cache)
      bgfx::destroy(uniform.second);
  }

  bgfx::UniformHandle& UniformCache::handle(const char* name, Type type, int size) const {
    if (cache_->cache.count(name))
      return cache_->cache[name];

    bgfx::UniformType::Enum bgfx_type = bgfx::UniformType::Vec4;
    switch (type) {
    case Sampler: bgfx_type = bgfx::UniformType::Sampler; break;
    case Vec4: bgfx_type = bgfx::UniformType::Vec4; break;
    case Mat3: bgfx_type = bgfx::UniformType::Mat3; break;
    case Mat4: bgfx_type = bgfx::UniformType::Mat4; break;
    }

    cache_->cache[name] = bgfx::createUniform(name, bgfx_type, size);
    return cache_->cache[name];
  }
}