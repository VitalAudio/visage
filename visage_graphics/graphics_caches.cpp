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

#include "graphics_caches.h"

#include <bgfx/bgfx.h>
#include <map>

namespace visage {
  struct ShaderCacheMap {
    std::map<const char*, bgfx::ShaderHandle> cache;
    std::map<const char*, bgfx::ShaderHandle> originals;
    std::map<std::string, const char*> name_lookup;
  };

  ShaderCache::ShaderCache() {
    cache_ = std::make_unique<ShaderCacheMap>();
  }

  ShaderCache::~ShaderCache() {
    for (const auto& shader : cache_->cache)
      bgfx::destroy(shader.second);
  }

  bgfx::ShaderHandle& ShaderCache::handle(const EmbeddedFile& file) const {
    auto file_data = reinterpret_cast<const char*>(file.data);
    if (cache_->cache.count(file_data))
      return cache_->cache[file_data];

    const bgfx::Memory* shader_memory = bgfx::copy(file.data, file.size);
    cache_->name_lookup[file.name] = file_data;
    cache_->cache[file_data] = bgfx::createShader(shader_memory);
    cache_->originals[file_data] = cache_->cache[file_data];

    return cache_->cache[file_data];
  }

  bgfx::ShaderHandle& ShaderCache::handle(const char* data) const {
    VISAGE_ASSERT(cache_->cache.count(data));
    return cache_->cache[data];
  }

  bool ShaderCache::swap(const char* original_data, const char* data, int size) const {
    bgfx::ShaderHandle handle = bgfx::createShader(bgfx::copy(data, size));
    if (!bgfx::isValid(handle))
      return false;

    cache_->cache[original_data] = handle;
    return true;
  }

  bool ShaderCache::swap(const EmbeddedFile& file, const char* data, int size) const {
    return swap(reinterpret_cast<const char*>(file.data), data, size);
  }

  bool ShaderCache::swap(const std::string& name, const char* data, int size) const {
    return swap(cache_->name_lookup[name], data, size);
  }

  const char* ShaderCache::shaderData(const std::string& name) const {
    return cache_->name_lookup[name];
  }

  void ShaderCache::restore(const EmbeddedFile& file) const {
    auto file_data = reinterpret_cast<const char*>(file.data);
    cache_->cache[file_data] = cache_->originals[file_data];
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
    auto vertex_data = reinterpret_cast<const char*>(vertex.data);
    auto fragment_data = reinterpret_cast<const char*>(fragment.data);

    if (cache_->cache.count(vertex_data) && cache_->cache[vertex_data].count(fragment_data))
      return cache_->cache[vertex_data][fragment_data];

    bgfx::ProgramHandle program = bgfx::createProgram(ShaderCache::shaderHandle(vertex),
                                                      ShaderCache::shaderHandle(fragment), false);
    cache_->cache[vertex_data][fragment_data] = program;
    cache_->shader_lookup[vertex_data][fragment_data] = { vertex, fragment };

    cache_->originals[vertex_data][fragment_data] = cache_->cache[vertex_data][fragment_data];
    return cache_->cache[vertex_data][fragment_data];
  }

  void ProgramCache::reload(const EmbeddedFile& vertex, const EmbeddedFile& fragment) const {
    bgfx::ProgramHandle handle = bgfx::createProgram(ShaderCache::shaderHandle(vertex),
                                                     ShaderCache::shaderHandle(fragment), false);
    auto vertex_data = reinterpret_cast<const char*>(vertex.data);
    auto fragment_data = reinterpret_cast<const char*>(fragment.data);

    if (bgfx::isValid(handle))
      cache_->cache[vertex_data][fragment_data] = handle;
  }

  void ProgramCache::reloadAll(const char* shader_data) const {
    for (const auto& vertex : cache_->cache) {
      for (const auto& fragment : vertex.second) {
        if (shader_data == vertex.first || shader_data == fragment.first) {
          bgfx::ProgramHandle handle = bgfx::createProgram(ShaderCache::shaderHandle(vertex.first),
                                                           ShaderCache::shaderHandle(fragment.first),
                                                           false);
          if (bgfx::isValid(handle))
            cache_->cache[vertex.first][fragment.first] = handle;
        }
      }
    }
  }
  void ProgramCache::reloadAll(const std::string& shader_name) const {
    reloadAll(ShaderCache::originalData(shader_name));
  }

  void ProgramCache::reloadAll(const EmbeddedFile& shader) const {
    reloadAll(ShaderCache::originalData(reinterpret_cast<const char*>(shader.data)));
  }

  void ProgramCache::restore(const EmbeddedFile& vertex, const EmbeddedFile& fragment) const {
    auto vertex_data = reinterpret_cast<const char*>(vertex.data);
    auto fragment_data = reinterpret_cast<const char*>(fragment.data);
    cache_->cache[vertex_data][fragment_data] = cache_->originals[vertex_data][fragment_data];
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