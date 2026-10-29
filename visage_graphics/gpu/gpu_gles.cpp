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

#include "gpu_backend.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <emscripten/html5.h>
#include <emscripten/html5_webgl.h>
#include <GLES3/gl3.h>
#include <unordered_map>

namespace visage::gpu {
  namespace {
    constexpr const char* kCanvas = "#canvas";

    struct GlFormat {
      GLenum internal_format;
      GLenum format;
      GLenum type;
      uint32_t bytes;
    };

    GlFormat glFormat(TextureFormat format) {
      switch (format) {
      case TextureFormat::RGBA8:
      case TextureFormat::BGRA8: return { GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, 4 };
      case TextureFormat::R8: return { GL_R8, GL_RED, GL_UNSIGNED_BYTE, 1 };
      case TextureFormat::R32F: return { GL_R32F, GL_RED, GL_FLOAT, 4 };
      case TextureFormat::RGBA16F: return { GL_RGBA16F, GL_RGBA, GL_HALF_FLOAT, 8 };
      case TextureFormat::RGBA32F: return { GL_RGBA32F, GL_RGBA, GL_FLOAT, 16 };
      default: return { GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, 4 };
      }
    }

    GLenum glBlendFactor(BlendFactor factor) {
      switch (factor) {
      case BlendFactor::Zero: return GL_ZERO;
      case BlendFactor::One: return GL_ONE;
      case BlendFactor::SrcColor: return GL_SRC_COLOR;
      case BlendFactor::InvSrcColor: return GL_ONE_MINUS_SRC_COLOR;
      case BlendFactor::SrcAlpha: return GL_SRC_ALPHA;
      case BlendFactor::InvSrcAlpha: return GL_ONE_MINUS_SRC_ALPHA;
      case BlendFactor::DstColor: return GL_DST_COLOR;
      case BlendFactor::InvDstColor: return GL_ONE_MINUS_DST_COLOR;
      case BlendFactor::DstAlpha: return GL_DST_ALPHA;
      case BlendFactor::InvDstAlpha: return GL_ONE_MINUS_DST_ALPHA;
      }
      return GL_ONE;
    }

    GLenum glBlendOp(BlendOp op) {
      switch (op) {
      case BlendOp::Add: return GL_FUNC_ADD;
      case BlendOp::Subtract: return GL_FUNC_SUBTRACT;
      case BlendOp::ReverseSubtract: return GL_FUNC_REVERSE_SUBTRACT;
      case BlendOp::Min: return GL_MIN;
      case BlendOp::Max: return GL_MAX;
      }
      return GL_FUNC_ADD;
    }

    void swizzleRedBlue(uint8_t* data, size_t pixels) {
      for (size_t i = 0; i < pixels; ++i)
        std::swap(data[i * 4], data[i * 4 + 2]);
    }

    struct Texture {
      bool used = false;
      TextureDesc desc;
      GLuint texture = 0;
      GlFormat format {};
    };

    struct Buffer {
      bool used = false;
      BufferDesc desc;
      GLuint buffer = 0;
    };

    struct Shader {
      bool used = false;
      ShaderInfo info;
      GLuint shader = 0;
    };

    struct ProgramUniform {
      GLint location = -1;
      UniformType type = UniformType::Vec4;
      uint16_t num = 1;
      uint16_t offset = 0;
      bool fragment = false;
    };

    struct Program {
      bool used = false;
      GLuint program = 0;
      GLint vertex_block = -1;
      GLint fragment_block = -1;
      std::vector<ShaderAttribute> attributes;
      std::vector<ProgramUniform> uniforms;
    };

    struct FrameBuffer {
      bool used = false;
      bool swap_chain = false;
      uint16_t texture = kInvalidHandle;
      GLuint framebuffer = 0;
      int width = 0;
      int height = 0;
    };

    template<typename T>
    T& entry(std::vector<T>& entries, uint16_t idx) {
      if (entries.size() <= idx)
        entries.resize(idx + 1);
      return entries[idx];
    }
  }

  class GlesBackend : public Backend {
  public:
    ~GlesBackend() override { shutdown(); }

    bool init(const Init& init, Caps& caps) override {
      init_ = init;
      context_ = emscripten_webgl_get_current_context();
      if (context_ <= 0) {
        EmscriptenWebGLContextAttributes attributes;
        emscripten_webgl_init_context_attributes(&attributes);
        attributes.alpha = true;
        attributes.premultipliedAlpha = false;
        attributes.depth = false;
        attributes.stencil = false;
        attributes.antialias = false;
        attributes.enableExtensionsByDefault = true;
        attributes.majorVersion = 2;
        attributes.minorVersion = 0;
        context_ = emscripten_webgl_create_context(kCanvas, &attributes);
        if (context_ <= 0)
          return false;
        owns_context_ = true;
      }
      if (emscripten_webgl_make_context_current(context_) != EMSCRIPTEN_RESULT_SUCCESS)
        return false;

      bool color_buffer_float = emscripten_webgl_enable_extension(context_, "EXT_color_buffer_float");
      bool color_buffer_half_float = emscripten_webgl_enable_extension(context_, "EXT_color_buffer_half_float");
      emscripten_webgl_enable_extension(context_, "OES_texture_float_linear");

      GLint max_texture_size = 0;
      glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_texture_size);

      caps.renderer = RendererType::OpenGLES;
      caps.origin_bottom_left = true;
      caps.compute = false;
      caps.swap_chain = true;
      caps.max_texture_size = max_texture_size;
      uint32_t all = kFormatTexture | kFormatRenderTarget | kFormatBlend;
      caps.formats[static_cast<int>(TextureFormat::RGBA8)] = all;
      caps.formats[static_cast<int>(TextureFormat::BGRA8)] = kFormatTexture;
      caps.formats[static_cast<int>(TextureFormat::R8)] = all;
      caps.formats[static_cast<int>(TextureFormat::R32F)] = kFormatTexture |
                                                            (color_buffer_float ? kFormatRenderTarget : 0);
      caps.formats[static_cast<int>(TextureFormat::RGBA16F)] =
          kFormatTexture | (color_buffer_float || color_buffer_half_float ? kFormatRenderTarget | kFormatBlend : 0);
      caps.formats[static_cast<int>(TextureFormat::RGBA32F)] = kFormatTexture |
                                                               (color_buffer_float ? kFormatRenderTarget : 0);

      glGenVertexArrays(1, &vertex_array_);
      glBindVertexArray(vertex_array_);
      glGenFramebuffers(1, &read_framebuffer_);
      glGenFramebuffers(1, &draw_framebuffer_);
      glDisable(GL_DEPTH_TEST);
      glDisable(GL_CULL_FACE);
      glDisable(GL_STENCIL_TEST);
      glDisable(GL_DITHER);
      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
      glPixelStorei(GL_PACK_ALIGNMENT, 1);
      return true;
    }

    void shutdown() override {
      if (context_ <= 0)
        return;

      for (Texture& texture : textures_) {
        if (texture.used)
          glDeleteTextures(1, &texture.texture);
      }
      for (Buffer& buffer : buffers_) {
        if (buffer.used)
          glDeleteBuffers(1, &buffer.buffer);
      }
      for (Program& program : programs_) {
        if (program.used)
          glDeleteProgram(program.program);
      }
      for (Shader& shader : shaders_) {
        if (shader.used)
          glDeleteShader(shader.shader);
      }
      for (FrameBuffer& frame_buffer : frame_buffers_) {
        if (frame_buffer.used && frame_buffer.framebuffer)
          glDeleteFramebuffers(1, &frame_buffer.framebuffer);
      }
      for (auto& sampler : samplers_)
        glDeleteSamplers(1, &sampler.second);
      textures_.clear();
      buffers_.clear();
      programs_.clear();
      shaders_.clear();
      frame_buffers_.clear();
      samplers_.clear();

      glDeleteFramebuffers(1, &read_framebuffer_);
      glDeleteFramebuffers(1, &draw_framebuffer_);
      glDeleteVertexArrays(1, &vertex_array_);
      if (owns_context_)
        emscripten_webgl_destroy_context(context_);
      context_ = 0;
    }

    bool createTexture(uint16_t idx, const TextureDesc& desc, const uint8_t* data, int pitch) override {
      Texture& texture = entry(textures_, idx);
      texture = Texture();
      texture.desc = desc;
      texture.format = glFormat(desc.format);

      glGenTextures(1, &texture.texture);
      glBindTexture(GL_TEXTURE_2D, texture.texture);
      glTexStorage2D(GL_TEXTURE_2D, 1, texture.format.internal_format, desc.width, desc.height);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
      if (glGetError() != GL_NO_ERROR) {
        glDeleteTextures(1, &texture.texture);
        texture = Texture();
        return false;
      }
      texture.used = true;

      std::vector<uint8_t> zeros;
      if (data == nullptr) {
        zeros.resize(static_cast<size_t>(desc.width) * desc.height * texture.format.bytes);
        data = zeros.data();
        pitch = 0;
      }
      updateTexture(idx, 0, 0, desc.width, desc.height, data, pitch);
      return true;
    }

    void updateTexture(uint16_t idx, int x, int y, int width, int height, const uint8_t* data,
                       int pitch) override {
      Texture& texture = textures_[idx];
      uint32_t row_bytes = width * texture.format.bytes;
      uint32_t source_pitch = pitch ? pitch : row_bytes;

      std::vector<uint8_t> swizzled;
      if (texture.desc.format == TextureFormat::BGRA8) {
        swizzled.resize(static_cast<size_t>(row_bytes) * height);
        for (int row = 0; row < height; ++row)
          std::memcpy(swizzled.data() + row * row_bytes, data + row * source_pitch, row_bytes);
        swizzleRedBlue(swizzled.data(), static_cast<size_t>(width) * height);
        data = swizzled.data();
        source_pitch = row_bytes;
      }

      glBindTexture(GL_TEXTURE_2D, texture.texture);
      glPixelStorei(GL_UNPACK_ROW_LENGTH, source_pitch / texture.format.bytes);
      glTexSubImage2D(GL_TEXTURE_2D, 0, x, y, width, height, texture.format.format, texture.format.type, data);
      glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    }

    void destroyTexture(uint16_t idx) override {
      glDeleteTextures(1, &textures_[idx].texture);
      textures_[idx] = Texture();
    }

    bool createBuffer(uint16_t idx, const BufferDesc& desc, const uint8_t* data) override {
      Buffer& buffer = entry(buffers_, idx);
      buffer = Buffer();
      buffer.desc = desc;
      glGenBuffers(1, &buffer.buffer);
      GLenum target = desc.index ? GL_ELEMENT_ARRAY_BUFFER : GL_ARRAY_BUFFER;
      glBindBuffer(target, buffer.buffer);
      std::vector<uint8_t> zeros;
      if (data == nullptr) {
        zeros.resize(desc.size);
        data = zeros.data();
      }
      glBufferData(target, desc.size, data, desc.dynamic ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
      if (glGetError() != GL_NO_ERROR) {
        glDeleteBuffers(1, &buffer.buffer);
        buffer = Buffer();
        return false;
      }
      buffer.used = true;
      return true;
    }

    void updateBuffer(uint16_t idx, uint32_t offset, const uint8_t* data, uint32_t size) override {
      Buffer& buffer = buffers_[idx];
      GLenum target = buffer.desc.index ? GL_ELEMENT_ARRAY_BUFFER : GL_ARRAY_BUFFER;
      glBindBuffer(target, buffer.buffer);
      glBufferSubData(target, offset, size, data);
    }

    void destroyBuffer(uint16_t idx) override {
      glDeleteBuffers(1, &buffers_[idx].buffer);
      buffers_[idx] = Buffer();
    }

    bool createShader(uint16_t idx, const ShaderInfo& info) override {
      if (info.code_type != ShaderCode::Essl || info.stage == ShaderStage::Compute)
        return false;

      Shader& shader = entry(shaders_, idx);
      shader = Shader();
      GLenum type = info.stage == ShaderStage::Vertex ? GL_VERTEX_SHADER : GL_FRAGMENT_SHADER;
      shader.shader = glCreateShader(type);
      const GLchar* source = reinterpret_cast<const GLchar*>(info.code.data());
      GLint length = static_cast<GLint>(info.code.size());
      glShaderSource(shader.shader, 1, &source, &length);
      glCompileShader(shader.shader);
      GLint compiled = 0;
      glGetShaderiv(shader.shader, GL_COMPILE_STATUS, &compiled);
      if (!compiled) {
        char log[2048] {};
        glGetShaderInfoLog(shader.shader, sizeof(log), nullptr, log);
        logMessage(log);
        glDeleteShader(shader.shader);
        shader = Shader();
        return false;
      }

      shader.used = true;
      shader.info = info;
      shader.info.code.clear();
      return true;
    }

    void destroyShader(uint16_t idx) override {
      glDeleteShader(shaders_[idx].shader);
      shaders_[idx] = Shader();
    }

    bool createProgram(uint16_t idx, uint16_t vertex, uint16_t fragment) override {
      Program& program = entry(programs_, idx);
      program = Program();
      program.program = glCreateProgram();
      glAttachShader(program.program, shaders_[vertex].shader);
      glAttachShader(program.program, shaders_[fragment].shader);
      glLinkProgram(program.program);
      GLint linked = 0;
      glGetProgramiv(program.program, GL_LINK_STATUS, &linked);
      if (!linked) {
        char log[2048] {};
        glGetProgramInfoLog(program.program, sizeof(log), nullptr, log);
        logMessage(log);
        glDeleteProgram(program.program);
        program = Program();
        return false;
      }

      program.used = true;
      program.attributes = shaders_[vertex].info.attributes;
      glUseProgram(program.program);
      bindUniforms(program, shaders_[vertex].info, false);
      bindUniforms(program, shaders_[fragment].info, true);
      glUseProgram(0);
      return true;
    }

    bool createComputeProgram(uint16_t, uint16_t) override { return false; }

    void destroyProgram(uint16_t idx) override {
      glDeleteProgram(programs_[idx].program);
      programs_[idx] = Program();
    }

    bool createFrameBuffer(uint16_t idx, uint16_t texture_idx) override {
      FrameBuffer& frame_buffer = entry(frame_buffers_, idx);
      frame_buffer = FrameBuffer();
      Texture& texture = textures_[texture_idx];
      glGenFramebuffers(1, &frame_buffer.framebuffer);
      glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer.framebuffer);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture.texture, 0);
      GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      if (status != GL_FRAMEBUFFER_COMPLETE) {
        glDeleteFramebuffers(1, &frame_buffer.framebuffer);
        frame_buffer = FrameBuffer();
        return false;
      }

      frame_buffer.used = true;
      frame_buffer.texture = texture_idx;
      frame_buffer.width = texture.desc.width;
      frame_buffer.height = texture.desc.height;
      return true;
    }

    bool createSwapChain(uint16_t idx, void*, void*, int width, int height, TextureFormat) override {
      FrameBuffer& frame_buffer = entry(frame_buffers_, idx);
      frame_buffer = FrameBuffer();
      frame_buffer.used = true;
      frame_buffer.swap_chain = true;
      frame_buffer.width = width;
      frame_buffer.height = height;
      return true;
    }

    void resizeSwapChain(uint16_t idx, int width, int height) override {
      frame_buffers_[idx].width = width;
      frame_buffers_[idx].height = height;
    }

    void destroyFrameBuffer(uint16_t idx) override {
      if (frame_buffers_[idx].framebuffer)
        glDeleteFramebuffers(1, &frame_buffers_[idx].framebuffer);
      frame_buffers_[idx] = FrameBuffer();
    }

    bool waitForFrame(uint16_t, int) override { return true; }

    void beginFrame(uint32_t) override { emscripten_webgl_make_context_current(context_); }

    void submitFrame(const FrameData& frame, Stats&) override {
      for (const ViewData& view : frame.views)
        executeView(frame, view);

      for (const ReadRequest& request : frame.reads) {
        if (request.texture < textures_.size() && textures_[request.texture].used)
          readTexture(textures_[request.texture], request.data);
      }

      for (uint16_t idx : frame.screenshots) {
        if (idx >= frame_buffers_.size() || !frame_buffers_[idx].used)
          continue;
        FrameBuffer& frame_buffer = frame_buffers_[idx];
        std::vector<uint8_t> data(static_cast<size_t>(frame_buffer.width) * frame_buffer.height * 4);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, frame_buffer.framebuffer);
        glReadPixels(0, 0, frame_buffer.width, frame_buffer.height, GL_RGBA, GL_UNSIGNED_BYTE, data.data());
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        if (init_.screenshot_callback)
          init_.screenshot_callback({ idx }, data.data(), frame_buffer.width, frame_buffer.height,
                                    frame_buffer.width * 4, false);
      }
    }

  private:
    void logMessage(const char* message) const {
      if (init_.log_callback)
        init_.log_callback(message);
      else
        std::fprintf(stderr, "visage gles: %s\n", message);
    }

    static bool matchesUniform(const std::string& active, const std::string& name) {
      std::string base = active;
      size_t bracket = base.find('[');
      if (bracket != std::string::npos)
        base = base.substr(0, bracket);
      if (base == name)
        return true;
      if (base.size() <= name.size())
        return false;
      char separator = base[base.size() - name.size() - 1];
      return (separator == '.' || separator == '_') && base.compare(base.size() - name.size(), name.size(), name) == 0;
    }

    void bindUniforms(Program& program, const ShaderInfo& info, bool fragment) {
      GLint block = glGetUniformLocation(program.program, fragment ? "visage_fragment_uniforms" : "visage_vertex_uniforms");
      (fragment ? program.fragment_block : program.vertex_block) = block;
      GLint count = 0;
      glGetProgramiv(program.program, GL_ACTIVE_UNIFORMS, &count);
      std::vector<std::string> names;
      for (GLint i = 0; i < count; ++i) {
        char name[256] {};
        GLsizei length = 0;
        GLint size = 0;
        GLenum type = 0;
        glGetActiveUniform(program.program, i, sizeof(name), &length, &size, &type, name);
        names.emplace_back(name, length);
      }

      for (const ShaderUniform& uniform : info.uniforms) {
        if (block >= 0)
          break;
        for (const std::string& name : names) {
          if (!matchesUniform(name, uniform.name))
            continue;
          ProgramUniform result;
          result.location = glGetUniformLocation(program.program, name.c_str());
          result.type = uniform.type;
          result.num = uniform.num;
          result.offset = uniform.offset;
          result.fragment = fragment;
          if (result.location >= 0)
            program.uniforms.push_back(result);
          break;
        }
      }

      for (const ShaderSampler& sampler : info.samplers) {
        for (const std::string& name : names) {
          if (!matchesUniform(name, sampler.name))
            continue;
          GLint location = glGetUniformLocation(program.program, name.c_str());
          if (location >= 0)
            glUniform1i(location, sampler.stage);
          break;
        }
      }
    }

    GLuint sampler(uint32_t flags) {
      flags &= kSamplerMask;
      auto found = samplers_.find(flags);
      if (found != samplers_.end())
        return found->second;

      auto wrap = [](bool clamp, bool mirror) {
        if (clamp)
          return GL_CLAMP_TO_EDGE;
        return mirror ? GL_MIRRORED_REPEAT : GL_REPEAT;
      };

      GLuint result = 0;
      glGenSamplers(1, &result);
      glSamplerParameteri(result, GL_TEXTURE_MIN_FILTER, (flags & kSamplerMinPoint) ? GL_NEAREST : GL_LINEAR);
      glSamplerParameteri(result, GL_TEXTURE_MAG_FILTER, (flags & kSamplerMagPoint) ? GL_NEAREST : GL_LINEAR);
      glSamplerParameteri(result, GL_TEXTURE_WRAP_S, wrap(flags & kSamplerUClamp, flags & kSamplerUMirror));
      glSamplerParameteri(result, GL_TEXTURE_WRAP_T, wrap(flags & kSamplerVClamp, flags & kSamplerVMirror));
      samplers_[flags] = result;
      return result;
    }

    void applyState(const RenderState& state) {
      glColorMask((state.write_mask & kWriteR) != 0, (state.write_mask & kWriteG) != 0,
                  (state.write_mask & kWriteB) != 0, (state.write_mask & kWriteA) != 0);
      if (state.blend) {
        glEnable(GL_BLEND);
        glBlendFuncSeparate(glBlendFactor(state.src_rgb), glBlendFactor(state.dst_rgb),
                            glBlendFactor(state.src_alpha), glBlendFactor(state.dst_alpha));
        glBlendEquationSeparate(glBlendOp(state.op_rgb), glBlendOp(state.op_alpha));
      }
      else
        glDisable(GL_BLEND);
    }

    void applyUniforms(const FrameData& frame, const DrawItem& draw, const Program& program) {
      if (program.vertex_block >= 0 && draw.vertex_uniform_size) {
        glUniform4fv(program.vertex_block, draw.vertex_uniform_size / 16,
                     reinterpret_cast<const float*>(frame.uniform_data.data() + draw.vertex_uniform_offset));
      }
      if (program.fragment_block >= 0 && draw.fragment_uniform_size) {
        glUniform4fv(program.fragment_block, draw.fragment_uniform_size / 16,
                     reinterpret_cast<const float*>(frame.uniform_data.data() + draw.fragment_uniform_offset));
      }
      for (const ProgramUniform& uniform : program.uniforms) {
        uint32_t base = uniform.fragment ? draw.fragment_uniform_offset : draw.vertex_uniform_offset;
        uint32_t size = uniform.fragment ? draw.fragment_uniform_size : draw.vertex_uniform_size;
        if (uniform.offset >= size)
          continue;
        const float* values = reinterpret_cast<const float*>(frame.uniform_data.data() + base + uniform.offset);
        switch (uniform.type) {
        case UniformType::Mat4: glUniformMatrix4fv(uniform.location, uniform.num, GL_FALSE, values); break;
        case UniformType::Mat3: {
          std::vector<float> packed(9 * uniform.num);
          for (int n = 0; n < uniform.num; ++n) {
            for (int column = 0; column < 3; ++column)
              std::memcpy(&packed[n * 9 + column * 3], values + (n * 3 + column) * 4, 3 * sizeof(float));
          }
          glUniformMatrix3fv(uniform.location, uniform.num, GL_FALSE, packed.data());
          break;
        }
        default: glUniform4fv(uniform.location, uniform.num, values); break;
        }
      }
    }

    void bindFrameBuffer(const ViewData& view) {
      FrameBuffer& frame_buffer = frame_buffers_[view.frame_buffer];
      glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer.framebuffer);
      int height = frame_buffer.height;
      if (frame_buffer.swap_chain) {
        int width = 0;
        emscripten_webgl_get_drawing_buffer_size(context_, &width, &height);
      }
      glViewport(view.x, height - view.height - view.y, view.width, view.height);
    }

    void executeDraw(const FrameData& frame, const DrawItem& draw) {
      if (draw.program >= programs_.size() || !programs_[draw.program].used)
        return;
      Program& program = programs_[draw.program];
      glUseProgram(program.program);
      applyState(draw.state);
      applyUniforms(frame, draw, program);

      for (int i = 0; i < kMaxTextureStages; ++i) {
        uint16_t idx = draw.textures[i].texture;
        glActiveTexture(GL_TEXTURE0 + i);
        if (idx < textures_.size() && textures_[idx].used) {
          glBindTexture(GL_TEXTURE_2D, textures_[idx].texture);
          glBindSampler(i, sampler(draw.textures[i].sampler_flags));
        }
        else {
          glBindTexture(GL_TEXTURE_2D, 0);
          glBindSampler(i, 0);
        }
      }

      for (int i = 0; i < static_cast<int>(Attrib::Count); ++i)
        glDisableVertexAttribArray(i);

      bool has_vertex_buffer = draw.vertex_buffer < buffers_.size() && buffers_[draw.vertex_buffer].used;
      if (has_vertex_buffer) {
        const Buffer& buffer = buffers_[draw.vertex_buffer];
        const VertexLayout& layout = buffer.desc.layout;
        glBindBuffer(GL_ARRAY_BUFFER, buffer.buffer);
        uintptr_t base = static_cast<uintptr_t>(draw.vertex_start) * layout.stride();
        for (const ShaderAttribute& attribute : program.attributes) {
          if (!layout.has(attribute.attrib))
            continue;
          GLuint location = static_cast<GLuint>(attribute.attrib);
          glEnableVertexAttribArray(location);
          glVertexAttribPointer(location, layout.count(attribute.attrib), GL_FLOAT, GL_FALSE, layout.stride(),
                                reinterpret_cast<const void*>(base + layout.offset(attribute.attrib)));
        }
      }

      if (draw.index_buffer < buffers_.size() && buffers_[draw.index_buffer].used) {
        const Buffer& buffer = buffers_[draw.index_buffer];
        bool index32 = buffer.desc.flags & kBufferIndex32;
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer.buffer);
        uintptr_t offset = static_cast<uintptr_t>(draw.index_start) * (index32 ? 4 : 2);
        glDrawElements(GL_TRIANGLES, draw.index_count, index32 ? GL_UNSIGNED_INT : GL_UNSIGNED_SHORT,
                       reinterpret_cast<const void*>(offset));
      }
      else if (has_vertex_buffer)
        glDrawArrays(GL_TRIANGLES, 0, draw.vertex_count);
      else
        glDrawArrays(GL_TRIANGLES, draw.vertex_start, draw.vertex_count);
    }

    void executeBlit(const BlitItem& item) {
      if (item.source >= textures_.size() || !textures_[item.source].used ||
          item.destination >= textures_.size() || !textures_[item.destination].used)
        return;

      glBindFramebuffer(GL_READ_FRAMEBUFFER, read_framebuffer_);
      glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                             textures_[item.source].texture, 0);
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, draw_framebuffer_);
      glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                             textures_[item.destination].texture, 0);
      glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
      glBlitFramebuffer(item.source_x, item.source_y, item.source_x + item.width, item.source_y + item.height,
                        item.dest_x, item.dest_y, item.dest_x + item.width, item.dest_y + item.height,
                        GL_COLOR_BUFFER_BIT, GL_NEAREST);
      glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    }

    void executeView(const FrameData& frame, const ViewData& view) {
      bool has_target = view.frame_buffer < frame_buffers_.size() && frame_buffers_[view.frame_buffer].used;
      bool bound = false;
      for (const ViewItem& item : view.items) {
        if (item.type == ViewItem::Draw) {
          if (!has_target)
            continue;
          if (!bound) {
            bindFrameBuffer(view);
            bound = true;
          }
          executeDraw(frame, frame.draws[item.index]);
        }
        else if (item.type == ViewItem::Blit) {
          executeBlit(frame.blits[item.index]);
          bound = false;
        }
      }
    }

    static uint16_t halfFromFloat(float value) {
      uint32_t bits = 0;
      std::memcpy(&bits, &value, sizeof(bits));
      uint32_t sign = (bits >> 16) & 0x8000;
      int exponent = static_cast<int>((bits >> 23) & 0xff) - 127 + 15;
      uint32_t mantissa = bits & 0x7fffff;
      if (((bits >> 23) & 0xff) == 0xff)
        return static_cast<uint16_t>(sign | 0x7c00 | (mantissa ? 0x200 : 0));
      if (exponent >= 31)
        return static_cast<uint16_t>(sign | 0x7c00);
      if (exponent <= 0) {
        if (exponent < -10)
          return static_cast<uint16_t>(sign);
        mantissa |= 0x800000;
        return static_cast<uint16_t>(sign | (mantissa >> (14 - exponent)));
      }
      return static_cast<uint16_t>(sign | (exponent << 10) | (mantissa >> 13));
    }

    void readTexture(Texture& texture, void* data) {
      int width = texture.desc.width;
      int height = texture.desc.height;
      size_t pixels = static_cast<size_t>(width) * height;
      glBindFramebuffer(GL_READ_FRAMEBUFFER, read_framebuffer_);
      glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture.texture, 0);

      switch (texture.desc.format) {
      case TextureFormat::RGBA8:
      case TextureFormat::BGRA8:
        glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, data);
        if (texture.desc.format == TextureFormat::BGRA8)
          swizzleRedBlue(static_cast<uint8_t*>(data), pixels);
        break;
      case TextureFormat::R8: {
        std::vector<uint8_t> rgba(pixels * 4);
        glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
        auto result = static_cast<uint8_t*>(data);
        for (size_t i = 0; i < pixels; ++i)
          result[i] = rgba[i * 4];
        break;
      }
      default: {
        std::vector<float> rgba(pixels * 4);
        glReadPixels(0, 0, width, height, GL_RGBA, GL_FLOAT, rgba.data());
        if (texture.desc.format == TextureFormat::RGBA32F)
          std::memcpy(data, rgba.data(), rgba.size() * sizeof(float));
        else if (texture.desc.format == TextureFormat::R32F) {
          auto result = static_cast<float*>(data);
          for (size_t i = 0; i < pixels; ++i)
            result[i] = rgba[i * 4];
        }
        else {
          auto result = static_cast<uint16_t*>(data);
          for (size_t i = 0; i < rgba.size(); ++i)
            result[i] = halfFromFloat(rgba[i]);
        }
        break;
      }
      }
      glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    }

    Init init_;
    EMSCRIPTEN_WEBGL_CONTEXT_HANDLE context_ = 0;
    bool owns_context_ = false;
    GLuint vertex_array_ = 0;
    GLuint read_framebuffer_ = 0;
    GLuint draw_framebuffer_ = 0;
    std::vector<Texture> textures_;
    std::vector<Buffer> buffers_;
    std::vector<Shader> shaders_;
    std::vector<Program> programs_;
    std::vector<FrameBuffer> frame_buffers_;
    std::unordered_map<uint32_t, GLuint> samplers_;
  };

  std::unique_ptr<Backend> createGlesBackend() {
    return std::make_unique<GlesBackend>();
  }
}
