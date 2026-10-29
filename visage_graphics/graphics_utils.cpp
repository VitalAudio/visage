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

#include "graphics_utils.h"

#include "gpu/gpu.h"

#include <cctype>
#include <locale>
#include <sstream>
#include <vector>

#define STB_RECT_PACK_IMPLEMENTATION
#include <stb/stb_rect_pack.h>

#if VISAGE_EMSCRIPTEN
#include "gpu/gpu_backend.h"

#include <emscripten/emscripten.h>
#include <GLES2/gl2.h>
#include <regex>
#include <visage_utils/string_utils.h>

namespace visage {
  static constexpr const char* kWebGlPrelude =
      "precision mediump float;\n"
      "#define texture2DLod    textureLod\n"
      "#define texture3DLod    textureLod\n"
      "#define textureCubeLod  textureLod\n"
      "#define texture2DGrad   textureGrad\n"
      "#define texture3DGrad   textureGrad\n"
      "#define textureCubeGrad textureGrad\n"
      "#define varying        in\n"
      "#define texture2D      texture\n"
      "#define texture2DArray texture\n"
      "#define texture2DProj  textureProj\n"
      "#define shadow2D(_sampler, _coord) (textureProj(_sampler, vec4(_coord, 1.0) ) ) \n"
      "#define shadow2DProj(_sampler, _coord) (textureProj(_sampler, _coord) )\n"
      "#define texture3D   texture\n"
      "#define textureCube texture\n"
      "out vec4 bgfx_FragColor;\n"
      "#define gl_FragColor bgfx_FragColor\n"
      "vec2 vec2_splat(float _x) { return vec2(_x, _x); }\n"
      "vec3 vec3_splat(float _x) { return vec3(_x, _x, _x); }\n"
      "vec4 vec4_splat(float _x) { return vec4(_x, _x, _x, _x); }\n";

  static std::map<std::string, std::string> parseVarying(const std::string& input) {
    std::map<std::string, std::string> varying;
    std::istringstream file_stream(input);
    std::string line;

    while (std::getline(file_stream, line)) {
      std::vector<std::string> tokens;
      std::istringstream line_stream(line);
      std::string token;

      while (line_stream >> token)
        tokens.push_back(token);

      if (tokens.size() >= 2)
        varying[tokens[1]] = tokens[0];
    }

    return varying;
  }

  static void parseUniforms(std::string& input, gpu::ShaderInfo& info) {
    std::istringstream stream(input);
    std::string line;
    std::string result;

    while (std::getline(stream, line)) {
      if (line.rfind("uniform ", 0)) {
        result += line + "\n";
        continue;
      }

      std::string content = line.substr(8);
      std::istringstream line_stream(content);
      std::string type, name;
      std::string precision = "highp";
      line_stream >> type;
      if (type == "highp" || type == "mediump" || type == "lowp") {
        precision = type;
        line_stream >> type;
      }

      line_stream >> name;
      size_t end_pos = name.find_first_of(';');
      if (end_pos != std::string::npos)
        name = name.erase(end_pos);

      gpu::ShaderUniform uniform;
      uniform.name = name;
      uniform.offset = static_cast<uint16_t>(info.uniform_block_size);
      if (type == "vec4" || type == "float4") {
        uniform.type = gpu::UniformType::Vec4;
        info.uniform_block_size += 16;
        info.uniforms.push_back(uniform);
      }
      else if (type == "mat3" || type == "float3x3") {
        uniform.type = gpu::UniformType::Mat3;
        info.uniform_block_size += 48;
        info.uniforms.push_back(uniform);
      }
      else if (type == "mat4" || type == "float4x4") {
        uniform.type = gpu::UniformType::Mat4;
        info.uniform_block_size += 64;
        info.uniforms.push_back(uniform);
      }

      result.append("uniform ");
      result.append(precision);
      result.append(" ");
      result.append(type);
      result.append(" " + name + ";\n");
    }

    input = result;
  }

  static void parseSamplers(const std::string& input, gpu::ShaderInfo& info) {
    std::regex sampler_regex(R"(\bSAMPLER2D\(\s*([A-Za-z_][A-Za-z0-9_]*)\s*,\s*([0-9]+)\s*\))");
    for (auto it = std::sregex_iterator(input.begin(), input.end(), sampler_regex); it != std::sregex_iterator(); ++it) {
      gpu::ShaderSampler sampler;
      sampler.name = (*it)[1].str();
      sampler.stage = static_cast<uint8_t>(std::stoi((*it)[2].str()));
      info.samplers.push_back(sampler);
    }
  }

  static std::vector<std::string> parseInputs(const std::string& input) {
    std::vector<std::string> names;
    std::istringstream stream(input);
    std::string line;

    while (std::getline(stream, line)) {
      if (line.rfind("$input", 0) == 0) {
        std::string content = line.substr(7);
        std::istringstream line_stream(content);
        std::string name;

        while (std::getline(line_stream, name, ',')) {
          name.erase(0, name.find_first_not_of(' '));
          name.erase(name.find_last_not_of(' ') + 1);
          names.push_back(name);
        }
        break;
      }
    }

    return names;
  }

  static bool testCompileShader(const std::string& code, std::string& error) {
    std::string full_shader = "#version 300 es\n" + code;
    error = "";
    GLuint shader_id = glCreateShader(GL_FRAGMENT_SHADER);
    const GLchar* str = (const GLchar*)full_shader.c_str();
    int32_t length = full_shader.length();
    glShaderSource(shader_id, 1, &str, &length);

    glCompileShader(shader_id);

    GLint compiled = 0;
    glGetShaderiv(shader_id, GL_COMPILE_STATUS, &compiled);

    if (!compiled) {
      GLsizei len;
      char log[1024];
      glGetShaderInfoLog(shader_id, sizeof(log), &len, log);
      error = log;
    }

    glDeleteShader(shader_id);
    return compiled;
  }

  bool preprocessWebGlShader(std::string& result, const std::string& code,
                             const std::string& utils_source, const std::string& varying_source) {
    std::vector<std::string> inputs = parseInputs(code);
    std::sort(inputs.begin(), inputs.end());

    String trimmed_code = code;
    trimmed_code = trimmed_code.trim();
    while (!trimmed_code.isEmpty() && trimmed_code[0] == '$')
      trimmed_code = trimmed_code.substring(trimmed_code.find('\n')).trim();
    result = trimmed_code.toUtf8();

    std::string utils_include = "#include <shader_include.sh>";
    size_t utils_pos = result.find(utils_include);
    if (utils_pos != std::string::npos)
      result.replace(utils_pos, utils_include.length(), utils_source);

    result = std::regex_replace(result, std::regex(R"(\bkPi\b)"), "3.1415926535897932");
    result = std::regex_replace(result, std::regex(R"(\bmul\(([^,]+),([^)]+)\))"), R"((($1) * ($2)))");
    result = std::regex_replace(result, std::regex(R"(\bsaturate\(([^,]+)\))"), R"(clamp(($1), 0.0, 1.0))");
    result = std::regex_replace(result, std::regex(R"(\batan2\(([^,]+),([^)]+)\))"), R"(atan(($1),($2)))");
    gpu::ShaderInfo info;
    info.stage = gpu::ShaderStage::Fragment;
    info.code_type = gpu::ShaderCode::Essl;
    parseSamplers(result, info);
    result = std::regex_replace(result, std::regex(R"(\bSAMPLER2D\(([^,]+),([^)]+)\))"),
                                R"(uniform sampler2D $1)");
    parseUniforms(result, info);

    std::map<std::string, std::string> varying = parseVarying(varying_source);
    std::string varying_list;
    for (auto& input : inputs) {
      if (varying.count(input))
        varying_list += "varying highp " + varying[input] + " " + input + ";\n";
    }
    result = kWebGlPrelude + varying_list + result;

    std::string error;
    if (testCompileShader(result, error)) {
      std::string code = "#version 300 es\n" + result;
      info.code.assign(code.begin(), code.end());
      std::vector<uint8_t> blob = gpu::writeShader(info);
      result.assign(blob.begin(), blob.end());
      return true;
    }

    result = error;
    return false;
  }
}
#else
namespace visage {
  bool preprocessWebGlShader(std::string& result, const std::string& code,
                             const std::string& utils_source, const std::string& varying_source) {
    return false;
  }
}
#endif

namespace visage {
  bool readFloat(const std::string& string, float& result, size_t* number_end) {
    if (number_end)
      *number_end = 0;

    thread_local std::istringstream stream = [] {
      std::istringstream classic_stream;
      classic_stream.imbue(std::locale::classic());
      return classic_stream;
    }();

    size_t start = string.find_first_not_of(" \t\n\r\f\v");
    if (start == std::string::npos)
      return false;

    auto is_digit = [&](size_t i) {
      return i < string.size() && std::isdigit(static_cast<unsigned char>(string[i]));
    };
    size_t end = start;
    if (string[end] == '+' || string[end] == '-')
      end++;
    size_t digits_start = end;
    while (is_digit(end))
      end++;
    size_t num_digits = end - digits_start;
    if (end < string.size() && string[end] == '.') {
      size_t fraction_start = ++end;
      while (is_digit(end))
        end++;
      num_digits += end - fraction_start;
    }
    if (num_digits == 0)
      return false;

    if (end < string.size() && (string[end] == 'e' || string[end] == 'E')) {
      size_t exponent = end + 1;
      if (exponent < string.size() && (string[exponent] == '+' || string[exponent] == '-'))
        exponent++;
      if (is_digit(exponent)) {
        end = exponent;
        while (is_digit(end))
          end++;
      }
    }

    if (number_end)
      *number_end = end;

    stream.clear();
    stream.str(string.substr(start, end - start));
    float value = 0.0f;
    if (!(stream >> value))
      return false;

    result = value;
    return true;
  }

  struct PackedAtlasData {
    stbrp_context pack_context {};
    std::unique_ptr<stbrp_node[]> pack_nodes;
  };

  AtlasPacker::AtlasPacker() : data_(std::make_unique<PackedAtlasData>()) { }

  AtlasPacker::~AtlasPacker() = default;

  bool AtlasPacker::addRect(PackedRect& rect) {
    if (!packed_)
      return false;

    stbrp_rect r;
    r.w = rect.w + padding_;
    r.h = rect.h + padding_;
    r.id = rect_index_++;

    packed_ = stbrp_pack_rects(&data_->pack_context, &r, 1);
    if (packed_) {
      rect.x = r.x;
      rect.y = r.y;
    }
    return packed_;
  }

  void AtlasPacker::clear() {
    packed_ = false;
  }

  int maxTextureDimension() {
    static constexpr int kDefaultMaxDimension = 1 << 14;
    int max_texture_size = gpu::getCaps()->max_texture_size;
    return max_texture_size ? std::min(kDefaultMaxDimension, max_texture_size) : kDefaultMaxDimension;
  }

  bool AtlasPacker::pack(std::vector<PackedRect>& rects, int width, int height) {
    data_->pack_nodes = std::make_unique<stbrp_node[]>(width);
    std::vector<stbrp_rect> packed_rects;
    rect_index_ = 0;
    for (const PackedRect& rect : rects) {
      stbrp_rect r;
      r.w = rect.w + padding_;
      r.h = rect.h + padding_;
      r.id = rect_index_++;
      packed_rects.push_back(r);
    }

    stbrp_init_target(&data_->pack_context, width, height, data_->pack_nodes.get(), width);
    packed_ = stbrp_pack_rects(&data_->pack_context, packed_rects.data(), packed_rects.size());
    if (packed_) {
      for (int i = 0; i < packed_rects.size(); ++i) {
        rects[i].x = packed_rects[i].x;
        rects[i].y = packed_rects[i].y;
      }
    }
    return packed_;
  }

  gpu::VertexLayout& UvVertex::layout() {
    static gpu::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin().add(gpu::Attrib::Position, 4).end();
    }

    return layout;
  }

  gpu::VertexLayout& ShapeVertex::layout() {
    static gpu::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(gpu::Attrib::Position, 4)
          .add(gpu::Attrib::Color0, 4)
          .add(gpu::Attrib::Color1, 4)
          .add(gpu::Attrib::Color2, 4)
          .add(gpu::Attrib::TexCoord0, 4)
          .add(gpu::Attrib::TexCoord1, 4)
          .add(gpu::Attrib::TexCoord2, 4)
          .end();
    }

    return layout;
  }

  gpu::VertexLayout& ComplexShapeVertex::layout() {
    static gpu::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(gpu::Attrib::Position, 4)
          .add(gpu::Attrib::Color0, 4)
          .add(gpu::Attrib::Color1, 4)
          .add(gpu::Attrib::Color2, 4)
          .add(gpu::Attrib::TexCoord0, 4)
          .add(gpu::Attrib::TexCoord1, 4)
          .add(gpu::Attrib::TexCoord2, 4)
          .add(gpu::Attrib::TexCoord3, 4)
          .end();
    }

    return layout;
  }

  gpu::VertexLayout& TextureVertex::layout() {
    static gpu::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(gpu::Attrib::Position, 4)
          .add(gpu::Attrib::Color0, 4)
          .add(gpu::Attrib::Color1, 4)
          .add(gpu::Attrib::Color2, 4)
          .add(gpu::Attrib::TexCoord0, 4)
          .add(gpu::Attrib::TexCoord1, 4)
          .end();
    }

    return layout;
  }

  gpu::VertexLayout& PostEffectVertex::layout() {
    static gpu::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(gpu::Attrib::Position, 4)
          .add(gpu::Attrib::Color0, 4)
          .add(gpu::Attrib::Color1, 4)
          .add(gpu::Attrib::Color2, 4)
          .add(gpu::Attrib::TexCoord0, 4)
          .add(gpu::Attrib::TexCoord1, 4)
          .end();
    }

    return layout;
  }
}