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

#include <bgfx/bgfx.h>
#include <vector>

#define STB_RECT_PACK_IMPLEMENTATION
#include <stb/stb_rect_pack.h>

#if VISAGE_EMSCRIPTEN
#include <bx/hash.h>
#include <emscripten/emscripten.h>
#include <GLES2/gl2.h>
#include <regex>
#include <visage_utils/string_utils.h>

namespace visage {
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

  static std::string parseUniforms(std::string& input) {
    std::string uniform_header;
    std::istringstream stream(input);
    std::string line;
    std::string result;

    int count = 0;
    while (std::getline(stream, line)) {
      if (line.rfind("uniform ", 0))
        result += line + "\n";
      else {
        ++count;
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

        uniform_header += (unsigned char)name.size();
        uniform_header += name;

        if (type == "vec4" || type == "float4")
          uniform_header += (unsigned char)2;
        else if (type == "mat3" || type == "float3x3")
          uniform_header += (unsigned char)3;
        else if (type == "mat4" || type == "float4x4")
          uniform_header += (unsigned char)4;
        else
          uniform_header += (unsigned char)0;

        uniform_header += (unsigned char)1;
        uniform_header += (unsigned char)0;
        uniform_header += (unsigned char)0;
        uniform_header += (unsigned char)1;
        uniform_header += (unsigned char)0;
        uniform_header += (unsigned char)0;
        uniform_header += (unsigned char)0;

        result.append("uniform ");
        result.append(precision);
        result.append(" ");
        result.append(type);
        result.append(" " + name + ";\n");
      }
    }

    input = result;
    std::string size;
    size += char(count);
    size += char(0);
    return size + uniform_header;
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
    std::string head =
        "#version 300 es\n"
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
        "#define gl_FragColor bgfx_FragColor\n";

    std::string full_shader = head + code;
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
    result = std::regex_replace(result, std::regex(R"(\bSAMPLER2D\(([^,]+),([^)]+)\))"),
                                R"(uniform sampler2D $1)");

    auto write_int = [](std::string& str, int integer) {
      str += static_cast<char>(integer & 0xFF);
      str += static_cast<char>((integer >> 8) & 0xFF);
      str += static_cast<char>((integer >> 16) & 0xFF);
      str += static_cast<char>((integer >> 24) & 0xFF);
    };

    std::string header = "FSH";
    header += char(9);

    bx::HashMurmur2A murmur {};
    murmur.begin();
    for (auto& input : inputs)
      murmur.add(input.c_str(), (uint32_t)input.size());

    write_int(header, murmur.end());
    write_int(header, 0);
    header += parseUniforms(result);

    std::map<std::string, std::string> varying = parseVarying(varying_source);
    std::string varying_list;
    for (auto& input : inputs) {
      if (varying.count(input))
        varying_list += "varying highp " + varying[input] + " " + input + ";\n";
    }
    result = varying_list + result;

    write_int(header, result.length());
    std::string error;
    if (testCompileShader(result, error)) {
      result = header + result;
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

  bgfx::VertexLayout& UvVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin().add(bgfx::Attrib::Position, 4, bgfx::AttribType::Float).end();
    }

    return layout;
  }

  bgfx::VertexLayout& PathVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 4, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }

  bgfx::VertexLayout& ShapeVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color1, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color2, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord1, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord2, 4, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }

  bgfx::VertexLayout& ComplexShapeVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color1, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color2, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord1, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord2, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord3, 4, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }

  bgfx::VertexLayout& TextureVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color1, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color2, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord1, 4, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }

  bgfx::VertexLayout& PostEffectVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color1, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color2, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord1, 4, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }
}