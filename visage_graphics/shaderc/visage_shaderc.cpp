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

#include "gpu/gpu_backend.h"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <glslang/Public/ResourceLimits.h>
#include <glslang/Public/ShaderLang.h>
#include <SPIRV/GlslangToSpv.h>
#include <iostream>
#include <map>
#include <set>
#include <spirv_glsl.hpp>
#include <spirv_hlsl.hpp>
#include <spirv_msl.hpp>
#include <sstream>

using namespace visage::gpu;

namespace {
  constexpr int kResourceBindingOffset = 16;
  constexpr int kMslVertexBufferIndex = 30;
  constexpr const char* kEsslVertexUniforms = "visage_vertex_uniforms";
  constexpr const char* kEsslFragmentUniforms = "visage_fragment_uniforms";

  struct Options {
    std::string input;
    std::string output;
    std::string varying_path;
    std::vector<std::string> include_dirs;
    std::vector<std::string> defines;
    ShaderStage stage = ShaderStage::Fragment;
    ShaderCode target = ShaderCode::Spirv;
    bool stage_set = false;
  };

  struct Varying {
    std::string interpolation;
    std::string type;
    std::string name;
    std::string semantic;
    int location = 0;
  };

  bool readFile(const std::filesystem::path& path, std::string& result) {
    std::ifstream file(path, std::ios::binary);
    if (!file)
      return false;
    std::stringstream stream;
    stream << file.rdbuf();
    result = stream.str();
    return true;
  }

  std::string trim(const std::string& value) {
    size_t start = value.find_first_not_of(" \t\r\n");
    if (start == std::string::npos)
      return "";
    size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(start, end - start + 1);
  }

  std::vector<std::string> splitNames(const std::string& value) {
    std::vector<std::string> names;
    std::string separated = value;
    std::replace(separated.begin(), separated.end(), ',', ' ');
    std::stringstream stream(separated);
    std::string name;
    while (stream >> name)
      names.push_back(name);
    return names;
  }

  std::vector<Varying> parseVaryings(const std::string& source) {
    static const std::set<std::string> kInterpolation = { "flat", "smooth", "noperspective", "centroid" };
    std::vector<Varying> varyings;
    std::stringstream stream(source);
    std::string line;
    while (std::getline(stream, line)) {
      size_t comment = line.find("//");
      if (comment != std::string::npos)
        line = line.substr(0, comment);
      size_t equals = line.find('=');
      if (equals != std::string::npos)
        line = line.substr(0, equals);
      size_t semicolon = line.find(';');
      if (semicolon != std::string::npos)
        line = line.substr(0, semicolon);

      std::string semantic;
      size_t colon = line.find(':');
      if (colon != std::string::npos) {
        semantic = trim(line.substr(colon + 1));
        line = line.substr(0, colon);
      }

      std::stringstream tokens(line);
      std::vector<std::string> words;
      std::string word;
      while (tokens >> word)
        words.push_back(word);
      if (words.size() < 2)
        continue;

      Varying varying;
      size_t index = 0;
      while (index + 2 < words.size() && kInterpolation.count(words[index]))
        varying.interpolation += words[index++] + " ";
      if (index + 1 >= words.size())
        continue;
      varying.type = words[index];
      varying.name = words[index + 1];
      varying.semantic = semantic;
      varyings.push_back(varying);
    }

    int location = 0;
    for (Varying& varying : varyings) {
      if (varying.name.rfind("a_", 0) != 0 && varying.name.rfind("i_", 0) != 0)
        varying.location = location++;
    }
    return varyings;
  }

  int attributeLocation(const std::string& semantic) {
    static const std::map<std::string, Attrib> kSemantics = {
      { "POSITION", Attrib::Position },   { "COLOR0", Attrib::Color0 },       { "COLOR1", Attrib::Color1 },
      { "COLOR2", Attrib::Color2 },       { "COLOR3", Attrib::Color3 },       { "TEXCOORD0", Attrib::TexCoord0 },
      { "TEXCOORD1", Attrib::TexCoord1 }, { "TEXCOORD2", Attrib::TexCoord2 }, { "TEXCOORD3", Attrib::TexCoord3 },
      { "TEXCOORD4", Attrib::TexCoord4 }, { "TEXCOORD5", Attrib::TexCoord5 }, { "TEXCOORD6", Attrib::TexCoord6 },
      { "TEXCOORD7", Attrib::TexCoord7 },
    };
    auto found = kSemantics.find(semantic);
    return found == kSemantics.end() ? -1 : static_cast<int>(found->second);
  }

  std::string stripDirectives(const std::string& source, std::vector<std::string>& inputs,
                              std::vector<std::string>& outputs) {
    std::stringstream stream(source);
    std::string line;
    std::string result;
    while (std::getline(stream, line)) {
      std::string trimmed = trim(line);
      if (trimmed.rfind("$input", 0) == 0) {
        std::vector<std::string> names = splitNames(trimmed.substr(6));
        inputs.insert(inputs.end(), names.begin(), names.end());
        result += "\n";
      }
      else if (trimmed.rfind("$output", 0) == 0) {
        std::vector<std::string> names = splitNames(trimmed.substr(7));
        outputs.insert(outputs.end(), names.begin(), names.end());
        result += "\n";
      }
      else
        result += line + "\n";
    }
    return result;
  }

  const Varying* findVarying(const std::vector<Varying>& varyings, const std::string& name) {
    for (const Varying& varying : varyings) {
      if (varying.name == name)
        return &varying;
    }
    return nullptr;
  }

  bool buildPrelude(const Options& options, const std::vector<Varying>& varyings,
                    const std::vector<std::string>& inputs, const std::vector<std::string>& outputs,
                    std::string& prelude, std::string& error) {
    std::stringstream stream;
    stream << "#extension GL_GOOGLE_include_directive : require\n";
    stream << "#define BGFX_SHADER_LANGUAGE_GLSL 450\n";
    stream << "#define VISAGE_SHADER_LANGUAGE_GLSL 450\n";
    if (options.stage == ShaderStage::Vertex)
      stream << "#define BGFX_SHADER_TYPE_VERTEX 1\n";
    else if (options.stage == ShaderStage::Fragment)
      stream << "#define BGFX_SHADER_TYPE_FRAGMENT 1\n";
    else
      stream << "#define BGFX_SHADER_TYPE_COMPUTE 1\n";

    for (const std::string& define : options.defines) {
      size_t equals = define.find('=');
      if (equals == std::string::npos)
        stream << "#define " << define << " 1\n";
      else
        stream << "#define " << define.substr(0, equals) << " " << define.substr(equals + 1) << "\n";
    }

    stream << "#define SAMPLER2D(_name, _reg) layout(binding = " << kResourceBindingOffset
           << " + _reg) uniform sampler2D _name\n";
    stream << "#define BUFFER_RO(_name, _type, _reg) layout(std430, binding = " << kResourceBindingOffset
           << " + _reg) readonly buffer _name##Buffer { _type _name[]; }\n";
    stream << "#define BUFFER_RW(_name, _type, _reg) layout(std430, binding = " << kResourceBindingOffset
           << " + _reg) buffer _name##Buffer { _type _name[]; }\n";
    stream << "#define BUFFER_WO(_name, _type, _reg) layout(std430, binding = " << kResourceBindingOffset
           << " + _reg) writeonly buffer _name##Buffer { _type _name[]; }\n";
    stream << "#define IMAGE2D_RO(_name, _format, _reg) layout(_format, binding = " << kResourceBindingOffset
           << " + _reg) readonly uniform highp image2D _name\n";
    stream << "#define IMAGE2D_RW(_name, _format, _reg) layout(_format, binding = " << kResourceBindingOffset
           << " + _reg) uniform highp image2D _name\n";
    stream << "#define IMAGE2D_WO(_name, _format, _reg) layout(_format, binding = " << kResourceBindingOffset
           << " + _reg) writeonly uniform highp image2D _name\n";
    stream << "#define NUM_THREADS(_x, _y, _z) layout(local_size_x = _x, local_size_y = _y, local_size_z = _z) in;\n";
    stream << "#define atomicFetchAndAdd(_mem, _data, _original) _original = atomicAdd(_mem, _data)\n";
    stream << "#define gl_VertexID gl_VertexIndex\n";
    stream << "#define gl_InstanceID gl_InstanceIndex\n";

    if (options.stage == ShaderStage::Vertex) {
      for (const std::string& name : inputs) {
        const Varying* varying = findVarying(varyings, name);
        if (varying == nullptr) {
          error = "Unknown vertex input " + name;
          return false;
        }
        int location = attributeLocation(varying->semantic);
        if (location < 0) {
          error = "Unsupported attribute semantic " + varying->semantic + " for " + name;
          return false;
        }
        stream << "layout(location = " << location << ") in " << varying->type << " " << name << ";\n";
      }
      for (const std::string& name : outputs) {
        const Varying* varying = findVarying(varyings, name);
        if (varying == nullptr) {
          error = "Unknown vertex output " + name;
          return false;
        }
        stream << "layout(location = " << varying->location << ") " << varying->interpolation << "out "
               << varying->type << " " << name << ";\n";
      }
    }
    else if (options.stage == ShaderStage::Fragment) {
      for (const std::string& name : inputs) {
        const Varying* varying = findVarying(varyings, name);
        if (varying == nullptr) {
          error = "Unknown fragment input " + name;
          return false;
        }
        stream << "layout(location = " << varying->location << ") " << varying->interpolation << "in "
               << varying->type << " " << name << ";\n";
      }
      stream << "layout(location = 0) out vec4 bgfx_FragColor;\n";
      stream << "#define gl_FragColor bgfx_FragColor\n";
    }

    prelude = stream.str();
    return true;
  }

  class Includer : public glslang::TShader::Includer {
  public:
    explicit Includer(std::vector<std::filesystem::path> directories) :
        directories_(std::move(directories)) { }

    IncludeResult* includeSystem(const char* header_name, const char* includer_name, size_t depth) override {
      return find(header_name, includer_name, false);
    }

    IncludeResult* includeLocal(const char* header_name, const char* includer_name, size_t depth) override {
      return find(header_name, includer_name, true);
    }

    void releaseInclude(IncludeResult* result) override {
      if (result == nullptr)
        return;
      delete static_cast<std::string*>(result->userData);
      delete result;
    }

  private:
    IncludeResult* find(const char* header_name, const char* includer_name, bool local) {
      std::vector<std::filesystem::path> search;
      if (local && includer_name && includer_name[0])
        search.push_back(std::filesystem::path(includer_name).parent_path());
      search.insert(search.end(), directories_.begin(), directories_.end());

      for (const auto& directory : search) {
        std::filesystem::path path = directory / header_name;
        auto contents = std::make_unique<std::string>();
        if (readFile(path, *contents)) {
          std::string* data = contents.release();
          return new IncludeResult(path.string(), data->data(), data->size(), data);
        }
      }
      return nullptr;
    }

    std::vector<std::filesystem::path> directories_;
  };

  bool compileSpirv(const Options& options, const std::string& prelude, const std::string& source,
                    std::vector<uint32_t>& spirv, std::string& error) {
    EShLanguage language = EShLangFragment;
    if (options.stage == ShaderStage::Vertex)
      language = EShLangVertex;
    else if (options.stage == ShaderStage::Compute)
      language = EShLangCompute;

    glslang::TShader shader(language);
    const char* sources[] = { source.c_str() };
    const char* names[] = { options.input.c_str() };
    shader.setStringsWithLengthsAndNames(sources, nullptr, names, 1);
    shader.setPreamble(prelude.c_str());
    shader.setEnvInput(glslang::EShSourceGlsl, language, glslang::EShClientVulkan, 100);
    shader.setEnvClient(glslang::EShClientVulkan, glslang::EShTargetVulkan_1_1);
    shader.setEnvTarget(glslang::EShTargetSpv, glslang::EShTargetSpv_1_3);
    shader.setEnvInputVulkanRulesRelaxed();
    shader.setGlobalUniformBlockName("Uniforms");
    shader.setGlobalUniformSet(0);
    shader.setGlobalUniformBinding(options.stage == ShaderStage::Fragment ? 1 : 0);
    shader.setAutoMapLocations(false);

    std::vector<std::filesystem::path> directories;
    directories.push_back(std::filesystem::path(options.input).parent_path());
    for (const std::string& directory : options.include_dirs)
      directories.emplace_back(directory);
    Includer includer(directories);

    auto messages = static_cast<EShMessages>(EShMsgSpvRules | EShMsgVulkanRules);
    if (!shader.parse(GetDefaultResources(), 450, ECoreProfile, false, false, messages, includer)) {
      error = shader.getInfoLog();
      return false;
    }

    glslang::TProgram program;
    program.addShader(&shader);
    if (!program.link(messages)) {
      error = program.getInfoLog();
      return false;
    }

    glslang::SpvOptions spv_options;
    spv_options.generateDebugInfo = false;
    spv_options.disableOptimizer = true;
    spv_options.validate = false;
    glslang::GlslangToSpv(*program.getIntermediate(language), spirv, &spv_options);
    return !spirv.empty();
  }

  bool reflect(const std::vector<uint32_t>& spirv, ShaderInfo& info, std::string& error) {
    spirv_cross::Compiler compiler(spirv);
    spirv_cross::ShaderResources resources = compiler.get_shader_resources();

    for (const auto& buffer : resources.uniform_buffers) {
      const spirv_cross::SPIRType& type = compiler.get_type(buffer.base_type_id);
      info.uniform_block_size = static_cast<uint32_t>(compiler.get_declared_struct_size(type));
      for (uint32_t i = 0; i < type.member_types.size(); ++i) {
        const spirv_cross::SPIRType& member = compiler.get_type(type.member_types[i]);
        ShaderUniform uniform;
        uniform.name = compiler.get_member_name(buffer.base_type_id, i);
        uniform.offset = static_cast<uint16_t>(compiler.type_struct_member_offset(type, i));
        uniform.num = member.array.empty() ? 1 : static_cast<uint16_t>(member.array[0]);
        if (member.columns == 4 && member.vecsize == 4)
          uniform.type = UniformType::Mat4;
        else if (member.columns == 3 && member.vecsize == 3)
          uniform.type = UniformType::Mat3;
        else if (member.columns == 1 && member.vecsize == 4 && member.basetype == spirv_cross::SPIRType::Float)
          uniform.type = UniformType::Vec4;
        else {
          error = "Uniform " + uniform.name + " must be a vec4, mat3 or mat4";
          return false;
        }
        info.uniforms.push_back(uniform);
      }
    }
    if (resources.uniform_buffers.size() > 1) {
      error = "Only one uniform block is supported";
      return false;
    }

    auto stage_for = [&](spirv_cross::ID id, int limit, uint8_t& stage) {
      int binding = static_cast<int>(compiler.get_decoration(id, spv::DecorationBinding)) -
                    kResourceBindingOffset;
      if (binding < 0 || binding >= limit) {
        error = "Resource " + compiler.get_name(id) + " has an invalid register";
        return false;
      }
      stage = static_cast<uint8_t>(binding);
      return true;
    };

    for (const auto& sampler : resources.sampled_images) {
      ShaderSampler result;
      result.name = compiler.get_name(sampler.id);
      if (!stage_for(sampler.id, kMaxTextureStages, result.stage))
        return false;
      info.samplers.push_back(result);
    }

    for (const auto& buffer : resources.storage_buffers) {
      ShaderStorage storage;
      storage.name = compiler.get_name(buffer.id);
      if (storage.name.empty())
        storage.name = compiler.get_name(buffer.base_type_id);
      if (!stage_for(buffer.id, kMaxComputeBindings, storage.stage))
        return false;
      spirv_cross::Bitset flags = compiler.get_buffer_block_flags(buffer.id);
      bool readable = !flags.get(spv::DecorationNonReadable);
      bool writable = !flags.get(spv::DecorationNonWritable);
      storage.access = readable && writable ? Access::ReadWrite : (writable ? Access::Write : Access::Read);
      info.storage.push_back(storage);
    }

    for (const auto& image : resources.storage_images) {
      ShaderStorage storage;
      storage.name = compiler.get_name(image.id);
      storage.image = true;
      if (!stage_for(image.id, kMaxComputeBindings, storage.stage))
        return false;
      bool readable = !compiler.has_decoration(image.id, spv::DecorationNonReadable);
      bool writable = !compiler.has_decoration(image.id, spv::DecorationNonWritable);
      storage.access = readable && writable ? Access::ReadWrite : (writable ? Access::Write : Access::Read);
      info.storage.push_back(storage);
    }

    if (info.stage == ShaderStage::Vertex) {
      for (const auto& input : resources.stage_inputs) {
        ShaderAttribute attribute;
        uint32_t location = compiler.get_decoration(input.id, spv::DecorationLocation);
        if (location >= static_cast<uint32_t>(Attrib::Count)) {
          error = "Vertex input " + input.name + " has an invalid location";
          return false;
        }
        attribute.attrib = static_cast<Attrib>(location);
        attribute.num_floats = static_cast<uint8_t>(compiler.get_type(input.type_id).vecsize);
        info.attributes.push_back(attribute);
      }
    }

    if (info.stage == ShaderStage::Compute) {
      for (int i = 0; i < 3; ++i)
        info.workgroup_size[i] = static_cast<uint16_t>(
            compiler.get_execution_mode_argument(spv::ExecutionModeLocalSize, i));
    }
    return true;
  }

  std::string compileMsl(const std::vector<uint32_t>& spirv, ShaderStage stage) {
    spirv_cross::CompilerMSL compiler(spirv);
    spirv_cross::CompilerMSL::Options options;
    options.platform = spirv_cross::CompilerMSL::Options::macOS;
    options.set_msl_version(2, 1);
    options.enable_decoration_binding = false;
    compiler.set_msl_options(options);

    spv::ExecutionModel model = spv::ExecutionModelFragment;
    if (stage == ShaderStage::Vertex)
      model = spv::ExecutionModelVertex;
    else if (stage == ShaderStage::Compute)
      model = spv::ExecutionModelGLCompute;

    spirv_cross::ShaderResources resources = compiler.get_shader_resources();
    for (const auto& buffer : resources.uniform_buffers) {
      spirv_cross::MSLResourceBinding binding;
      binding.stage = model;
      binding.desc_set = 0;
      binding.binding = compiler.get_decoration(buffer.id, spv::DecorationBinding);
      binding.msl_buffer = 0;
      compiler.add_msl_resource_binding(binding);
    }
    for (const auto& sampler : resources.sampled_images) {
      uint32_t register_index = compiler.get_decoration(sampler.id, spv::DecorationBinding);
      spirv_cross::MSLResourceBinding binding;
      binding.stage = model;
      binding.desc_set = 0;
      binding.binding = register_index;
      binding.msl_texture = register_index - kResourceBindingOffset;
      binding.msl_sampler = register_index - kResourceBindingOffset;
      compiler.add_msl_resource_binding(binding);
    }
    for (const auto& buffer : resources.storage_buffers) {
      uint32_t register_index = compiler.get_decoration(buffer.id, spv::DecorationBinding);
      spirv_cross::MSLResourceBinding binding;
      binding.stage = model;
      binding.desc_set = 0;
      binding.binding = register_index;
      binding.msl_buffer = 1 + register_index - kResourceBindingOffset;
      compiler.add_msl_resource_binding(binding);
    }
    for (const auto& image : resources.storage_images) {
      uint32_t register_index = compiler.get_decoration(image.id, spv::DecorationBinding);
      spirv_cross::MSLResourceBinding binding;
      binding.stage = model;
      binding.desc_set = 0;
      binding.binding = register_index;
      binding.msl_texture = register_index - kResourceBindingOffset;
      compiler.add_msl_resource_binding(binding);
    }
    if (stage == ShaderStage::Vertex) {
      spirv_cross::MSLShaderInterfaceVariable input;
      for (const auto& stage_input : resources.stage_inputs) {
        input.location = compiler.get_decoration(stage_input.id, spv::DecorationLocation);
        compiler.add_msl_shader_input(input);
      }
    }
    return compiler.compile();
  }

  std::string compileHlsl(const std::vector<uint32_t>& spirv, ShaderStage stage) {
    spirv_cross::CompilerHLSL compiler(spirv);
    spirv_cross::CompilerHLSL::Options options;
    options.shader_model = 50;
    compiler.set_hlsl_options(options);

    spv::ExecutionModel model = spv::ExecutionModelFragment;
    if (stage == ShaderStage::Vertex)
      model = spv::ExecutionModelVertex;
    else if (stage == ShaderStage::Compute)
      model = spv::ExecutionModelGLCompute;

    spirv_cross::ShaderResources resources = compiler.get_shader_resources();
    for (const auto& buffer : resources.uniform_buffers) {
      spirv_cross::HLSLResourceBinding binding;
      binding.stage = model;
      binding.desc_set = 0;
      binding.binding = compiler.get_decoration(buffer.id, spv::DecorationBinding);
      binding.cbv.register_binding = 0;
      compiler.add_hlsl_resource_binding(binding);
    }
    for (const auto& sampler : resources.sampled_images) {
      uint32_t register_index = compiler.get_decoration(sampler.id, spv::DecorationBinding);
      spirv_cross::HLSLResourceBinding binding;
      binding.stage = model;
      binding.desc_set = 0;
      binding.binding = register_index;
      binding.srv.register_binding = register_index - kResourceBindingOffset;
      binding.sampler.register_binding = register_index - kResourceBindingOffset;
      compiler.add_hlsl_resource_binding(binding);
    }
    for (const auto& buffer : resources.storage_buffers) {
      uint32_t register_index = compiler.get_decoration(buffer.id, spv::DecorationBinding);
      spirv_cross::HLSLResourceBinding binding;
      binding.stage = model;
      binding.desc_set = 0;
      binding.binding = register_index;
      binding.srv.register_binding = register_index - kResourceBindingOffset;
      binding.uav.register_binding = register_index - kResourceBindingOffset;
      compiler.add_hlsl_resource_binding(binding);
    }
    for (const auto& image : resources.storage_images) {
      uint32_t register_index = compiler.get_decoration(image.id, spv::DecorationBinding);
      spirv_cross::HLSLResourceBinding binding;
      binding.stage = model;
      binding.desc_set = 0;
      binding.binding = register_index;
      binding.uav.register_binding = register_index - kResourceBindingOffset;
      compiler.add_hlsl_resource_binding(binding);
    }
    return compiler.compile();
  }

  std::string compileEssl(const std::vector<uint32_t>& spirv, ShaderStage stage) {
    spirv_cross::CompilerGLSL compiler(spirv);
    spirv_cross::CompilerGLSL::Options options;
    options.version = 300;
    options.es = true;
    options.fragment.default_float_precision = spirv_cross::CompilerGLSL::Options::Highp;
    options.fragment.default_int_precision = spirv_cross::CompilerGLSL::Options::Highp;
    options.enable_420pack_extension = false;
    compiler.set_common_options(options);

    spirv_cross::ShaderResources resources = compiler.get_shader_resources();
    for (const auto& sampler : resources.sampled_images) {
      compiler.unset_decoration(sampler.id, spv::DecorationBinding);
      compiler.unset_decoration(sampler.id, spv::DecorationDescriptorSet);
    }
    for (const auto& buffer : resources.uniform_buffers) {
      compiler.unset_decoration(buffer.id, spv::DecorationBinding);
      compiler.unset_decoration(buffer.id, spv::DecorationDescriptorSet);
      const char* name = stage == ShaderStage::Vertex ? kEsslVertexUniforms : kEsslFragmentUniforms;
      compiler.set_name(buffer.id, name);
      compiler.set_name(buffer.base_type_id, name);
      compiler.flatten_buffer_block(buffer.id);
    }
    return compiler.compile();
  }

  bool parseArguments(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };

      if (arg == "-f")
        options.input = next();
      else if (arg == "-o")
        options.output = next();
      else if (arg == "-i")
        options.include_dirs.push_back(next());
      else if (arg == "--varyingdef")
        options.varying_path = next();
      else if (arg == "--define")
        options.defines.push_back(next());
      else if (arg == "--type") {
        std::string type = next();
        options.stage_set = true;
        if (type == "v" || type == "vertex")
          options.stage = ShaderStage::Vertex;
        else if (type == "c" || type == "compute")
          options.stage = ShaderStage::Compute;
        else
          options.stage = ShaderStage::Fragment;
      }
      else if (arg == "--target" || arg == "-p" || arg == "--profile") {
        std::string target = next();
        if (target == "spirv")
          options.target = ShaderCode::Spirv;
        else if (target == "msl" || target == "metal")
          options.target = ShaderCode::Msl;
        else if (target == "hlsl" || target.rfind("s_", 0) == 0)
          options.target = ShaderCode::Hlsl;
        else if (target == "dxbc")
          options.target = ShaderCode::Dxbc;
        else if (target == "essl" || target == "300_es")
          options.target = ShaderCode::Essl;
        else {
          std::cerr << "Unknown target " << target << std::endl;
          return false;
        }
      }
      else if (arg == "--platform" || arg == "-O")
        next();
      else {
        std::cerr << "Unknown argument " << arg << std::endl;
        return false;
      }
    }

    if (options.input.empty() || options.output.empty()) {
      std::cerr << "Usage: visage_shaderc -f input.sc -o output --type v|f|c --target spirv|msl|hlsl|dxbc|essl"
                   " [-i include_dir] [--varyingdef varying.def.sc] [--define NAME=VALUE]"
                << std::endl;
      return false;
    }

    if (!options.stage_set) {
      std::string name = std::filesystem::path(options.input).filename().string();
      if (name.rfind("vs_", 0) == 0)
        options.stage = ShaderStage::Vertex;
      else if (name.rfind("cs_", 0) == 0)
        options.stage = ShaderStage::Compute;
    }
    if (options.varying_path.empty())
      options.varying_path = (std::filesystem::path(options.input).parent_path() / "varying.def.sc").string();
    return true;
  }
}

#if _WIN32
bool compileDxbc(const std::string& hlsl, ShaderStage stage, std::vector<uint8_t>& result, std::string& error);
#endif

int main(int argc, char** argv) {
  Options options;
  if (!parseArguments(argc, argv, options))
    return 1;

  std::string source;
  if (!readFile(options.input, source)) {
    std::cerr << "Could not read " << options.input << std::endl;
    return 1;
  }

  std::vector<Varying> varyings;
  std::string varying_source;
  if (readFile(options.varying_path, varying_source))
    varyings = parseVaryings(varying_source);

  std::vector<std::string> inputs, outputs;
  std::string body = stripDirectives(source, inputs, outputs);

  std::string prelude;
  std::string error;
  if (!buildPrelude(options, varyings, inputs, outputs, prelude, error)) {
    std::cerr << options.input << ": " << error << std::endl;
    return 1;
  }

  glslang::InitializeProcess();
  std::vector<uint32_t> spirv;
  bool compiled = compileSpirv(options, prelude, body, spirv, error);
  glslang::FinalizeProcess();
  if (!compiled) {
    std::cerr << options.input << ":\n" << error << std::endl;
    return 1;
  }

  ShaderInfo info;
  info.stage = options.stage;
  info.code_type = options.target;
  if (!reflect(spirv, info, error)) {
    std::cerr << options.input << ": " << error << std::endl;
    return 1;
  }

  try {
    std::string code;
    switch (options.target) {
    case ShaderCode::Spirv:
      info.code.resize(spirv.size() * sizeof(uint32_t));
      std::memcpy(info.code.data(), spirv.data(), info.code.size());
      break;
    case ShaderCode::Msl: code = compileMsl(spirv, options.stage); break;
    case ShaderCode::Hlsl: code = compileHlsl(spirv, options.stage); break;
    case ShaderCode::Essl:
      if (options.stage == ShaderStage::Compute) {
        std::cerr << options.input << ": compute shaders are not supported on ESSL" << std::endl;
        return 1;
      }
      code = compileEssl(spirv, options.stage);
      break;
    case ShaderCode::Dxbc: {
#if _WIN32
      if (!compileDxbc(compileHlsl(spirv, options.stage), options.stage, info.code, error)) {
        std::cerr << options.input << ":\n" << error << std::endl;
        return 1;
      }
#else
      std::cerr << "DXBC output needs a Windows host" << std::endl;
      return 1;
#endif
      break;
    }
    }
    if (!code.empty())
      info.code.assign(code.begin(), code.end());
  }
  catch (const std::exception& exception) {
    std::cerr << options.input << ": " << exception.what() << std::endl;
    return 1;
  }

  std::vector<uint8_t> blob = writeShader(info);
  std::filesystem::path output(options.output);
  if (output.has_parent_path())
    std::filesystem::create_directories(output.parent_path());
  std::ofstream file(output, std::ios::binary);
  file.write(reinterpret_cast<const char*>(blob.data()), static_cast<std::streamsize>(blob.size()));
  return file ? 0 : 1;
}
