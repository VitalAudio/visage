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

#if VISAGE_LINUX

#include "visage_graphics/gpu/gpu_backend.h"

#include <catch2/catch_test_macros.hpp>
#include <cstring>

using namespace visage;

TEST_CASE("Shader blobs keep their reflection and code through a round trip", "[graphics]") {
  gpu::ShaderInfo info;
  info.stage = gpu::ShaderStage::Compute;
  info.code_type = gpu::ShaderCode::Msl;
  info.uniforms.push_back({ "u_bounds", gpu::UniformType::Vec4, 1, 0 });
  info.uniforms.push_back({ "u_transform", gpu::UniformType::Mat4, 2, 16 });
  info.uniform_block_size = 144;
  info.samplers.push_back({ "s_texture", 1 });
  info.storage.push_back({ "b_tiles", 3, false, gpu::Access::ReadWrite });
  info.storage.push_back({ "i_segments", 4, true, gpu::Access::Write });
  info.workgroup_size[0] = 64;
  info.code = { 1, 2, 3, 4, 5 };

  std::vector<uint8_t> blob = gpu::writeShader(info);
  gpu::ShaderInfo parsed;
  REQUIRE(gpu::parseShader(blob.data(), static_cast<uint32_t>(blob.size()), parsed));
  REQUIRE(parsed.stage == info.stage);
  REQUIRE(parsed.code_type == info.code_type);
  REQUIRE(parsed.uniforms.size() == 2);
  REQUIRE(parsed.uniforms[1].name == "u_transform");
  REQUIRE(parsed.uniforms[1].type == gpu::UniformType::Mat4);
  REQUIRE(parsed.uniforms[1].num == 2);
  REQUIRE(parsed.uniforms[1].offset == 16);
  REQUIRE(parsed.uniform_block_size == 144);
  REQUIRE(parsed.samplers[0].stage == 1);
  REQUIRE(parsed.storage[1].image);
  REQUIRE(parsed.storage[1].access == gpu::Access::Write);
  REQUIRE(parsed.workgroup_size[0] == 64);
  REQUIRE(parsed.code == info.code);

  blob.resize(blob.size() - 1);
  REQUIRE_FALSE(gpu::parseShader(blob.data(), static_cast<uint32_t>(blob.size()), parsed));
  REQUIRE_FALSE(gpu::parseShader(reinterpret_cast<const uint8_t*>("bad"), 3, parsed));
}

TEST_CASE("GPU handles are released after the frame that destroys them", "[graphics]") {
  gpu::Init init;
  init.type = gpu::RendererType::Noop;
  REQUIRE(gpu::init(init));

  gpu::TextureHandle texture = gpu::createTexture2D(8, 8, gpu::TextureFormat::RGBA8);
  gpu::FrameBufferHandle frame_buffer = gpu::createFrameBuffer(16, 16, gpu::TextureFormat::RGBA16F);
  gpu::VertexLayout layout;
  layout.begin().add(gpu::Attrib::Position, 4).end();
  gpu::BufferHandle buffer = gpu::createDynamicVertexBuffer(32, layout);
  REQUIRE(gpu::isValid(texture));
  REQUIRE(gpu::isValid(frame_buffer));
  REQUIRE(gpu::isValid(buffer));
  REQUIRE(gpu::isValid(gpu::getTexture(frame_buffer)));
  REQUIRE(gpu::getStats()->num_textures == 2);
  REQUIRE(gpu::getStats()->num_buffers == 1);

  gpu::destroy(texture);
  gpu::destroy(frame_buffer);
  gpu::destroy(buffer);
  REQUIRE(gpu::getStats()->num_textures == 2);
  gpu::frame();
  REQUIRE(gpu::getStats()->num_textures == 0);
  REQUIRE(gpu::getStats()->num_frame_buffers == 0);
  REQUIRE(gpu::getStats()->num_buffers == 0);

  REQUIRE_FALSE(gpu::isValid(gpu::createShader(gpu::copy("bad", 3))));
  gpu::shutdown();
  REQUIRE_FALSE(gpu::initialized());
}

TEST_CASE("Vulkan textures round trip uploads, partial updates and blits", "[graphics]") {
  gpu::Init init;
  init.type = gpu::RendererType::Vulkan;
  if (!gpu::init(init)) {
    SUCCEED("No Vulkan device");
    return;
  }

  static constexpr int kSize = 4;
  std::vector<uint8_t> pixels(kSize * kSize * 4);
  for (size_t i = 0; i < pixels.size(); ++i)
    pixels[i] = static_cast<uint8_t>(i * 7);

  gpu::TextureHandle source = gpu::createTexture2D(kSize, kSize, gpu::TextureFormat::RGBA8, 0,
                                                   gpu::copy(pixels.data(), pixels.size()));
  gpu::TextureHandle destination = gpu::createTexture2D(kSize, kSize, gpu::TextureFormat::RGBA8,
                                                        gpu::kTextureBlitDst | gpu::kTextureReadBack);
  REQUIRE(gpu::isValid(source));
  REQUIRE(gpu::isValid(destination));

  std::vector<uint8_t> read(pixels.size());
  uint32_t ready = gpu::readTexture(source, read.data());
  while (gpu::frame() < ready) {
  }
  REQUIRE(read == pixels);

  uint8_t patch[4] = { 9, 8, 7, 6 };
  gpu::updateTexture2D(source, 1, 2, 1, 1, gpu::copy(patch, sizeof(patch)));
  std::memcpy(pixels.data() + (2 * kSize + 1) * 4, patch, sizeof(patch));
  gpu::blit(0, destination, 0, 0, source);
  ready = gpu::readTexture(destination, read.data());
  while (gpu::frame() < ready) {
  }
  REQUIRE(read == pixels);

  gpu::destroy(source);
  gpu::destroy(destination);
  gpu::frame();
  gpu::shutdown();
}

#endif
