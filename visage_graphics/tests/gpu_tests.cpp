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

#include "visage_graphics/gpu/gpu_backend.h"

#include <catch2/catch_test_macros.hpp>
#include <cstring>

using namespace visage;

#if VISAGE_LINUX

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

  std::vector<uint8_t> huge_code = blob;
  size_t code_size_offset = blob.size() - info.code.size() - 4;
  for (int i = 0; i < 4; ++i)
    huge_code[code_size_offset + i] = 0xff;
  REQUIRE_FALSE(gpu::parseShader(huge_code.data(), static_cast<uint32_t>(huge_code.size()), parsed));

  gpu::ShaderInfo overflowing = info;
  overflowing.uniforms[1].offset = 128;
  std::vector<uint8_t> overflowing_blob = gpu::writeShader(overflowing);
  REQUIRE_FALSE(gpu::parseShader(overflowing_blob.data(),
                                 static_cast<uint32_t>(overflowing_blob.size()), parsed));

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

  gpu::destroy(gpu::getTexture(frame_buffer));
  gpu::frame();
  REQUIRE(gpu::getStats()->num_textures == 2);

  REQUIRE_FALSE(gpu::isValid(gpu::createTexture2D(8, 8, gpu::TextureFormat::RGBA8, 0,
                                                  gpu::alloc(8 * 8 * 4 - 1))));
  REQUIRE(gpu::getStats()->num_textures == 2);

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

TEST_CASE("Swap chain damage keeps separate areas apart and merges overlaps", "[graphics]") {
  gpu::DamageRegion region;
  region.add({ 0, 0, 0, 10, 10 });
  region.add({ 0, 90, 90, 10, 10 });
  REQUIRE(region.rects().size() == 2);

  region.add({ 0, 5, 5, 10, 10 });
  REQUIRE(region.rects().size() == 2);
  REQUIRE(region.rects()[1].x == 0);
  REQUIRE(region.rects()[1].width == 15);

  for (int i = 0; i < 20; ++i)
    region.add({ 0, i * 20, 50, 5, 5 });
  REQUIRE(region.rects().size() <= gpu::DamageRegion::kMaxRects);

  int64_t covered = 0;
  for (const gpu::Damage& rect : region.rects())
    covered += int64_t(rect.width) * rect.height;
  REQUIRE(covered < 100 * 100);
}

TEST_CASE("Swap chain buffers redraw the damage of every frame they missed", "[graphics]") {
  gpu::DamageRegion left;
  left.add({ 0, 0, 0, 10, 100 });
  gpu::DamageRegion right;
  right.add({ 0, 90, 0, 10, 100 });

  gpu::DamageHistory history;
  REQUIRE(history.bufferDamage(left, 0, 2, 100, 100).rects()[0].width == 100);

  history.push(left);
  history.push(right);
  gpu::DamageRegion current;
  current.add({ 0, 50, 50, 5, 5 });
  REQUIRE(history.bufferDamage(current, 0, 1, 100, 100).rects().size() == 1);
  REQUIRE(history.bufferDamage(current, 0, 2, 100, 100).rects().size() == 2);
  REQUIRE(history.bufferDamage(current, 0, 3, 100, 100).rects().size() == 3);
  REQUIRE(history.bufferDamage(current, 0, 4, 100, 100).rects()[0].width == 100);
  REQUIRE(history.bufferDamage(current, 0, 0, 100, 100).rects()[0].width == 100);
}

#endif

TEST_CASE("Clears fill only the requested area of a frame buffer", "[graphics]") {
  gpu::Init init;
  init.type = gpu::defaultRendererType();
  if (!gpu::init(init))
    SKIP("No GPU device");
  if (!gpu::getCaps()->clear_rects) {
    gpu::shutdown();
    SKIP("Renderer has no rectangle clears");
  }

  static constexpr int kSize = 8;
  gpu::FrameBufferHandle frame_buffer = gpu::createFrameBuffer(kSize, kSize, gpu::TextureFormat::RGBA8);
  gpu::TextureHandle read_back = gpu::createTexture2D(kSize, kSize, gpu::TextureFormat::RGBA8,
                                                      gpu::kTextureBlitDst | gpu::kTextureReadBack);
  REQUIRE(gpu::isValid(frame_buffer));
  REQUIRE(gpu::isValid(read_back));

  gpu::setViewFrameBuffer(0, frame_buffer);
  gpu::setViewRect(0, 0, 0, kSize, kSize);
  gpu::clear(0, 0, 0, kSize, kSize, 0xff0000ff);
  gpu::clear(0, 2, 3, 4, 2, 0x8000ff00);
  gpu::blit(1, read_back, 0, 0, gpu::getTexture(frame_buffer));

  std::vector<uint8_t> pixels(kSize * kSize * 4);
  uint32_t ready = gpu::readTexture(read_back, pixels.data());
  while (gpu::frame() < ready) {
  }

  for (int y = 0; y < kSize; ++y) {
    for (int x = 0; x < kSize; ++x) {
      const uint8_t* pixel = pixels.data() + (y * kSize + x) * 4;
      bool inside = x >= 2 && x < 6 && y >= 3 && y < 5;
      REQUIRE(pixel[0] == 0);
      REQUIRE(pixel[1] == (inside ? 255 : 0));
      REQUIRE(pixel[2] == (inside ? 0 : 255));
      REQUIRE(pixel[3] == (inside ? 128 : 255));
    }
  }

  gpu::destroy(read_back);
  gpu::destroy(frame_buffer);
  gpu::frame();
  gpu::shutdown();
}
