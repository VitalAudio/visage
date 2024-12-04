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

#include "shapes.h"

namespace visage {
  class Layer;

  class PostEffect {
  public:
    explicit PostEffect(bool hdr = false) : hdr_(hdr) { }

    virtual ~PostEffect() = default;
    virtual int preprocess(Region* region, int submit_pass) { return submit_pass; }
    virtual void submit(const SampleRegion& source, Layer& destination, int submit_pass, int x, int y) { }
    bool hdr() const { return hdr_; }

  private:
    bool hdr_ = false;
  };

  struct BlurBloomHandles;

  class BlurBloomPostEffect : public PostEffect {
  public:
    static constexpr int kMaxDownsamples = 6;

    BlurBloomPostEffect();
    ~BlurBloomPostEffect() override;

    void setInitialVertices(Region* region);
    int preprocess(Region* region, int submit_pass) override;
    void submit(const SampleRegion& source, Layer& destination, int submit_pass, int x, int y) override;
    void submitPassthrough(const SampleRegion& source, const Layer& destination, int submit_pass,
                           int x, int y) const;
    void submitBloom(const SampleRegion& source, const Layer& destination, int submit_pass, int x,
                     int y) const;
    void checkBuffers(const Region* region);

    void setBloomSize(float size) { bloom_size_ = std::log2(size); }
    void setBloomIntensity(float intensity) { bloom_intensity_ = intensity; }
    void setBlurSize(float size) { blur_size_ = std::log2(size); }
    void setBlurAmount(float amount) { blur_amount_ = amount; }

  private:
    float bloom_size_ = 0.0f;
    float bloom_intensity_ = 1.0f;
    float blur_size_ = 0.0f;
    float blur_amount_ = 0.0f;

    int full_width_ = 0;
    int full_height_ = 0;
    int format_ = 0;
    int widths_[kMaxDownsamples] {};
    int heights_[kMaxDownsamples] {};

    int cutoff_ = 0;

    UvVertex screen_vertices_[4] {};
    std::unique_ptr<BlurBloomHandles> handles_;

    VISAGE_LEAK_CHECKER(BlurBloomPostEffect)
  };

  class ShaderPostEffect : public PostEffect {
  public:
    struct UniformData {
      float data[4];
    };

    ShaderPostEffect(const EmbeddedFile& vertex_shader, const EmbeddedFile& fragment_shader) :
        vertex_shader_(vertex_shader), fragment_shader_(fragment_shader) { }

    void submit(const SampleRegion& source, Layer& destination, int submit_pass, int x, int y) override;

    BlendMode state() const { return state_; }
    void setState(BlendMode state) { state_ = state; }

    const EmbeddedFile& vertexShader() const { return vertex_shader_; }
    const EmbeddedFile& fragmentShader() const { return fragment_shader_; }

    void setUniformValue(const std::string& name, float value) {
      uniforms_[name] = { value, value, value, value };
    }
    void setUniformValue(const std::string& name, float value1, float value2, float value3, float value4) {
      uniforms_[name] = { value1, value2, value3, value4 };
    }
    void removeUniform(const std::string& name) { uniforms_.erase(name); }

  private:
    std::map<std::string, UniformData> uniforms_;
    EmbeddedFile vertex_shader_;
    EmbeddedFile fragment_shader_;
    BlendMode state_ = BlendMode::Alpha;

    VISAGE_LEAK_CHECKER(ShaderPostEffect)
  };
}