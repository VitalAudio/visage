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

#pragma once

#include "shapes.h"

namespace visage {
  class Layer;

  class PostEffect {
  public:
    explicit PostEffect(bool hdr = false) : hdr_(hdr) { }

    virtual ~PostEffect() = default;
    virtual int preprocess(Region* region, int submit_pass) { return submit_pass; }
    virtual void submit(const BatchVector<SampleRegion>& batches, Layer& destination, int submit_pass) { }
    void submitPassthrough(const BatchVector<SampleRegion>& batches, const Layer& destination,
                           int submit_pass) const;
    bool hdr() const { return hdr_; }

  private:
    bool hdr_ = false;
  };

  struct DownsampleHandles;

  class DownsamplePostEffect : public PostEffect {
  public:
    static constexpr int kMaxDownsamples = 6;

    explicit DownsamplePostEffect(bool hdr = false);
    ~DownsamplePostEffect() override;

  protected:
    void setInitialVertices(Region* region);
    void checkBuffers(const Region* region, bool full_resolution);
    void setScreenVertexBuffer(bool inverted);

    int full_width_ = 0;
    int full_height_ = 0;
    int widths_[kMaxDownsamples + 1] {};
    int heights_[kMaxDownsamples + 1] {};
    std::unique_ptr<DownsampleHandles> handles_;
    UvVertex screen_vertices_[4] {};
    UvVertex inv_screen_vertices_[4] {};
    int format_ = 0;
  };

  class BlurPostEffect : public DownsamplePostEffect {
  public:
    static constexpr float kMinSigma = 0.01f;

    BlurPostEffect();
    ~BlurPostEffect() override;

    int preprocess(Region* region, int submit_pass) override;
    void submit(const BatchVector<SampleRegion>& batches, Layer& destination, int submit_pass) override;

    float blurRadius() const { return blur_radius_; }
    void setBlurRadius(float size) { blur_radius_ = std::max(0.0f, size); }

  private:
    float blur_radius_ = 0.0f;
    float sigma_ = 0.0f;
    int downsample_stages_ = 0;

    VISAGE_LEAK_CHECKER(BlurPostEffect)
  };

  class BloomPostEffect : public DownsamplePostEffect {
  public:
    BloomPostEffect();
    ~BloomPostEffect() override;

    int preprocess(Region* region, int submit_pass) override;
    void submit(const BatchVector<SampleRegion>& batches, Layer& destination, int submit_pass) override;
    void submitBloom(const BatchVector<SampleRegion>& batches, const Layer& destination,
                     int submit_pass) const;

    void setBloomSize(float size) { bloom_size_ = std::log2(std::max(1.0f, size)); }
    void setBloomIntensity(float intensity) { bloom_intensity_ = intensity; }

  private:
    float bloom_size_ = 0.0f;
    float bloom_intensity_ = 1.0f;

    int downsamples_ = 0;

    VISAGE_LEAK_CHECKER(BloomPostEffect)
  };

  class ShaderPostEffect : public PostEffect {
  public:
    struct UniformData {
      float data[4];
    };

    ShaderPostEffect(const EmbeddedFile& vertex_shader, const EmbeddedFile& fragment_shader) :
        vertex_shader_(vertex_shader), fragment_shader_(fragment_shader) { }

    void submit(const BatchVector<SampleRegion>& batches, Layer& destination, int submit_pass) override;

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