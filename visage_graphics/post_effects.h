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
    virtual void submit(const SampleRegion& source, Layer& destination, int submit_pass, int x, int y) { }
    bool hdr() const { return hdr_; }

  private:
    bool hdr_ = false;
  };

  struct DownsampleHandles;

  class DownsamplePostEffect : public PostEffect {
  public:
    static constexpr int kMaxDownsamples = 6;

    DownsamplePostEffect(bool hdr = false);

  protected:
    void setInitialVertices(Region* region);
    void checkBuffers(const Region* region);

    int full_width_ = 0;
    int full_height_ = 0;
    int widths_[kMaxDownsamples] {};
    int heights_[kMaxDownsamples] {};
    std::unique_ptr<DownsampleHandles> handles_;
    UvVertex screen_vertices_[4] {};
    int format_ = 0;
  };

  class BlurPostEffect : public DownsamplePostEffect {
  public:
    BlurPostEffect();
    ~BlurPostEffect() override;

    int preprocess(Region* region, int submit_pass) override;
    int preprocessBlend(int submit_pass);
    void submitPassthrough(const SampleRegion& source, Layer& destination, int submit_pass, int x, int y);
    void blendPassthrough(const SampleRegion& source, Layer& destination, int submit_pass, int x, int y);
    void submitBlurred(const SampleRegion& source, Layer& destination, int submit_pass, int x, int y);
    void submit(const SampleRegion& source, Layer& destination, int submit_pass, int x, int y) override;

    void setBlurSize(float size) { blur_size_ = std::log2(size); }
    void setBlurAmount(float amount) { blur_amount_ = amount; }

  private:
    float blur_size_ = 0.0f;
    float blur_amount_ = 0.0f;
    float stages_ = 0.0f;

    VISAGE_LEAK_CHECKER(BlurPostEffect)
  };

  class BloomPostEffect : public DownsamplePostEffect {
  public:
    BloomPostEffect();
    ~BloomPostEffect() override;

    int preprocess(Region* region, int submit_pass) override;
    void submit(const SampleRegion& source, Layer& destination, int submit_pass, int x, int y) override;
    void submitPassthrough(const SampleRegion& source, const Layer& destination, int submit_pass,
                           int x, int y) const;
    void submitBloom(const SampleRegion& source, const Layer& destination, int submit_pass, int x,
                     int y) const;

    void setBloomSize(float size) { bloom_size_ = std::log2(size); }
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