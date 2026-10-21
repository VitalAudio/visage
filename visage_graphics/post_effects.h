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
    virtual int preprocessBackdrop(Region* source, const Region* region, int submit_pass) {
      return preprocess(source, submit_pass);
    }
    virtual void submit(const BatchVector<SampleRegion>& batches, Layer& destination, int submit_pass) { }
    virtual void submitBackdrop(const BatchVector<SampleRegion>& batches, Layer& destination,
                                int submit_pass) {
      submit(batches, destination, submit_pass);
    }
    virtual bool appliedInPlace() const { return false; }
    virtual BlendMode state() const { return BlendMode::Alpha; }
    void submitPassthrough(const BatchVector<SampleRegion>& batches, const Layer& destination,
                           int submit_pass) const;
    bool hdr() const { return hdr_; }

  private:
    bool hdr_ = false;
  };

  struct BackdropHandles;

  class BackdropCopy {
  public:
    BackdropCopy();
    ~BackdropCopy();

    int copy(Region* source, const Region* region, int submit_pass);
    void release();

    bool hasCopy() const;
    Layer* layer() const;
    bgfx::FrameBufferHandle frameBuffer() const;
    int width() const { return bounds_.width(); }
    int height() const { return bounds_.height(); }
    void setUniforms(const BatchVector<SampleRegion>& batches, PostEffectVertex* vertices,
                     bgfx::FrameBufferHandle texture) const;

  private:
    std::unique_ptr<BackdropHandles> handles_;
    const Region* source_ = nullptr;
    IBounds bounds_;
  };

  struct DownsampleHandles;

  class DownsamplePostEffect : public PostEffect {
  public:
    static constexpr int kMaxDownsamples = 6;

    explicit DownsamplePostEffect(bool hdr = false);
    ~DownsamplePostEffect() override;

  protected:
    void setLevelDimensions(int width, int height);
    void checkBuffers(const Region* region);
    void setScreenVertexBuffer(bool inverted);
    void setTexturePositions(const BatchVector<SampleRegion>& batches, PostEffectVertex* vertices,
                             int level) const;

    int full_width_ = 0;
    int full_height_ = 0;
    int widths_[kMaxDownsamples + 1] {};
    int heights_[kMaxDownsamples + 1] {};
    std::unique_ptr<DownsampleHandles> handles_;
    int format_ = 0;
    const Region* region_ = nullptr;
  };

  class BlurPostEffect : public DownsamplePostEffect {
  public:
    static constexpr float kMinSigma = 0.01f;

    BlurPostEffect();
    ~BlurPostEffect() override;

    int preprocess(Region* region, int submit_pass) override;
    int preprocessBackdrop(Region* source, const Region* region, int submit_pass) override;
    void submitBackdrop(const BatchVector<SampleRegion>& batches, Layer& destination, int submit_pass) override;
    bool appliedInPlace() const override { return true; }

    float blurRadius() const { return blur_radius_; }
    void setBlurRadius(float size) { blur_radius_ = std::max(0.0f, size); }

  private:
    int blur(Region* region, int submit_pass);

    float blur_radius_ = 0.0f;
    BackdropCopy backdrop_;

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

    int preprocessBackdrop(Region* source, const Region* region, int submit_pass) override;
    void submit(const BatchVector<SampleRegion>& batches, Layer& destination, int submit_pass) override;
    void submitBackdrop(const BatchVector<SampleRegion>& batches, Layer& destination, int submit_pass) override;

    BlendMode state() const override { return state_; }
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
    void submitShader(const BatchVector<SampleRegion>& batches, Layer& destination, int submit_pass,
                      bool backdrop);

    std::map<std::string, UniformData> uniforms_;
    EmbeddedFile vertex_shader_;
    EmbeddedFile fragment_shader_;
    BlendMode state_ = BlendMode::Alpha;
    BackdropCopy backdrop_;

    VISAGE_LEAK_CHECKER(ShaderPostEffect)
  };

  class LuminanceMaskPostEffect : public ShaderPostEffect {
  public:
    LuminanceMaskPostEffect();
  };
}