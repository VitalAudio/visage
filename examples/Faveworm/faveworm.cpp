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

#include <visage/app.h>
#include <cmath>
#include <vector>

// Analog oscilloscope waveform visualization based on faveworm by Laurent de Soras
// Physics-based rendering: beam brightness inversely proportional to movement speed

class Oscilloscope : public visage::Frame {
public:
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr int kHistoryFrames = 4;   // Reduced for performance
  static constexpr float kMaxDist = 1.5f;   // Larger = fewer circles (was 0.5 in faveworm)
  static constexpr int kOversampleRate = 16;  // Audio oversampling factor

  struct Sample {
    float x, y;
  };

  Oscilloscope() {
    setIgnoresMouseEvents(true, false);
  }

  void setPhosphorEnabled(bool enabled) { phosphor_enabled_ = enabled; }
  bool phosphorEnabled() const { return phosphor_enabled_; }
  void setPhosphorDecay(float decay) { phosphor_decay_ = decay; }
  void setBeamSize(float size) { beam_size_ = size; }
  void setBeamGain(float gain) { beam_gain_ = gain; }

  void generateWaveform(double time, std::vector<Sample>& samples) {
    // Sample count balanced for quality and performance
    const int num_samples = 300;
    samples.resize(num_samples);

    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    if (w <= 0 || h <= 0) return;

    // Complex waveform with multiple harmonics
    const float freq1 = 3.0f + 0.5f * std::sin(static_cast<float>(time) * 0.3f);
    const float freq2 = 7.0f + std::sin(static_cast<float>(time) * 0.2f);
    const float freq3 = 11.0f + 0.3f * std::sin(static_cast<float>(time) * 0.15f);
    const float time_offset = static_cast<float>(time) * 2.0f;

    for (int i = 0; i < num_samples; ++i) {
      float t = static_cast<float>(i) / (num_samples - 1);
      float phase = t * 2.0f * kPi;

      // Horizontal sweep (time axis)
      samples[i].x = t * w;

      // Vertical signal (complex audio waveform)
      float signal = 0.5f * std::sin(phase * freq1 + time_offset);
      signal += 0.25f * std::sin(phase * freq2 + time_offset * 1.3f);
      signal += 0.15f * std::sin(phase * freq3 + time_offset * 0.7f);
      // Amplitude modulation
      signal *= 0.8f + 0.2f * std::sin(static_cast<float>(time) * 0.5f);

      samples[i].y = (0.5f - signal * 0.4f) * h;
    }
  }

  // Core faveworm algorithm: interpolate between samples, brightness inversely proportional to speed
  void renderWaveform(visage::Canvas& canvas, const std::vector<Sample>& samples, float alpha_mult) {
    if (samples.size() < 2) return;

    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    if (w <= 0 || h <= 0) return;

    const float diag = std::sqrt(w * w + h * h);

    // Base gain calculation (from faveworm)
    const float ref_energy = 50.0f;
    const float shutter_ratio = 1.0f;
    const float base_gain = (ref_energy * diag) / (shutter_ratio * samples.size() * kOversampleRate);
    const float unit_gain = base_gain * beam_gain_ * alpha_mult;

    const int num_samples = static_cast<int>(samples.size());
    const float half_beam = beam_size_ * 0.5f;

    for (int pos = 0; pos < num_samples - 1; ++pos) {
      float x = samples[pos].x;
      float y = samples[pos].y;
      const float dx = samples[pos + 1].x - x;
      const float dy = samples[pos + 1].y - y;
      const float d = std::sqrt(dx * dx + dy * dy);

      // Number of interpolation steps (max kMaxDist pixel gap)
      const int n = std::max(static_cast<int>(std::ceil(d / kMaxDist)), 1);
      const float nr = 1.0f / static_cast<float>(n);
      const float ix = dx * nr;
      const float iy = dy * nr;

      // KEY PHYSICS: gain inversely proportional to number of steps
      const float g = unit_gain * nr;
      const int end = (pos == num_samples - 2) ? n + 1 : n;

      for (int k = 0; k < end; ++k) {
        float brightness = std::min(1.0f, g);
        // Green phosphor color (P31 phosphor style)
        visage::Color color(brightness * 0.9f, brightness * 0.15f, brightness, brightness * 0.85f);
        canvas.setColor(color);
        canvas.fadeCircle(x - half_beam, y - half_beam, beam_size_, beam_size_ * 0.35f);
        x += ix;
        y += iy;
      }
    }
  }

  void draw(visage::Canvas& canvas) override {
    int iw = width();
    int ih = height();
    double time = canvas.time();

    // Dark CRT background
    canvas.setColor(0xff050508);
    canvas.fill(0, 0, iw, ih);

    if (iw > 0 && ih > 0) {
      const float w = static_cast<float>(iw);
      const float h = static_cast<float>(ih);

      // Draw graticule (oscilloscope grid)
      canvas.setColor(visage::Color(1.0f, 0.08f, 0.14f, 0.1f));
      for (int i = 1; i < 10; ++i) {
        float gx = w * i / 10.0f;
        float gy = h * i / 10.0f;
        canvas.fill(gx, 0, 1, h);
        canvas.fill(0, gy, w, 1);
      }
      // Center crosshair slightly brighter
      canvas.setColor(visage::Color(1.0f, 0.12f, 0.2f, 0.15f));
      canvas.fill(w * 0.5f, 0, 1, h);
      canvas.fill(0, h * 0.5f, w, 1);

      // Generate current waveform
      generateWaveform(time, current_samples_);

      if (current_samples_.size() >= 2) {
        // Additive blending for phosphor glow effect
        canvas.setBlendMode(visage::BlendMode::Add);

        // Draw phosphor persistence trails (older frames with decay)
        if (phosphor_enabled_) {
          for (int age = kHistoryFrames - 1; age >= 1; --age) {
            int idx = (history_index_ - age + kHistoryFrames) % kHistoryFrames;
            if (history_[idx].size() >= 2) {
              float decay = std::pow(phosphor_decay_, static_cast<float>(age));
              if (decay >= 0.02f)
                renderWaveform(canvas, history_[idx], decay);
            }
          }

          // Store current frame in history
          history_[history_index_] = current_samples_;
          history_index_ = (history_index_ + 1) % kHistoryFrames;
        }

        // Draw current frame at full brightness
        renderWaveform(canvas, current_samples_, 1.0f);

        // Reset blend mode
        canvas.setBlendMode(visage::BlendMode::Alpha);
      }
    }

    redraw();
  }

private:
  std::vector<Sample> current_samples_;
  std::vector<Sample> history_[kHistoryFrames];
  int history_index_ = 0;
  bool phosphor_enabled_ = true;
  float phosphor_decay_ = 0.1f;  // One step up from minimum (0.1 increments)
  float beam_size_ = 3.0f;
  float beam_gain_ = 1.5f;
};

class ExampleEditor : public visage::ApplicationWindow {
public:
  ExampleEditor() {
    // Enable keyboard input
    setAcceptsKeystrokes(true);

    addChild(&oscilloscope_);
    oscilloscope_.layout().setMargin(0);

    bloom_.setBloomSize(20.0f);
    bloom_.setBloomIntensity(1.5f);
    setPostEffect(&bloom_);
  }

  void setBloomEnabled(bool enabled) {
    bloom_enabled_ = enabled;
    setPostEffect(bloom_enabled_ ? &bloom_ : nullptr);
  }

  bool bloomEnabled() const { return bloom_enabled_; }
  void setPhosphorEnabled(bool enabled) { oscilloscope_.setPhosphorEnabled(enabled); }
  bool phosphorEnabled() const { return oscilloscope_.phosphorEnabled(); }

  void draw(visage::Canvas& canvas) override {
    canvas.setColor(0xff050508);
    canvas.fill(0, 0, width(), height());
  }

  bool keyPress(const visage::KeyEvent& event) override {
    if (event.keyCode() == visage::KeyCode::B) {
      setBloomEnabled(!bloom_enabled_);
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::P) {
      setPhosphorEnabled(!phosphorEnabled());
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Up) {
      bloom_intensity_ = std::min(5.0f, bloom_intensity_ + 0.3f);
      bloom_.setBloomIntensity(bloom_intensity_);
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Down) {
      bloom_intensity_ = std::max(0.0f, bloom_intensity_ - 0.3f);
      bloom_.setBloomIntensity(bloom_intensity_);
      return true;
    }
    return false;
  }

private:
  Oscilloscope oscilloscope_;
  visage::BloomPostEffect bloom_;
  float bloom_intensity_ = 2.0f;
  bool bloom_enabled_ = true;
};

int runExample() {
  ExampleEditor editor;
  editor.setWindowDecoration(visage::Window::Decoration::Client);

  if (visage::isMobileDevice())
    editor.showMaximized();
  else
    editor.show(900, 550);

  editor.runEventLoop();

  return 0;
}
