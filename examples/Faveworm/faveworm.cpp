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
#include <fstream>
#include <atomic>
#include <mutex>
#include <algorithm>

#include "FilterMorpher.h"
#include "FilterJoystick.h"
#include "TestSignalGenerator.h"
#include "embedded/example_fonts.h"

// Help overlay showing keyboard shortcuts
class HelpOverlay : public visage::Frame {
public:
  HelpOverlay() {
    // Start with mouse events ignored (hidden by default)
    setIgnoresMouseEvents(true, true);
  }

  void draw(visage::Canvas& canvas) override {
    if (!visible_) return;

    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());

    // Semi-transparent background
    canvas.setColor(visage::Color(0.85f, 0.0f, 0.0f, 0.0f));
    canvas.fill(0, 0, w, h);

    // Title and help text
    visage::Font title_font(24, resources::fonts::Lato_Regular_ttf);
    visage::Font font(14, resources::fonts::DroidSansMono_ttf);

    float y = 30;
    float col1 = 30;
    float col2 = 130;
    float line_h = 20;

    canvas.setColor(visage::Color(1.0f, 0.4f, 1.0f, 0.9f));
    canvas.text("Faveworm Help", title_font, visage::Font::kTopLeft, col1, y, w - 60, 30);
    y += 40;

    canvas.setColor(visage::Color(0.9f, 0.8f, 0.9f, 0.95f));

    auto drawSection = [&](const char* title) {
      canvas.setColor(visage::Color(1.0f, 0.5f, 0.9f, 0.8f));
      canvas.text(title, font, visage::Font::kTopLeft, col1, y, w - 60, line_h);
      y += line_h + 5;
      canvas.setColor(visage::Color(0.9f, 0.8f, 0.9f, 0.95f));
    };

    auto drawKey = [&](const char* key, const char* desc) {
      canvas.setColor(visage::Color(1.0f, 0.6f, 1.0f, 0.7f));
      canvas.text(key, font, visage::Font::kTopLeft, col1, y, 90, line_h);
      canvas.setColor(visage::Color(0.9f, 0.8f, 0.9f, 0.95f));
      canvas.text(desc, font, visage::Font::kTopLeft, col2, y, w - col2 - 30, line_h);
      y += line_h;
    };

    drawSection("General");
    drawKey("H / ?", "Toggle this help");
    drawKey("M", "Cycle display mode");
    drawKey("Space", "Play/pause audio");
    drawKey("B", "Toggle bloom");
    drawKey("P", "Toggle phosphor");
    drawKey("Up/Down", "Adjust bloom intensity");
    y += 10;

    drawSection("Trigger Mode");
    drawKey("L", "Toggle waveform lock");
    drawKey("E", "Toggle trigger edge");
    drawKey("Shift+Up/Dn", "Adjust threshold");
    y += 10;

    drawSection("XY Mode - Filter");
    drawKey("F", "Toggle filter");
    drawKey("S", "Toggle split mode");
    drawKey("D", "Cycle split presets");
    drawKey("Left/Right", "Adjust cutoff");
    y += 10;

    drawSection("XY Mode - Test Signal");
    drawKey("T", "Toggle test signal");
    drawKey("W", "Cycle waveform");
    drawKey("[ / ]", "Adjust frequency");
    drawKey("Shift+[/]", "Adjust detune");
    y += 15;

    canvas.setColor(visage::Color(0.6f, 0.5f, 0.6f, 0.6f));
    canvas.text("Drag audio file to load", font, visage::Font::kTopLeft, col1, y, w - 60, line_h);

    redraw();
  }

  void mouseDown(const visage::MouseEvent&) override {
    setVisible(false);
  }

  void setVisible(bool v) {
    visible_ = v;
    // Block mouse events when visible, pass through when hidden
    setIgnoresMouseEvents(!visible_, !visible_);
    if (visible_) redraw();
  }

  bool isVisible() const { return visible_; }
  void toggle() { setVisible(!visible_); }

private:
  bool visible_ = false;
};

// Vintage control panel background
class ControlPanel : public visage::Frame {
public:
  void draw(visage::Canvas& canvas) override {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());

    // Dark metallic background
    canvas.setColor(visage::Color(0.95f, 0.06f, 0.07f, 0.08f));
    canvas.fill(0, 0, w, h);

    // Subtle border/bevel (thin rectangle)
    canvas.setColor(visage::Color(0.3f, 0.12f, 0.12f, 0.10f));
    canvas.fill(0, 0, 2, h);

    // Vintage screws/rivets at corners
    canvas.setColor(visage::Color(0.4f, 0.15f, 0.15f, 0.12f));
    float rivet = 6.0f;
    canvas.circle(8 - rivet/2, 8 - rivet/2, rivet);
    canvas.circle(w - 8 - rivet/2, 8 - rivet/2, rivet);
    canvas.circle(8 - rivet/2, h - 8 - rivet/2, rivet);
    canvas.circle(w - 8 - rivet/2, h - 8 - rivet/2, rivet);

    redraw();
  }
};

#ifdef __APPLE__
#include <AudioToolbox/AudioToolbox.h>
#endif

// Display modes
enum class DisplayMode {
  TimeFree,     // Horizontal sweep, free running
  TimeTrigger,  // Horizontal sweep with trigger and waveform locking
  XY            // X-Y mode (Lissajous)
};

// Simple WAV file loader
struct AudioData {
  std::vector<float> left;
  std::vector<float> right;
  int sample_rate = 44100;
  bool stereo = false;

  bool loadWav(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;

    char riff[4];
    file.read(riff, 4);
    if (std::string(riff, 4) != "RIFF") return false;

    uint32_t file_size;
    file.read(reinterpret_cast<char*>(&file_size), 4);

    char wave[4];
    file.read(wave, 4);
    if (std::string(wave, 4) != "WAVE") return false;

    uint16_t audio_format = 0, num_channels = 0, bits_per_sample = 0;
    uint32_t byte_rate = 0, block_align = 0;

    while (file) {
      char chunk_id[4];
      uint32_t chunk_size;
      file.read(chunk_id, 4);
      file.read(reinterpret_cast<char*>(&chunk_size), 4);

      if (std::string(chunk_id, 4) == "fmt ") {
        file.read(reinterpret_cast<char*>(&audio_format), 2);
        file.read(reinterpret_cast<char*>(&num_channels), 2);
        file.read(reinterpret_cast<char*>(&sample_rate), 4);
        file.read(reinterpret_cast<char*>(&byte_rate), 4);
        file.read(reinterpret_cast<char*>(&block_align), 2);
        file.read(reinterpret_cast<char*>(&bits_per_sample), 2);
        if (chunk_size > 16) file.seekg(chunk_size - 16, std::ios::cur);
      }
      else if (std::string(chunk_id, 4) == "data") {
        stereo = (num_channels == 2);
        int num_samples = chunk_size / (bits_per_sample / 8) / num_channels;
        left.resize(num_samples);
        if (stereo) right.resize(num_samples);

        for (int i = 0; i < num_samples; ++i) {
          if (bits_per_sample == 16) {
            int16_t sample;
            file.read(reinterpret_cast<char*>(&sample), 2);
            left[i] = sample / 32768.0f;
            if (stereo) {
              file.read(reinterpret_cast<char*>(&sample), 2);
              right[i] = sample / 32768.0f;
            }
          }
          else if (bits_per_sample == 24) {
            uint8_t bytes[3];
            file.read(reinterpret_cast<char*>(bytes), 3);
            int32_t sample = (bytes[2] << 24) | (bytes[1] << 16) | (bytes[0] << 8);
            left[i] = sample / 2147483648.0f;
            if (stereo) {
              file.read(reinterpret_cast<char*>(bytes), 3);
              sample = (bytes[2] << 24) | (bytes[1] << 16) | (bytes[0] << 8);
              right[i] = sample / 2147483648.0f;
            }
          }
          else if (bits_per_sample == 32) {
            if (audio_format == 3) {
              float sample;
              file.read(reinterpret_cast<char*>(&sample), 4);
              left[i] = sample;
              if (stereo) {
                file.read(reinterpret_cast<char*>(&sample), 4);
                right[i] = sample;
              }
            }
            else {
              int32_t sample;
              file.read(reinterpret_cast<char*>(&sample), 4);
              left[i] = sample / 2147483648.0f;
              if (stereo) {
                file.read(reinterpret_cast<char*>(&sample), 4);
                right[i] = sample / 2147483648.0f;
              }
            }
          }
        }
        break;
      }
      else {
        file.seekg(chunk_size, std::ios::cur);
      }
    }

    if (!stereo && !left.empty()) {
      right = left;
      stereo = true;
    }

    return !left.empty();
  }
};

// Ring buffer for visualization and trigger detection
class RingBuffer {
public:
  static constexpr size_t kSize = 16384;  // Must be power of 2

  void write(float left, float right) {
    size_t pos = write_pos_.load(std::memory_order_relaxed);
    left_[pos & (kSize - 1)] = left;
    right_[pos & (kSize - 1)] = right;
    write_pos_.store(pos + 1, std::memory_order_release);
  }

  void read(std::vector<float>& left, std::vector<float>& right, int num_samples, int offset = 0) {
    size_t pos = write_pos_.load(std::memory_order_acquire);
    left.resize(num_samples);
    right.resize(num_samples);

    size_t start = (pos >= static_cast<size_t>(num_samples + offset)) ? pos - num_samples - offset : 0;
    for (int i = 0; i < num_samples; ++i) {
      size_t idx = (start + i) & (kSize - 1);
      left[i] = left_[idx];
      right[i] = right_[idx];
    }
  }

  // Read a single sample at offset from current position (negative = past)
  float readLeft(int offset) const {
    size_t pos = write_pos_.load(std::memory_order_acquire);
    return left_[(pos + offset) & (kSize - 1)];
  }

  float readRight(int offset) const {
    size_t pos = write_pos_.load(std::memory_order_acquire);
    return right_[(pos + offset) & (kSize - 1)];
  }

  size_t writePos() const { return write_pos_.load(std::memory_order_acquire); }

private:
  float left_[kSize] = {};
  float right_[kSize] = {};
  std::atomic<size_t> write_pos_{0};
};

// Audio player with trigger detection
class AudioPlayer {
public:
  static constexpr int kSweepSamples = 1024;  // Samples per sweep
  static constexpr int kDeadSamples = 256;     // Dead time after trigger
  static constexpr int kEvalSamples = 512;     // Evaluation window for waveform locking

  AudioPlayer() = default;
  ~AudioPlayer() { stop(); }

  bool load(const std::string& path) {
    if (!audio_data_.loadWav(path))
      return false;
    play_position_ = 0;
    return true;
  }

  void play() {
#ifdef __APPLE__
    if (is_playing_ || audio_data_.left.empty()) return;

    AudioStreamBasicDescription format = {};
    format.mSampleRate = audio_data_.sample_rate;
    format.mFormatID = kAudioFormatLinearPCM;
    format.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
    format.mBitsPerChannel = 32;
    format.mChannelsPerFrame = 2;
    format.mFramesPerPacket = 1;
    format.mBytesPerFrame = 8;
    format.mBytesPerPacket = 8;

    AudioQueueNewOutput(&format, audioCallback, this, nullptr, nullptr, 0, &audio_queue_);

    for (int i = 0; i < 3; ++i) {
      AudioQueueAllocateBuffer(audio_queue_, 512 * 8, &buffers_[i]);
      fillBuffer(buffers_[i]);
      AudioQueueEnqueueBuffer(audio_queue_, buffers_[i], 0, nullptr);
    }

    AudioQueueStart(audio_queue_, nullptr);
    is_playing_ = true;
#endif
  }

  void stop() {
#ifdef __APPLE__
    if (!is_playing_) return;
    AudioQueueStop(audio_queue_, true);
    AudioQueueDispose(audio_queue_, true);
    is_playing_ = false;
#endif
  }

  bool isPlaying() const { return is_playing_; }

  void getCurrentSamples(std::vector<float>& left, std::vector<float>& right, int num_samples) {
    ring_buffer_.read(left, right, num_samples);
  }

  // Get triggered samples with waveform locking
  bool getTriggeredSamples(std::vector<float>& out, int num_samples, float threshold, bool rising_edge, bool lock_enabled) {
    std::lock_guard<std::mutex> lock(trigger_mutex_);

    size_t current_pos = ring_buffer_.writePos();
    int search_start = -kDeadSamples - kSweepSamples;
    int search_end = -kDeadSamples;

    // Look for trigger point
    std::vector<int> candidates;

    for (int i = search_start; i < search_end - 1; ++i) {
      float s0 = ring_buffer_.readLeft(i);
      float s1 = ring_buffer_.readLeft(i + 1);

      bool crossed = rising_edge ? (s0 <= threshold && s1 > threshold)
                                 : (s0 >= threshold && s1 < threshold);
      if (crossed) {
        candidates.push_back(i);
      }
    }

    if (candidates.empty()) {
      // No trigger found, free run
      out.resize(num_samples);
      for (int i = 0; i < num_samples; ++i) {
        out[i] = ring_buffer_.readLeft(-num_samples + i);
      }
      return false;
    }

    int best_trigger = candidates.back();

    // Waveform locking: find best match with reference
    if (lock_enabled && !reference_waveform_.empty() && candidates.size() > 1) {
      float best_score = -1e30f;

      for (int cand : candidates) {
        float score = 0.0f;
        int ref_size = static_cast<int>(reference_waveform_.size());
        for (int j = 0; j < ref_size; ++j) {
          float s = ring_buffer_.readLeft(cand + j);
          score += s * reference_waveform_[j];
        }
        if (score > best_score) {
          best_score = score;
          best_trigger = cand;
        }
      }
    }

    // Read samples from trigger point
    out.resize(num_samples);
    for (int i = 0; i < num_samples; ++i) {
      out[i] = ring_buffer_.readLeft(best_trigger + i);
    }

    // Update reference waveform
    reference_waveform_ = out;

    return true;
  }

  bool hasAudio() const { return !audio_data_.left.empty(); }
  int sampleRate() const { return audio_data_.sample_rate; }

private:
#ifdef __APPLE__
  static void audioCallback(void* user_data, AudioQueueRef queue, AudioQueueBufferRef buffer) {
    auto* player = static_cast<AudioPlayer*>(user_data);
    player->fillBuffer(buffer);
    AudioQueueEnqueueBuffer(queue, buffer, 0, nullptr);
  }

  void fillBuffer(AudioQueueBufferRef buffer) {
    float* data = static_cast<float*>(buffer->mAudioData);
    int frames = buffer->mAudioDataBytesCapacity / 8;
    size_t total = audio_data_.left.size();

    for (int i = 0; i < frames; ++i) {
      size_t idx = play_position_ % total;
      float l = audio_data_.left[idx];
      float r = audio_data_.stereo ? audio_data_.right[idx] : audio_data_.left[idx];
      data[i * 2] = l;
      data[i * 2 + 1] = r;

      ring_buffer_.write(l, r);

      play_position_++;
    }

    buffer->mAudioDataByteSize = frames * 8;
  }

  AudioQueueRef audio_queue_ = nullptr;
  AudioQueueBufferRef buffers_[3] = {};
#endif

  AudioData audio_data_;
  RingBuffer ring_buffer_;
  size_t play_position_ = 0;
  bool is_playing_ = false;

  std::mutex trigger_mutex_;
  std::vector<float> reference_waveform_;
};

class Oscilloscope : public visage::Frame {
public:
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr int kHistoryFrames = 4;
  static constexpr float kMaxDist = 1.5f;
  static constexpr int kOversampleRate = 16;

  struct Sample {
    float x, y;
  };

  Oscilloscope() {
    setIgnoresMouseEvents(false, false);
    // Initialize SVF with default settings for XY visualization
    svf_.setSampleRate(44100.0);
    svf_.setCutoff(150.0);      // Good starting frequency for XY visuals
    svf_.setResonance(0.7);     // Moderate resonance for defined shapes
    // Stereo router for split X/Y filter modes
    stereo_router_.setSampleRate(44100.0);
    stereo_router_.setCutoff(150.0);
    stereo_router_.setResonance(0.7);
    stereo_router_.setSplitMode(StereoFilterRouter::SplitMode::LpHp);
  }

  bool receivesDragDropFiles() override { return true; }

  void dropFiles(const std::vector<std::string>& paths) override {
    if (!paths.empty() && audio_player_) {
      audio_player_->stop();
      if (audio_player_->load(paths[0])) {
        display_mode_ = DisplayMode::TimeTrigger;
        audio_player_->play();
      }
    }
  }

  void setPhosphorEnabled(bool enabled) { phosphor_enabled_ = enabled; }
  bool phosphorEnabled() const { return phosphor_enabled_; }
  void setPhosphorDecay(float decay) { phosphor_decay_ = decay; }
  void setBeamSize(float size) { beam_size_ = size; }
  void setBeamGain(float gain) { beam_gain_ = gain; }

  void setDisplayMode(DisplayMode mode) { display_mode_ = mode; }
  DisplayMode displayMode() const { return display_mode_; }
  void cycleDisplayMode() {
    switch (display_mode_) {
      case DisplayMode::TimeFree: display_mode_ = DisplayMode::TimeTrigger; break;
      case DisplayMode::TimeTrigger: display_mode_ = DisplayMode::XY; break;
      case DisplayMode::XY: display_mode_ = DisplayMode::TimeFree; break;
    }
  }

  void setTriggerThreshold(float thr) { trigger_threshold_ = thr; }
  float triggerThreshold() const { return trigger_threshold_; }
  void setTriggerRising(bool rising) { trigger_rising_ = rising; }
  bool triggerRising() const { return trigger_rising_; }
  void setWaveformLock(bool lock) { waveform_lock_ = lock; }
  bool waveformLock() const { return waveform_lock_; }

  void setAudioPlayer(AudioPlayer* player) {
    audio_player_ = player;
    if (player && player->hasAudio()) {
      svf_.setSampleRate(player->sampleRate());
    }
  }

  // SVF controls
  FilterMorpher& morpher() { return morpher_; }
  StereoFilterRouter& stereoRouter() { return stereo_router_; }
  float filterCutoff() const { return filter_cutoff_; }
  float filterResonance() const { return filter_resonance_; }
  void setFilterCutoff(float fc) {
    filter_cutoff_ = std::clamp(fc, 20.0f, 2000.0f);
    svf_.setCutoff(filter_cutoff_);
    stereo_router_.setCutoff(filter_cutoff_);
  }
  void setFilterResonance(float r) {
    filter_resonance_ = std::clamp(r, 0.0f, 1.0f);
    svf_.setResonance(filter_resonance_);
    stereo_router_.setResonance(filter_resonance_);
  }
  void setFilterEnabled(bool enabled) { filter_enabled_ = enabled; }
  bool filterEnabled() const { return filter_enabled_; }

  // Stereo split mode (mono source -> X/Y via different filter outputs)
  void setStereoSplitMode(bool enabled) { stereo_split_mode_ = enabled; }
  bool stereoSplitMode() const { return stereo_split_mode_; }
  void cycleSplitMode() { stereo_router_.cycleSplitMode(); }

  // Test signal generator controls
  TestSignalGenerator& testSignal() { return test_signal_; }
  void setTestSignalEnabled(bool enabled) { test_signal_enabled_ = enabled; }
  bool testSignalEnabled() const { return test_signal_enabled_; }

  void generateWaveform(double time, std::vector<Sample>& samples) {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    if (w <= 0 || h <= 0) return;

    // XY mode with audio - apply SVF filter for Lissajous shapes
    if (display_mode_ == DisplayMode::XY && audio_player_ && audio_player_->hasAudio()) {
      const int num_samples = 512;
      std::vector<float> left, right;
      audio_player_->getCurrentSamples(left, right, num_samples);

      samples.resize(num_samples);
      float cx = w * 0.5f;
      float cy = h * 0.5f;
      float scale = std::min(w, h) * 0.4f;

      for (int i = 0; i < num_samples; ++i) {
        float x_val = left[i];
        float y_val = right[i];

        if (filter_enabled_) {
          if (stereo_split_mode_) {
            // Split mode: mix to mono, route through different filter outputs
            float mono = (left[i] + right[i]) * 0.5f;
            auto xy = stereo_router_.process(static_cast<double>(mono));
            x_val = static_cast<float>(xy.x);
            y_val = static_cast<float>(xy.y);
          } else {
            // Morph mode: filter Y channel with joystick-controlled mix
            auto outputs = svf_.process(static_cast<double>(y_val));
            y_val = static_cast<float>(morpher_.apply(outputs.lp, outputs.bp, outputs.hp));
          }
        }

        samples[i].x = cx + x_val * scale;
        samples[i].y = cy - y_val * scale;
      }
      return;
    }

    // XY mode with test signal generator (no audio loaded)
    if (display_mode_ == DisplayMode::XY && test_signal_enabled_) {
      const int num_samples = 512;
      samples.resize(num_samples);
      float cx = w * 0.5f;
      float cy = h * 0.5f;
      float scale = std::min(w, h) * 0.4f;

      for (int i = 0; i < num_samples; ++i) {
        float x_val, y_val;
        test_signal_.getSample(x_val, y_val);

        if (filter_enabled_) {
          if (stereo_split_mode_) {
            // Split mode: mono source -> X/Y via different filter outputs
            // Use X oscillator as mono source for interesting Lissajous
            auto xy = stereo_router_.process(static_cast<double>(x_val));
            x_val = static_cast<float>(xy.x);
            y_val = static_cast<float>(xy.y);
          } else {
            // Morph mode: filter Y channel with joystick-controlled mix
            auto outputs = svf_.process(static_cast<double>(y_val));
            y_val = static_cast<float>(morpher_.apply(outputs.lp, outputs.bp, outputs.hp));
          }
        }

        samples[i].x = cx + x_val * scale;
        samples[i].y = cy - y_val * scale;
      }
      return;
    }

    // Time mode with trigger
    if (display_mode_ == DisplayMode::TimeTrigger && audio_player_ && audio_player_->hasAudio()) {
      const int num_samples = 512;
      std::vector<float> audio;
      audio_player_->getTriggeredSamples(audio, num_samples, trigger_threshold_, trigger_rising_, waveform_lock_);

      samples.resize(num_samples);
      float cy = h * 0.5f;
      float scale = h * 0.4f;

      for (int i = 0; i < num_samples; ++i) {
        float t = static_cast<float>(i) / (num_samples - 1);
        samples[i].x = t * w;
        samples[i].y = cy - audio[i] * scale;
      }
      return;
    }

    // Time mode free running with audio
    if (display_mode_ == DisplayMode::TimeFree && audio_player_ && audio_player_->hasAudio()) {
      const int num_samples = 512;
      std::vector<float> left, right;
      audio_player_->getCurrentSamples(left, right, num_samples);

      samples.resize(num_samples);
      float cy = h * 0.5f;
      float scale = h * 0.4f;

      for (int i = 0; i < num_samples; ++i) {
        float t = static_cast<float>(i) / (num_samples - 1);
        samples[i].x = t * w;
        samples[i].y = cy - left[i] * scale;
      }
      return;
    }

    // Demo mode: synthetic waveform
    const int num_samples = 300;
    samples.resize(num_samples);

    const float freq1 = 3.0f + 0.5f * std::sin(static_cast<float>(time) * 0.3f);
    const float freq2 = 7.0f + std::sin(static_cast<float>(time) * 0.2f);
    const float freq3 = 11.0f + 0.3f * std::sin(static_cast<float>(time) * 0.15f);
    const float time_offset = static_cast<float>(time) * 2.0f;

    for (int i = 0; i < num_samples; ++i) {
      float t = static_cast<float>(i) / (num_samples - 1);
      float phase = t * 2.0f * kPi;

      if (display_mode_ == DisplayMode::XY) {
        float cx = w * 0.5f;
        float cy = h * 0.5f;
        float scale = std::min(w, h) * 0.35f;
        samples[i].x = cx + scale * std::sin(phase * 3.0f + time_offset);
        samples[i].y = cy + scale * std::sin(phase * 2.0f + time_offset * 0.7f);
      }
      else {
        samples[i].x = t * w;
        float signal = 0.5f * std::sin(phase * freq1 + time_offset);
        signal += 0.25f * std::sin(phase * freq2 + time_offset * 1.3f);
        signal += 0.15f * std::sin(phase * freq3 + time_offset * 0.7f);
        signal *= 0.8f + 0.2f * std::sin(static_cast<float>(time) * 0.5f);
        samples[i].y = (0.5f - signal * 0.4f) * h;
      }
    }
  }

  void renderWaveform(visage::Canvas& canvas, const std::vector<Sample>& samples, float alpha_mult) {
    if (samples.size() < 2) return;

    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    if (w <= 0 || h <= 0) return;

    const float diag = std::sqrt(w * w + h * h);
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

      const int n = std::max(static_cast<int>(std::ceil(d / kMaxDist)), 1);
      const float nr = 1.0f / static_cast<float>(n);
      const float ix = dx * nr;
      const float iy = dy * nr;
      const float g = unit_gain * nr;
      const int end = (pos == num_samples - 2) ? n + 1 : n;

      for (int k = 0; k < end; ++k) {
        float brightness = std::min(1.0f, g);
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

    canvas.setColor(0xff050508);
    canvas.fill(0, 0, iw, ih);

    if (iw > 0 && ih > 0) {
      const float w = static_cast<float>(iw);
      const float h = static_cast<float>(ih);

      // Draw graticule
      canvas.setColor(visage::Color(1.0f, 0.08f, 0.14f, 0.1f));
      for (int i = 1; i < 10; ++i) {
        float gx = w * i / 10.0f;
        float gy = h * i / 10.0f;
        canvas.fill(gx, 0, 1, h);
        canvas.fill(0, gy, w, 1);
      }
      canvas.setColor(visage::Color(1.0f, 0.12f, 0.2f, 0.15f));
      canvas.fill(w * 0.5f, 0, 1, h);
      canvas.fill(0, h * 0.5f, w, 1);

      // Draw trigger level indicator in trigger mode
      if (display_mode_ == DisplayMode::TimeTrigger) {
        float trig_y = h * 0.5f - trigger_threshold_ * h * 0.4f;
        canvas.setColor(visage::Color(1.0f, 0.8f, 0.2f, 0.3f));
        canvas.fill(0, trig_y - 1, 20, 2);
      }

      generateWaveform(time, current_samples_);

      if (current_samples_.size() >= 2) {
        canvas.setBlendMode(visage::BlendMode::Add);

        if (phosphor_enabled_) {
          for (int age = kHistoryFrames - 1; age >= 1; --age) {
            int idx = (history_index_ - age + kHistoryFrames) % kHistoryFrames;
            if (history_[idx].size() >= 2) {
              float decay = std::pow(phosphor_decay_, static_cast<float>(age));
              if (decay >= 0.02f)
                renderWaveform(canvas, history_[idx], decay);
            }
          }
          history_[history_index_] = current_samples_;
          history_index_ = (history_index_ + 1) % kHistoryFrames;
        }

        renderWaveform(canvas, current_samples_, 1.0f);
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
  float phosphor_decay_ = 0.1f;
  float beam_size_ = 3.0f;
  float beam_gain_ = 1.5f;

  DisplayMode display_mode_ = DisplayMode::TimeFree;
  float trigger_threshold_ = 0.0f;
  bool trigger_rising_ = true;
  bool waveform_lock_ = true;

  // SVF filter for XY mode
  mutable SimpleSVF svf_;
  FilterMorpher morpher_;
  mutable StereoFilterRouter stereo_router_;
  float filter_cutoff_ = 150.0f;
  float filter_resonance_ = 0.7f;
  bool filter_enabled_ = true;
  bool stereo_split_mode_ = false;  // false = morph (joystick), true = split (X/Y routing)

  // Test signal generator for XY mode without audio
  TestSignalGenerator test_signal_;
  bool test_signal_enabled_ = true;  // Enabled by default

  AudioPlayer* audio_player_ = nullptr;
};

class ExampleEditor : public visage::ApplicationWindow {
public:
  ExampleEditor() {
    setAcceptsKeystrokes(true);

    addChild(&oscilloscope_);
    oscilloscope_.layout().setMargin(0);
    oscilloscope_.setAudioPlayer(&audio_player_);

    // Control panel background (add first so it's behind controls)
    addChild(&control_panel_);
    control_panel_.setVisible(false);

    // Set up filter joystick for morph control
    addChild(&filter_joystick_);
    filter_joystick_.setMorpher(&oscilloscope_.morpher());
    filter_joystick_.setVisible(false);  // Hidden by default (TimeFree mode)

    // Cutoff knob (logarithmic for frequency)
    addChild(&cutoff_knob_);
    cutoff_knob_.setValue(&filter_cutoff_);
    cutoff_knob_.setRange(20.0f, 2000.0f);
    cutoff_knob_.setColor(visage::Color(1.0f, 0.4f, 0.9f, 0.9f));  // Cyan
    cutoff_knob_.setCallback([this](float v) { oscilloscope_.setFilterCutoff(v); });
    cutoff_knob_.setVisible(false);

    // Resonance knob (linear)
    addChild(&resonance_knob_);
    resonance_knob_.setValue(&filter_resonance_);
    resonance_knob_.setRange(0.0f, 1.0f);
    resonance_knob_.setColor(visage::Color(1.0f, 0.9f, 0.6f, 0.4f));  // Orange
    resonance_knob_.setCallback([this](float v) { oscilloscope_.setFilterResonance(v); });
    resonance_knob_.setVisible(false);

    // Help overlay (covers entire window)
    addChild(&help_overlay_);

    bloom_.setBloomSize(20.0f);
    bloom_.setBloomIntensity(0.2f);  // Subtle glow by default
    setPostEffect(&bloom_);
  }

  void resized() override {
    bool show_panel = oscilloscope_.displayMode() == DisplayMode::XY;

    if (show_panel) {
      // Vintage control panel on right side
      const int panel_width = 140;
      const int margin = 8;
      const int knob_size = 60;
      const int js_size = 100;

      int panel_x = width() - panel_width;
      control_panel_.setBounds(panel_x, 0, panel_width, height());

      // Joystick at top of panel
      int y = margin + 10;
      filter_joystick_.setBounds(panel_x + (panel_width - js_size) / 2, y, js_size, js_size);
      y += js_size + margin + 10;

      // Cutoff knob
      cutoff_knob_.setBounds(panel_x + (panel_width - knob_size) / 2, y, knob_size, knob_size);
      y += knob_size + margin;

      // Resonance knob
      resonance_knob_.setBounds(panel_x + (panel_width - knob_size) / 2, y, knob_size, knob_size);

      // Oscilloscope fills remaining space
      oscilloscope_.setBounds(0, 0, width() - panel_width, height());
    } else {
      // No panel - oscilloscope fills entire window
      oscilloscope_.setBounds(0, 0, width(), height());
    }

    // Help overlay covers entire window
    help_overlay_.setBounds(0, 0, width(), height());
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
    else if (event.keyCode() == visage::KeyCode::H ||
             (event.isShiftDown() && event.keyCode() == visage::KeyCode::Slash)) {
      help_overlay_.toggle();
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::M) {
      oscilloscope_.cycleDisplayMode();
      bool show_panel = oscilloscope_.displayMode() == DisplayMode::XY;
      control_panel_.setVisible(show_panel);
      filter_joystick_.setVisible(show_panel);
      cutoff_knob_.setVisible(show_panel);
      resonance_knob_.setVisible(show_panel);
      // Resize to update oscilloscope bounds
      resized();
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::L) {
      oscilloscope_.setWaveformLock(!oscilloscope_.waveformLock());
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::E) {
      oscilloscope_.setTriggerRising(!oscilloscope_.triggerRising());
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::F) {
      oscilloscope_.setFilterEnabled(!oscilloscope_.filterEnabled());
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::S) {
      // Toggle stereo split mode (mono -> X/Y via different filter outputs)
      oscilloscope_.setStereoSplitMode(!oscilloscope_.stereoSplitMode());
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::D) {
      // Cycle through split mode presets
      oscilloscope_.cycleSplitMode();
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::W) {
      // Cycle test signal waveform
      oscilloscope_.testSignal().cycleWaveform();
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::T) {
      // Toggle test signal
      oscilloscope_.setTestSignalEnabled(!oscilloscope_.testSignalEnabled());
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::LeftBracket) {
      auto& ts = oscilloscope_.testSignal();
      if (event.isShiftDown()) {
        // Decrease detune
        ts.setDetune(ts.detune() * 0.999);
      } else {
        // Decrease frequency
        ts.setFrequency(ts.frequency() * 0.9);
      }
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::RightBracket) {
      auto& ts = oscilloscope_.testSignal();
      if (event.isShiftDown()) {
        // Increase detune
        ts.setDetune(ts.detune() * 1.001);
      } else {
        // Increase frequency
        ts.setFrequency(ts.frequency() * 1.1);
      }
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Left) {
      // Decrease cutoff
      filter_cutoff_ = std::max(20.0f, filter_cutoff_ * 0.9f);
      oscilloscope_.setFilterCutoff(filter_cutoff_);
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Right) {
      // Increase cutoff
      filter_cutoff_ = std::min(2000.0f, filter_cutoff_ * 1.1f);
      oscilloscope_.setFilterCutoff(filter_cutoff_);
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Space) {
      if (audio_player_.isPlaying())
        audio_player_.stop();
      else
        audio_player_.play();
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Up) {
      if (event.isShiftDown()) {
        // Adjust trigger threshold
        oscilloscope_.setTriggerThreshold(std::min(1.0f, oscilloscope_.triggerThreshold() + 0.05f));
      } else {
        bloom_intensity_ = std::min(5.0f, bloom_intensity_ + 0.3f);
        bloom_.setBloomIntensity(bloom_intensity_);
      }
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Down) {
      if (event.isShiftDown()) {
        oscilloscope_.setTriggerThreshold(std::max(-1.0f, oscilloscope_.triggerThreshold() - 0.05f));
      } else {
        bloom_intensity_ = std::max(0.0f, bloom_intensity_ - 0.3f);
        bloom_.setBloomIntensity(bloom_intensity_);
      }
      return true;
    }
    return false;
  }

private:
  Oscilloscope oscilloscope_;
  ControlPanel control_panel_;
  FilterJoystick filter_joystick_;
  FilterKnob cutoff_knob_{"Cutoff", true};  // logarithmic
  FilterKnob resonance_knob_{"Resonance", false};  // linear
  HelpOverlay help_overlay_;
  visage::BloomPostEffect bloom_;
  AudioPlayer audio_player_;
  float bloom_intensity_ = 0.2f;
  bool bloom_enabled_ = true;
  float filter_cutoff_ = 150.0f;
  float filter_resonance_ = 0.7f;
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
