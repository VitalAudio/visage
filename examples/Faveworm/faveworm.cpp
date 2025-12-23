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

#include "embedded/example_fonts.h"
#include "FilterJoystick.h"
#include "FilterMorpher.h"
#include "TestSignalGenerator.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>
#include <visage/app.h>

// Help overlay showing keyboard shortcuts
class HelpOverlay : public visage::Frame {
public:
  HelpOverlay() {
    // Start with mouse events ignored (hidden by default)
    setIgnoresMouseEvents(true, true);
  }

  void draw(visage::Canvas& canvas) override {
    if (!visible_)
      return;

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
    drawKey("O", "Toggle speaker output");
    drawKey("B", "Toggle bloom");
    drawKey("P", "Toggle phosphor");
    drawKey("Up/Down", "Adjust bloom intensity");
    drawKey(", / .", "Step back / forward (when frozen)");
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

  void mouseDown(const visage::MouseEvent&) override { setVisible(false); }

  void setVisible(bool v) {
    visible_ = v;
    // Block mouse events when visible, pass through when hidden
    setIgnoresMouseEvents(!visible_, !visible_);
    if (visible_)
      redraw();
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
    canvas.circle(8 - rivet / 2, 8 - rivet / 2, rivet);
    canvas.circle(w - 8 - rivet / 2, 8 - rivet / 2, rivet);
    canvas.circle(8 - rivet / 2, h - 8 - rivet / 2, rivet);
    canvas.circle(w - 8 - rivet / 2, h - 8 - rivet / 2, rivet);

    redraw();
  }
};

#include <portaudio.h>

// Helper structure for PortAudio initialization
struct PortAudioRAII {
  PortAudioRAII() { Pa_Initialize(); }
  ~PortAudioRAII() { Pa_Terminate(); }
};
static PortAudioRAII pa_raii;

// Display modes
enum class DisplayMode {
  TimeFree,  // Horizontal sweep, free running
  TimeTrigger,  // Horizontal sweep with trigger and waveform locking
  XY  // X-Y mode (Lissajous)
};

// Simple WAV file loader
struct AudioData {
  std::vector<float> left;
  std::vector<float> right;
  int sample_rate = 44100;
  bool stereo = false;

  bool loadWav(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file)
      return false;

    char riff[4];
    file.read(riff, 4);
    if (std::string(riff, 4) != "RIFF")
      return false;

    uint32_t file_size;
    file.read(reinterpret_cast<char*>(&file_size), 4);

    char wave[4];
    file.read(wave, 4);
    if (std::string(wave, 4) != "WAVE")
      return false;

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
        if (chunk_size > 16)
          file.seekg(chunk_size - 16, std::ios::cur);
      }
      else if (std::string(chunk_id, 4) == "data") {
        stereo = (num_channels == 2);
        int num_samples = chunk_size / (bits_per_sample / 8) / num_channels;
        left.resize(num_samples);
        if (stereo)
          right.resize(num_samples);

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
  std::atomic<size_t> write_pos_ { 0 };
};

// Audio player with trigger detection
class AudioPlayer {
public:
  static constexpr int kSweepSamples = 1024;  // Samples per sweep
  static constexpr int kDeadSamples = 256;  // Dead time after trigger
  static constexpr int kEvalSamples = 512;  // Evaluation window for waveform locking

  ~AudioPlayer() { stop(); }

  AudioPlayer() {
    // Initialize filter defaults
    svf_.setSampleRate(44100.0);
    svf_.setCutoff(150.0);
    svf_.setResonance(0.7);
    stereo_router_.setSampleRate(44100.0);
    stereo_router_.setCutoff(150.0);
    stereo_router_.setResonance(0.7);
    stereo_router_.setSplitMode(StereoFilterRouter::SplitMode::LpHp);
  }

  bool load(const std::string& path) {
    if (!audio_data_.loadWav(path))
      return false;
    play_position_ = 0;
    return true;
  }

  void setTestGenerator(TestSignalGenerator* gen) { test_generator_ = gen; }

  void play() {
    if (is_playing_)
      return;

    if (audio_data_.left.empty() && !test_generator_)
      return;

    PaError init_err = Pa_Initialize();
    if (init_err != paNoError)
      return;

    PaStreamParameters outputParameters;
    outputParameters.device = getOutputDevice(use_speaker_);
    if (outputParameters.device == paNoDevice) {
      outputParameters.device = Pa_GetDefaultOutputDevice();
    }

    if (outputParameters.device == paNoDevice)
      return;

    const PaDeviceInfo* info = Pa_GetDeviceInfo(outputParameters.device);
    if (!info)
      return;

    outputParameters.channelCount = 2;
    outputParameters.sampleFormat = paFloat32;
    outputParameters.suggestedLatency = info->defaultLowOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = nullptr;

    double sr = audio_data_.left.empty() ? 44100.0 : audio_data_.sample_rate;
    sample_rate_ = sr;

    PaError err = Pa_OpenStream(&stream_, nullptr, &outputParameters, sr, 512, paClipOff, paCallback, this);
    if (err != paNoError)
      return;

    err = Pa_StartStream(stream_);
    if (err != paNoError) {
      Pa_CloseStream(stream_);
      stream_ = nullptr;
      return;
    }

    is_playing_ = true;
  }

  void stop() {
    if (!is_playing_ || !stream_)
      return;

    // We don't wait for ramp here because target_gain_ is managed by UI
    Pa_StopStream(stream_);
    Pa_CloseStream(stream_);
    stream_ = nullptr;
    is_playing_ = false;
  }

  bool isPlaying() const { return is_playing_; }

  bool isPaused() const { return paused_; }
  void setPaused(bool p) { paused_ = p; }

  void getCurrentSamples(std::vector<float>& left, std::vector<float>& right, int num_samples) {
    ring_buffer_.read(left, right, num_samples);
  }

  // Step forward or backward in time (only used when frozen)
  void step(int samples) {
    if (audio_data_.left.empty()) {
      if (test_generator_) {
        for (int i = 0; i < std::abs(samples); ++i) {
          float l, r;
          test_generator_->getSample(l, r);
          if (samples > 0)
            ring_buffer_.write(l, r);
          // Note: Test generator doesn't support seeking backward easily,
          // so we just advance or stay still for now.
        }
      }
      return;
    }

    size_t total = audio_data_.left.size();
    if (samples > 0) {
      for (int i = 0; i < samples; ++i) {
        size_t idx = play_position_ % total;
        float l = audio_data_.left[idx];
        float r = audio_data_.stereo ? audio_data_.right[idx] : audio_data_.left[idx];
        ring_buffer_.write(l, r);
        play_position_++;
      }
    }
    else {
      int abs_samples = -samples;
      play_position_ = (play_position_ + total - abs_samples) % total;

      // Update ring buffer so the visualization shows the new position
      int update_samples = kSweepSamples;
      for (int i = 0; i < update_samples; ++i) {
        size_t idx = (play_position_ - update_samples + i + total) % total;
        float l = audio_data_.left[idx];
        float r = audio_data_.stereo ? audio_data_.right[idx] : audio_data_.left[idx];
        ring_buffer_.write(l, r);
      }
    }
  }

  // Get triggered samples with waveform locking
  bool getTriggeredSamples(std::vector<float>& out, int num_samples, float threshold,
                           bool rising_edge, bool lock_enabled) {
    std::lock_guard<std::mutex> lock(trigger_mutex_);

    size_t current_pos = ring_buffer_.writePos();
    int search_start = -kDeadSamples - kSweepSamples;
    int search_end = -kDeadSamples;

    // Look for trigger point
    std::vector<int> candidates;

    for (int i = search_start; i < search_end - 1; ++i) {
      float s0 = ring_buffer_.readLeft(i);
      float s1 = ring_buffer_.readLeft(i + 1);

      bool crossed = rising_edge ? (s0 <= threshold && s1 > threshold) :
                                   (s0 >= threshold && s1 < threshold);
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

  bool isMuted() const { return muted_; }
  void setMuted(bool m) { muted_ = m; }

  void setSpeakerOutput(bool use_speaker) {
    if (use_speaker_ == use_speaker)
      return;

    use_speaker_ = use_speaker;
    // If we're already playing, we have to restart the stream to change the device.
    // To avoid clicks, we should technically ramp down first, but for now we'll
    // just allow the hardware switch to be a bit abrupt, or the user can mute first.
    if (is_playing_) {
      stop();
      play();
    }
  }

  int getOutputDevice(bool speaker) {
    int numDevices = Pa_GetDeviceCount();
    if (numDevices < 0)
      return paNoDevice;

    int fallback = paNoDevice;
    for (int i = 0; i < numDevices; ++i) {
      const PaDeviceInfo* deviceInfo = Pa_GetDeviceInfo(i);
      if (deviceInfo->maxOutputChannels <= 0)
        continue;

      if (fallback == paNoDevice)
        fallback = i;

      const char* name = deviceInfo->name;
      if (speaker) {
        if (strstr(name, "Speaker") || strstr(name, "Built-in") || strstr(name, "Internal"))
          return i;
      }
      else {
        // For non-speaker, we prefer something that ISN'T the internal speaker if possible,
        // but really the user just wants the "default" if they didn't toggle speaker.
        // On macOS, PortAudio's default is usually correct.
      }
    }
    return fallback;
  }

  bool useSpeaker() const { return use_speaker_; }

private:
  static int paCallback(const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags,
                        void* userData) {
    auto* player = static_cast<AudioPlayer*>(userData);
    float* out = static_cast<float*>(outputBuffer);
    size_t total = player->audio_data_.left.size();

    if (total == 0 && !player->test_generator_) {
      memset(out, 0, framesPerBuffer * 2 * sizeof(float));
      return paContinue;
    }

    const float target_gain = (player->muted_ || player->paused_) ? 0.0f : 1.0f;
    const float ramp_inc = 1.0f / (0.050f * player->sample_rate_);

    for (unsigned int i = 0; i < framesPerBuffer; ++i) {
      // Apply gain ramp first to determine if we should still be pulling samples for output
      if (player->current_gain_ < target_gain)
        player->current_gain_ = std::min(target_gain, player->current_gain_ + ramp_inc);
      else if (player->current_gain_ > target_gain)
        player->current_gain_ = std::max(target_gain, player->current_gain_ - ramp_inc);

      // The waveform/engine is "running" if not paused OR if we are still ramping down audio
      bool is_running = !player->paused_ || player->current_gain_ > 0.0f;

      float l = 0.0f, r = 0.0f;
      if (is_running) {
        if (total > 0) {
          size_t idx = player->play_position_ % total;
          l = player->audio_data_.left[idx];
          r = player->audio_data_.stereo ? player->audio_data_.right[idx] : player->audio_data_.left[idx];
          player->play_position_++;
        }
        else if (player->test_generator_) {
          player->test_generator_->getSample(l, r);
        }

        // Apply visualizer-style filtering to the audio output
        if (player->filter_enabled_) {
          // Note: accessing filter members from audio thread while UI thread writes to them
          // is theoretically racy but practically safe for simple float params.
          if (player->stereo_split_mode_) {
            double mono = (static_cast<double>(l) + static_cast<double>(r)) * 0.5;
            auto xy = player->stereo_router_.process(mono);
            l = static_cast<float>(xy.x);
            r = static_cast<float>(xy.y);
          }
          else {
            // Morph mode: only filter Y (right) channel
            auto outputs = player->svf_.process(static_cast<double>(r));
            r = static_cast<float>(player->morpher_.apply(outputs.lp, outputs.bp, outputs.hp));
          }
        }

        // Only update ring buffer (waveform) if the user hasn't explicitly paused it
        if (!player->paused_)
          player->ring_buffer_.write(l, r);
      }

      out[i * 2] = l * player->current_gain_;
      out[i * 2 + 1] = r * player->current_gain_;
    }

    return paContinue;
  }

  PaStream* stream_ = nullptr;

  AudioData audio_data_;
  RingBuffer ring_buffer_;
  size_t play_position_ = 0;
  bool is_playing_ = false;

public:
  // Filter controls
  void setFilterCutoff(float fc) {
    filter_cutoff_ = std::clamp(fc, 20.0f, 2000.0f);
    svf_.setCutoff(filter_cutoff_);
    stereo_router_.setCutoff(filter_cutoff_);
  }
  float filterCutoff() const { return filter_cutoff_; }

  void setFilterResonance(float r) {
    filter_resonance_ = std::clamp(r, 0.0f, 0.99f);
    svf_.setResonance(filter_resonance_);
    stereo_router_.setResonance(filter_resonance_);
  }
  float filterResonance() const { return filter_resonance_; }

  void setFilterEnabled(bool enabled) { filter_enabled_ = enabled; }
  bool filterEnabled() const { return filter_enabled_; }

  void setStereoSplitMode(bool enabled) { stereo_split_mode_ = enabled; }
  bool stereoSplitMode() const { return stereo_split_mode_; }
  void cycleSplitMode() { stereo_router_.cycleSplitMode(); }

  // Expose for UI
  FilterMorpher& morpher() { return morpher_; }
  StereoFilterRouter& stereoRouter() { return stereo_router_; }

  // Filter state
  mutable SimpleSVF svf_;
  FilterMorpher morpher_;
  mutable StereoFilterRouter stereo_router_;
  std::atomic<float> filter_cutoff_ { 150.0f };
  std::atomic<float> filter_resonance_ { 0.7f };
  std::atomic<bool> filter_enabled_ { false };
  std::atomic<bool> stereo_split_mode_ { false };

  std::mutex trigger_mutex_;
  std::vector<float> reference_waveform_;
  bool use_speaker_ = false;
  TestSignalGenerator* test_generator_ = nullptr;
  float current_gain_ = 0.0f;
  std::atomic<bool> paused_ { false };
  std::atomic<bool> muted_ { true };
  float sample_rate_ = 44100.0f;
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

  Oscilloscope() { setIgnoresMouseEvents(true, false); }

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

  void setAudioPlayer(AudioPlayer* player) { audio_player_ = player; }

  // SVF controls
  // SVF controls delegates
  FilterMorpher& morpher() { return audio_player_->morpher(); }
  StereoFilterRouter& stereoRouter() { return audio_player_->stereoRouter(); }
  float filterCutoff() const { return audio_player_ ? audio_player_->filterCutoff() : 150.0f; }
  float filterResonance() const { return audio_player_ ? audio_player_->filterResonance() : 0.7f; }

  void setFilterCutoff(float fc) {
    if (audio_player_)
      audio_player_->setFilterCutoff(fc);
  }
  void setFilterResonance(float r) {
    if (audio_player_)
      audio_player_->setFilterResonance(r);
  }
  void setFilterEnabled(bool enabled) {
    if (audio_player_)
      audio_player_->setFilterEnabled(enabled);
  }
  bool filterEnabled() const { return audio_player_ ? audio_player_->filterEnabled() : false; }

  // Stereo split mode delegates
  void setStereoSplitMode(bool enabled) {
    if (audio_player_)
      audio_player_->setStereoSplitMode(enabled);
  }
  bool stereoSplitMode() const { return audio_player_ ? audio_player_->stereoSplitMode() : false; }
  void cycleSplitMode() {
    if (audio_player_)
      audio_player_->cycleSplitMode();
  }

  // Test signal generator controls
  TestSignalGenerator& testSignal() { return test_signal_; }
  void setTestSignalEnabled(bool enabled) { test_signal_enabled_ = enabled; }
  bool testSignalEnabled() const { return test_signal_enabled_; }

  void generateWaveform(double time, std::vector<Sample>& samples) {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    if (w <= 0 || h <= 0)
      return;

    // Use audio player data for all primary modes
    if (audio_player_) {
      if (display_mode_ == DisplayMode::XY) {
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

          samples[i].x = cx + x_val * scale;
          samples[i].y = cy - y_val * scale;
        }
        return;
      }
      else if (display_mode_ == DisplayMode::TimeTrigger) {
        const int num_samples = 512;
        std::vector<float> audio;
        audio_player_->getTriggeredSamples(audio, num_samples, trigger_threshold_, trigger_rising_,
                                           waveform_lock_);

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
      else if (display_mode_ == DisplayMode::TimeFree) {
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
    if (samples.size() < 2)
      return;

    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    if (w <= 0 || h <= 0)
      return;

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

      // Only generate new waveform if not paused (freeze display when paused)
      bool is_paused = test_signal_.isPaused();
      if (!is_paused || current_samples_.empty()) {
        generateWaveform(time + time_offset_, current_samples_);
      }

      if (current_samples_.size() >= 2) {
        canvas.setBlendMode(visage::BlendMode::Add);

        // Always render phosphor trails (frozen snapshot when paused)
        if (phosphor_enabled_) {
          for (int age = kHistoryFrames - 1; age >= 1; --age) {
            int idx = (history_index_ - age + kHistoryFrames) % kHistoryFrames;
            if (history_[idx].size() >= 2) {
              float decay = std::pow(phosphor_decay_, static_cast<float>(age));
              if (decay >= 0.02f)
                renderWaveform(canvas, history_[idx], decay);
            }
          }
          // Only update history when not paused
          if (!is_paused) {
            history_[history_index_] = current_samples_;
            history_index_ = (history_index_ + 1) % kHistoryFrames;
          }
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

  DisplayMode display_mode_ = DisplayMode::XY;
  double time_offset_ = 0.0;
  float trigger_threshold_ = 0.0f;
  bool trigger_rising_ = true;
  bool waveform_lock_ = true;

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
    audio_player_.setTestGenerator(&oscilloscope_.testSignal());

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

    // Resonance knob (linear) - capped just below 1.0 for stability
    addChild(&resonance_knob_);
    resonance_knob_.setValue(&filter_resonance_);
    resonance_knob_.setRange(0.0f, 0.99f);
    resonance_knob_.setColor(visage::Color(1.0f, 0.9f, 0.6f, 0.4f));  // Orange
    resonance_knob_.setCallback([this](float v) {
      oscilloscope_.setFilterResonance(v);
      filter_resonance_pct_ = v * 100.0f;
    });
    resonance_knob_.setVisible(false);

    // Numeric displays for filter values
    addChild(&cutoff_display_);
    cutoff_display_.setValue(&filter_cutoff_);
    cutoff_display_.setColor(visage::Color(1.0f, 0.4f, 0.9f, 0.9f));  // Match cutoff knob
    cutoff_display_.setDecimals(0);
    cutoff_display_.setVisible(false);

    addChild(&resonance_display_);
    resonance_display_.setValue(&filter_resonance_pct_);
    resonance_display_.setColor(visage::Color(1.0f, 0.9f, 0.6f, 0.4f));  // Match resonance knob
    resonance_display_.setDecimals(0);
    resonance_display_.setVisible(false);

    // Frequency knob for test signal (logarithmic)
    addChild(&freq_knob_);
    freq_knob_.setValue(&signal_freq_);
    freq_knob_.setRange(10.0f, 500.0f);
    freq_knob_.setColor(visage::Color(1.0f, 0.5f, 0.9f, 0.5f));  // Match waveform selector
    freq_knob_.setCallback([this](float v) { oscilloscope_.testSignal().setFrequency(v); });
    freq_knob_.setVisible(false);

    // Detune knob for test signal (linear around 1.0)
    addChild(&detune_knob_);
    detune_knob_.setValue(&signal_detune_);
    detune_knob_.setRange(0.9f, 1.1f);
    detune_knob_.setColor(visage::Color(1.0f, 0.6f, 0.8f, 0.6f));  // Lighter purple
    detune_knob_.setCallback([this](float v) { oscilloscope_.testSignal().setDetune(v); });
    detune_knob_.setVisible(false);

    // Numeric displays for signal generator
    addChild(&freq_display_);
    freq_display_.setValue(&signal_freq_);
    freq_display_.setColor(visage::Color(1.0f, 0.5f, 0.9f, 0.5f));  // Match freq knob
    freq_display_.setDecimals(0);
    freq_display_.setVisible(false);

    addChild(&detune_display_);
    detune_display_.setValue(&signal_detune_);
    detune_display_.setColor(visage::Color(1.0f, 0.6f, 0.8f, 0.6f));  // Match detune knob
    detune_display_.setDecimals(3);
    detune_display_.setVisible(false);

    // Mode selector (display mode)
    addChild(&mode_selector_);
    mode_selector_.setLabel("Mode");
    mode_selector_.setOptions({ "Time", "Trig", "XY" });
    mode_selector_.setColor(visage::Color(1.0f, 0.4f, 0.8f, 0.9f));
    mode_selector_.setCallback([this](int i) {
      oscilloscope_.setDisplayMode(static_cast<DisplayMode>(i));
      updatePanelVisibility();
    });
    mode_selector_.setVisible(false);

    // Waveform selector
    addChild(&waveform_selector_);
    waveform_selector_.setLabel("Wave");
    waveform_selector_.setOptions({ "Sin", "Tri", "Saw", "Sqr", "Nz" });
    waveform_selector_.setColor(visage::Color(1.0f, 0.5f, 0.9f, 0.5f));
    waveform_selector_.setCallback([this](int i) {
      oscilloscope_.testSignal().setWaveform(static_cast<TestSignalGenerator::Waveform>(i));
    });
    waveform_selector_.setVisible(false);

    // Split mode selector
    addChild(&split_selector_);
    split_selector_.setLabel("Split");
    split_selector_.setOptions({ "LP/HP", "BP/AP", "BR/AP", "LP/BP", "AP/HP", "BP/BR", "Morph" });
    split_selector_.setColor(visage::Color(1.0f, 0.3f, 0.7f, 0.9f));
    split_selector_.setCallback([this](int i) {
      oscilloscope_.stereoRouter().setSplitMode(static_cast<StereoFilterRouter::SplitMode>(i));
    });
    split_selector_.setVisible(false);

    // Filter on/off switch
    addChild(&filter_switch_);
    filter_switch_.setValue(false);
    filter_switch_.setColor(visage::Color(1.0f, 0.4f, 0.9f, 0.9f));
    filter_switch_.setCallback([this](bool v) {
      oscilloscope_.setFilterEnabled(v);
      updateFilterControlsEnabled(v);
    });
    filter_switch_.setVisible(false);

    // Split mode on/off switch
    addChild(&split_switch_);
    split_switch_.setValue(false);
    split_switch_.setColor(visage::Color(1.0f, 0.3f, 0.7f, 0.9f));
    split_switch_.setCallback([this](bool v) {
      oscilloscope_.setStereoSplitMode(v);
      split_selector_.setEnabled(v && filter_switch_.value());
    });
    split_switch_.setVisible(false);

    // Speaker Switch (Sound toggle)
    addChild(&speaker_switch_);
    speaker_switch_.setValue(false);  // Default: Sound OFF (Muted)
    speaker_switch_.setColor(visage::Color(1.0f, 1.0f, 0.8f, 0.2f));  // Yellowish
    speaker_switch_.setCallback([this](bool v) {
      audio_player_.setMuted(!v);  // Switch ON (v=true) -> Muted = false
    });
    speaker_switch_.setVisible(false);

    // Help overlay (covers entire window)
    addChild(&help_overlay_);

    bloom_.setBloomSize(20.0f);
    bloom_.setBloomIntensity(0.2f);  // Subtle glow by default
    setPostEffect(&bloom_);

    // Initialize panel visibility
    updatePanelVisibility();

    // Start audio engine immediately (it will be muted by default)
    audio_player_.play();
  }

  void updatePanelVisibility() {
    bool show_panel = true;  // Always show panel
    bool is_xy = oscilloscope_.displayMode() == DisplayMode::XY;

    control_panel_.setVisible(show_panel);
    mode_selector_.setVisible(show_panel);

    // XY-specific controls
    filter_joystick_.setVisible(is_xy);
    cutoff_knob_.setVisible(is_xy);
    resonance_knob_.setVisible(is_xy);
    cutoff_display_.setVisible(is_xy);
    resonance_display_.setVisible(is_xy);
    freq_knob_.setVisible(is_xy);
    detune_knob_.setVisible(is_xy);
    freq_display_.setVisible(is_xy);
    detune_display_.setVisible(is_xy);
    waveform_selector_.setVisible(is_xy);
    split_selector_.setVisible(is_xy);
    filter_switch_.setVisible(is_xy);
    split_switch_.setVisible(is_xy);
    speaker_switch_.setVisible(show_panel);

    // Update enabled state based on filter switch
    updateFilterControlsEnabled(filter_switch_.value());

    resized();
  }

  void updateFilterControlsEnabled(bool enabled) {
    // Dim/enable filter-related controls based on filter switch
    filter_joystick_.setEnabled(enabled);
    cutoff_knob_.setEnabled(enabled);
    resonance_knob_.setEnabled(enabled);
    split_switch_.setEnabled(enabled);
    // Split selector only enabled if both filter AND split switch are on
    split_selector_.setEnabled(enabled && split_switch_.value());
  }

  void resized() override {
    // Vintage control panel on right side (always visible)
    const int panel_width = 140;
    const int margin = 4;
    const int knob_size = 60;  // Taller to fit label
    const int js_size = 90;  // Taller to fit label
    const int selector_h = 34;  // Taller to fit label
    const int switch_h = 32;  // Taller to fit label

    int panel_x = width() - panel_width;
    control_panel_.setBounds(panel_x, 0, panel_width, height());

    int y = margin;
    int cx = panel_x + (panel_width - knob_size) / 2;

    // Mode selector at top (always visible)
    mode_selector_.setBounds(panel_x + 10, y, panel_width - 20, selector_h);
    y += selector_h + margin;

    // XY mode controls
    if (oscilloscope_.displayMode() == DisplayMode::XY) {
      // Waveform selector
      waveform_selector_.setBounds(panel_x + 10, y, panel_width - 20, selector_h);
      y += selector_h + margin;

      // Signal freq and detune knobs side by side (smaller) with displays below
      int small_knob = 50;
      int small_display_h = 16;
      int left_x = panel_x + 10;
      int right_x = panel_x + panel_width - 10 - small_knob;
      freq_knob_.setBounds(left_x, y, small_knob, small_knob);
      detune_knob_.setBounds(right_x, y, small_knob, small_knob);
      y += small_knob + 2;
      freq_display_.setBounds(left_x, y, small_knob, small_display_h);
      detune_display_.setBounds(right_x, y, small_knob, small_display_h);
      y += small_display_h + margin;

      // Filter switch + label area
      filter_switch_.setBounds(panel_x + 10, y, panel_width - 20, switch_h);
      y += switch_h + margin;

      // Joystick
      filter_joystick_.setBounds(panel_x + (panel_width - js_size) / 2, y, js_size, js_size);
      y += js_size + margin;

      // Cutoff knob with display
      int display_w = 50;
      int display_h = 18;
      cutoff_knob_.setBounds(panel_x + 10, y, knob_size, knob_size);
      cutoff_display_.setBounds(panel_x + 10 + knob_size + 4, y + (knob_size - display_h) / 2,
                                display_w, display_h);
      y += knob_size + margin;

      // Resonance knob with display
      resonance_knob_.setBounds(panel_x + 10, y, knob_size, knob_size);
      resonance_display_.setBounds(panel_x + 10 + knob_size + 4, y + (knob_size - display_h) / 2,
                                   display_w, display_h);
      y += knob_size + margin;

      // Split switch
      split_switch_.setBounds(panel_x + 10, y, panel_width - 20, switch_h);
      y += switch_h + margin;

      // Split mode selector
      split_selector_.setBounds(panel_x + 10, y, panel_width - 20, selector_h);
    }

    // Speaker switch at the bottom of the control panel
    speaker_switch_.setBounds(panel_x + 10, height() - switch_h - margin, panel_width - 20, switch_h);

    // Oscilloscope fills remaining space
    oscilloscope_.setBounds(0, 0, width() - panel_width, height());

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
      mode_selector_.setIndex(static_cast<int>(oscilloscope_.displayMode()));
      updatePanelVisibility();
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
      if (event.isShiftDown()) {
        // Decrease detune
        signal_detune_ = std::max(0.9f, signal_detune_ * 0.999f);
        oscilloscope_.testSignal().setDetune(signal_detune_);
      }
      else {
        // Decrease frequency
        signal_freq_ = std::max(10.0f, signal_freq_ * 0.9f);
        oscilloscope_.testSignal().setFrequency(signal_freq_);
      }
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::RightBracket) {
      if (event.isShiftDown()) {
        // Increase detune
        signal_detune_ = std::min(1.1f, signal_detune_ * 1.001f);
        oscilloscope_.testSignal().setDetune(signal_detune_);
      }
      else {
        // Increase frequency
        signal_freq_ = std::min(500.0f, signal_freq_ * 1.1f);
        oscilloscope_.testSignal().setFrequency(signal_freq_);
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
      // Toggle pause state (Freezes waveform and audio)
      audio_player_.setPaused(!audio_player_.isPaused());
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::O) {
      // Toggle audio mute (Speaker switch)
      speaker_switch_.setValue(!speaker_switch_.value(), true);
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Up) {
      if (event.isShiftDown()) {
        // Adjust trigger threshold
        oscilloscope_.setTriggerThreshold(std::min(1.0f, oscilloscope_.triggerThreshold() + 0.05f));
      }
      else {
        bloom_intensity_ = std::min(5.0f, bloom_intensity_ + 0.3f);
        bloom_.setBloomIntensity(bloom_intensity_);
      }
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Down) {
      if (event.isShiftDown()) {
        oscilloscope_.setTriggerThreshold(std::max(-1.0f, oscilloscope_.triggerThreshold() - 0.05f));
      }
      else {
        bloom_intensity_ = std::max(0.0f, bloom_intensity_ - 0.3f);
        bloom_.setBloomIntensity(bloom_intensity_);
      }
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Comma) {
      // Step back in time
      if (audio_player_.isPaused()) {
        int samples = event.isShiftDown() ? -4410 : -441;
        audio_player_.step(samples);
      }
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Period) {
      // Step forward in time
      if (audio_player_.isPaused()) {
        int samples = event.isShiftDown() ? 4410 : 441;
        audio_player_.step(samples);
      }
      return true;
    }
    return false;
  }

private:
  Oscilloscope oscilloscope_;
  ControlPanel control_panel_;
  ModeSelector mode_selector_;
  ModeSelector waveform_selector_;
  ModeSelector split_selector_;
  ToggleSwitch filter_switch_ { "Filter" };
  ToggleSwitch split_switch_ { "Split" };
  ToggleSwitch speaker_switch_ { "Speaker" };
  FilterJoystick filter_joystick_;
  FilterKnob cutoff_knob_ { "Cutoff", true };  // logarithmic
  FilterKnob resonance_knob_ { "Resonance", false };  // linear
  NumericDisplay cutoff_display_ { "Hz" };
  NumericDisplay resonance_display_ { "%" };
  FilterKnob freq_knob_ { "Freq", true };  // logarithmic
  FilterKnob detune_knob_ { "Detune", false };  // linear (around 1.0)
  NumericDisplay freq_display_ { "Hz" };
  NumericDisplay detune_display_ { "x" };
  HelpOverlay help_overlay_;
  visage::BloomPostEffect bloom_;
  AudioPlayer audio_player_;
  float bloom_intensity_ = 0.2f;
  bool bloom_enabled_ = true;
  float filter_cutoff_ = 150.0f;
  float filter_resonance_ = 0.7f;
  float filter_resonance_pct_ = 70.0f;  // For display (0-100%)
  float signal_freq_ = 80.0f;
  float signal_detune_ = 1.003f;
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
