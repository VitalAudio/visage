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
#include <visage_ui/scroll_bar.h>

// Global constants for parameter ranges
static constexpr float kMinFilterCutoff = 20.0f;
static constexpr float kMaxFilterCutoff = 2000.0f;
static constexpr float kMinFilterResonance = 0.0f;
static constexpr float kMaxFilterResonance = 1.0f;  // Note: UI limits to 0.99f for safety
static constexpr float kMinSplitAngle = -180.0f;
static constexpr float kMaxSplitAngle = 180.0f;
static constexpr float kMinSplitDepth = 0.0f;
static constexpr float kMaxSplitDepth = 1.0f;
static constexpr float kMinVolume = 0.0f;
static constexpr float kMaxVolume = 1.0f;
static constexpr float kMinPostScale = 0.1f;
static constexpr float kMaxPostScale = 3.0f;
static constexpr float kMinPostRotate = 0.0f;
static constexpr float kMaxPostRotate = 360.0f;
static constexpr float kMinPreGain = 0.0f;
static constexpr float kMaxPreGain = 4.0f;

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
    drawKey("P", "Toggle phosphor");
    drawKey("G", "Toggle grid");
    drawKey("Up/Down", "Adjust bloom intensity");
    drawKey(", / .", "Step back / forward (when frozen)");
    y += 10;

    drawSection("Audio");
    drawKey("Space", "Play/pause audio");
    drawKey("V/Shift+V", "Adjust volume");
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

    drawSection("XY Mode - Oscillator");
    drawKey("T", "Toggle test signal");
    drawKey("W", "Toggle square mode");
    drawKey("B/Shift+B", "Adjust beta");
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
// Vintage control panel background
class ControlPanel : public visage::ScrollableFrame {
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
  }
};

// Section frame with a subtle border and title for modular hardware look
class SectionFrame : public visage::Frame {
public:
  SectionFrame(const std::string& title) : title_(title) { setIgnoresMouseEvents(true, true); }

  void draw(visage::Canvas& canvas) override {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());

    // Subtle box border - metallic/etched look
    canvas.setColor(visage::Color(1.0f, 0.15f, 0.15f, 0.15f));
    canvas.rectangleBorder(2, 2, w - 4, h - 4, 1.0f);

    // Etched highlight
    canvas.setColor(visage::Color(0.2f, 1.0f, 1.0f, 1.0f));
    canvas.fill(2, 2, w - 4, 1);
    canvas.fill(2, 2, 1, h - 4);

    // Title label background bit (cut into the border)
    visage::Font font(10, resources::fonts::Lato_Regular_ttf);
    visage::String title_str(title_);
    float text_w = font.stringWidth(title_str.c_str(), title_str.length());
    canvas.setColor(visage::Color(0.95f, 0.06f, 0.07f, 0.08f));  // Match panel bg
    canvas.fill(12, 0, text_w + 8, 5);

    // Title text
    canvas.setColor(visage::Color(1.0f, 0.5f, 0.8f, 0.8f));
    canvas.text(title_, font, visage::Font::kTopLeft, 16, -2, text_w, 12);
  }

private:
  std::string title_;
};

#if VISAGE_EMSCRIPTEN
#include <SDL2/SDL.h>
#else
#include <portaudio.h>

// Helper structure for PortAudio initialization
struct PortAudioRAII {
  PortAudioRAII() { Pa_Initialize(); }
  ~PortAudioRAII() { Pa_Terminate(); }
};
static PortAudioRAII pa_raii;
#endif

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

  float readLeftAt(size_t index) const { return left_[index & (kSize - 1)]; }
  float readRightAt(size_t index) const { return right_[index & (kSize - 1)]; }

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

  ~AudioPlayer() {
#if !VISAGE_EMSCRIPTEN
    // Ramp down audio before stopping to prevent pops/clicks
    if (is_playing_ && stream_) {
      shutting_down_ = true;
      // Wait for gain to ramp down (max ~50ms at 44.1kHz)
      for (int i = 0; i < 100 && current_gain_ > 0.001f; ++i) {
        Pa_Sleep(1);
      }
    }
#endif
    stop();
  }

  AudioPlayer() {
    // Initialize filter defaults
    svf_.setSampleRate(44100.0);
    svf_.setCutoff(150.0);
    svf_.setResonance(0.7);
    stereo_router_.setSampleRate(44100.0);
    stereo_router_.setCutoff(150.0);
    stereo_router_.setResonance(0.7);
    stereo_router_.setSplitMode(StereoFilterRouter::SplitMode::LpHp);
#if VISAGE_EMSCRIPTEN
    volume_ = 0.0f;
#else
    volume_ = 0.5f;
#endif
    paused_ = false;
    current_gain_ = 0.0f;
  }

  bool load(const std::string& path) {
    if (!audio_data_.loadWav(path))
      return false;
    play_position_ = 0;
    return true;
  }

  void setTestGenerator(TestSignalGenerator* gen) { test_generator_ = gen; }

  void play() {
#if !VISAGE_EMSCRIPTEN
    if (is_playing_)
      return;
#endif

    if (audio_data_.left.empty() && !test_generator_)
      return;

#if VISAGE_EMSCRIPTEN
    if (is_playing_) {
      if (device_ != 0)
        SDL_PauseAudioDevice(device_, 0);
      return;
    }
    static bool sdl_inited = false;
    if (!sdl_inited) {
      if (SDL_Init(SDL_INIT_AUDIO) < 0)
        return;
      sdl_inited = true;
    }
    SDL_AudioSpec want, have;
    SDL_memset(&want, 0, sizeof(want));
    want.freq = audio_data_.left.empty() ? 44100 : audio_data_.sample_rate;
    want.format = AUDIO_F32;
    want.channels = 2;
    want.samples = 1024;  // Larger buffer for stability on web
    want.callback = sdlCallback;
    want.userdata = this;

    device_ = SDL_OpenAudioDevice(nullptr, 0, &want, &have, SDL_AUDIO_ALLOW_FREQUENCY_CHANGE);
    if (device_ == 0)
      return;

    sample_rate_ = have.freq;

    SDL_PauseAudioDevice(device_, 0);
    is_playing_ = true;
#else
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
#endif
  }

  void stop() {
    if (!is_playing_)
      return;

#if VISAGE_EMSCRIPTEN
    if (device_ != 0) {
      SDL_CloseAudioDevice(device_);
      device_ = 0;
    }
#else
    if (!stream_)
      return;

    // We don't wait for ramp here because target_gain_ is managed by UI
    Pa_StopStream(stream_);
    Pa_CloseStream(stream_);
    stream_ = nullptr;
#endif
    is_playing_ = false;
  }

  bool isPlaying() const { return is_playing_; }

  bool isPaused() const { return paused_; }
  void setPaused(bool p) {
    paused_ = p;
    if (test_generator_)
      test_generator_->setPaused(p);
  }

  void getCurrentSamples(std::vector<float>& left, std::vector<float>& right, int num_samples) {
    ring_buffer_.read(left, right, num_samples);
  }

  // Step forward or backward in time (only used when frozen)
  void step(int samples) {
    if (audio_data_.left.empty()) {
      if (test_generator_) {
        // Shift phase to new position - history window
        test_generator_->advance(samples - kSweepSamples);

        // Regenerate history window to update visualization
        for (int i = 0; i < kSweepSamples; ++i) {
          float l, r;
          test_generator_->getSample(l, r);

          if (filter_enabled_) {
            // Process both channels through filter with unified morpher
            double mono = (static_cast<double>(l) + static_cast<double>(r)) * 0.5;
            auto outputs = svf_.process(mono);
            auto xy = morpher_.applyXY(outputs.lp, outputs.bp, outputs.hp);
            l = static_cast<float>(xy.x);
            r = static_cast<float>(xy.y);
          }
          ring_buffer_.write(l, r);
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

  // Read triggered samples from the audio thread's latest discovered trigger point
  bool getTriggeredSamples(std::vector<float>& out, int num_samples) {
    size_t write_pos = ring_buffer_.writePos();
    size_t trigger_pos = latest_trigger_pos_.load(std::memory_order_acquire);

    // If no trigger yet, or it's too old, just free run from the end
    if (trigger_pos == 0 || (write_pos - trigger_pos) > RingBuffer::kSize / 2) {
      out.resize(num_samples);
      for (int i = 0; i < num_samples; ++i)
        out[i] = ring_buffer_.readLeft(-num_samples + i);
      return false;
    }

    out.resize(num_samples);
    for (int i = 0; i < num_samples; ++i) {
      // Use the stored trigger position as the start of our sweep
      size_t idx = (trigger_pos + i) & (RingBuffer::kSize - 1);
      out[i] = ring_buffer_.readLeftAt(idx);
    }

    return true;
  }

  void setTriggerThreshold(float v) { trigger_threshold_.store(v); }
  void setTriggerRising(bool v) { trigger_rising_.store(v); }
  void setTriggerLock(bool v) { trigger_lock_.store(v); }
  void startShutdown() { shutting_down_ = true; }
  bool hasAudio() const { return !audio_data_.left.empty(); }
  int sampleRate() const { return audio_data_.sample_rate; }

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
#if VISAGE_EMSCRIPTEN
    return 0;
#else
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
#endif
  }

  bool useSpeaker() const { return use_speaker_; }

  void setVolume(float v) { volume_ = std::clamp(v, 0.0f, 1.0f); }
  float volume() const { return volume_; }

private:
#if VISAGE_EMSCRIPTEN
  static void sdlCallback(void* userData, Uint8* stream, int len) {
    auto* player = static_cast<AudioPlayer*>(userData);
    float* out = reinterpret_cast<float*>(stream);
    int num_samples = len / (2 * sizeof(float));
    player->process(out, num_samples);
  }
#else
  static int paCallback(const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags,
                        void* userData) {
    auto* player = static_cast<AudioPlayer*>(userData);
    float* out = static_cast<float*>(outputBuffer);
    player->process(out, framesPerBuffer);
    return paContinue;
  }
#endif

  void process(float* out, unsigned long framesPerBuffer) {
    size_t total = audio_data_.left.size();

    const float target_gain = (paused_ || shutting_down_) ? 0.0f : volume_.load();
    const float ramp_inc = 1.0f / (0.050f * sample_rate_);

    for (unsigned int i = 0; i < framesPerBuffer; ++i) {
      if (current_gain_ < target_gain)
        current_gain_ = std::min(target_gain, current_gain_ + ramp_inc);
      else if (current_gain_ > target_gain)
        current_gain_ = std::max(target_gain, current_gain_ - ramp_inc);

      bool is_running = !paused_ || current_gain_ > 0.0f;

      float l = 0.0f, r = 0.0f;
      float speaker_l = 0.0f, speaker_r = 0.0f;

      if (is_running) {
        if (total > 0) {
          size_t idx = play_position_ % total;
          l = audio_data_.left[idx];
          r = audio_data_.stereo ? audio_data_.right[idx] : audio_data_.left[idx];
          play_position_++;
        }
        else if (test_generator_) {
          test_generator_->getSample(l, r);
        }

        // For scope visualization: X = raw, Y = filtered (unless split mode)
        float scope_l = l;
        float scope_r = r;

        // Speaker output values (default to raw if filter disabled)
        speaker_l = l;
        speaker_r = r;

        if (filter_enabled_) {
          // Process GLOBAL filter once per sample (preserving state)
          double mono = (static_cast<double>(l) + static_cast<double>(r)) * 0.5;
          auto outputs = svf_.process(mono);

          // 1. Calculate SCOPE values
          if (morpher_.hasSplit()) {
            // Split mode: both X and Y get offset-based filtering
            auto xy = morpher_.applyXY(outputs.lp, outputs.bp, outputs.hp);
            scope_l = static_cast<float>(xy.x);
            scope_r = static_cast<float>(xy.y);
          }
          else {
            // No split: X = raw, Y = filtered with morpher
            scope_r = static_cast<float>(morpher_.apply(outputs.lp, outputs.bp, outputs.hp));
          }

          // 2. Calculate SPEAKER values (always filtered)
          // Note: If no split, applyXY returns identical L/R based on main position
          auto xy_speaker = morpher_.applyXY(outputs.lp, outputs.bp, outputs.hp);
          speaker_l = static_cast<float>(xy_speaker.x);
          speaker_r = static_cast<float>(xy_speaker.y);
        }

        if (!paused_) {
          // Write scope samples to ring buffer (X=raw, Y=filtered or split)
          ring_buffer_.write(scope_l, scope_r);

          // Trigger detection in audio thread
          float thresh = trigger_threshold_.load(std::memory_order_relaxed);
          bool rising = trigger_rising_.load(std::memory_order_relaxed);
          bool crossed = rising ? (prev_trigger_l_ <= thresh && scope_l > thresh) :
                                  (prev_trigger_l_ >= thresh && scope_l < thresh);

          bool find_trigger = (trigger_holdoff_ >= kSweepSamples);

          if (crossed && find_trigger) {
            // Waveform locking: find best match with reference
            bool lock_enabled = trigger_lock_.load(std::memory_order_relaxed);
            bool triggered = true;

            if (lock_enabled && !reference_buffer_.empty()) {
              // Standard cross-correlation score
              float score = 0.0f;
              size_t pos = ring_buffer_.writePos();
              for (int j = 0; j < (int)reference_buffer_.size(); ++j) {
                score += ring_buffer_.readLeftAt(pos + j) * reference_buffer_[j];
              }
            }

            if (triggered) {
              latest_trigger_pos_.store(ring_buffer_.writePos() & (RingBuffer::kSize - 1),
                                        std::memory_order_release);
              trigger_holdoff_ = 0;

              // Update reference occasionally
              if (reference_buffer_.size() != kSweepSamples)
                reference_buffer_.resize(kSweepSamples);
              for (int j = 0; j < kSweepSamples; ++j)
                reference_buffer_[j] = ring_buffer_.readLeftAt((latest_trigger_pos_.load() + j) &
                                                               (RingBuffer::kSize - 1));
            }
          }

          prev_trigger_l_ = scope_l;
          trigger_holdoff_++;
        }
      }

      out[i * 2] = speaker_l * current_gain_;
      out[i * 2 + 1] = speaker_r * current_gain_;
    }
  }

#if VISAGE_EMSCRIPTEN
  SDL_AudioDeviceID device_ = 0;
#else
  PaStream* stream_ = nullptr;
#endif

  AudioData audio_data_;
  RingBuffer ring_buffer_;
  size_t play_position_ = 0;
  bool is_playing_ = false;

  // Trigger state (audio thread)
  float prev_trigger_l_ = 0.0f;
  int trigger_holdoff_ = 0;
  std::vector<float> reference_buffer_;
  std::atomic<size_t> latest_trigger_pos_ { 0 };
  std::atomic<float> trigger_threshold_ { 0.0f };
  std::atomic<bool> trigger_rising_ { true };
  std::atomic<bool> trigger_lock_ { true };

public:
  // Filter controls
  void setFilterCutoff(float fc) {
    filter_cutoff_ = std::clamp(fc, kMinFilterCutoff, kMaxFilterCutoff);
    svf_.setCutoff(filter_cutoff_);
    stereo_router_.setCutoff(filter_cutoff_);
  }
  float filterCutoff() const { return filter_cutoff_; }

  void setFilterResonance(float r) {
    filter_resonance_ = std::clamp(r, kMinFilterResonance, kMaxFilterResonance);
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
  std::atomic<bool> filter_enabled_ { true };
  std::atomic<bool> stereo_split_mode_ { false };
  std::atomic<float> pre_gain_ { 1.0f };

  // RPM controls
  void setBeta(float b) {
    if (test_generator_)
      test_generator_->setBeta(b);
  }
  void setExponent(int e) {
    if (test_generator_)
      test_generator_->setExponent(e);
  }

  void setPreGain(float v) {
    pre_gain_ = v;
    svf_.setPreGain(v);
    stereo_router_.setPreGain(v);
  }
  float preGain() const { return pre_gain_; }

  bool use_speaker_ = false;
  TestSignalGenerator* test_generator_ = nullptr;
  float current_gain_ = 0.0f;
  std::atomic<bool> paused_ { false };
  std::atomic<bool> shutting_down_ { false };
  std::atomic<float> volume_ { 0.0f };
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

  Oscilloscope() {
    setIgnoresMouseEvents(true, false);
    // Demo mode: synthetic waveform if nothing else
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
  void setWaveformHue(float hue) { waveform_hue_ = hue; }
  void setPostScale(float scale) { post_scale_ = scale; }
  void setPostRotate(float rotate) { post_rotate_ = rotate; }
  void setGridEnabled(bool enabled) { grid_enabled_ = enabled; }
  bool gridEnabled() const { return grid_enabled_; }

  void setDisplayMode(DisplayMode mode) { display_mode_ = mode; }
  DisplayMode displayMode() const { return display_mode_; }
  void cycleDisplayMode() {
    switch (display_mode_) {
    case DisplayMode::TimeFree: display_mode_ = DisplayMode::TimeTrigger; break;
    case DisplayMode::TimeTrigger: display_mode_ = DisplayMode::XY; break;
    case DisplayMode::XY: display_mode_ = DisplayMode::TimeFree; break;
    }
  }

  void setTriggerThreshold(float thr) {
    trigger_threshold_ = thr;
    if (audio_player_)
      audio_player_->setTriggerThreshold(thr);
  }
  float triggerThreshold() const { return trigger_threshold_; }
  void setTriggerRising(bool rising) {
    trigger_rising_ = rising;
    if (audio_player_)
      audio_player_->setTriggerRising(rising);
  }
  bool triggerRising() const { return trigger_rising_; }
  void setWaveformLock(bool lock) {
    waveform_lock_ = lock;
    if (audio_player_)
      audio_player_->setTriggerLock(lock);
  }
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

  void step() { needs_step_update_ = true; }

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
  TestSignalGenerator& testSignal() {
    if (audio_player_ && audio_player_->test_generator_)
      return *audio_player_->test_generator_;
    return test_signal_;
  }
  void setTestSignalEnabled(bool enabled) { test_signal_enabled_ = enabled; }
  bool testSignalEnabled() const { return test_signal_enabled_; }

  void generateWaveform(double time, std::vector<Sample>& samples) {
    // Note: This now generates NORMALIZED coordinates (-1 to 1 range typically)
    // they are mapped to screen pixels in renderWaveform.

    // Use audio player data for all primary modes
    if (audio_player_ && audio_player_->isPlaying()) {
      if (display_mode_ == DisplayMode::XY) {
        const int num_samples = 512;
        std::vector<float> left, right;
        audio_player_->getCurrentSamples(left, right, num_samples);

        samples.resize(num_samples);
        for (int i = 0; i < num_samples; ++i) {
          // Store raw signal values (clamped for safety)
          samples[i].x = std::isfinite(left[i]) ? std::clamp(left[i], -2.0f, 2.0f) : 0.0f;
          samples[i].y = std::isfinite(right[i]) ? std::clamp(right[i], -2.0f, 2.0f) : 0.0f;
        }
        return;
      }
      else if (display_mode_ == DisplayMode::TimeTrigger) {
        const int num_samples = 512;
        std::vector<float> audio;
        audio_player_->getTriggeredSamples(audio, num_samples);

        samples.resize(num_samples);
        for (int i = 0; i < num_samples; ++i) {
          samples[i].x = static_cast<float>(i) / (num_samples - 1);
          samples[i].y = audio[i];
        }
        return;
      }
      else if (display_mode_ == DisplayMode::TimeFree) {
        const int num_samples = 512;
        std::vector<float> left, right;
        audio_player_->getCurrentSamples(left, right, num_samples);

        samples.resize(num_samples);
        for (int i = 0; i < num_samples; ++i) {
          samples[i].x = static_cast<float>(i) / (num_samples - 1);
          samples[i].y = left[i];
        }
        return;
      }
    }

    // If audio isn't playing, use the test signal for visualization feedback
    if (test_signal_enabled_) {
      const int num_samples = 512;
      samples.resize(num_samples);
      TestSignalGenerator& gen = testSignal();

      for (int i = 0; i < num_samples; ++i) {
        float l, r;
        gen.getSample(l, r);
        l = std::clamp(l, -2.0f, 2.0f);
        r = std::clamp(r, -2.0f, 2.0f);

        if (display_mode_ == DisplayMode::XY) {
          samples[i].x = l;
          samples[i].y = r;
        }
        else {
          samples[i].x = static_cast<float>(i) / (num_samples - 1);
          samples[i].y = l;
        }
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
        samples[i].x = std::sin(phase * 3.0f + time_offset);
        samples[i].y = std::sin(phase * 2.0f + time_offset * 0.7f);
      }
      else {
        float signal = 0.5f * std::sin(phase * freq1 + time_offset);
        signal += 0.25f * std::sin(phase * freq2 + time_offset * 1.3f);
        signal += 0.15f * std::sin(phase * freq3 + time_offset * 0.7f);
        signal *= 0.8f + 0.2f * std::sin(static_cast<float>(time) * 0.5f);
        samples[i].x = t;
        samples[i].y = signal;
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

    auto toPixel = [&](const Sample& s) -> Sample {
      Sample p;
      if (display_mode_ == DisplayMode::XY) {
        float cx = w * 0.5f;
        float cy = h * 0.5f;
        float base_scale = std::min(w, h) * 0.4f * post_scale_;

        float rad = post_rotate_ * kPi / 180.0f;
        float sn = std::sin(rad);
        float cs = std::cos(rad);

        float rx = s.x * cs - s.y * sn;
        float ry = s.x * sn + s.y * cs;

        p.x = cx + rx * base_scale;
        p.y = cy - ry * base_scale;
      }
      else {
        float cy = h * 0.5f;
        float base_scale = h * 0.4f * post_scale_;
        p.x = s.x * w;
        p.y = cy - s.y * base_scale;
      }
      return p;
    };

    const float diag = std::sqrt(w * w + h * h);
    const float ref_energy = 50.0f;
    const float shutter_ratio = 1.0f;
    const float base_gain = (ref_energy * diag) / (shutter_ratio * samples.size() * kOversampleRate);
    const float unit_gain = base_gain * beam_gain_ * alpha_mult;

    const int num_samples = static_cast<int>(samples.size());
    const float half_beam = beam_size_ * 0.5f;

    for (int pos = 0; pos < num_samples - 1; ++pos) {
      Sample p1 = toPixel(samples[pos]);
      Sample p2 = toPixel(samples[pos + 1]);

      float x = p1.x;
      float y = p1.y;
      const float dx = p2.x - x;
      const float dy = p2.y - y;
      const float d = std::sqrt(dx * dx + dy * dy);

      float beta = std::abs(static_cast<float>(testSignal().beta()));
      float step_dist = kMaxDist * (1.0f + 0.2f * beta * beta);

      const int n = std::min(100, std::max(static_cast<int>(std::ceil(d / step_dist)), 1));
      const float nr = 1.0f / static_cast<float>(n);
      const float ix = dx * nr;
      const float iy = dy * nr;
      const float g = unit_gain * nr;
      const int end = (pos == num_samples - 2) ? n + 1 : n;

      for (int k = 0; k < end; ++k) {
        float brightness = std::min(1.0f, g);
        visage::Color color = visage::Color::fromAHSV(brightness * 0.9f, waveform_hue_, 0.85f, brightness);
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
      if (grid_enabled_) {
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
      }

      // Draw trigger level indicator in trigger mode
      if (display_mode_ == DisplayMode::TimeTrigger) {
        float trig_y = h * 0.5f - trigger_threshold_ * h * 0.4f;
        canvas.setColor(visage::Color(1.0f, 0.8f, 0.2f, 0.3f));
        canvas.fill(0, trig_y - 1, 20, 2);
      }

      // Only generate new waveform if not paused (freeze display when paused)
      bool is_paused = testSignal().isPaused();
      if (!is_paused || current_samples_.empty() || needs_step_update_) {
        generateWaveform(time + time_offset_, current_samples_);
        needs_step_update_ = false;
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
  float waveform_hue_ = 170.0f;

  DisplayMode display_mode_ = DisplayMode::XY;
  double time_offset_ = 0.0;
  float trigger_threshold_ = 0.0f;
  bool trigger_rising_ = true;
  bool waveform_lock_ = true;

  // Test signal generator for XY mode without audio
  TestSignalGenerator test_signal_;
  bool test_signal_enabled_ = true;  // Enabled by default

  AudioPlayer* audio_player_ = nullptr;
  bool needs_step_update_ = false;
  float post_scale_ = 1.0f;
  float post_rotate_ = 0.0f;
  bool grid_enabled_ = true;
};

class ExampleEditor;
static ExampleEditor* g_editor = nullptr;

class FadeOutTimer : public visage::EventTimer {
public:
  FadeOutTimer(visage::Frame* editor, std::function<void()> on_complete) :
      editor_(editor), on_complete_(std::move(on_complete)) { }

  void timerCallback() override {
    alpha_ -= 0.04f;
    if (alpha_ <= 0.0f) {
      stopTimer();
      if (on_complete_)
        on_complete_();
    }
    else {
      editor_->setAlphaTransparency(alpha_);
      editor_->redraw();
    }
  }

private:
  visage::Frame* editor_;
  std::function<void()> on_complete_;
  float alpha_ = 1.0f;
};

class ExampleEditor : public visage::ApplicationWindow {
public:
  ExampleEditor() {
    g_editor = this;
    setAcceptsKeystrokes(true);

    addChild(&oscilloscope_);
    oscilloscope_.layout().setMargin(0);
    oscilloscope_.setAudioPlayer(&audio_player_);
    audio_player_.setTestGenerator(&audio_test_signal_);

    // Control panel background (added first)
    addChild(&control_panel_);
    control_panel_.setVisible(true);

    // Add controls to scrollable control panel
    control_panel_.addScrolledChild(&mode_selector_);
    mode_selector_.setLabel("Mode");
    mode_selector_.setOptions({ "Time", "Trig", "XY" });
    mode_selector_.setColor(visage::Color(1.0f, 0.4f, 0.8f, 0.9f));
    mode_selector_.setCallback([this](int i) {
      oscilloscope_.setDisplayMode(static_cast<DisplayMode>(i));
      updatePanelVisibility();
    });

    control_panel_.addScrolledChild(&signal_box_);
    control_panel_.addScrolledChild(&filter_box_);
    control_panel_.addScrolledChild(&display_box_);

    control_panel_.addScrolledChild(&beta_knob_);
    beta_knob_.setValue(&signal_beta_);
    beta_knob_.setRange(TestSignalGenerator::kMinBeta, TestSignalGenerator::kMaxBeta);
    beta_knob_.setColor(visage::Color(1.0f, 0.5f, 0.9f, 0.5f));
    beta_knob_.setCallback([this](float v) { audio_player_.setBeta(v); });

    control_panel_.addScrolledChild(&exponent_switch_);
    exponent_switch_.setValue(false);  // false = 1, true = 2
    exponent_switch_.setColor(visage::Color(1.0f, 0.6f, 0.8f, 0.6f));
    exponent_switch_.setCallback([this](bool v) { audio_player_.setExponent(v ? 2 : 1); });

    control_panel_.addScrolledChild(&pre_gain_knob_);
    pre_gain_knob_.setValue(&pre_gain_val_);
    pre_gain_knob_.setRange(kMinPreGain, kMaxPreGain);
    pre_gain_knob_.setColor(visage::Color(1.0f, 0.9f, 0.6f, 0.5f));
    pre_gain_knob_.setCallback([this](float v) { audio_player_.setPreGain(v); });

    control_panel_.addScrolledChild(&pre_gain_display_);
    pre_gain_display_.setValue(&pre_gain_val_);
    pre_gain_display_.setColor(visage::Color(1.0f, 0.9f, 0.6f, 0.5f));
    pre_gain_display_.setDecimals(1);

    control_panel_.addScrolledChild(&grid_switch_);
    grid_switch_.setValue(true);
    grid_switch_.setColor(visage::Color(1.0f, 0.8f, 0.8f, 0.4f));
    grid_switch_.setCallback([this](bool v) { oscilloscope_.setGridEnabled(v); });

    control_panel_.addScrolledChild(&freq_knob_);
    freq_knob_.setValue(&signal_freq_);
    freq_knob_.setRange(TestSignalGenerator::kMinFrequency, TestSignalGenerator::kMaxFrequency);
    freq_knob_.setColor(visage::Color(1.0f, 0.5f, 0.9f, 0.5f));
    freq_knob_.setCallback([this](float v) { oscilloscope_.testSignal().setFrequency(v); });

    control_panel_.addScrolledChild(&detune_knob_);
    detune_knob_.setValue(&signal_detune_);
    detune_knob_.setRange(TestSignalGenerator::kMinDetune, TestSignalGenerator::kMaxDetune);
    detune_knob_.setColor(visage::Color(1.0f, 0.6f, 0.8f, 0.6f));
    detune_knob_.setCallback([this](float v) { oscilloscope_.testSignal().setDetune(v); });

    control_panel_.addScrolledChild(&freq_display_);
    freq_display_.setValue(&signal_freq_);
    freq_display_.setColor(visage::Color(1.0f, 0.5f, 0.9f, 0.5f));
    freq_display_.setDecimals(0);

    control_panel_.addScrolledChild(&detune_display_);
    detune_display_.setValue(&signal_detune_);
    detune_display_.setColor(visage::Color(1.0f, 0.6f, 0.8f, 0.6f));
    detune_display_.setDecimals(3);

    control_panel_.addScrolledChild(&filter_switch_);
    filter_switch_.setValue(true);
    filter_switch_.setColor(visage::Color(1.0f, 0.4f, 0.9f, 0.9f));
    filter_switch_.setCallback([this](bool v) {
      oscilloscope_.setFilterEnabled(v);
      updateFilterControlsEnabled(v);
    });

    control_panel_.addScrolledChild(&filter_joystick_);
    filter_joystick_.setMorpher(&oscilloscope_.morpher());

    control_panel_.addScrolledChild(&cutoff_knob_);
    cutoff_knob_.setValue(&filter_cutoff_);
    cutoff_knob_.setRange(kMinFilterCutoff, kMaxFilterCutoff);
    cutoff_knob_.setColor(visage::Color(1.0f, 0.4f, 0.9f, 0.9f));
    cutoff_knob_.setCallback([this](float v) { oscilloscope_.setFilterCutoff(v); });

    control_panel_.addScrolledChild(&cutoff_display_);
    cutoff_display_.setValue(&filter_cutoff_);
    cutoff_display_.setColor(visage::Color(1.0f, 0.4f, 0.9f, 0.9f));
    cutoff_display_.setDecimals(0);

    control_panel_.addScrolledChild(&resonance_knob_);
    resonance_knob_.setValue(&filter_resonance_);
    resonance_knob_.setRange(kMinFilterResonance, 1.0f);
    resonance_knob_.setColor(visage::Color(1.0f, 0.9f, 0.6f, 0.4f));
    resonance_knob_.setCallback([this](float v) {
      oscilloscope_.setFilterResonance(v);
      filter_resonance_pct_ = v * 100.0f;
    });

    control_panel_.addScrolledChild(&resonance_display_);
    resonance_display_.setValue(&filter_resonance_pct_);
    resonance_display_.setColor(visage::Color(1.0f, 0.9f, 0.6f, 0.4f));
    resonance_display_.setDecimals(0);

    control_panel_.addScrolledChild(&split_angle_knob_);
    split_angle_knob_.setValue(&split_angle_);
    split_angle_knob_.setRange(kMinSplitAngle, kMaxSplitAngle);
    split_angle_knob_.setColor(visage::Color(1.0f, 0.3f, 0.7f, 0.9f));
    split_angle_knob_.setCallback([this](float v) { oscilloscope_.morpher().setSplitAngle(v); });

    control_panel_.addScrolledChild(&angle_display_);
    angle_display_.setValue(&split_angle_);
    angle_display_.setColor(visage::Color(1.0f, 0.3f, 0.7f, 0.9f));
    angle_display_.setDecimals(0);

    control_panel_.addScrolledChild(&split_depth_knob_);
    split_depth_knob_.setValue(&split_depth_);
    split_depth_knob_.setRange(kMinSplitDepth, kMaxSplitDepth);
    split_depth_knob_.setColor(visage::Color(1.0f, 0.3f, 0.7f, 0.9f));
    split_depth_knob_.setCallback([this](float v) { oscilloscope_.morpher().setSplitDepth(v); });

    control_panel_.addScrolledChild(&depth_display_);
    depth_display_.setValue(&split_depth_);
    depth_display_.setColor(visage::Color(1.0f, 0.3f, 0.7f, 0.9f));
    depth_display_.setDecimals(2);

    control_panel_.addScrolledChild(&post_scale_knob_);
    post_scale_knob_.setValue(&post_scale_val_);
    post_scale_knob_.setRange(kMinPostScale, kMaxPostScale);
    post_scale_knob_.setColor(visage::Color(1.0f, 0.5f, 0.7f, 0.9f));
    post_scale_knob_.setCallback([this](float v) { oscilloscope_.setPostScale(v); });

    control_panel_.addScrolledChild(&post_rotate_knob_);
    post_rotate_knob_.setValue(&post_rotate_val_);
    post_rotate_knob_.setRange(kMinPostRotate, kMaxPostRotate);
    post_rotate_knob_.setColor(visage::Color(1.0f, 0.5f, 0.7f, 0.9f));
    post_rotate_knob_.setCallback([this](float v) { oscilloscope_.setPostRotate(v); });

    control_panel_.addScrolledChild(&volume_knob_);
    volume_knob_.setValue(&volume_val_);
    volume_knob_.setRange(kMinVolume, kMaxVolume);
    volume_knob_.setColor(visage::Color(1.0f, 0.4f, 0.9f, 0.9f));
    volume_knob_.setCallback([this](float v) { audio_player_.setVolume(v); });

    control_panel_.addScrolledChild(&hue_slider_);
    hue_slider_.setValue(&waveform_hue_);
    hue_slider_.setRange(0.0f, 360.0f);
    hue_slider_.setColor(visage::Color::fromAHSV(1.0f, waveform_hue_, 0.85f, 0.9f));
    hue_slider_.setCallback([this](float v) {
      oscilloscope_.setWaveformHue(v);
      hue_slider_.setColor(visage::Color::fromAHSV(1.0f, v, 0.85f, 0.9f));
    });

    control_panel_.addScrolledChild(&bloom_slider_);
    bloom_slider_.setValue(&bloom_intensity_);
    bloom_slider_.setLedIntensity(&bloom_intensity_);
    bloom_slider_.setRange(0.0f, 2.5f);
    bloom_slider_.setColor(visage::Color(1.0f, 1.0f, 0.8f, 0.2f));
    bloom_slider_.setCallback([this](float v) {
      bloom_intensity_ = v;
      bloom_.setBloomIntensity(v);
    });

    // Help overlay (covers entire window)
    addChild(&help_overlay_);

    bloom_.setBloomSize(20.0f);
    bloom_.setBloomIntensity(0.5f);  // Slightly higher default glow
    setPostEffect(&bloom_);

    // Initialize panel visibility
    updatePanelVisibility();

    // Start audio engine immediately (it will be muted by default)
    audio_player_.play();
#if VISAGE_EMSCRIPTEN
    setVolume(0.0f);
#else
    onCloseRequest() = [this] {
      if (shutting_down_)
        return true;

      shutting_down_ = true;
      audio_player_.startShutdown();
      fade_out_timer_.startTimer(10);
      return false;
    };
#endif
    setMinimumDimensions(318.0f, 183.0f);
  }

  void updatePanelVisibility() {
    bool show_panel = true;  // Always show panel
    bool is_xy = oscilloscope_.displayMode() == DisplayMode::XY;

    control_panel_.setVisible(show_panel);
    mode_selector_.setVisible(show_panel);
    grid_switch_.setVisible(show_panel);

    // XY-specific controls
    filter_joystick_.setVisible(is_xy);
    cutoff_knob_.setVisible(is_xy);
    resonance_knob_.setVisible(is_xy);
    cutoff_display_.setVisible(is_xy);
    resonance_display_.setVisible(is_xy);
    beta_knob_.setVisible(is_xy);
    exponent_switch_.setVisible(is_xy);
    pre_gain_knob_.setVisible(is_xy);
    pre_gain_display_.setVisible(is_xy);
    freq_knob_.setVisible(is_xy);
    detune_knob_.setVisible(is_xy);
    freq_display_.setVisible(is_xy);
    detune_display_.setVisible(is_xy);
    split_angle_knob_.setVisible(is_xy);
    split_depth_knob_.setVisible(is_xy);
    angle_display_.setVisible(is_xy);
    depth_display_.setVisible(is_xy);
    filter_switch_.setVisible(is_xy);
    post_scale_knob_.setVisible(is_xy);
    post_rotate_knob_.setVisible(is_xy);
    volume_knob_.setVisible(show_panel);
    hue_slider_.setVisible(show_panel);
    bloom_slider_.setVisible(show_panel);

    signal_box_.setVisible(is_xy);
    filter_box_.setVisible(is_xy);
    display_box_.setVisible(show_panel);

    // Update enabled state based on filter switch
    updateFilterControlsEnabled(filter_switch_.value());

    resized();
  }

  void updateFilterControlsEnabled(bool enabled) {
    // Dim/enable filter-related controls based on filter switch
    filter_joystick_.setEnabled(enabled);
    cutoff_knob_.setEnabled(enabled);
    resonance_knob_.setEnabled(enabled);
    cutoff_display_.setEnabled(enabled);
    resonance_display_.setEnabled(enabled);
    split_angle_knob_.setEnabled(enabled);
    split_depth_knob_.setEnabled(enabled);
    angle_display_.setEnabled(enabled);
    depth_display_.setEnabled(enabled);
  }

  void resized() override {
    // Vintage control panel on right side (always visible)
    const int panel_width = 140;
    const int margin = 4;
    const int knob_size = 60;  // Taller to fit label
    const int js_size = 90;  // Taller to fit label
    const int selector_h = 40;  // Taller for multi-switch with LEDs
    const int btn_size = 24;  // Half size buttons
    const int margin_x = 10;
    const int switch_h = 42;  // Square push button height (needs room for LED + button + label)

    int panel_x = width() - panel_width;
    control_panel_.setBounds(panel_x, 0, panel_width, height());

    int y = margin;
    int inner_width = panel_width - 20;

    // Mode selector at top (always visible)
    mode_selector_.setBounds(10, y, inner_width, selector_h);
    y += selector_h + margin + 8;

    // XY mode controls
    if (oscilloscope_.displayMode() == DisplayMode::XY) {
      // --- SIGNAL SECTION ---
      int signal_start_y = y;
      y += 12;  // Title padding

      // Beta, Sq switch, and Gain side by side
      int small_knob = 50;
      int extra_small_btn = 20;
      int left_x = 10 + margin;
      int right_x = panel_width - 10 - margin - small_knob;
      int mid_x = (panel_width - extra_small_btn) / 2;

      beta_knob_.setBounds(left_x, y, small_knob, small_knob);
      exponent_switch_.setBounds(mid_x, y + (small_knob - extra_small_btn) / 2, extra_small_btn,
                                 extra_small_btn);
      pre_gain_knob_.setBounds(right_x, y, small_knob, small_knob);
      y += small_knob + 2;

      // Indicator below Gain
      pre_gain_display_.setBounds(right_x, y, small_knob, 14);
      y += 14 + margin;

      // Signal freq and detune knobs
      int small_display_h = 16;
      int left_x = 10 + margin;
      int right_x = panel_width - 10 - margin - small_knob;
      freq_knob_.setBounds(left_x, y, small_knob, small_knob);
      detune_knob_.setBounds(right_x, y, small_knob, small_knob);
      y += small_knob + 2;
      freq_display_.setBounds(left_x, y, small_knob, small_display_h);
      detune_display_.setBounds(right_x, y, small_knob, small_display_h);
      y += small_display_h + 10;

      signal_box_.setBounds(5, signal_start_y, panel_width - 10, y - signal_start_y);
      y += 12;

      // --- FILTER SECTION ---
      int filter_start_y = y;
      y += 12;

      // Joystick and Filter Switch
      int js_x = (panel_width - js_size) / 2;
      filter_joystick_.setBounds(js_x, y, js_size, js_size);

      // Filter switch at upper right of the joystick frame
      filter_switch_.setBounds(js_x + js_size - btn_size, y, btn_size, btn_size);

      y += js_size + margin;

      // Cutoff and Resonance knobs
      int left_f_x = 10 + margin;
      int right_f_x = panel_width - 10 - margin - small_knob;
      cutoff_knob_.setBounds(left_f_x, y, small_knob, small_knob);
      resonance_knob_.setBounds(right_f_x, y, small_knob, small_knob);
      y += small_knob + 2;
      cutoff_display_.setBounds(left_f_x, y, small_knob, small_display_h);
      resonance_display_.setBounds(right_f_x, y, small_knob, small_display_h);
      y += small_display_h + 8;

      // Split angle and depth
      split_angle_knob_.setBounds(left_f_x, y, small_knob, small_knob);
      split_depth_knob_.setBounds(right_f_x, y, small_knob, small_knob);
      y += small_knob + 2;
      angle_display_.setBounds(left_f_x, y, small_knob, small_display_h);
      depth_display_.setBounds(right_f_x, y, small_knob, small_display_h);
      y += small_display_h + 10;

      filter_box_.setBounds(5, filter_start_y, panel_width - 10, y - filter_start_y);
      y += 12;
    }

    // --- DISPLAY / OUTPUT SECTION ---
    int display_start_y = y;
    y += 12;

    if (oscilloscope_.displayMode() == DisplayMode::XY) {
      // Small post-filter scale and rotate knobs with Grid button in between
      int extra_small_knob = 40;
      int extra_left_x = 10 + margin;
      int extra_right_x = panel_width - 10 - margin - extra_small_knob;
      post_scale_knob_.setBounds(extra_left_x, y, extra_small_knob, extra_small_knob);
      post_rotate_knob_.setBounds(extra_right_x, y, extra_small_knob, extra_small_knob);
      grid_switch_.setBounds((panel_width - btn_size) / 2, y + (extra_small_knob - btn_size) / 2,
                             btn_size, btn_size);
      y += extra_small_knob + 10;
    }
    else {
      // Grid switch alone centered if not in XY
      grid_switch_.setBounds((panel_width - btn_size) / 2, y, btn_size, btn_size);
      y += btn_size + 10;
    }

    int slider_w = 40;
    int slider_h = 100;
    int sliders_x = (panel_width - (slider_w * 2 + margin)) / 2;
    hue_slider_.setBounds(sliders_x, y, slider_w, slider_h);
    bloom_slider_.setBounds(sliders_x + slider_w + margin, y, slider_w, slider_h);
    y += slider_h + 8;

    volume_knob_.setBounds((panel_width - knob_size) / 2, y, knob_size, knob_size);
    y += knob_size + 10;

    display_box_.setBounds(5, display_start_y, panel_width - 10, y - display_start_y);
    y += 10;

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
    if (event.keyCode() == visage::KeyCode::P) {
      setPhosphorEnabled(!phosphorEnabled());
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::H ||
             (event.isShiftDown() && event.keyCode() == visage::KeyCode::Slash)) {
      help_overlay_.toggle();
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::G) {
      oscilloscope_.setGridEnabled(!oscilloscope_.gridEnabled());
      grid_switch_.setValue(oscilloscope_.gridEnabled());
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
      // Toggle exponent
      bool next = !exponent_switch_.value();
      exponent_switch_.setValue(next);
      audio_player_.setExponent(next ? 2 : 1);
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::T) {
      // Toggle test signal
      oscilloscope_.setTestSignalEnabled(!oscilloscope_.testSignalEnabled());
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Left) {
      // Decrease cutoff
      filter_cutoff_ = std::max(kMinFilterCutoff, filter_cutoff_ * 0.9f);
      oscilloscope_.setFilterCutoff(filter_cutoff_);
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Right) {
      // Increase cutoff
      filter_cutoff_ = std::min(kMaxFilterCutoff, filter_cutoff_ * 1.1f);
      oscilloscope_.setFilterCutoff(filter_cutoff_);
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Space) {
      // Toggle pause state (Freezes waveform and audio)
      audio_player_.setPaused(!audio_player_.isPaused());
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Up) {
      if (event.isShiftDown()) {
        // Adjust trigger threshold
        oscilloscope_.setTriggerThreshold(std::min(1.0f, oscilloscope_.triggerThreshold() + 0.05f));
      }
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Down) {
      if (event.isShiftDown()) {
        // Adjust trigger threshold
        oscilloscope_.setTriggerThreshold(std::max(-1.0f, oscilloscope_.triggerThreshold() - 0.05f));
      }
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Comma) {
      // Step back in time
      if (audio_player_.isPaused()) {
        int samples = event.isShiftDown() ? -4410 : -441;
        audio_player_.step(samples);
        oscilloscope_.step();
      }
      return true;
    }
    else if (event.keyCode() == visage::KeyCode::Period) {
      // Step forward in time
      if (audio_player_.isPaused()) {
        int samples = event.isShiftDown() ? 4410 : 441;
        audio_player_.step(samples);
        oscilloscope_.step();
      }
      return true;
    }
    return false;
  }

  void setVolume(float v) {
    volume_val_ = std::clamp(v, kMinVolume, kMaxVolume);
    audio_player_.setVolume(volume_val_);
    volume_knob_.redraw();
  }

  void setFilterCutoff(float fc) {
    filter_cutoff_ = std::clamp(fc, kMinFilterCutoff, kMaxFilterCutoff);
    oscilloscope_.setFilterCutoff(filter_cutoff_);
    cutoff_knob_.redraw();
  }

  void setFilterResonance(float r) {
    filter_resonance_ = std::clamp(r, kMinFilterResonance, 0.99f);
    filter_resonance_pct_ = filter_resonance_ * 100.0f;
    oscilloscope_.setFilterResonance(filter_resonance_);
    resonance_knob_.redraw();
  }

  void setWindowSize(int w, int h) { setBounds(0, 0, w, h); }

  void setParam(const std::string& name, float value) {
    if (name == "volume") {
      setVolume(value);
    }
    else if (name == "cutoff") {
      setFilterCutoff(value);
    }
    else if (name == "resonance") {
      setFilterResonance(value);
    }
    else if (name == "freq") {
      signal_freq_ = std::clamp(value, static_cast<float>(TestSignalGenerator::kMinFrequency),
                                static_cast<float>(TestSignalGenerator::kMaxFrequency));
      oscilloscope_.testSignal().setFrequency(signal_freq_);
    }
    else if (name == "detune") {
      signal_detune_ = std::clamp(value, static_cast<float>(TestSignalGenerator::kMinDetune),
                                  static_cast<float>(TestSignalGenerator::kMaxDetune));
      oscilloscope_.testSignal().setDetune(signal_detune_);
    }
    else if (name == "beta") {
      signal_beta_ = std::clamp(value, static_cast<float>(TestSignalGenerator::kMinBeta),
                                static_cast<float>(TestSignalGenerator::kMaxBeta));
      audio_player_.setBeta(signal_beta_);
    }
    else if (name == "exponent") {
      audio_player_.setExponent(value > 1.5f ? 2 : 1);
    }
    else if (name == "pregain") {
      pre_gain_val_ = std::clamp(value, kMinPreGain, kMaxPreGain);
      audio_player_.setPreGain(pre_gain_val_);
      pre_gain_knob_.redraw();
    }
    else if (name == "bloom") {
      bloom_intensity_ = std::clamp(value, 0.0f, 2.5f);
      bloom_.setBloomIntensity(bloom_intensity_);
    }
    else if (name == "hue") {
      waveform_hue_ = std::clamp(value, 0.0f, 360.0f);
      oscilloscope_.setWaveformHue(waveform_hue_);
    }
    else if (name == "display") {
      int m = std::clamp(static_cast<int>(value), 0, 2);
      oscilloscope_.setDisplayMode(static_cast<DisplayMode>(m));
      mode_selector_.setIndex(m);
      updatePanelVisibility();
    }
    redraw();
  }

  void mouseDown(const visage::MouseEvent& event) override {
    // Satisfy browser autoplay policy on first click
    audio_player_.play();
  }

private:
  Oscilloscope oscilloscope_;
  ControlPanel control_panel_;
  ModeSelector mode_selector_;
  PushButtonSwitch filter_switch_ { "On" };
  PushButtonSwitch exponent_switch_ { "Sq" };
  PushButtonSwitch grid_switch_ { "Grid" };
  FilterKnob pre_gain_knob_ { "Gain" };
  FilterKnob split_angle_knob_ { "Angle", false, false, true };  // bidirectional
  FilterKnob split_depth_knob_ { "Depth", false, false, false };  // 0 to 1 range
  FilterKnob post_scale_knob_ { "Scale" };
  FilterKnob post_rotate_knob_ { "Rotate" };
  FilterKnob volume_knob_ { "Vol" };
  FilterSlider hue_slider_ { "Hue" };
  FilterSlider bloom_slider_ { "Bloom" };
#if VISAGE_EMSCRIPTEN
  float volume_val_ = 0.0f;
#else
  float volume_val_ = 0.5f;
#endif
  FilterJoystick filter_joystick_;
  FilterKnob cutoff_knob_ { "Cutoff", true };  // logarithmic
  FilterKnob resonance_knob_ { "Resonance", false, false, false, true };  // reverse logarithmic
  FilterKnob beta_knob_ { "Beta", false, true, true };  // bipolar logarithmic, bidirectional
  FilterKnob freq_knob_ { "Freq", true };  // logarithmic
  FilterKnob detune_knob_ { "Detune", false, false, true };  // linear (around 1.0), bidirectional
  NumericDisplay freq_display_ { "Hz" };
  NumericDisplay detune_display_ { "x" };
  NumericDisplay cutoff_display_ { "Hz" };
  NumericDisplay resonance_display_ { "%" };
  NumericDisplay angle_display_ { "°" };
  NumericDisplay depth_display_ { "" };
  NumericDisplay pre_gain_display_ { "x" };
  SectionFrame signal_box_ { "SIGNAL" };
  SectionFrame filter_box_ { "FILTER" };
  SectionFrame display_box_ { "DISPLAY" };
  HelpOverlay help_overlay_;
  visage::BloomPostEffect bloom_;
  TestSignalGenerator audio_test_signal_;
  AudioPlayer audio_player_;
  float pre_gain_val_ = 1.0f;
  float bloom_intensity_ = 0.5f;
  bool bloom_enabled_ = true;
  float waveform_hue_ = 170.0f;
  float filter_cutoff_ = 150.0f;
  float filter_resonance_ = 0.7f;
  float filter_resonance_pct_ = 70.0f;  // For display (0-100%)
  float signal_beta_ = 0.00f;
  float signal_freq_ = 80.0f;
  float signal_detune_ = 1.003f;
  float split_angle_ = 0.0f;  // -180 to +180 degrees
  float split_depth_ = 0.0f;  // -1 to +1
  float post_scale_val_ = 1.0f;
  float post_rotate_val_ = 0.0f;

  bool shutting_down_ = false;
  FadeOutTimer fade_out_timer_ { this, [this] { visage::closeApplication(); } };
};

int runExample() {
  ExampleEditor editor;
  editor.setWindowDecoration(visage::Window::Decoration::Client);

#if VISAGE_EMSCRIPTEN
  editor.showMaximized();
#else
  if (visage::isMobileDevice())
    editor.showMaximized();
  else
    editor.show(900, 550);
#endif

  editor.runEventLoop();

  return 0;
}

#if VISAGE_EMSCRIPTEN
#include <emscripten.h>

extern "C" {
EMSCRIPTEN_KEEPALIVE
void setVolume(float volume) {
  if (g_editor)
    g_editor->setVolume(volume);
}

EMSCRIPTEN_KEEPALIVE
void setParameter(const char* name, float value) {
  if (g_editor && name)
    g_editor->setParam(name, value);
}

EMSCRIPTEN_KEEPALIVE
void resizeWindow(int w, int h) {
  if (g_editor)
    g_editor->setWindowSize(w, h);
}
}
#endif
