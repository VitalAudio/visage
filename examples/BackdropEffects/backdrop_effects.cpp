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
#include "embedded/example_shaders.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <visage/app.h>

namespace {
  constexpr float kPi = 3.14159265358979323846f;

  enum class Kind {
    None,
    Blur,
    Shader
  };

  struct EffectSpec {
    const char* name;
    Kind kind;
    const visage::EmbeddedFile* fragment;
    const char* parameter;
    float min;
    float max;
    float initial;
    float step;
    bool animated;
  };

  const std::vector<EffectSpec>& effectSpecs() {
    using namespace resources::shaders;
    static const std::vector<EffectSpec> kSpecs = {
      { "None", Kind::None, nullptr, nullptr, 0, 0, 0, 0, false },
      { "Blur", Kind::Blur, nullptr, "radius", 1.0f, 300.0f, 30.0f, 1.25f, false },
      { "Gray scale", Kind::Shader, &fs_gray_scale, nullptr, 0, 0, 0, 0, false },
      { "Sepia", Kind::Shader, &fs_sepia, nullptr, 0, 0, 0, 0, false },
      { "Invert", Kind::Shader, &fs_backdrop_invert, nullptr, 0, 0, 0, 0, false },
      { "Posterize", Kind::Shader, &fs_backdrop_posterize, "levels", 2.0f, 16.0f, 4.0f, 1.0f, false },
      { "Pixelate", Kind::Shader, &fs_backdrop_pixelate, "block px", 2.0f, 120.0f, 14.0f, 1.25f, false },
      { "Chromatic aberration", Kind::Shader, &fs_backdrop_chromatic, "spread", 0.005f, 0.3f, 0.04f,
        1.25f, false },
      { "Frost", Kind::Shader, &fs_backdrop_frost, "jitter px", 1.0f, 60.0f, 6.0f, 1.25f, false },
      { "Ripple", Kind::Shader, &fs_backdrop_ripple, "amplitude px", 1.0f, 60.0f, 8.0f, 1.25f, true },
      { "Magnify", Kind::Shader, &fs_backdrop_magnify, "zoom", 1.1f, 6.0f, 1.8f, 1.15f, false },
      { "Hue rotate", Kind::Shader, &fs_backdrop_hue, "speed", 0.1f, 20.0f, 1.5f, 1.25f, true },
      { "Glitch", Kind::Shader, &fs_glitch, nullptr, 0, 0, 0, 0, true },
      { "Rainbow warp", Kind::Shader, &fs_warp, nullptr, 0, 0, 0, 0, true },
    };
    return kSpecs;
  }

  std::string formatAmount(float value) {
    char buffer[32];
    if (value < 0.1f)
      std::snprintf(buffer, sizeof(buffer), "%.3f", value);
    else if (value < 10.0f)
      std::snprintf(buffer, sizeof(buffer), "%.2f", value);
    else
      std::snprintf(buffer, sizeof(buffer), "%.0f", value);
    return buffer;
  }

  const char* kShapeNames[] = { "rounded", "pill", "square" };
  constexpr int kNumShapes = 3;

  int effectIndex(const std::string& name) {
    const auto& specs = effectSpecs();
    for (int i = 0; i < specs.size(); ++i) {
      if (name == specs[i].name)
        return i;
    }
    return 0;
  }

  const visage::Font& labelFont() {
    static const visage::Font font(15, resources::fonts::Lato_Regular_ttf);
    return font;
  }
}

class GlassLayer : public visage::Frame {
public:
  static constexpr float kResizeHandle = 22.0f;

  explicit GlassLayer(int effect) {
    for (const EffectSpec& spec : effectSpecs()) {
      std::unique_ptr<visage::PostEffect> instance;
      if (spec.kind == Kind::Blur)
        instance = std::make_unique<visage::BlurPostEffect>();
      else if (spec.kind == Kind::Shader)
        instance = std::make_unique<visage::ShaderPostEffect>(resources::shaders::vs_custom, *spec.fragment);
      effects_.push_back(std::move(instance));
      amounts_.push_back(spec.initial);
    }

    setAcceptsKeystrokes(true);
    onDrawMask() = [this](visage::Canvas& canvas) {
      canvas.setColor(0xffffffff);
      canvas.roundedRectangle(0, 0, width(), height(), cornerRadius());
    };
    setEffect(effect);
  }

  ~GlassLayer() override { setBackdropEffect(nullptr); }

  int effect() const { return effect_; }
  const EffectSpec& spec() const { return effectSpecs()[effect_]; }
  float amount() const {
    const EffectSpec& s = spec();
    return s.step > 1.0f ? amounts_[effect_] : std::round(amounts_[effect_]);
  }

  void setEffect(int effect) {
    int count = effectSpecs().size();
    effect_ = ((effect % count) + count) % count;
    apply();
  }

  void adjustAmount(float steps) {
    const EffectSpec& s = spec();
    if (s.parameter == nullptr)
      return;
    float& value = amounts_[effect_];
    if (s.step > 1.0f)
      value *= std::pow(s.step, steps);
    else
      value += steps * s.step;
    value = std::clamp(value, s.min, s.max);
    apply();
  }

  void cycleShape() {
    shape_ = (shape_ + 1) % kNumShapes;
    redraw();
  }
  int shape() const { return shape_; }

  void setTinted(bool tinted) {
    tinted_ = tinted;
    redraw();
  }
  bool tinted() const { return tinted_; }

  void setSelected(bool selected) {
    if (selected_ == selected)
      return;
    selected_ = selected;
    redraw();
  }

  void setAnimating(bool animating) {
    animating_ = animating;
    redraw();
  }

  std::string description() const {
    std::string result = spec().name;
    if (spec().parameter)
      result += "   " + std::string(spec().parameter) + " " + formatAmount(amount());
    return result;
  }

  void draw(visage::Canvas& canvas) override {
    float w = width();
    float h = height();
    float radius = cornerRadius();
    if (tinted_) {
      canvas.setColor(0x18ffffff);
      canvas.fill(0, 0, w, h);
      canvas.setColor(visage::Brush::vertical(0x66ffffff, 0x22ffffff));
      canvas.roundedRectangleBorder(0, 0, w, h, radius, 2.0f);
    }
    if (selected_) {
      canvas.setColor(0xffffd040);
      canvas.roundedRectangleBorder(0, 0, w, h, radius, 4.0f);
    }

    float label_height = 26.0f;
    float label_width = std::min(w - 20.0f, 300.0f);
    if (label_width > 40.0f && h > 2.0f * label_height) {
      float label_x = (w - label_width) * 0.5f;
      float label_y = std::min(h - label_height - 10.0f, std::max(10.0f, radius * 0.3f));
      canvas.setColor(0x99000000);
      canvas.roundedRectangle(label_x, label_y, label_width, label_height, label_height * 0.5f);
      canvas.setColor(0xffffffff);
      canvas.text(description(), labelFont(), visage::Font::kCenter, label_x, label_y, label_width,
                  label_height);
    }

    canvas.setColor(0x80ffffff);
    for (int i = 1; i <= 3; ++i) {
      float offset = i * 5.0f + 2.0f;
      canvas.segment(w - offset, h - 4.0f, w - 4.0f, h - offset, 1.5f, true);
    }

    if (animating_ && spec().animated)
      redraw();
  }

  void mouseDown(const visage::MouseEvent& e) override {
    if (on_select_)
      on_select_(this);
    Point position = e.position;
    resizing_ = position.x > width() - kResizeHandle && position.y > height() - kResizeHandle;
    drag_start_ = e.windowPosition();
    start_bounds_ = bounds();
    if (!resizing_)
      setCursorStyle(visage::MouseCursor::Dragging);
  }

  void mouseDrag(const visage::MouseEvent& e) override {
    Point delta = e.windowPosition() - drag_start_;
    if (resizing_) {
      setBounds(start_bounds_.x(), start_bounds_.y(), std::max(80.0f, start_bounds_.width() + delta.x),
                std::max(60.0f, start_bounds_.height() + delta.y));
    }
    else
      setBounds(start_bounds_.x() + delta.x, start_bounds_.y() + delta.y, start_bounds_.width(),
                start_bounds_.height());
  }

  void mouseUp(const visage::MouseEvent& e) override { mouseMove(e); }

  void mouseMove(const visage::MouseEvent& e) override {
    bool corner = e.position.x > width() - kResizeHandle && e.position.y > height() - kResizeHandle;
    setCursorStyle(corner ? visage::MouseCursor::MultiDirectionalResize : visage::MouseCursor::Drag);
  }

  void mouseExit(const visage::MouseEvent& e) override {
    setCursorStyle(visage::MouseCursor::Arrow);
  }

  bool mouseWheel(const visage::MouseEvent& e) override {
    float delta = e.precise_wheel_delta_y != 0.0f ? e.precise_wheel_delta_y : e.wheel_delta_y;
    if (delta == 0.0f)
      return false;

    auto now = std::chrono::steady_clock::now();
    bool new_scroll = now - last_wheel_time_ > kWheelGestureGap;
    last_wheel_time_ = now;

    float steps = delta;
    if (new_scroll && std::abs(delta) < kMinWheelSteps)
      steps = delta > 0.0f ? kMinWheelSteps : -kMinWheelSteps;
    adjustAmount(steps);
    if (on_change_)
      on_change_();
    return true;
  }

  auto& onSelect() { return on_select_; }
  auto& onChange() { return on_change_; }

private:
  using Point = visage::Point;
  static constexpr std::chrono::milliseconds kWheelGestureGap { 200 };
  static constexpr float kMinWheelSteps = 0.25f;

  float cornerRadius() const {
    float min = std::min(width(), height());
    if (shape_ == 0)
      return min * 0.12f;
    if (shape_ == 1)
      return min * 0.5f;
    return 0.0f;
  }

  void apply() {
    visage::PostEffect* effect = effects_[effect_].get();
    float value = amount();
    const EffectSpec& s = spec();
    if (s.kind == Kind::Blur)
      static_cast<visage::BlurPostEffect*>(effect)->setBlurRadius(value);
    else if (s.kind == Kind::Shader)
      static_cast<visage::ShaderPostEffect*>(effect)->setUniformValue("u_amount", value);
    setBackdropEffect(effect);
    redraw();
  }

  std::vector<std::unique_ptr<visage::PostEffect>> effects_;
  std::vector<float> amounts_;
  int effect_ = 0;
  int shape_ = 0;
  bool tinted_ = true;
  bool selected_ = false;
  bool animating_ = true;
  bool resizing_ = false;
  Point drag_start_;
  visage::Bounds start_bounds_;
  std::chrono::steady_clock::time_point last_wheel_time_;
  std::function<void(GlassLayer*)> on_select_;
  std::function<void()> on_change_;
};

class ExampleEditor : public visage::ApplicationWindow {
public:
  ExampleEditor() {
    setAcceptsKeystrokes(true);
    onKeyPress() = [this](const visage::KeyEvent& e) { return handleKey(e); };

    scene_.onDraw() = [this](visage::Canvas& canvas) {
      if (!paused_)
        scene_time_ = canvas.time();
      drawScene(canvas, scene_.width(), scene_.height(), scene_time_);
      if (!paused_)
        scene_.redraw();
    };
    scene_.setIgnoresMouseEvents(true, false);
    addChild(scene_);

    hud_.onDraw() = [this](visage::Canvas& canvas) { drawHud(canvas); };
    hud_.setIgnoresMouseEvents(true, false);
    addChild(hud_);

    addLayer(effectIndex("Blur"), 0.08f, 0.12f, 0.36f, 0.42f);
    addLayer(effectIndex("Pixelate"), 0.32f, 0.36f, 0.34f, 0.4f);
    addLayer(effectIndex("Chromatic aberration"), 0.56f, 0.18f, 0.32f, 0.5f);
    select(0);
  }

  void resized() override {
    scene_.setBounds(0, 0, width(), height());
    hud_.setBounds(0, height() - kHudHeight, width(), kHudHeight);
    if (!placed_) {
      placed_ = true;
      for (auto& layer : layers_)
        layer.frame->setBounds(layer.start_x * width(), layer.start_y * height(),
                               layer.start_w * width(), layer.start_h * height());
    }
  }

  void draw(visage::Canvas& canvas) override {
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, width(), height());
  }

private:
  static constexpr float kHudHeight = 96.0f;

  struct LayerEntry {
    std::unique_ptr<GlassLayer> frame;
    float start_x, start_y, start_w, start_h;
  };

  void addLayer(int effect, float x, float y, float w, float h) {
    auto layer = std::make_unique<GlassLayer>(effect);
    GlassLayer* raw = layer.get();
    raw->onSelect() = [this](GlassLayer* selected) {
      for (int i = 0; i < layers_.size(); ++i) {
        if (layers_[i].frame.get() == selected)
          select(i);
      }
    };
    raw->onChange() = [this] { hud_.redraw(); };
    raw->onKeyPress() = [this](const visage::KeyEvent& e) { return handleKey(e); };
    raw->setAnimating(!paused_);
    addChild(raw);
    if (placed_)
      raw->setBounds(x * width(), y * height(), w * width(), h * height());
    layers_.push_back({ std::move(layer), x, y, w, h });
    keepHudOnTop();
  }

  void keepHudOnTop() {
    removeChild(hud_);
    addChild(hud_);
  }

  void select(int index) {
    if (layers_.empty()) {
      selected_ = -1;
      hud_.redraw();
      return;
    }
    selected_ = std::clamp(index, 0, static_cast<int>(layers_.size()) - 1);
    for (int i = 0; i < layers_.size(); ++i)
      layers_[i].frame->setSelected(i == selected_);
    hud_.redraw();
  }

  const char* layerPosition(int index) const {
    if (index == layers_.size() - 1)
      return "top";
    if (index == 0)
      return "bottom";
    return "middle";
  }

  GlassLayer* selectedLayer() { return selected_ >= 0 ? layers_[selected_].frame.get() : nullptr; }

  void bringToFront(int index) {
    LayerEntry entry = std::move(layers_[index]);
    layers_.erase(layers_.begin() + index);
    removeChild(entry.frame.get());
    addChild(entry.frame.get());
    layers_.push_back(std::move(entry));
    keepHudOnTop();
    select(layers_.size() - 1);
    layers_.back().frame->requestKeyboardFocus();
  }

  void sendToBack(int index) {
    LayerEntry entry = std::move(layers_[index]);
    layers_.erase(layers_.begin() + index);
    for (auto& layer : layers_)
      removeChild(layer.frame.get());
    removeChild(entry.frame.get());
    addChild(entry.frame.get());
    for (auto& layer : layers_)
      addChild(layer.frame.get());
    layers_.insert(layers_.begin(), std::move(entry));
    keepHudOnTop();
    select(0);
    layers_.front().frame->requestKeyboardFocus();
  }

  void removeSelected() {
    if (selected_ < 0)
      return;
    removeChild(layers_[selected_].frame.get());
    layers_.erase(layers_.begin() + selected_);
    select(std::min(selected_, static_cast<int>(layers_.size()) - 1));
  }

  bool handleKey(const visage::KeyEvent& e) {
    using visage::KeyCode;
    KeyCode key = e.keyCode();
    GlassLayer* layer = selectedLayer();
    int number = static_cast<int>(key) - static_cast<int>(KeyCode::Number1);

    if (key == KeyCode::Right && layer)
      layer->setEffect(layer->effect() + 1);
    else if (key == KeyCode::Left && layer)
      layer->setEffect(layer->effect() - 1);
    else if (key == KeyCode::Up && layer)
      layer->adjustAmount(1);
    else if (key == KeyCode::Down && layer)
      layer->adjustAmount(-1);
    else if (key == KeyCode::Tab && !layers_.empty())
      select((selected_ + (e.isShiftDown() ? -1 : 1) + layers_.size()) % layers_.size());
    else if (number >= 0 && number < 9 && number < layers_.size())
      select(number);
    else if (key == KeyCode::N) {
      std::uniform_real_distribution<float> position(0.05f, 0.6f);
      std::uniform_int_distribution<int> effect(1, effectSpecs().size() - 1);
      addLayer(effect(random_), position(random_), position(random_) * 0.8f, 0.3f, 0.36f);
      select(layers_.size() - 1);
    }
    else if ((key == KeyCode::Backspace || key == KeyCode::Delete) && layer)
      removeSelected();
    else if (key == KeyCode::S && layer)
      layer->cycleShape();
    else if (key == KeyCode::T && layer)
      layer->setTinted(!layer->tinted());
    else if (key == KeyCode::F && layer)
      bringToFront(selected_);
    else if (key == KeyCode::B && layer)
      sendToBack(selected_);
    else if (key == KeyCode::R) {
      std::uniform_int_distribution<int> effect(1, effectSpecs().size() - 1);
      for (auto& entry : layers_)
        entry.frame->setEffect(effect(random_));
    }
    else if (key == KeyCode::Space) {
      paused_ = !paused_;
      for (auto& entry : layers_)
        entry.frame->setAnimating(!paused_);
      scene_.redraw();
    }
    else if (key == KeyCode::H)
      show_help_ = !show_help_;
    else
      return false;

    hud_.redraw();
    return true;
  }

  void drawScene(visage::Canvas& canvas, float w, float h, float time) {
    canvas.setColor(visage::Brush::vertical(0xff1b1235, 0xff06303c));
    canvas.fill(0, 0, w, h);

    static constexpr int kColumns = 28;
    static constexpr int kRows = 16;
    float cell_w = w / kColumns;
    float cell_h = h / kRows;
    for (int r = 0; r < kRows; ++r) {
      for (int c = 0; c < kColumns; ++c) {
        float wave = std::sin(time * 1.3f + c * 0.35f + r * 0.5f);
        float hue = std::fmod(c * 9.0f + r * 6.0f + time * 20.0f, 360.0f);
        canvas.setColor(visage::Color::fromAHSV(0.55f + 0.25f * wave, hue, 0.75f, 0.9f));
        float size = 0.35f + 0.25f * (wave * 0.5f + 0.5f);
        canvas.roundedRectangle(c * cell_w + cell_w * (1.0f - size) * 0.5f,
                                r * cell_h + cell_h * (1.0f - size) * 0.5f, cell_w * size,
                                cell_h * size, cell_w * size * 0.2f);
      }
    }

    for (int i = 0; i < 6; ++i) {
      float x = std::fmod(time * (60.0f + i * 25.0f) + i * w / 6.0f, w + 200.0f) - 100.0f;
      canvas.setColor(visage::Color::fromAHSV(0.35f, i * 60.0f, 0.5f, 1.0f));
      canvas.fill(x, 0, 18.0f + i * 4.0f, h);
    }

    float min = std::min(w, h);
    float ring_radius = min * 0.32f;
    float dot = min * 0.09f;
    for (int i = 0; i < 12; ++i) {
      float angle = time * 0.4f + i * 2.0f * kPi / 12.0f;
      float x = w * 0.5f + ring_radius * std::cos(angle) - dot * 0.5f;
      float y = h * 0.45f + ring_radius * std::sin(angle) - dot * 0.5f;
      canvas.setColor(visage::Color::fromAHSV(1.0f, i * 30.0f + time * 30.0f, 0.85f, 1.0f));
      canvas.circle(x, y, dot);
    }

    const visage::Font title(min * 0.16f, resources::fonts::Lato_Regular_ttf);
    canvas.setColor(0xf0ffffff);
    canvas.text("visage", title, visage::Font::kCenter, 0, h * 0.45f - min * 0.12f, w, min * 0.24f);
  }

  void drawHud(visage::Canvas& canvas) {
    float w = hud_.width();
    float h = hud_.height();
    canvas.setColor(0xcc101014);
    canvas.fill(0, 0, w, h);

    const visage::Font font(16, resources::fonts::Lato_Regular_ttf);
    std::string status = std::to_string(layers_.size()) + " layers";
    if (GlassLayer* layer = selectedLayer()) {
      status += "     selected: layer " + std::to_string(selected_ + 1) + " (" +
                layerPosition(selected_) + ")   " + layer->description() + "   shape " +
                kShapeNames[layer->shape()] + (layer->tinted() ? ", tinted" : "");
    }
    if (paused_)
      status += "     PAUSED";
    canvas.setColor(0xffffffff);
    canvas.text(status, font, visage::Font::kLeft, 16, 8, w - 32, 26);

    if (show_help_) {
      canvas.setColor(0xffb8b8c0);
      canvas.text("left/right: cycle effect    up/down or wheel: amount    "
                  "tab / 1-9: select layer    drag: move    drag corner: resize",
                  font, visage::Font::kLeft, 16, 36, w - 32, 24);
      canvas.text("N: new layer    delete: remove    F/B: front/back    S: shape    T: tint    "
                  "R: randomize    space: pause    H: hide help",
                  font, visage::Font::kLeft, 16, 62, w - 32, 24);
    }
  }

  visage::Frame scene_;
  visage::Frame hud_;
  std::vector<LayerEntry> layers_;
  int selected_ = -1;
  bool placed_ = false;
  bool paused_ = false;
  bool show_help_ = true;
  float scene_time_ = 0.0f;
  std::mt19937 random_ { 7 };
};

int runExample() {
  ExampleEditor editor;
  editor.setTitle("Backdrop Effects");
  editor.showMaximized();
  editor.requestKeyboardFocus();
  editor.runEventLoop();

  return 0;
}
