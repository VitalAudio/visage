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

#include "embedded/example_fonts.h"
#include "embedded/example_shaders.h"

#include <complex>
#include <visage_app/application_editor.h>
#include <visage_widgets/shader_editor.h>
#include <visage_windowing/windowing.h>

using namespace visage::dimension;

static void drawRing(visage::Canvas& canvas, int width, int height, float radius,
                     float circle_diameter, int num, float phase_offset) {
  static constexpr float kPi = 3.14159265358979323846f;

  float phase_inc = 2.0f * kPi / num;
  std::complex<float> tick(cosf(phase_inc), sinf(phase_inc));
  std::complex<float> position(radius * cosf(phase_offset), radius * sinf(phase_offset));
  float center_x = (width - circle_diameter) / 2.0f;
  float center_y = (height - circle_diameter) / 2.0f;

  for (int i = 0; i < num; ++i) {
    position *= tick;
    canvas.circle(center_x + position.real(), center_y + position.imag(), circle_diameter);
  }
}

class PostEffectSelector : public visage::Frame {
public:
  enum PostEffect {
    kNone,
    kGrayScale,
    kSepia,
    kGlitch,
    kBlur,
    kNumOptions
  };

  PostEffectSelector() {
    static const std::string option_names[kNumOptions] = { "None", "Gray Scale", "Sepia", "Glitch", "Blur" };

    setFlexLayout(true);
    layout().setPadding(3_vmin);
    layout().setFlexGap(3_vmin);
    for (int i = 0; i < kNumOptions; ++i) {
      options_[i].layout().setFlexGrow(1.0f);
      options_[i].setText(option_names[i]);

      addChild(&options_[i]);
      options_[i].onToggle() = [this](visage::Button* button, bool on) {
        for (int i = 0; i < kNumOptions; ++i) {
          if (&options_[i] == button && on_effect_change_)
            on_effect_change_(static_cast<PostEffect>(i));
        }
      };
    };
  }

  void resized() override {
    visage::Font font(height() * 0.05f, resources::fonts::Lato_Regular_ttf);

    for (auto& button : options_)
      button.setFont(font);
  }

  void setCallback(std::function<void(PostEffect)> on_effect_change) {
    on_effect_change_ = std::move(on_effect_change);
  }

private:
  visage::UiButton options_[kNumOptions];
  std::function<void(PostEffect)> on_effect_change_;
};

class ExampleEditor : public visage::WindowedEditor {
public:
  ExampleEditor() {
    shapes_.onDraw() = [this](visage::Canvas& canvas) {
      int width = shapes_.width();
      int height = shapes_.height();
      int min = std::min(width, height);

      canvas.setColor(0xff222233);
      canvas.fill(0, 0, width, height);
      canvas.setColor(0xffaa88ff);
      drawRing(canvas, width, height, min * 0.3f, min * 0.2f, 8, canvas.time() * 0.1f);
      shapes_.redraw();
    };

    gray_scale_ = std::make_unique<visage::ShaderPostEffect>(resources::shaders::vs_gray_scale,
                                                             resources::shaders::fs_gray_scale);

    sepia_ = std::make_unique<visage::ShaderPostEffect>(resources::shaders::vs_sepia,
                                                        resources::shaders::fs_sepia);

    glitch_ = std::make_unique<visage::ShaderPostEffect>(resources::shaders::vs_glitch,
                                                         resources::shaders::fs_glitch);

    blur_ = std::make_unique<visage::BlurBloomPostEffect>();
    blur_->setBloomSize(0.0f);
    blur_->setBloomIntensity(0.0f);
    blur_->setBlurSize(40.0f);
    blur_->setBlurAmount(1.0f);

    addChild(&shapes_);
    addChild(&selector_);

    selector_.setCallback([this](PostEffectSelector::PostEffect effect) {
      if (effect == PostEffectSelector::kNone)
        shapes_.setPostEffect(nullptr);
      else if (effect == PostEffectSelector::kGrayScale)
        shapes_.setPostEffect(gray_scale_.get());
      else if (effect == PostEffectSelector::kSepia)
        shapes_.setPostEffect(sepia_.get());
      else if (effect == PostEffectSelector::kGlitch)
        shapes_.setPostEffect(glitch_.get());
      else if (effect == PostEffectSelector::kBlur)
        shapes_.setPostEffect(blur_.get());
    });
  }

  void draw(visage::Canvas& canvas) override {
    canvas.setColor(0xff222233);
    canvas.fill(0, 0, width(), height());
  }

  void resized() override {
    int center = width() / 2;
    int shapes_width = std::min(center, height());
    shapes_.setBounds((center - shapes_width) / 2, (height() - shapes_width) / 2, shapes_width, shapes_width);
    selector_.setBounds(center, 0, width() - center, height());
  }

private:
  PostEffectSelector selector_;
  Frame shapes_;
  std::unique_ptr<visage::ShaderPostEffect> gray_scale_;
  std::unique_ptr<visage::ShaderPostEffect> sepia_;
  std::unique_ptr<visage::ShaderPostEffect> glitch_;
  std::unique_ptr<visage::BlurBloomPostEffect> blur_;
};

int runExample() {
  ExampleEditor editor;
  editor.showMaximized();
  editor.runEventLoop();

  return 0;
}
