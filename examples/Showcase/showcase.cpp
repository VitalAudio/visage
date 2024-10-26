/* Copyright Matt Tytel
 *
 * test plugin is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * test plugin is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with test plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "showcase.h"

#include "embedded/example_fonts.h"
#include "embedded/example_shaders.h"

#include <visage_graphics/post_effects.h>
#include <visage_windowing/windowing.h>

namespace {
  constexpr int kPaletteWidth = 200;
  constexpr int kShaderEditorWidth = 600;
}  // namespace

THEME_COLOR(OverlayBody, 0xff212529);
THEME_COLOR(OverlayBorder, 0x66ffffff);

THEME_VALUE(BloomSize, 25.0f, ScaledHeight, false);
THEME_VALUE(BloomIntensity, 3.0f, Constant, false);
THEME_VALUE(BlurSize, 25.0f, ScaledHeight, false);
THEME_VALUE(OverlayRounding, 25.0f, ScaledHeight, false);

class DebugInfo : public visage::Frame {
public:
  DebugInfo() { setIgnoresMouseEvents(true, true); }

  void draw(visage::Canvas& canvas) override {
    canvas.setColor(0x88000000);
    canvas.fill(0, 0, width(), height());

    canvas.setColor(0xffffffff);

    std::vector<std::string> info = canvas.debugInfo();
    int line_height = height() / info.size();
    int text_height = line_height * 0.65f;

    const visage::Font font(text_height, resources::fonts::Lato_Regular_ttf);
    for (int i = 0; i < info.size(); ++i)
      canvas.text(info[i], font, visage::Font::kLeft, line_height, line_height * i, width(), line_height);
  }
};

Overlay::Overlay() :
    animation_(visage::Animation<float>::kRegularTime, visage::Animation<float>::kLinear,
               visage::Animation<float>::kLinear) {
  animation_.setAnimationTime(160.0f);
  animation_.setTargetValue(1.0f);
}

void Overlay::resized() { }

void Overlay::draw(visage::Canvas& canvas) {
  float overlay_amount = animation_.update();
  if (!animation_.isTargeting() && overlay_amount == 0.0f)
    setVisible(false);

  visage::Bounds body = getBodyBounds();
  float rounding = getBodyRounding();

  canvas.setColor(0);
  canvas.clearArea(body.x() - 1, body.y() - 1, body.width() + 2, body.height() + 2);

  canvas.setPaletteColor(kOverlayBody);
  canvas.roundedRectangle(body.x(), body.y(), body.width(), body.height(), rounding);

  canvas.setPaletteColor(kOverlayBorder);
  canvas.roundedRectangleBorder(body.x(), body.y(), body.width(), body.height(), rounding, 1.0f);
}

visage::Bounds Overlay::getBodyBounds() const {
  int x_border = width() / 4;
  int y_border = height() / 4;

  return { x_border, y_border, width() - 2 * x_border, height() - 2 * y_border };
}

float Overlay::getBodyRounding() {
  return paletteValue(kOverlayRounding);
}

Showcase::Showcase() : color_editor_(palette()), value_editor_(palette()) {
  setAcceptsKeystrokes(true);
  setFixedAspectRatio(true);

  palette_.initWithDefaults();
  setPalette(&palette_);
  color_editor_.setEditedPalette(&palette_);
  value_editor_.setEditedPalette(&palette_);

  blur_bloom_ = std::make_unique<visage::BlurBloomPostEffect>();
  test_component_ = std::make_unique<TestDrawableComponent>();
  test_component_->setPostEffect(blur_bloom_.get());
  addChild(test_component_.get());
  test_component_->addListener(this);

  addChild(&color_editor_, false);
  addChild(&value_editor_, false);
  addChild(&shader_editor_, false);

  overlay_zoom_ = std::make_unique<visage::ShaderPostEffect>(resources::shaders::vs_overlay,
                                                             resources::shaders::fs_overlay);
  overlay_.setPostEffect(overlay_zoom_.get());
  addChild(&overlay_);
  overlay_.setOnTop(true);

  debug_info_ = std::make_unique<DebugInfo>();
  addChild(debug_info_.get());
  debug_info_->setOnTop(true);
  debug_info_->setVisible(false);

  addChild(&popup_, false);
  popup_.setOnTop(true);
  addChild(&primary_value_display_, false);
  primary_value_display_.setOnTop(true);
  addChild(&secondary_value_display_, false);
  secondary_value_display_.setOnTop(true);

  popup_.setFont(visage::Font(10, resources::fonts::Lato_Regular_ttf));
  primary_value_display_.setFont(visage::Font(10, resources::fonts::Lato_Regular_ttf));
  secondary_value_display_.setFont(visage::Font(10, resources::fonts::Lato_Regular_ttf));
}

Showcase::~Showcase() {
  removeFromWindow();
}

void Showcase::showPopup(const visage::PopupOptions& options, visage::Frame* frame, visage::Bounds bounds,
                         std::function<void(int)> callback, std::function<void()> cancel) {
  visage::Bounds selection_bounds = relativeBounds(frame) + bounds.topLeft();
  popup_.showMenu(options, selection_bounds, callback, nullptr);
}

void Showcase::showValueDisplay(const std::string& text, visage::Frame* frame, visage::Bounds bounds,
                                visage::Font::Justification justification, bool primary) {
  visage::Bounds selection_bounds = relativeBounds(frame) + bounds.topLeft();
  if (primary)
    primary_value_display_.showDisplay(text, selection_bounds, justification);
  else
    secondary_value_display_.showDisplay(text, selection_bounds, justification);
}

void Showcase::hideValueDisplay(bool primary) {
  if (primary)
    primary_value_display_.setVisible(false);
  else
    secondary_value_display_.setVisible(false);
}

void Showcase::editorResized() {
  int w = width();
  int h = height();
  int main_width = w;
  if (color_editor_.isVisible() || value_editor_.isVisible() || shader_editor_.isVisible())
    main_width = std::round((h * kDefaultWidth * 1.0f) / kDefaultHeight);

  debug_info_->setBounds(0, 0, main_width, h);

  test_component_->setBounds(0, 0, main_width, h);
  color_editor_.setBounds(main_width, 0, w - main_width, h);
  value_editor_.setBounds(main_width, 0, w - main_width, h);
  shader_editor_.setBounds(main_width, 0, w - main_width, h);

  overlay_.setBounds(test_component_->bounds());
  popup_.setBounds(localBounds());
}

void Showcase::draw(visage::Canvas& canvas) {
  static constexpr float kMaxZoom = 0.075f;
  VISAGE_ASSERT(width() && height());

  canvas.setPalette(palette());
  blur_bloom_->setBlurSize(canvas.value(kBlurSize));
  blur_bloom_->setBloomSize(canvas.value(kBloomSize));
  blur_bloom_->setBloomIntensity(canvas.value(kBloomIntensity));

  float overlay_amount = overlay_.animationProgress();
  test_component_->setShadow(overlay_.getBodyBounds(), overlay_amount, overlay_.getBodyRounding());
  blur_bloom_->setBlurAmount(overlay_amount);
  overlay_zoom_->setUniformValue("u_zoom", kMaxZoom * (1.0f - overlay_amount) + 1.0f);
  overlay_zoom_->setUniformValue("u_alpha", overlay_amount * overlay_amount);

  ApplicationEditor::draw(canvas);
}

void Showcase::clearEditors() {
  color_editor_.setVisible(false);
  value_editor_.setVisible(false);
  shader_editor_.setVisible(false);

  int new_width = std::round((height() * kDefaultWidth * 1.0f) / kDefaultHeight);
  color_editor_.setVisible(false);
  setBounds(0, 0, new_width, height());
}

void Showcase::showEditor(const Frame* editor, int default_width) {
  color_editor_.setVisible(editor == &color_editor_);
  value_editor_.setVisible(editor == &value_editor_);
  shader_editor_.setVisible(editor == &shader_editor_);

  int new_default_width = kDefaultWidth + default_width;
  int new_width = std::round((height() * new_default_width * 1.0f) / kDefaultHeight);
  setBounds(0, 0, new_width, height());
}

bool Showcase::keyPress(const visage::KeyEvent& key) {
  bool modifier = key.isMainModifier();
  if (key.keyCode() == visage::KeyCode::Number0 && key.isMainModifier())
    clearEditors();
  else if (key.keyCode() == visage::KeyCode::Number1 && modifier) {
    if (color_editor_.isVisible())
      clearEditors();
    else
      showEditor(&color_editor_, kPaletteWidth);
    return true;
  }
  else if (key.keyCode() == visage::KeyCode::Number2 && modifier) {
    if (value_editor_.isVisible())
      clearEditors();
    else
      showEditor(&value_editor_, kPaletteWidth);
    return true;
  }
  else if (key.keyCode() == visage::KeyCode::Number3 && modifier) {
    if (shader_editor_.isVisible())
      clearEditors();
    else
      showEditor(&shader_editor_, kShaderEditorWidth);
    return true;
  }
  else if (key.keyCode() == visage::KeyCode::D && modifier && key.isShiftDown()) {
    debug_info_->setVisible(!debug_info_->isVisible());
    return true;
  }
  else if (key.keyCode() == visage::KeyCode::Z && modifier) {
    undo();
    return true;
  }
  else if (key.keyCode() == visage::KeyCode::Y && modifier) {
    redo();
    return true;
  }

  return false;
}

int runExample() {
  std::unique_ptr<Showcase> editor = std::make_unique<Showcase>();
  editor->showWithEventLoop(0.6f);

  return 0;
}
