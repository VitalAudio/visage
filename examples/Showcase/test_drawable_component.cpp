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

#include "test_drawable_component.h"

#include "embedded/example_fonts.h"
#include "embedded/example_icons.h"
#include "embedded/example_shaders.h"

#include <filesystem>
#include <visage_graphics/theme.h>
#include <visage_utils/file_system.h>

inline float quickSin1(float phase) {
  phase = 0.5f - phase;
  return phase * (8.0f - 16.0f * fabsf(phase));
}

inline float sin1(float phase) {
  float approx = quickSin1(phase - floorf(phase));
  return approx * (0.776f + 0.224f * fabsf(approx));
}

THEME_COLOR(BackgroundColor, 0xff33393f);
THEME_COLOR(TextColor, 0xffffffff);
THEME_COLOR(ShapeColor, 0xffaaff88);
THEME_COLOR(LabelColor, 0x44212529);
THEME_COLOR(LogoColor1, 0xffaa88ff);
THEME_COLOR(LogoColor2, 0xffffffff);
THEME_COLOR(LogoBackgroundColor, 0xff212529);
THEME_COLOR(OverlayShadowColor, 0xbb000000);
THEME_COLOR(ShadowColor, 0x88000000);

class DragDropTarget : public visage::Frame {
public:
  DragDropTarget() { setIgnoresMouseEvents(false, false); }

  void setPositions(int* rectangles, const float* data, int data_size) const {
    if (data == nullptr)
      return;

    int h = height() / 6;
    int w = width();
    for (int i = 0; i < w; ++i) {
      int start_index = i * data_size / w;
      int end_index = (i + 1) * data_size / w;
      float max = 0.0f;
      float min = 0.0f;
      for (int j = start_index; j < end_index; ++j) {
        max = std::max(max, data[j]);
        min = std::min(min, data[j]);
      }
      rectangles[i] = h * std::max(max, -min);
    }
  }

  void resetPositions() {
    rectangles_left_ = std::make_unique<int[]>(width());
    rectangles_right_ = std::make_unique<int[]>(width());

    //TODO
    // setPositions(rectangles_left_.get(), buffer_.left_data.get(), buffer_.length);
    // setPositions(rectangles_right_.get(), buffer_.right_data.get(), buffer_.length);
  }

  void resized() override { resetPositions(); }

  void draw(visage::Canvas& canvas) override {
    canvas.setPaletteColor(kLogoBackgroundColor);
    canvas.roundedRectangle(0, 0, width(), height(), height() / 16);

    canvas.setPaletteColor(kTextColor);

    const visage::Font font(height() / 8, resources::fonts::Lato_Regular_ttf);
    if (dragging_)
      canvas.text(filename_, font, visage::Font::kCenter, 0, 0, width(), height());
    else
      canvas.text("Drag audio files", font, visage::Font::kCenter, 0, 0, width(), height());

    // if (buffer_.left_data == nullptr)
    //   return;

    int y1 = height() / 4;
    int y2 = y1 + height() / 2;
    for (int i = 0; i < width(); ++i) {
      int left = rectangles_left_[i];
      int right = rectangles_right_[i];
      canvas.rectangle(i, y1 - left, 1, left * 2);
      canvas.rectangle(i, y2 - right, 1, right * 2);
    }
  }

  bool receivesDragDropFiles() override { return true; }
  std::string dragDropFileExtensionRegex() override { return "(wav)|(mp3)|(ogg)"; }

  void dragFilesEnter(const std::vector<std::string>& paths) override {
    dragging_ = true;
    filename_ = visage::fileName(paths[0]);
  }

  void dragFilesExit() override { dragging_ = false; }

  void dropFiles(const std::vector<std::string>& paths) override {
    /* TODO
    int size = 0;
    std::unique_ptr<char[]> data = visage::loadFileData(paths[0], size);
    if (size) {
      buffer_ = visage::decodeAudioFile(visage::getFileName(paths[0]),
                                        reinterpret_cast<unsigned char*>(data.get()), size);
    }

    dragging_ = false;
    resetPositions();
     */
  }

  bool isDragDropSource() override {
    // TOOD
    // return buffer_.length > 0;
    return false;
  }

  std::string startDragDropSource() override {
    // TODO
    // source_file_ = visage::createTemporaryFile("wav");
    // visage::encodeWavFile(buffer_, source_file_.string());
    // return source_file_;
    return "";
  }

  void cleanupDragDropSource() override {
    if (std::filesystem::exists(source_file_))
      std::filesystem::remove(source_file_);
  }

private:
  std::string filename_;
  bool dragging_ = false;

  std::unique_ptr<int[]> rectangles_left_;
  std::unique_ptr<int[]> rectangles_right_;
  visage::File source_file_;
};

class TextImage : public visage::CachedFrame {
public:
  TextImage() {
    std::string text = "\nThis is a bunch of center justified and wrapped text fit into an area.";
    text_ = std::make_unique<visage::Text>(text, visage::Font(10, resources::fonts::Lato_Regular_ttf));
    text_->setMultiLine(true);
    text_->setJustification(visage::Font::kCenter);
  }

  void drawToCache(visage::Canvas& canvas) override {
    int font_height = height() / 6;
    text_->setFont(visage::Font(font_height, resources::fonts::Lato_Regular_ttf));
    canvas.setColor(0xffffffff);
    canvas.text(text_.get(), 0, 0, width(), height());
  }

private:
  std::unique_ptr<visage::Text> text_;
};

TestDrawableComponent::TestDrawableComponent() {
  static constexpr int kBars = kNumBars;

  drag_drop_target_ = std::make_unique<DragDropTarget>();
  addChild(drag_drop_target_.get());

  bar_component_ = std::make_unique<visage::BarComponent>(kBars);
  addChild(bar_component_.get());
  bar_component_->setHorizontalAntiAliasing(false);

  shader_quad_ = std::make_unique<visage::ShaderQuad>(resources::shaders::vs_shape,
                                                      resources::shaders::fs_shader_quad,
                                                      visage::BlendState::Alpha);
  addChild(shader_quad_.get());

  icon_button_ = std::make_unique<visage::ToggleIconButton>(resources::icons::check_circle_svg.data,
                                                            resources::icons::check_circle_svg.size, true);
  addChild(icon_button_.get());

  text_button_ = std::make_unique<visage::ToggleTextButton>("Toggle",
                                                            visage::Font(24, resources::fonts::Lato_Regular_ttf));
  addChild(text_button_.get());

  text_ = std::make_unique<TextImage>();
  addChild(text_.get());

  ui_button_ = std::make_unique<visage::UiButton>("Trigger Overlay",
                                                  visage::Font(24, resources::fonts::Lato_Regular_ttf));
  ui_button_->onToggle() = [this](visage::Button* button, bool toggled) {
    for (Listener* listener : listeners_)
      listener->showOverlay();
  };

  addChild(ui_button_.get());
  ui_button_->setToggleOnMouseDown(true);

  action_button_ = std::make_unique<visage::UiButton>("Popup Menu",
                                                      visage::Font(24, resources::fonts::Lato_Regular_ttf));
  addChild(action_button_.get());
  action_button_->setActionButton();
  action_button_->onToggle() = [this](visage::Button* button, bool toggled) {
    visage::PopupOptions options;
    options.addOption(0, "Test 1");
    options.addOption(1, "Hello World");
    options.addBreak();
    options.addOption(2, "Another Item 1");
    visage::PopupOptions sub_options;
    sub_options.name = "Sub Menu";
    sub_options.addOption(3, "Sub Item 1");
    sub_options.addBreak();
    sub_options.addOption(4, "Sub Item 2");
    sub_options.addOption(5, "Sub Item 3");
    sub_options.addOption(6, "Sub Item 4");
    options.addOption(sub_options);
    visage::PopupOptions sub_options2;
    sub_options2.name = "Other Sub Menu";
    sub_options2.addOption(7, "Other Sub Item 1");
    sub_options2.addBreak();
    sub_options2.addOption(8, "Other Sub Item 2");
    sub_options2.addOption(9, "Other Sub Item 3");
    sub_options2.addOption(10, "Other Sub Item 4");
    options.addOption(sub_options2);
    options.addOption(11, "Another Item 3");
    options.addBreak();
    options.addOption(12, "Force Crash");

    showPopupMenu(options, action_button_->bounds(), [=](int id) {
      if (id == 12)
        VISAGE_FORCE_CRASH();
    });
  };
  action_button_->setToggleOnMouseDown(true);

  text_editor_ = std::make_unique<visage::TextEditor>();
  addChild(text_editor_.get());

  left_text_editor_ = std::make_unique<visage::TextEditor>();
  addChild(left_text_editor_.get());
  left_text_editor_->setJustification(visage::Font::kLeft);
  left_text_editor_->setDefaultText("Left Text");

  number_editor_ = std::make_unique<visage::TextEditor>();
  addChild(number_editor_.get());
  number_editor_->setDefaultText("Center Select");
  number_editor_->setNumberEntry();

  right_text_editor_ = std::make_unique<visage::TextEditor>();
  addChild(right_text_editor_.get());
  right_text_editor_->setJustification(visage::Font::kRight);
  right_text_editor_->setDefaultText("Right Text");

  for (auto& line_component : line_components_) {
    line_component = std::make_unique<visage::LineComponent>(400);
    addChild(line_component.get());
  }

  setIgnoresMouseEvents(true, true);
}

TestDrawableComponent::~TestDrawableComponent() = default;

void TestDrawableComponent::resized() {
  int w = width();
  int h = height();
  int x_division = w / 2;
  int right_width = w - x_division;
  int section_height = h / 4;
  int section_head_height = section_height / 4;
  int section_body_height = section_height - section_head_height;

  int line_offset = section_body_height / kNumLines;
  for (int i = 0; i < kNumLines; ++i) {
    line_components_[i]->setBounds(0, section_head_height + line_offset * i, x_division, line_offset);
    line_components_[i]->setFill(true);
  }

  bar_component_->setBounds(0, section_height + section_head_height, x_division / 2, section_body_height);
  int shader_x = x_division / 2 + (x_division / 2 - section_body_height) / 2;
  shader_quad_->setBounds(shader_x, section_height + section_head_height, section_body_height,
                          section_body_height);

  int font_height = section_head_height / 2;
  int text_y = 2 * section_height + section_head_height;
  int text_section_padding = w / 50;
  int text_section_width = (w - text_section_padding) / 4 - text_section_padding;
  int padding = section_body_height / 16;
  int single_line_height = (section_body_height + padding) / 3 - padding;
  int margin = font_height / 3;
  left_text_editor_->setBounds(text_section_padding, text_y, text_section_width, single_line_height);
  left_text_editor_->setFont(visage::Font(font_height, resources::fonts::Lato_Regular_ttf));
  left_text_editor_->setMargin(margin, 0);
  left_text_editor_->setBackgroundRounding(margin / 2);

  number_editor_->setBounds(text_section_padding, text_y + single_line_height + padding,
                            text_section_width, single_line_height);
  number_editor_->setFont(visage::Font(font_height, resources::fonts::Lato_Regular_ttf));
  number_editor_->setMargin(margin, 0);
  number_editor_->setBackgroundRounding(margin / 2);

  right_text_editor_->setBounds(text_section_padding, text_y + 2 * (single_line_height + padding),
                                text_section_width, single_line_height);
  right_text_editor_->setFont(visage::Font(font_height, resources::fonts::Lato_Regular_ttf));
  right_text_editor_->setMargin(margin, 0);
  right_text_editor_->setBackgroundRounding(margin / 2);

  text_editor_->setMargin(font_height / 2, font_height / 2);
  text_editor_->setFont(visage::Font(section_head_height / 2, resources::fonts::Lato_Regular_ttf));
  text_editor_->setMultiLine(true);
  text_editor_->setJustification(visage::Font::kTopLeft);
  text_editor_->setBounds(text_section_width + 2 * text_section_padding, text_y, text_section_width,
                          section_body_height);
  text_editor_->setBackgroundRounding(margin / 2);
  text_editor_->setDefaultText("Multiline Text");

  text_->setBounds(x_division, text_editor_->y(), right_width / 2, text_editor_->height());

  int widget_y = 3 * section_height + section_head_height;
  int buttons_width = right_width / 2;
  int button_padding = buttons_width / 16;
  int button_width = (buttons_width - button_padding) / 2;
  int button_height = button_width / 3;
  int button_font_height = button_height / 3;
  action_button_->setBounds(x_division, widget_y, button_width, button_height);
  action_button_->setFont(visage::Font(button_font_height, resources::fonts::Lato_Regular_ttf));
  ui_button_->setBounds(x_division, (h + widget_y) / 2, button_width, button_height);
  ui_button_->setFont(visage::Font(button_font_height, resources::fonts::Lato_Regular_ttf));

  text_button_->setBounds(x_division + button_width + button_padding, widget_y, button_width, button_height);
  text_button_->setFont(visage::Font(button_font_height, resources::fonts::Lato_Regular_ttf));
  icon_button_->setBounds(x_division + button_width + button_padding, (h + widget_y) / 2,
                          button_height, button_height);

  drag_drop_target_->setBounds(x_division + right_width / 2, widget_y, right_width / 2, h - widget_y);
}

void TestDrawableComponent::draw(visage::Canvas& canvas) {
  static constexpr float kHalfPi = 3.14159265358979323846f * 0.5f;
  int w = width();
  int h = height();
  canvas.setPaletteColor(kBackgroundColor);
  canvas.fill(0, 0, w, h);

  int section_height = h / 4;
  int section_head_height = section_height / 4;
  int x_division = w / 2;
  int right_width = w - x_division;

  int label_height = section_head_height / 2;
  int label_offset = (section_head_height - label_height) / 2;
  canvas.setPaletteColor(kLabelColor);
  canvas.fill(0, label_offset, w, label_height);
  canvas.fill(0, section_height + label_offset, w, label_height);
  canvas.fill(0, 2 * section_height + label_offset, w, label_height);
  canvas.fill(0, 3 * section_height + label_offset, w, label_height);

  canvas.setPaletteColor(kTextColor);
  visage::Font font(section_head_height / 3, resources::fonts::Lato_Regular_ttf);

  canvas.text("Line Rendering", font, visage::Font::kCenter, 0, 0, x_division, section_head_height);
  canvas.text("Line Editing", font, visage::Font::kCenter, x_division, 0, right_width, section_head_height);

  canvas.text("Bars", font, visage::Font::kCenter, 0, section_height, x_division / 2, section_head_height);
  canvas.text("Shaders", font, visage::Font::kCenter, x_division / 2, section_height,
              x_division / 2, section_head_height);
  canvas.text("Shapes", font, visage::Font::kCenter, x_division, section_height, right_width,
              section_head_height);

  canvas.text("Text Editing", font, visage::Font::kCenter, 0, 2 * section_height, x_division,
              section_head_height);
  canvas.text("Text", font, visage::Font::kCenter, x_division, 2 * section_height, right_width / 2,
              section_head_height);
  canvas.text("SVG", font, visage::Font::kCenter, x_division + right_width / 2, 2 * section_height,
              right_width / 2, section_head_height);

  canvas.text("Controls", font, visage::Font::kCenter, 0, 3 * section_height, x_division,
              section_head_height);
  canvas.text("Buttons", font, visage::Font::kCenter, x_division, 3 * section_height,
              right_width / 2, section_head_height);
  canvas.text("Drag + Drop", font, visage::Font::kCenter, x_division + right_width / 2,
              3 * section_height, right_width / 2, section_head_height);

  int icon_width = std::min(w / 4, text_editor_->height());
  int icon_x = x_division + right_width / 2 + right_width / 4 - icon_width / 2;
  int icon_y = text_editor_->y();
  int blur_radius = icon_width / 16;

  canvas.setPaletteColor(kLogoBackgroundColor);
  canvas.circle(icon_x, icon_y, icon_width);
  canvas.setPaletteColor(kShadowColor);
  canvas.icon(resources::icons::vital_ring_svg, icon_x, icon_y, icon_width, icon_width, blur_radius);
  canvas.icon(resources::icons::vital_v_svg, icon_x, icon_y, icon_width, icon_width, blur_radius);
  canvas.setPaletteColor(kLogoColor1);
  canvas.icon(resources::icons::vital_ring_svg, icon_x, icon_y, icon_width, icon_width);
  canvas.setPaletteColor(kLogoColor2);
  canvas.icon(resources::icons::vital_v_svg, icon_x, icon_y, icon_width, icon_width);

  double render_time = canvas.time();
  for (int r = 0; r < kNumLines; ++r) {
    int render_height = line_components_[r]->height();
    int render_width = line_components_[r]->width();
    int line_height = render_height * 0.9f;
    int offset = render_height * 0.05f;

    float position = 0.0f;
    for (int i = 0; i < 400; ++i) {
      float t = i / 399.0f;
      float delta = std::min(t, 1.0f - t);
      position += 0.1f * delta * delta + 0.003f;
      double phase = (render_time + r) * 0.5;
      line_components_[r]->setXAt(i, t * render_width);
      line_components_[r]->setYAt(i, offset + (sin1(phase + position) * 0.5f + 0.5f) * line_height);
    }
  }

  float space = 1;
  float bar_width = (bar_component_->width() + space) / kNumBars;
  int bar_height = bar_component_->height();
  for (int i = 0; i < kNumBars; ++i) {
    float x = bar_width * i;
    float current_height = (sin1((render_time * 60.0 + i * 30) / 600.0f) + 1.0f) * 0.5f * bar_height;
    bar_component_->positionBar(i, x, current_height, bar_width - space, bar_height - current_height);
  }

  float center_radians = render_time * 1.2f;
  double phase = render_time * 0.1;
  float radians = kHalfPi * sin1(phase) + kHalfPi;

  int shape_x = x_division;
  int shape_y = bar_component_->y();
  int shape_padding = (w - shape_x) / 20;
  shape_x += shape_padding;
  int shape_width = (w - shape_x) / 5 - shape_padding;
  shape_y += (bar_component_->height() - 2 * shape_width - shape_padding) / 2;
  int shape_y2 = shape_y + shape_width + shape_padding;
  int roundness = shape_width / 8;

  float shape_phase = canvas.time() * 0.1f;
  shape_phase -= std::floor(shape_phase);
  float shape_cycle = sin1(shape_phase) * 0.5f + 0.5f;
  float thickness = shape_width * shape_cycle / 8.0f + 1.0f;

  canvas.setPaletteColor(kShadowColor);
  canvas.rectangleShadow(shape_x, shape_y, shape_width, shape_width, thickness);
  canvas.roundedRectangleShadow(shape_x + 2 * (shape_width + shape_padding), shape_y, shape_width,
                                shape_width, roundness, thickness);

  canvas.setPaletteColor(kShapeColor);
  canvas.rectangle(shape_x, shape_y, shape_width, shape_width);
  canvas.rectangleBorder(shape_x, shape_y2, shape_width, shape_width, thickness);
  canvas.circle(shape_x + shape_width + shape_padding, shape_y, shape_width);
  canvas.ring(shape_x + shape_width + shape_padding, shape_y2, shape_width, thickness);
  canvas.roundedRectangle(shape_x + 2 * (shape_width + shape_padding), shape_y, shape_width,
                          shape_width, roundness);
  canvas.roundedRectangleBorder(shape_x + 2 * (shape_width + shape_padding), shape_y2, shape_width,
                                shape_width, roundness, thickness);
  canvas.arc(shape_x + 3 * (shape_width + shape_padding), shape_y, shape_width, thickness,
             center_radians, radians, false);
  canvas.arc(shape_x + 3 * (shape_width + shape_padding), shape_y2, shape_width, thickness,
             center_radians, radians, true);

  float max_separation = shape_padding / 2.0f;
  float separation = shape_cycle * max_separation;
  int triangle_x = shape_x + 4 * (shape_width + shape_padding) + max_separation;
  int triangle_y = shape_y + max_separation;
  int triangle_width = (shape_width - 2.0f * max_separation) / 2.0f;
  canvas.triangleDown(triangle_x, triangle_y - separation, triangle_width);
  canvas.triangleRight(triangle_x - separation, triangle_y, triangle_width);
  canvas.triangleUp(triangle_x, triangle_y + triangle_width + separation, triangle_width);
  canvas.triangleLeft(triangle_x + triangle_width + separation, triangle_y, triangle_width);

  float segment_x = shape_x + 4 * (shape_width + shape_padding);
  float segment_y = shape_y2;
  float shape_radius = shape_width / 2;
  std::pair<float, float> segment_positions[] = { { segment_x, segment_y + shape_radius },
                                                  { segment_x + shape_radius, segment_y },
                                                  { segment_x + shape_width, segment_y + shape_radius },
                                                  { segment_x + shape_radius, segment_y + shape_width } };

  int index = std::min<int>(shape_phase * 4.0f, 3);
  float movement_phase = shape_phase * 4.0f - index;
  float t1 = sin1(std::min(movement_phase * 0.5f, 0.25f) - 0.25f) + 1.0f;
  float t2 = sin1(std::max(movement_phase * 0.5f, 0.25f) - 0.25f);
  std::pair<float, float> from = segment_positions[index];
  std::pair<float, float> to = segment_positions[(index + 1) % 4];
  float delta_x = to.first - from.first;
  float delta_y = to.second - from.second;
  canvas.segment(from.first + delta_x * t1, from.second + delta_y * t1, from.first + delta_x * t2,
                 from.second + delta_y * t2, thickness, true);

  from = segment_positions[(index + 2) % 4];
  to = segment_positions[(index + 3) % 4];
  delta_x = to.first - from.first;
  delta_y = to.second - from.second;
  canvas.segment(from.first + delta_x * t1, from.second + delta_y * t1, from.first + delta_x * t2,
                 from.second + delta_y * t2, thickness, false);

  if (shadow_amount_) {
    float shadow_mult = std::max(0.0f, 2.0f * shadow_amount_ - 1.0f);
    shadow_mult = shadow_mult * shadow_mult;
    canvas.setColor(canvas.color(kOverlayShadowColor).withMultipliedAlpha(shadow_mult));
    canvas.roundedRectangle(shadow_bounds_.x(), shadow_bounds_.y(), shadow_bounds_.width(),
                            shadow_bounds_.height(), shadow_rounding_);
  }
}