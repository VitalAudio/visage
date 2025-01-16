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

#include "examples_frame.h"

#include "embedded/example_fonts.h"
#include "embedded/example_icons.h"
#include "embedded/example_images.h"
#include "embedded/example_shaders.h"

#include <filesystem>
#include <visage_graphics/theme.h>
#include <visage_ui/popup_menu.h>
#include <visage_utils/file_system.h>

inline float quickSin1(float phase) {
  phase = 0.5f - phase;
  return phase * (8.0f - 16.0f * fabsf(phase));
}

inline float sin1(float phase) {
  float approx = quickSin1(phase - floorf(phase));
  return approx * (0.776f + 0.224f * fabsf(approx));
}

static constexpr float kHalfPi = 3.14159265358979323846f * 0.5f;

THEME_COLOR(TextColor, 0xffffffff);
THEME_COLOR(ShapeColor, 0xffaaff88);
THEME_COLOR(LabelColor, 0x44212529);
THEME_COLOR(DarkBackgroundColor, 0xff212529);
THEME_COLOR(OverlayShadowColor, 0xbb000000);
THEME_COLOR(ShadowColor, 0x88000000);

class AnimatedLines : public visage::Frame {
public:
  static constexpr int kNumLines = 2;
  static constexpr int kNumPoints = 400;

  AnimatedLines() {
    for (auto& graph_line : graph_lines_) {
      graph_line = std::make_unique<visage::GraphLine>(kNumPoints);
      addChild(graph_line.get());
    }
  }

  void resized() override {
    int line_offset = height() / kNumLines;
    for (int i = 0; i < kNumLines; ++i) {
      graph_lines_[i]->setBounds(0, line_offset * i, width(), line_offset);
      graph_lines_[i]->setFill(true);
    }
  }

  void draw(visage::Canvas& canvas) override {
    double render_time = canvas.time();
    for (int r = 0; r < kNumLines; ++r) {
      int render_height = graph_lines_[r]->height();
      int render_width = graph_lines_[r]->width();
      int line_height = render_height * 0.9f;
      int offset = render_height * 0.05f;

      float position = 0.0f;
      for (int i = 0; i < kNumPoints; ++i) {
        float t = i / (kNumPoints - 1.0f);
        float delta = std::min(t, 1.0f - t);
        position += 0.1f * delta * delta + 0.003f;
        double phase = (render_time + r) * 0.5;
        graph_lines_[r]->setXAt(i, t * render_width);
        graph_lines_[r]->setYAt(i, offset + (sin1(phase + position) * 0.5f + 0.5f) * line_height);
      }
    }

    redraw();
  }

private:
  std::unique_ptr<visage::GraphLine> graph_lines_[kNumLines];
};

class DragDropSource : public visage::Frame {
  void draw(visage::Canvas& canvas) override {
    canvas.setPaletteColor(kDarkBackgroundColor);
    canvas.roundedRectangle(0, 0, width(), height(), height() / 16);

    canvas.setPaletteColor(kTextColor);

    const visage::Font font(height() / 4, resources::fonts::Lato_Regular_ttf);
    if (dragging_)
      canvas.text("Dragging source file", font, visage::Font::kCenter, 0, 0, width(), height());
    else
      canvas.text("Drag source", font, visage::Font::kCenter, 0, 0, width(), height());
  }

  bool isDragDropSource() override { return true; }

  std::string startDragDropSource() override {
    redraw();
    dragging_ = true;
    source_file_ = visage::createTemporaryFile("txt");
    visage::replaceFileWithText(source_file_, "Example drag and drop source file.");
    return source_file_.string();
  }

  void cleanupDragDropSource() override {
    redraw();
    dragging_ = false;
    if (std::filesystem::exists(source_file_))
      std::filesystem::remove(source_file_);
  }

private:
  bool dragging_ = false;
  visage::File source_file_;
};

class DragDropTarget : public visage::Frame {
  void draw(visage::Canvas& canvas) override {
    canvas.setPaletteColor(kDarkBackgroundColor);
    canvas.roundedRectangle(0, 0, width(), height(), height() / 16);

    canvas.setPaletteColor(kTextColor);

    const visage::Font font(height() / 4, resources::fonts::Lato_Regular_ttf);
    if (dragging_)
      canvas.text("Dragging " + filename_, font, visage::Font::kCenter, 0, 0, width(), height());
    else if (dropped_)
      canvas.text("Dropped " + filename_, font, visage::Font::kCenter, 0, 0, width(), height());
    else
      canvas.text("Drag destination", font, visage::Font::kCenter, 0, 0, width(), height());
  }

  bool receivesDragDropFiles() override { return true; }
  std::string dragDropFileExtensionRegex() override { return ".*"; }

  void dragFilesEnter(const std::vector<std::string>& paths) override {
    dragging_ = true;
    dropped_ = false;
    filename_ = visage::fileName(paths[0]);
    redraw();
  }

  void dragFilesExit() override {
    dragging_ = false;
    redraw();
  }

  void dropFiles(const std::vector<std::string>& paths) override {
    dragging_ = false;
    dropped_ = true;
    redraw();
  }

private:
  std::string filename_;
  bool dragging_ = false;
  bool dropped_ = false;
};

class DragDropExample : public visage::Frame {
public:
  DragDropExample() {
    addChild(&source_);
    addChild(&target_);
  }

  void resized() {
    int padding = height() / 16;
    int h = (height() - padding) / 2;
    source_.setBounds(0, 0, width(), h);
    target_.setBounds(0, height() - h, width(), h);
  }

private:
  DragDropSource source_;
  DragDropTarget target_;
};

class CachedText : public visage::Frame {
public:
  CachedText() {
    setCached(true);
    std::string text = "This is a bunch of center justified and wrapped text fit into an area.";
    text_ = std::make_unique<visage::Text>(text, visage::Font(10, resources::fonts::Lato_Regular_ttf));
    text_->setMultiLine(true);
    text_->setJustification(visage::Font::kCenter);
  }

  void draw(visage::Canvas& canvas) override {
    int font_height = height() / 6;
    text_->setFont(visage::Font(font_height, resources::fonts::Lato_Regular_ttf));
    canvas.setColor(0xffffffff);
    canvas.text(text_.get(), 0, 0, width(), height());
  }

private:
  std::unique_ptr<visage::Text> text_;
};

ExamplesFrame::ExamplesFrame() {
  static constexpr int kBars = kNumBars;

  drag_drop_ = std::make_unique<DragDropExample>();
  addChild(drag_drop_.get());

  bar_list_ = std::make_unique<visage::BarList>(kBars);
  addChild(bar_list_.get());
  bar_list_->setHorizontalAntiAliasing(false);

  bar_list_->onDraw() += [this](visage::Canvas& canvas) {
    double render_time = canvas.time();
    float space = 1;
    float bar_width = (bar_list_->width() + space) / kNumBars;
    int bar_height = bar_list_->height();
    for (int i = 0; i < kNumBars; ++i) {
      float x = bar_width * i;
      float current_height = (sin1((render_time * 60.0 + i * 30) / 600.0f) + 1.0f) * 0.5f * bar_height;
      bar_list_->positionBar(i, x, current_height, bar_width - space, bar_height - current_height);
    }
  };

  visage::Font font(24, resources::fonts::Lato_Regular_ttf);
  shader_quad_ = std::make_unique<visage::ShaderQuad>(resources::shaders::vs_shader_quad,
                                                      resources::shaders::fs_shader_quad,
                                                      visage::BlendMode::Alpha);
  addChild(shader_quad_.get());

  icon_button_ = std::make_unique<visage::ToggleIconButton>(resources::icons::check_circle_svg.data,
                                                            resources::icons::check_circle_svg.size, true);
  addChild(icon_button_.get());

  text_button_ = std::make_unique<visage::ToggleTextButton>("Toggle", font);
  addChild(text_button_.get());

  text_ = std::make_unique<CachedText>();
  addChild(text_.get());

  ui_button_ = std::make_unique<visage::UiButton>("Trigger Overlay", font);
  ui_button_->onToggle() = [this](visage::Button* button, bool toggled) {
    on_show_overlay_.callback();
  };

  addChild(ui_button_.get());
  ui_button_->setToggleOnMouseDown(true);

  action_button_ = std::make_unique<visage::UiButton>("Popup Menu", font);
  addChild(action_button_.get());
  action_button_->setActionButton();
  action_button_->onToggle() = [this](visage::Button* button, bool toggled) {
    visage::PopupMenu menu;
    menu.addOption(0, "Take Screenshot");
    menu.addOption(1, "Hello World");
    menu.addBreak();
    menu.addOption(2, "Another Item 1");
    visage::PopupMenu sub_menu("Sub Menu");
    sub_menu.addOption(3, "Sub Item 1");
    sub_menu.addBreak();
    sub_menu.addOption(4, "Sub Item 2");
    sub_menu.addOption(5, "Sub Item 3");
    sub_menu.addOption(6, "Sub Item 4");
    menu.addSubMenu(sub_menu);
    visage::PopupMenu sub_menu2("Other Sub Menu");
    sub_menu2.addOption(7, "Other Sub Item 1");
    sub_menu2.addBreak();
    sub_menu2.addOption(8, "Other Sub Item 2");
    sub_menu2.addOption(9, "Other Sub Item 3");
    sub_menu2.addOption(10, "Other Sub Item 4");
    menu.addSubMenu(sub_menu2);
    menu.addOption(11, "Another Item 3");
    menu.addBreak();
    menu.addOption(12, "Force Crash");

    menu.onSelection() = [this](int id) {
      if (id == 0) {
        visage::File file = visage::hostExecutable().parent_path() / "screenshot.png";
        on_screenshot_.callback(file.string());
      }
      else if (id == 12)
        VISAGE_FORCE_CRASH();
    };

    menu.show(action_button_.get());
  };
  action_button_->setToggleOnMouseDown(true);

  shapes_ = std::make_unique<visage::Frame>();
  addChild(shapes_.get());
  shapes_->onDraw() = [this](visage::Canvas& canvas) {
    double render_time = canvas.time();

    float center_radians = render_time * 1.2f;
    double phase = render_time * 0.1;
    float radians = kHalfPi * sin1(phase) + kHalfPi;

    int min_shape_padding = heightScale() * 10.0f;
    int shape_width = std::min(shapes_->width() / 5, shapes_->height() / 2) - min_shape_padding;
    int shape_padding_x = shapes_->width() / 5 - shape_width;
    int shape_padding_y = shapes_->height() / 2 - shape_width;

    int shape_x = shape_padding_x / 2;
    int shape_y = 0;
    int shape_y2 = shape_y + shape_width + shape_padding_y;
    int roundness = shape_width / 8;

    float shape_phase = canvas.time() * 0.1f;
    shape_phase -= std::floor(shape_phase);
    float shape_cycle = sin1(shape_phase) * 0.5f + 0.5f;
    float thickness = shape_width * shape_cycle / 8.0f + 1.0f;

    canvas.setPaletteColor(kShapeColor);
    canvas.rectangle(shape_x, shape_y, shape_width, shape_width);
    canvas.rectangleBorder(shape_x, shape_y2, shape_width, shape_width, thickness);
    canvas.circle(shape_x + shape_width + shape_padding_x, shape_y, shape_width);
    canvas.ring(shape_x + shape_width + shape_padding_x, shape_y2, shape_width, thickness);
    canvas.roundedRectangle(shape_x + 2 * (shape_width + shape_padding_x), shape_y, shape_width,
                            shape_width, roundness);
    canvas.roundedRectangleBorder(shape_x + 2 * (shape_width + shape_padding_x), shape_y2,
                                  shape_width, shape_width, roundness, thickness);
    canvas.arc(shape_x + 3 * (shape_width + shape_padding_x), shape_y, shape_width, thickness,
               center_radians, radians, false);
    canvas.arc(shape_x + 3 * (shape_width + shape_padding_x), shape_y2, shape_width, thickness,
               center_radians, radians, true);

    float max_separation = shape_padding_x / 2.0f;
    float separation = shape_cycle * max_separation;
    int triangle_x = shape_x + 4 * (shape_width + shape_padding_x) + max_separation;
    int triangle_y = shape_y + max_separation;
    int triangle_width = (shape_width - 2.0f * max_separation) / 2.0f;
    canvas.triangleDown(triangle_x, triangle_y - separation, triangle_width);
    canvas.triangleRight(triangle_x - separation, triangle_y, triangle_width);
    canvas.triangleUp(triangle_x, triangle_y + triangle_width + separation, triangle_width);
    canvas.triangleLeft(triangle_x + triangle_width + separation, triangle_y, triangle_width);

    float segment_x = shape_x + 4 * (shape_width + shape_padding_x);
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

    shapes_->redraw();
  };

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

  animated_lines_ = std::make_unique<AnimatedLines>();
  addChild(animated_lines_.get());

  setIgnoresMouseEvents(true, true);
}

ExamplesFrame::~ExamplesFrame() = default;

void ExamplesFrame::resized() {
  int w = width();
  int h = height();
  int x_division = w / 2;
  int right_width = w - x_division;
  int section_height = h / 4;
  int section_head_height = section_height / 4;
  int section_body_height = section_height - section_head_height;

  animated_lines_->setBounds(0, section_head_height, x_division, section_body_height);

  bar_list_->setBounds(0, section_height + section_head_height, x_division / 2, section_body_height);
  int shader_x = x_division / 2 + (x_division / 2 - section_body_height) / 2;
  shader_quad_->setBounds(shader_x, section_height + section_head_height, section_body_height,
                          section_body_height);
  shapes_->setBounds(x_division, bar_list_->y(), right_width, section_body_height);

  int font_height = section_head_height * 0.45f;
  int text_y = 2 * section_height + section_head_height;
  int text_section_padding = w / 50;
  int text_section_width = (w - text_section_padding) / 4 - text_section_padding;
  int padding = section_body_height / 16;
  int single_line_height = (section_body_height + padding) / 3 - padding;
  int margin = font_height / 3;
  left_text_editor_->setBounds(text_section_padding, text_y, text_section_width, single_line_height);
  left_text_editor_->setFont(visage::Font(font_height, resources::fonts::Lato_Regular_ttf));
  left_text_editor_->setBackgroundRounding(margin / 2);

  number_editor_->setBounds(text_section_padding, text_y + single_line_height + padding,
                            text_section_width, single_line_height);
  number_editor_->setFont(visage::Font(font_height, resources::fonts::Lato_Regular_ttf));
  number_editor_->setBackgroundRounding(margin / 2);

  right_text_editor_->setBounds(text_section_padding, text_y + 2 * (single_line_height + padding),
                                text_section_width, single_line_height);
  right_text_editor_->setFont(visage::Font(font_height, resources::fonts::Lato_Regular_ttf));
  right_text_editor_->setBackgroundRounding(margin / 2);

  text_editor_->setFont(visage::Font(font_height, resources::fonts::Lato_Regular_ttf));
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

  drag_drop_->setBounds(x_division + right_width / 2, widget_y, right_width / 2, h - widget_y);
}

void ExamplesFrame::draw(visage::Canvas& canvas) {
  int w = width();
  int h = height();

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
  canvas.text("Image", font, visage::Font::kCenter, x_division + right_width / 2,
              2 * section_height, right_width / 2, section_head_height);

  canvas.text("Controls", font, visage::Font::kCenter, 0, 3 * section_height, x_division,
              section_head_height);
  canvas.text("Buttons", font, visage::Font::kCenter, x_division, 3 * section_height,
              right_width / 2, section_head_height);
  canvas.text("Drag + Drop", font, visage::Font::kCenter, x_division + right_width / 2,
              3 * section_height, right_width / 2, section_head_height);

  int icon_width = std::min(w / 4, text_editor_->height());
  int icon_x = x_division + right_width / 2 + right_width / 4 - icon_width / 2;
  int icon_y = text_editor_->y();

  canvas.setColor(0xffffffff);
  canvas.image(resources::images::test_png, icon_x, icon_y, icon_width, icon_width);

  if (shadow_amount_) {
    float shadow_mult = std::max(0.0f, 2.0f * shadow_amount_ - 1.0f);
    shadow_mult = shadow_mult * shadow_mult;
    canvas.setColor(canvas.color(kOverlayShadowColor).withMultipliedAlpha(shadow_mult));
    canvas.roundedRectangle(shadow_bounds_.x(), shadow_bounds_.y(), shadow_bounds_.width(),
                            shadow_bounds_.height(), shadow_rounding_);
  }
}