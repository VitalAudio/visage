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

/// Custom gradient definitions:
///
/// OkLab: https://bottosson.github.io/posts/oklab/
/// Viridis: https://sjmgarnier.github.io/viridis/articles/intro-to-viridis.html

#include "embedded/example_fonts.h"

#include <array>
#include <visage/app.h>
#include <visage/widgets.h>

using namespace visage::dimension;

static constexpr unsigned int kViridisMapResolution = 128;
static constexpr unsigned int kViridisMap[kViridisMapResolution] = {
  0xFF440154, 0xFF450457, 0xFF46075A, 0xFF460A5D, 0xFF470D60, 0xFF471063, 0xFF471365, 0xFF481668,
  0xFF48186A, 0xFF481B6D, 0xFF481E6F, 0xFF482072, 0xFF482374, 0xFF482576, 0xFF482878, 0xFF472A7A,
  0xFF472D7B, 0xFF472F7D, 0xFF46327F, 0xFF463480, 0xFF453781, 0xFF443983, 0xFF443C84, 0xFF433E85,
  0xFF424086, 0xFF414387, 0xFF404588, 0xFF3F4788, 0xFF3E4A89, 0xFF3D4C8A, 0xFF3C4E8A, 0xFF3B508B,
  0xFF3A528B, 0xFF39558C, 0xFF38578C, 0xFF37598C, 0xFF375B8D, 0xFF365D8D, 0xFF355F8D, 0xFF34618D,
  0xFF33638D, 0xFF32658E, 0xFF31678E, 0xFF30698E, 0xFF2F6B8E, 0xFF2E6D8E, 0xFF2E6F8E, 0xFF2D718E,
  0xFF2C738E, 0xFF2B758E, 0xFF2A778E, 0xFF2A798E, 0xFF297A8E, 0xFF287C8E, 0xFF277E8E, 0xFF27808E,
  0xFF26828E, 0xFF25848E, 0xFF24868E, 0xFF24888E, 0xFF238A8D, 0xFF228B8D, 0xFF228D8D, 0xFF218F8D,
  0xFF20918C, 0xFF20938C, 0xFF1F958B, 0xFF1F978B, 0xFF1F998A, 0xFF1F9A8A, 0xFF1E9C89, 0xFF1F9E89,
  0xFF1FA088, 0xFF1FA287, 0xFF20A486, 0xFF21A685, 0xFF22A884, 0xFF23A983, 0xFF25AB82, 0xFF27AD81,
  0xFF29AF80, 0xFF2BB17E, 0xFF2EB37D, 0xFF30B47B, 0xFF33B67A, 0xFF36B878, 0xFF39BA76, 0xFF3DBB74,
  0xFF40BD73, 0xFF44BF71, 0xFF47C06F, 0xFF4BC26C, 0xFF4FC46A, 0xFF53C568, 0xFF57C766, 0xFF5BC863,
  0xFF60CA61, 0xFF64CB5E, 0xFF69CD5B, 0xFF6DCE59, 0xFF72CF56, 0xFF77D153, 0xFF7CD250, 0xFF81D34D,
  0xFF86D44A, 0xFF8BD647, 0xFF90D743, 0xFF95D840, 0xFF9AD93D, 0xFF9FDA39, 0xFFA5DB36, 0xFFAADC32,
  0xFFAFDD2F, 0xFFB5DD2B, 0xFFBADE28, 0xFFBFDF25, 0xFFC5E022, 0xFFCAE11F, 0xFFD0E11C, 0xFFD5E21A,
  0xFFDAE319, 0xFFDFE318, 0xFFE4E419, 0xFFEAE41A, 0xFFEFE51C, 0xFFF4E61E, 0xFFF8E621, 0xFFFDE725
};

visage::Color sampleViridis(float t) {
  int index = std::clamp<int>(std::round((1.0f - t) * (kViridisMapResolution - 1)), 0,
                              kViridisMapResolution - 1);
  return visage::Color(kViridisMap[index]);
}

visage::Color sampleOkLab(float t) {
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kL = 0.82f;
  static constexpr float kC = 0.15f;
  static constexpr float kOffset = 0.45f;

  float a = kC * std::cos(2.0f * kPi * t + kOffset);
  float b = kC * std::sin(2.0f * kPi * t + kOffset);

  float l_ = kL + 0.3963377774f * a + 0.2158037573f * b;
  float m_ = kL - 0.1055613458f * a - 0.0638541728f * b;
  float s_ = kL - 0.0894841775f * a - 1.2914855480f * b;

  float l = l_ * l_ * l_;
  float m = m_ * m_ * m_;
  float s = s_ * s_ * s_;

  return { 1.0f, +4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s,
           -1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s,
           -0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s };
}

void setPattern(visage::Gradient& gradient, const std::string& pattern) {
  if (pattern == "Single") {
    gradient.setRepeat(false);
    gradient.setReflect(false);
  }
  else if (pattern == "Repeat") {
    gradient.setRepeat(true);
    gradient.setReflect(false);
  }
  else {
    gradient.setRepeat(false);
    gradient.setReflect(true);
  }
}

class PointsFrame : public visage::Frame {
public:
  static constexpr int kDragRadius = 20;
  static constexpr int kDotRadius = 5;
  enum ActivePoint {
    kNone,
    kFrom,
    kTo
  };

  void draw(visage::Canvas& canvas) override {
    visage::Brush points = visage::Brush::linear(visage::Gradient(0xffffff00, 0xff00ffff), point1_, point2_);
    canvas.setColor(points);
    canvas.roundedRectangle(0, 0, width(), height(), 18.0f);

    canvas.setColor(0xff000000);
    visage::Font font(20, resources::fonts::Lato_Regular_ttf);
    canvas.text("Linear Points", font, visage::Font::kCenter, 0, 0, width(), height());

    canvas.setColor(mouse_down_ ? 0xaaffffff : 0x66ffffff);
    if (active_point_ == kFrom)
      canvas.circle(point1_.x - kDragRadius, point1_.y - kDragRadius, 2.0f * kDragRadius);

    else if (active_point_ == kTo)
      canvas.circle(point2_.x - kDragRadius, point2_.y - kDragRadius, 2.0f * kDragRadius);

    canvas.setColor(0xff000000);
    canvas.circle(point1_.x - kDotRadius, point1_.y - kDotRadius, 2.0f * kDotRadius);
    canvas.circle(point2_.x - kDotRadius, point2_.y - kDotRadius, 2.0f * kDotRadius);
  }

  void setActivePoint(ActivePoint active_point) {
    if (active_point == active_point_)
      return;

    active_point_ = active_point;
    redraw();
  }

  void mouseMove(const visage::MouseEvent& e) override {
    visage::Point point = e.position;
    visage::Point delta_from = point - point1_;
    visage::Point delta_to = point - point2_;

    if (delta_from.squareMagnitude() < kDragRadius * kDragRadius &&
        delta_from.squareMagnitude() < delta_to.squareMagnitude())
      setActivePoint(kFrom);
    else if (delta_to.squareMagnitude() < kDragRadius * kDragRadius)
      setActivePoint(kTo);
    else
      setActivePoint(kNone);
  }

  void mouseDown(const visage::MouseEvent& e) override {
    if (active_point_ == kNone)
      return;

    mouse_down_ = true;
    redraw();
  }

  void mouseUp(const visage::MouseEvent& e) override {
    if (active_point_ == kNone)
      return;

    mouse_down_ = false;
    redraw();
  }

  void mouseDrag(const visage::MouseEvent& e) override {
    if (active_point_ == kNone)
      return;

    if (active_point_ == kFrom)
      point1_ = localBounds().clampPoint(e.position);
    else if (active_point_ == kTo)
      point2_ = localBounds().clampPoint(e.position);
    redraw();
  }

  void setGradient(const visage::Gradient& gradient) {
    gradient_ = gradient;
    redraw();
  }

  void setPoints(const visage::Point& point1, const visage::Point& point2) {
    point1_ = point1;
    point2_ = point2;
    redraw();
  }

protected:
  ActivePoint active_point_ = kNone;
  bool mouse_down_ = false;
  visage::Point point1_;
  visage::Point point2_;
  visage::Gradient gradient_;
};

class LinearPointsFrame : public PointsFrame {
public:
  void draw(visage::Canvas& canvas) override {
    visage::Brush points = visage::Brush::linear(gradient_, point1_, point2_);
    canvas.setColor(points);
    canvas.roundedRectangle(0, 0, width(), height(), 18.0f);

    canvas.setColor(0xff000000);
    visage::Font font(20, resources::fonts::Lato_Regular_ttf);
    canvas.text("Linear Gradient", font, visage::Font::kCenter, 0, 0, width(), height());

    canvas.setColor(mouse_down_ ? 0xaaffffff : 0x66ffffff);
    if (active_point_ == kFrom)
      canvas.circle(point1_.x - kDragRadius, point1_.y - kDragRadius, 2.0f * kDragRadius);

    else if (active_point_ == kTo)
      canvas.circle(point2_.x - kDragRadius, point2_.y - kDragRadius, 2.0f * kDragRadius);

    canvas.setColor(0xff000000);
    canvas.circle(point1_.x - kDotRadius, point1_.y - kDotRadius, 2.0f * kDotRadius);
    canvas.circle(point2_.x - kDotRadius, point2_.y - kDotRadius, 2.0f * kDotRadius);
  }
};

class RadialPointsFrame : public PointsFrame {
public:
  void draw(visage::Canvas& canvas) override {
    auto radius = (point2_ - point1_).length();
    visage::Brush radial = visage::Brush::radial(gradient_, point1_, radius);
    canvas.setColor(radial);
    canvas.roundedRectangle(0, 0, width(), height(), 18.0f);

    canvas.setColor(0xff000000);
    visage::Font font(20, resources::fonts::Lato_Regular_ttf);
    canvas.text("Radial Gradient", font, visage::Font::kCenter, 0, 0, width(), height());

    canvas.setColor(mouse_down_ ? 0xaaffffff : 0x66ffffff);
    if (active_point_ == kFrom)
      canvas.circle(point1_.x - kDragRadius, point1_.y - kDragRadius, 2.0f * kDragRadius);

    else if (active_point_ == kTo)
      canvas.circle(point2_.x - kDragRadius, point2_.y - kDragRadius, 2.0f * kDragRadius);

    canvas.setColor(0xff000000);
    canvas.circle(point1_.x - kDotRadius, point1_.y - kDotRadius, 2.0f * kDotRadius);
    canvas.circle(point2_.x - kDotRadius, point2_.y - kDotRadius, 2.0f * kDotRadius);
  }
};

int runExample() {
  visage::ApplicationWindow app;

  app.layout().setFlex(true);
  app.layout().setFlexGap(8);
  app.layout().setPadding(8);

  visage::Frame gradients;
  gradients.layout().setFlex(true);
  gradients.layout().setFlexRows(false);
  gradients.layout().setFlexGap(8);
  gradients.layout().setFlexGrow(1.0f);
  app.addChild(gradients);

  app.onDraw() = [&app](visage::Canvas& canvas) {
    canvas.setColor(0xff222222);
    canvas.fill(0, 0, app.width(), app.height());
  };

  visage::Gradient gradient(0xffffff00, 0xff00aaff);

  std::unique_ptr<PointsFrame> linear_points_frame = std::make_unique<LinearPointsFrame>();
  gradients.addChild(linear_points_frame.get());
  linear_points_frame->layout().setFlexGrow(1.0f);
  linear_points_frame->setGradient(gradient);

  std::unique_ptr<PointsFrame> radial_points_frame = std::make_unique<RadialPointsFrame>();
  gradients.addChild(radial_points_frame.get());
  radial_points_frame->layout().setFlexGrow(1.0f);
  radial_points_frame->setGradient(gradient);

  visage::Font font(20, resources::fonts::Lato_Regular_ttf);
  visage::Frame controls;
  controls.layout().setFlexGrow(0.15f);
  controls.layout().setFlex(true);
  controls.layout().setFlexRows(false);
  controls.layout().setFlexGap(8);

  std::string pattern = "Single";
  visage::UiButton pattern_button("Pattern: " + pattern);
  pattern_button.setFont(font);
  pattern_button.layout().setFlexGrow(1.0f);
  controls.addChild(pattern_button);

  pattern_button.onToggle() = [&](visage::Button* button, bool on) {
    if (pattern == "Single")
      pattern = "Repeat";
    else if (pattern == "Repeat")
      pattern = "Reflect";
    else
      pattern = "Single";

    setPattern(gradient, pattern);
    linear_points_frame->setGradient(gradient);
    radial_points_frame->setGradient(gradient);
    pattern_button.setText("Pattern: " + pattern);
  };

  std::map<std::string, visage::Gradient> gradients_map = {
    { "Two Color", visage::Gradient(0xffffff00, 0xff00aaff) },
    { "Rainbow", visage::Gradient(0xffff0000, 0xffffff00, 0xff00ff00, 0xff00ffff, 0xff0000ff,
                                  0xffff00ff, 0xffff0000) },
    { "Rainbow (OkLab)", visage::Gradient::fromSampleFunction(100, sampleOkLab) },
    { "Viridis", visage::Gradient::fromSampleFunction(kViridisMapResolution, sampleViridis) }
  };

  visage::UiButton color_button("Gradient: Two Color");
  color_button.setFont(font);
  color_button.layout().setFlexGrow(1.0f);
  controls.addChild(color_button);

  app.addChild(controls);
  auto gradient_it = gradients_map.find("Two Color");
  color_button.onToggle() = [&](visage::Button* button, bool on) {
    gradient_it = std::next(gradient_it);
    if (gradient_it == gradients_map.end())
      gradient_it = gradients_map.begin();

    gradient = gradient_it->second;
    setPattern(gradient, pattern);
    linear_points_frame->setGradient(gradient);
    radial_points_frame->setGradient(gradient);
    color_button.setText("Gradient: " + gradient_it->first);
  };

  app.setTitle("Visage Gradient Example");
  app.show(80_vmin, 60_vmin);

  visage::Point from(linear_points_frame->width() * 0.33f, linear_points_frame->height() * 0.33f);
  visage::Point to(linear_points_frame->width() * 0.66f, linear_points_frame->height() * 0.66f);
  linear_points_frame->setPoints(from, to);
  radial_points_frame->setPoints(from, to);
  app.runEventLoop();
  return 0;
}
