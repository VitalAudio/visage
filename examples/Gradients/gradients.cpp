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

std::unique_ptr<visage::Frame> createFrame(visage::ApplicationWindow& app, visage::Brush& brush,
                                           std::string text) {
  std::unique_ptr<visage::Frame> frame = std::make_unique<visage::Frame>();
  app.addChild(frame.get());
  frame->layout().setFlexGrow(1.0f);
  frame->onDraw() = [f = frame.get(), brush, text](visage::Canvas& canvas) {
    canvas.setColor(brush);
    canvas.roundedRectangle(0, 0, f->width(), f->height(), 10.0f * canvas.dpiScale());

    canvas.setColor(0xff000000);
    visage::Font font(20.0f * canvas.dpiScale(), resources::fonts::Lato_Regular_ttf);
    canvas.text(text, font, visage::Font::kCenter, 0, 0, f->width(), f->height());
  };
  return frame;
}

visage::Color sampleViridis(float t) {
  int index = std::round((1.0f - t) * (kViridisMapResolution - 1));
  return visage::Color(kViridisMap[static_cast<int>(index)]);
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

class LinearPointsFrame : public visage::Frame {
public:
  static constexpr int kDragRadius = 20;
  static constexpr int kDotRadius = 5;
  enum ActivePoint {
    kNone,
    kFrom,
    kTo
  };

  void draw(visage::Canvas& canvas) {
    visage::Brush points = visage::Brush::linear(visage::Gradient(0xffffff00, 0xff00ffff),
                                                 from_point_, to_point_);
    canvas.setColor(points);
    canvas.roundedRectangle(0, 0, width(), height(), 10.0f * dpiScale());

    canvas.setColor(0xff000000);
    visage::Font font(20.0f * canvas.dpiScale(), resources::fonts::Lato_Regular_ttf);
    canvas.text("Linear Points", font, visage::Font::kCenter, 0, 0, width(), height());

    float drag_radius = canvas.dpiScale() * kDragRadius;
    canvas.setColor(mouse_down_ ? 0xaaffffff : 0x66ffffff);
    if (active_point_ == kFrom)
      canvas.circle(from_point_.x - drag_radius, from_point_.y - drag_radius, 2.0f * drag_radius);

    else if (active_point_ == kTo)
      canvas.circle(to_point_.x - drag_radius, to_point_.y - drag_radius, 2.0f * drag_radius);

    float dot_radius = canvas.dpiScale() * kDotRadius;
    canvas.setColor(0xff000000);
    canvas.circle(from_point_.x - dot_radius, from_point_.y - dot_radius, 2.0f * dot_radius);
    canvas.circle(to_point_.x - dot_radius, to_point_.y - dot_radius, 2.0f * dot_radius);
  }

  void setActivePoint(ActivePoint active_point) {
    if (active_point == active_point_)
      return;

    active_point_ = active_point;
    redraw();
  }

  void mouseMove(const visage::MouseEvent& e) {
    int radius = dpiScale() * kDragRadius;
    visage::FloatPoint point = e.position;
    visage::FloatPoint delta_from = point - from_point_;
    visage::FloatPoint delta_to = point - to_point_;

    if (delta_from.squareMagnitude() < radius * radius &&
        delta_from.squareMagnitude() < delta_to.squareMagnitude())
      setActivePoint(kFrom);
    else if (delta_to.squareMagnitude() < radius * radius)
      setActivePoint(kTo);
    else
      setActivePoint(kNone);
  }

  void mouseDown(const visage::MouseEvent& e) {
    if (active_point_ == kNone)
      return;

    mouse_down_ = true;
    redraw();
  }

  void mouseUp(const visage::MouseEvent& e) {
    if (active_point_ == kNone)
      return;

    mouse_down_ = true;
    redraw();
  }

  void mouseDrag(const visage::MouseEvent& e) {
    if (active_point_ == kNone)
      return;

    if (active_point_ == kFrom)
      from_point_ = localBounds().clampPoint(e.position);
    else if (active_point_ == kTo)
      to_point_ = localBounds().clampPoint(e.position);
    redraw();
  }

private:
  ActivePoint active_point_ = kNone;
  bool mouse_down_ = false;
  visage::FloatPoint from_point_ = { 20.0f, 20.0f };
  visage::FloatPoint to_point_ = { 60.0f, 100.0f };
};

int runExample() {
  visage::ApplicationWindow app;

  app.layout().setFlex(true);
  app.layout().setFlexRows(false);
  app.layout().setFlexGap(2_vmin);
  app.layout().setPadding(2_vmin);

  app.onDraw() = [&app](visage::Canvas& canvas) {
    canvas.setColor(0xff222222);
    canvas.fill(0, 0, app.width(), app.height());
  };

  visage::Brush linear = visage::Brush::vertical(0xffffff00, 0xff00ffff);
  std::unique_ptr<visage::Frame> linear_frame = createFrame(app, linear, "Linear");

  visage::Brush rainbow = visage::Brush::vertical(visage::Gradient(0xffff0000, 0xffffff00,
                                                                   0xff00ff00, 0xff00ffff, 0xff0000ff,
                                                                   0xffff00ff, 0xffff0000));
  std::unique_ptr<visage::Frame> rainbow_frame = createFrame(app, rainbow, "Rainbow");

  visage::Brush ok_lab = visage::Brush::vertical(visage::Gradient::fromSampleFunction(100, sampleOkLab));
  std::unique_ptr<visage::Frame> ok_lab_frame = createFrame(app, ok_lab, "OkLab Rainbow");

  visage::Brush viridis = visage::Brush::vertical(visage::Gradient::fromSampleFunction(kViridisMapResolution,
                                                                                       sampleViridis));
  std::unique_ptr<visage::Frame> viridis_frame = createFrame(app, viridis, "Viridis");

  std::unique_ptr<visage::Frame> linear_points_frame = std::make_unique<LinearPointsFrame>();
  app.addChild(linear_points_frame.get());
  linear_points_frame->layout().setFlexGrow(1.0f);

  app.setTitle("Visage Gradient Example");
  app.show(80_vmin, 60_vmin);
  app.runEventLoop();
  return 0;
}
