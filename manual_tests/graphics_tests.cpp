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

#include "embedded/example_images.h"
#include "graphics_cells.h"

#include <cmath>
#include <random>

namespace manual {
  namespace {
    constexpr float kPi = 3.14159265358979323846f;
#ifdef NDEBUG
    constexpr bool kDebugBuild = false;
#else
    constexpr bool kDebugBuild = true;
#endif
    constexpr unsigned int kShapeColor = 0xffaaff88;

    visage::Path starPath(float cx, float cy, float outer, float inner, int points = 5) {
      visage::Path path;
      for (int i = 0; i < 2 * points; ++i) {
        float angle = i * kPi / points - kPi * 0.5f;
        float r = (i % 2) ? inner : outer;
        visage::Point point(cx + r * std::cos(angle), cy + r * std::sin(angle));
        if (i == 0)
          path.moveTo(point);
        else
          path.lineTo(point);
      }
      path.close();
      return path;
    }

    visage::Path pentagram(float cx, float cy, float r) {
      visage::Path path;
      for (int i = 0; i < 5; ++i) {
        float angle = i * 4.0f * kPi / 5.0f - kPi * 0.5f;
        visage::Point point(cx + r * std::cos(angle), cy + r * std::sin(angle));
        if (i == 0)
          path.moveTo(point);
        else
          path.lineTo(point);
      }
      path.close();
      return path;
    }

    visage::Path arrowPath(float size) {
      visage::Path path;
      path.moveTo(0, -size * 0.08f);
      path.lineTo(size * 0.6f, -size * 0.08f);
      path.lineTo(size * 0.6f, -size * 0.25f);
      path.lineTo(size, 0);
      path.lineTo(size * 0.6f, size * 0.25f);
      path.lineTo(size * 0.6f, size * 0.08f);
      path.lineTo(0, size * 0.08f);
      path.close();
      return path;
    }

    struct ShapeSettings {
      float thickness = 6.0f;
      bool animate = true;
      bool unmatched_save = false;
    };

    class ShapesPage : public CellPage {
    public:
      ShapesPage() : CellPage(6) {
        auto s = &settings_;
        auto square = [](float w, float h) { return std::min(w, h) * 0.8f; };
        auto origin = [square](float w, float h) {
          float size = square(w, h);
          return visage::Point((w - size) * 0.5f, (h - size) * 0.5f);
        };

        addCell("rectangle / rectangleBorder", "Filled square on the left, outline on the right.",
                [s](visage::Canvas& canvas, float w, float h) {
                  float size = std::min(w * 0.45f, h * 0.8f);
                  canvas.setColor(kShapeColor);
                  canvas.rectangle(w * 0.25f - size * 0.5f, (h - size) * 0.5f, size, size);
                  canvas.rectangleBorder(w * 0.75f - size * 0.5f, (h - size) * 0.5f, size, size,
                                         s->thickness);
                });
        addCell("roundedRectangle / Border", "Smooth even corners on both, border has even width all around.",
                [s](visage::Canvas& canvas, float w, float h) {
                  float size = std::min(w * 0.45f, h * 0.8f);
                  canvas.setColor(kShapeColor);
                  canvas.roundedRectangle(w * 0.25f - size * 0.5f, (h - size) * 0.5f, size, size,
                                          size * 0.2f);
                  canvas.roundedRectangleBorder(w * 0.75f - size * 0.5f, (h - size) * 0.5f, size,
                                                size, size * 0.2f, s->thickness);
                });
        addCell("circle / ring", "Round disk and a ring of even width.",
                [s](visage::Canvas& canvas, float w, float h) {
                  float size = std::min(w * 0.45f, h * 0.8f);
                  canvas.setColor(kShapeColor);
                  canvas.circle(w * 0.25f - size * 0.5f, (h - size) * 0.5f, size);
                  canvas.ring(w * 0.75f - size * 0.5f, (h - size) * 0.5f, size, s->thickness);
                });
        addCell("squircle / squircleBorder",
                "No dot in the middle of the filled one, no blob in the middle of "
                "the border.",
                [s](visage::Canvas& canvas, float w, float h) {
                  float size = std::min(w * 0.45f, h * 0.8f);
                  canvas.setColor(kShapeColor);
                  canvas.squircle(w * 0.25f - size * 0.5f, (h - size) * 0.5f, size);
                  canvas.squircleBorder(w * 0.75f - size * 0.5f, (h - size) * 0.5f, size, 4.0f, s->thickness);
                });
        addCell("superEllipse 2.5 and 8",
                "Same shapes at any DPI: drag to a different scale screen to compare.",
                [](visage::Canvas& canvas, float w, float h) {
                  float size = std::min(w * 0.45f, h * 0.8f);
                  canvas.setColor(kShapeColor);
                  canvas.superEllipse(w * 0.25f - size * 0.5f, (h - size) * 0.5f, size, size, 2.5f);
                  canvas.superEllipse(w * 0.75f - size * 0.5f, (h - size) * 0.5f, size, size, 8.0f);
                });
        addCell("diamond", "A diamond with softly rounded corners.",
                [square, origin](visage::Canvas& canvas, float w, float h) {
                  float size = square(w, h);
                  visage::Point o = origin(w, h);
                  canvas.setColor(kShapeColor);
                  canvas.diamond(o.x, o.y, size, size * 0.1f);
                });
        addCell("triangle / triangleBorder", "Sharp filled triangle and an outline of even width.",
                [s](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(kShapeColor);
                  canvas.triangle(w * 0.05f, h * 0.85f, w * 0.45f, h * 0.85f, w * 0.25f, h * 0.15f);
                  canvas.triangleBorder(w * 0.55f, h * 0.85f, w * 0.95f, h * 0.85f, w * 0.75f,
                                        h * 0.15f, s->thickness);
                });
        addCell("roundedTriangle / Border", "Rounded corners on both triangles.",
                [s](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(kShapeColor);
                  canvas.roundedTriangle(w * 0.05f, h * 0.85f, w * 0.45f, h * 0.85f, w * 0.25f,
                                         h * 0.15f, 8);
                  canvas.roundedTriangleBorder(w * 0.55f, h * 0.85f, w * 0.95f, h * 0.85f,
                                               w * 0.75f, h * 0.15f, 8, s->thickness);
                });
        addCell("Degenerate triangle",
                "Three points on a line draw nothing. The circle next to it still draws.",
                [](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(kShapeColor);
                  canvas.roundedTriangle(w * 0.05f, h * 0.5f, w * 0.25f, h * 0.5f, w * 0.45f, h * 0.5f, 6);
                  canvas.roundedTriangleBorder(w * 0.05f, h * 0.3f, w * 0.25f, h * 0.3f, w * 0.45f,
                                               h * 0.3f, 6, 3);
                  float size = std::min(w * 0.4f, h * 0.7f);
                  canvas.circle(w * 0.75f - size * 0.5f, (h - size) * 0.5f, size);
                });
        addCell("segment rounded / flat",
                "Top: round ends. Middle: square ends. Bottom: a zero-length rounded "
                "segment is a dot, a zero-length flat one is nothing.",
                [s](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(kShapeColor);
                  canvas.segment(w * 0.15f, h * 0.2f, w * 0.85f, h * 0.35f, s->thickness, true);
                  canvas.segment(w * 0.15f, h * 0.5f, w * 0.85f, h * 0.6f, s->thickness, false);
                  canvas.segment(w * 0.3f, h * 0.85f, w * 0.3f, h * 0.85f, s->thickness * 2, true);
                  canvas.segment(w * 0.7f, h * 0.85f, w * 0.7f, h * 0.85f, s->thickness * 2, false);
                });
        addCell("quadratic", "A smooth curve of even thickness.", [s](visage::Canvas& canvas, float w, float h) {
          canvas.setColor(kShapeColor);
          canvas.quadratic(w * 0.1f, h * 0.8f, w * 0.5f, -h * 0.4f, w * 0.9f, h * 0.8f, s->thickness);
        });
        addCell("arc rounded / flat", "Animated arcs: rounded ends on the left, flat ends on the right.",
                [s](visage::Canvas& canvas, float w, float h) {
                  float size = std::min(w * 0.45f, h * 0.8f);
                  float t = s->animate ? canvas.time() : 0.0f;
                  float radians = kPi * (0.5f + 0.45f * std::sin(t));
                  canvas.setColor(kShapeColor);
                  canvas.arc(w * 0.25f - size * 0.5f, (h - size) * 0.5f, size, s->thickness, t,
                             radians, true);
                  canvas.arc(w * 0.75f - size * 0.5f, (h - size) * 0.5f, size, s->thickness, -t,
                             radians, false);
                });
        addCell("roundedArcShadow / flatArcShadow", "Soft glows following the arcs.",
                [s](visage::Canvas& canvas, float w, float h) {
                  float size = std::min(w * 0.45f, h * 0.7f);
                  canvas.setColor(0x88aaff88);
                  canvas.roundedArcShadow(w * 0.25f - size * 0.5f, (h - size) * 0.5f, size,
                                          s->thickness, 0.0f, kPi * 0.6f, 12);
                  canvas.flatArcShadow(w * 0.75f - size * 0.5f, (h - size) * 0.5f, size,
                                       s->thickness, 0.0f, kPi * 0.6f, 12);
                });
        addCell("rectangleShadow / roundedRectangleShadow", "Soft shadows with no hard edges or banding.",
                [](visage::Canvas& canvas, float w, float h) {
                  float size = std::min(w * 0.35f, h * 0.6f);
                  canvas.setColor(0xcc000000);
                  canvas.rectangleShadow(w * 0.25f - size * 0.5f, (h - size) * 0.5f, size, size, 16);
                  canvas.roundedRectangleShadow(w * 0.75f - size * 0.5f, (h - size) * 0.5f, size,
                                                size, 10, 16);
                  canvas.setColor(kShapeColor);
                  canvas.rectangle(w * 0.25f - size * 0.5f, (h - size) * 0.5f, size, size);
                  canvas.roundedRectangle(w * 0.75f - size * 0.5f, (h - size) * 0.5f, size, size, 10);
                });
        addCell("fadeCircle", "Disks with soft edges of 2px and 20px.",
                [](visage::Canvas& canvas, float w, float h) {
                  float size = std::min(w * 0.45f, h * 0.8f);
                  canvas.setColor(kShapeColor);
                  canvas.fadeCircle(w * 0.25f - size * 0.5f, (h - size) * 0.5f, size, 2);
                  canvas.fadeCircle(w * 0.75f - size * 0.5f, (h - size) * 0.5f, size, 20);
                });
        addCell("left/right/top/bottom rounded", "Each rectangle is rounded only on its named side.",
                [](visage::Canvas& canvas, float w, float h) {
                  float cw = w * 0.2f;
                  float ch = h * 0.6f;
                  canvas.setColor(kShapeColor);
                  canvas.leftRoundedRectangle(w * 0.03f, h * 0.2f, cw, ch, 14);
                  canvas.rightRoundedRectangle(w * 0.27f, h * 0.2f, cw, ch, 14);
                  canvas.topRoundedRectangle(w * 0.52f, h * 0.2f, cw, ch, 14);
                  canvas.bottomRoundedRectangle(w * 0.77f, h * 0.2f, cw, ch, 14);
                });
        addCell("triangleUp/Down/Left/Right", "Four arrows pointing up, down, left and right.",
                [](visage::Canvas& canvas, float w, float h) {
                  float size = std::min(w * 0.1f, h * 0.2f);
                  canvas.setColor(kShapeColor);
                  canvas.triangleUp(w * 0.05f, h * 0.5f - size, size);
                  canvas.triangleDown(w * 0.3f, h * 0.5f - size * 0.5f, size);
                  canvas.triangleLeft(w * 0.6f, h * 0.5f - size, size);
                  canvas.triangleRight(w * 0.8f, h * 0.5f - size, size);
                });
        addCell("No setColor() call", "Shapes drawn before any setColor are opaque white, not tinted.",
                [](visage::Canvas& canvas, float w, float h) {
                  float size = std::min(w * 0.4f, h * 0.7f);
                  canvas.circle(w * 0.25f - size * 0.5f, (h - size) * 0.5f, size);
                  canvas.image(resources::images::test_png, w * 0.75f - size * 0.5f,
                               (h - size) * 0.5f, size, size);
                });
        addCell("fill() inside clamp bounds", "Only the inset area is filled, not the whole cell.",
                [](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(kShapeColor);
                  canvas.setClampBounds(w * 0.25f, h * 0.25f, w * 0.5f, h * 0.5f);
                  canvas.fill();
                });
        addCell("Unmatched saveState()",
                "With the toggle on, draws a small red square and never calls "
                "restoreState. The next cell must not be clipped or offset.",
                [s](visage::Canvas& canvas, float w, float h) {
                  if (!s->unmatched_save) {
                    canvas.setColor(kDimText);
                    canvas.text("Toggle is off", font(12), visage::Font::kCenter, 0, 0, w, h);
                    return;
                  }
                  canvas.saveState();
                  canvas.setColor(0xffff4040);
                  canvas.trimClampBounds(0, 0, 20, 20);
                  canvas.setPosition(4, 4);
                  canvas.fill(0, 0, w, h);
                });
        addCell("After unmatched saveState", "A full green rectangle with a border fills this whole cell.",
                [](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(0xff2f8a5d);
                  canvas.fill(0, 0, w, h);
                  canvas.setColor(0xffffffff);
                  canvas.rectangleBorder(0, 0, w, h, 2);
                });

        addControl(&thickness_);
        thickness_.onChange() += [this](float value) {
          settings_.thickness = value;
          redrawCells();
        };
        animate_ = std::make_unique<visage::ToggleTextButton>("Animate");
        animate_->setToggled(true);
        animate_->setFont(font(13));
        animate_->setUndoable(false);
        animate_->onToggle() = [this](visage::Button*, bool on) { settings_.animate = on; };
        addControl(animate_.get());
        unmatched_ = std::make_unique<
            visage::ToggleTextButton>(kDebugBuild ? "Unmatched saveState (Release builds only)" :
                                                    "Unmatched saveState");
        unmatched_->setFont(font(13));
        unmatched_->setUndoable(false);
        unmatched_->onToggle() = [this](visage::Button*, bool on) {
          if (kDebugBuild) {
            unmatched_->setToggled(false);
            return;
          }
          settings_.unmatched_save = on;
          redrawCells();
        };
        addControl(unmatched_.get());
      }

    private:
      std::unique_ptr<visage::ToggleTextButton> unmatched_;
      ShapeSettings settings_;
      Slider thickness_ { "Thickness", 0.5f, 60.0f, 6.0f, 1 };
      std::unique_ptr<visage::ToggleTextButton> animate_;
    };

    class StrokeView : public visage::Frame {
    public:
      enum Shape {
        kStar,
        kCurves,
        kZigzag,
        kSubpaths,
        kRoundedRect,
        kCollinear,
        kCircle,
        kZeroLength,
        kNumShapes
      };

      static const char* shapeName(int shape) {
        static const char* kNames[] = { "Star (closed)",
                                        "Curves and corners (open)",
                                        "Zigzag (open)",
                                        "Three subpaths",
                                        "Rounded rect, 0 and 40 corners",
                                        "Collinear vertex",
                                        "Circle",
                                        "Zero-length segments" };
        return kNames[shape];
      }

      static const char* joinName(int join) {
        static const char* kNames[] = { "Round", "Miter", "Bevel", "Square" };
        return kNames[join];
      }

      static const char* capName(int cap) {
        static const char* kNames[] = { "Round", "Square", "Butt" };
        return kNames[cap];
      }

      visage::Path buildPath() const {
        float s = std::min(width(), height()) * 0.8f;
        float x0 = (width() - s) * 0.5f;
        float y0 = (height() - s) * 0.5f;
        visage::Path path;
        switch (shape) {
        case kStar: path = starPath(width() * 0.5f, height() * 0.5f, s * 0.5f, s * 0.22f); break;
        case kCurves:
          path.moveTo(x0, y0 + s * 0.8f);
          path.bezierTo(x0 + s * 0.1f, y0, x0 + s * 0.4f, y0, x0 + s * 0.45f, y0 + s * 0.5f);
          path.lineTo(x0 + s * 0.55f, y0 + s * 0.2f);
          path.quadraticTo(x0 + s * 0.9f, y0 + s * 0.1f, x0 + s * 0.8f, y0 + s * 0.6f);
          path.arcTo(s * 0.15f, s * 0.15f, 0, false, true, visage::Point(x0 + s, y0 + s * 0.9f));
          break;
        case kZigzag:
          path.moveTo(x0, y0 + s);
          for (int i = 1; i <= 6; ++i)
            path.lineTo(x0 + s * i / 6.0f, (i % 2) ? y0 : y0 + s * (1.0f - i * 0.05f));
          break;
        case kSubpaths:
          path.moveTo(x0, y0 + s * 0.15f);
          path.lineTo(x0 + s, y0 + s * 0.15f);
          path.moveTo(x0, y0 + s * 0.35f);
          path.lineTo(x0 + s * 0.5f, y0 + s * 0.45f);
          path.lineTo(x0 + s, y0 + s * 0.35f);
          path.addRectangle(x0 + s * 0.2f, y0 + s * 0.6f, s * 0.6f, s * 0.35f);
          break;
        case kRoundedRect:
          path.addRoundedRectangle(x0, y0 + s * 0.15f, s, s * 0.7f, 0, 0, 40, 40, 0, 0, 40, 40);
          break;
        case kCollinear:
          path.moveTo(x0, y0 + s * 0.3f);
          path.lineTo(x0 + s * 0.5f, y0 + s * 0.3f);
          path.lineTo(x0 + s, y0 + s * 0.3f);
          path.lineTo(x0 + s, y0 + s * 0.9f);
          path.lineTo(x0 + s * 0.5f, y0 + s * 0.9f);
          break;
        case kCircle: path.addCircle(width() * 0.5f, height() * 0.5f, s * 0.45f); break;
        case kZeroLength:
          path.moveTo(x0 + s * 0.2f, y0 + s * 0.5f);
          path.lineTo(x0 + s * 0.2f, y0 + s * 0.5f);
          path.moveTo(x0 + s * 0.5f, y0 + s * 0.5f);
          path.close();
          path.moveTo(x0 + s * 0.8f, y0 + s * 0.3f);
          path.lineTo(x0 + s * 0.8f, y0 + s * 0.3f);
          path.lineTo(x0 + s * 0.8f, y0 + s * 0.7f);
          break;
        default: break;
        }
        return path;
      }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(0xff111418);
        canvas.fill(0, 0, width(), height());

        visage::Path path = buildPath();
        std::vector<float> dashes;
        if (dashed)
          dashes = { dash_length, dash_length * 0.6f };
        float offset = animate_dash ? canvas.time() * dash_length : 0.0f;
        visage::Path stroked = path.stroke(stroke_width, static_cast<visage::Path::Join>(join),
                                           static_cast<visage::Path::EndCap>(cap), dashes, offset,
                                           miter_limit);
        canvas.setColor(0xccaa88ff);
        canvas.fill(stroked);

        if (centerline) {
          canvas.setColor(0xffffffff);
          canvas.fill(path.stroke(1.0f));
        }
        if (animate_dash)
          redraw();
      }

      int shape = kStar;
      int join = 0;
      int cap = 0;
      bool dashed = false;
      bool animate_dash = false;
      bool centerline = true;
      float stroke_width = 24.0f;
      float dash_length = 30.0f;
      float miter_limit = 4.0f;
    };

    class StrokesPage : public visage::Frame {
    public:
      StrokesPage() {
        addChild(&view_);
        shape_ = makeButton("Shape", [this] {
          view_.shape = (view_.shape + 1) % StrokeView::kNumShapes;
          update();
        });
        join_ = makeButton("Join", [this] {
          view_.join = (view_.join + 1) % 4;
          update();
        });
        cap_ = makeButton("Cap", [this] {
          view_.cap = (view_.cap + 1) % 3;
          update();
        });
        dash_ = makeButton("Dashes", [this] {
          view_.dashed = !view_.dashed;
          update();
        });
        animate_ = makeButton("Animate dash offset", [this] {
          view_.animate_dash = !view_.animate_dash;
          update();
        });
        centerline_ = makeButton("Centerline", [this] {
          view_.centerline = !view_.centerline;
          update();
        });
        tiny_ = makeButton("Dash length 0.01 (hang test)", [this] {
          view_.dashed = true;
          view_.dash_length = 0.01f;
          dash_length_.setValue(0.01f, false);
          update();
        });
        for (auto* button : { shape_.get(), join_.get(), cap_.get(), dash_.get(), animate_.get(),
                              centerline_.get(), tiny_.get() })
          addChild(button);

        width_.onChange() += [this](float value) {
          view_.stroke_width = value;
          update();
        };
        dash_length_.onChange() += [this](float value) {
          view_.dash_length = value;
          update();
        };
        miter_.onChange() += [this](float value) {
          view_.miter_limit = value;
          update();
        };
        addChild(&width_);
        addChild(&dash_length_);
        addChild(&miter_);
        update();
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        layoutButtonRow(takeTop(bounds, 34), { shape_.get(), join_.get(), cap_.get(), dash_.get() }, 6);
        layoutButtonRow(takeTop(bounds, 34), { animate_.get(), centerline_.get(), tiny_.get() }, 6);
        layoutButtonRow(takeTop(bounds, 38), { &width_, &dash_length_, &miter_ }, 16);
        view_.setBounds(bounds);
      }

    private:
      void update() {
        shape_->setText(std::string("Shape: ") + StrokeView::shapeName(view_.shape));
        join_->setText(std::string("Join: ") + StrokeView::joinName(view_.join));
        cap_->setText(std::string("Cap: ") + StrokeView::capName(view_.cap));
        dash_->setText(view_.dashed ? "Dashes: on" : "Dashes: off");
        animate_->setText(view_.animate_dash ? "Animate dash offset: on" : "Animate dash offset: off");
        centerline_->setText(view_.centerline ? "Centerline: on" : "Centerline: off");
        view_.redraw();
      }

      StrokeView view_;
      std::unique_ptr<visage::UiButton> shape_;
      std::unique_ptr<visage::UiButton> join_;
      std::unique_ptr<visage::UiButton> cap_;
      std::unique_ptr<visage::UiButton> dash_;
      std::unique_ptr<visage::UiButton> animate_;
      std::unique_ptr<visage::UiButton> centerline_;
      std::unique_ptr<visage::UiButton> tiny_;
      Slider width_ { "Stroke width", 0.5f, 160.0f, 24.0f, 1 };
      Slider dash_length_ { "Dash length", 0.5f, 120.0f, 30.0f, 1 };
      Slider miter_ { "Miter limit", 1.0f, 12.0f, 4.0f, 1 };
    };

    class PathFillingPage : public CellPage {
    public:
      PathFillingPage() : CellPage(4) {
        std::mt19937 random(1234);
        std::uniform_real_distribution<float> unit(0.0f, 1.0f);
        for (int i = 0; i < 3000; ++i) {
          float x = 0.2f + 0.6f * unit(random);
          float y = 0.2f + 0.6f * unit(random);
          dense_points_.push_back({ x, y });
          dense_points_.push_back({ x + (unit(random) - 0.5f) * 0.6f, y + (unit(random) - 0.5f) * 0.6f });
          dense_points_.push_back({ x + (unit(random) - 0.5f) * 0.6f, y + (unit(random) - 0.5f) * 0.6f });
        }

        addCell("Fill rules",
                "Pentagram filled NonZero (solid center), EvenOdd (hollow center) and Positive.",
                [](visage::Canvas& canvas, float w, float h) {
                  visage::Path::FillRule rules[] = { visage::Path::FillRule::NonZero,
                                                     visage::Path::FillRule::EvenOdd,
                                                     visage::Path::FillRule::Positive };
                  float r = std::min(w / 6.0f, h * 0.4f);
                  canvas.setColor(kShapeColor);
                  for (int i = 0; i < 3; ++i) {
                    visage::Path path = pentagram(w * (i + 0.5f) / 3.0f, h * 0.5f, r);
                    path.setFillRule(rules[i]);
                    canvas.fill(path);
                  }
                });
        addCell("Thin wedges",
                "60 thin wedges from the center. Edges are smooth, with no gaps or bright "
                "seams near the center.",
                [](visage::Canvas& canvas, float w, float h) {
                  visage::Path path;
                  float cx = w * 0.5f;
                  float cy = h * 0.5f;
                  float r = std::min(w, h) * 0.45f;
                  for (int i = 0; i < 60; ++i) {
                    float a0 = i * 2.0f * kPi / 60.0f;
                    float a1 = a0 + kPi / 60.0f;
                    path.moveTo(cx, cy);
                    path.lineTo(cx + r * std::cos(a0), cy + r * std::sin(a0));
                    path.lineTo(cx + r * std::cos(a1), cy + r * std::sin(a1));
                    path.close();
                  }
                  canvas.setColor(0xffffffff);
                  canvas.fill(path);
                });
        addCell("Concentric rings", "Rings 1.5px wide with 1.5px gaps. They look even all the way around.",
                [](visage::Canvas& canvas, float w, float h) {
                  visage::Path path;
                  float r = std::min(w, h) * 0.48f;
                  for (float radius = r; radius > 3.0f; radius -= 3.0f)
                    path.addCircle(w * 0.5f, h * 0.5f, radius);
                  path.setFillRule(visage::Path::FillRule::EvenOdd);
                  canvas.setColor(0xffffffff);
                  canvas.fill(path);
                });
        addCell("3000 overlapping triangles",
                "One dense NonZero path. The middle is solid blue with no stray "
                "dark or bright pixels.",
                [this](visage::Canvas& canvas, float w, float h) {
                  visage::Path path;
                  for (int i = 0; i + 2 < dense_points_.size(); i += 3) {
                    path.moveTo(dense_points_[i].x * w, dense_points_[i].y * h);
                    path.lineTo(dense_points_[i + 1].x * w, dense_points_[i + 1].y * h);
                    path.lineTo(dense_points_[i + 2].x * w, dense_points_[i + 2].y * h);
                    path.close();
                  }
                  path.setFillRule(visage::Path::FillRule::NonZero);
                  canvas.setColor(0xff5ab0ff);
                  canvas.fill(path);
                });
        addCell("Huge circle edge",
                "Path circle of radius 4000 (blue) and the analytic ring of the same "
                "circle (white). The white line stays on the blue edge.",
                [](visage::Canvas& canvas, float w, float h) {
                  float r = 4000.0f;
                  float cx = w * 0.5f;
                  float cy = h * 0.5f + r - h * 0.1f;
                  visage::Path path;
                  path.addCircle(cx, cy, r);
                  canvas.setColor(0xaa5ab0ff);
                  canvas.fill(path, 0, 0, w, h);
                  canvas.setColor(0xffffffff);
                  canvas.ring(cx - r - 0.75f, cy - r - 0.75f, 2 * r + 1.5f, 1.5f);
                });
        addCell("Path larger than the atlas",
                "A 12000px star centered here. The visible part draws and nothing "
                "crashes.",
                [](visage::Canvas& canvas, float w, float h) {
                  visage::Path path = starPath(6000, 6000, 6000, 2500);
                  canvas.setColor(kShapeColor);
                  canvas.fill(path, w * 0.5f - 6000, h * 0.5f - 6000, 12000, 12000);
                });
        addCell(
            "Path::rotated vs Transform::rotation",
            "Red: Path::rotated. White outline: "
            "transformed(Transform::rotation). They overlap exactly.",
            [](visage::Canvas& canvas, float w, float h) {
              float angle = canvas.time() * 40.0f;
              float size = std::min(w, h) * 0.45f;
              visage::Path arrow = arrowPath(size);
              visage::Path rotated = arrow.rotated(angle).translated(w * 0.5f, h * 0.5f);
              visage::Path transformed = arrow.transformed(visage::Transform::rotation(angle))
                                             .translated(w * 0.5f, h * 0.5f);
              canvas.setColor(0xffff4040);
              canvas.fill(rotated);
              canvas.setColor(0xffffffff);
              visage::Path outline = transformed.stroke(1.5f);
              canvas.fill(outline);
            },
            true);
        addCell(
            "400 animated stars", "Each star changes every frame. No flicker, missing stars or garbage.",
            [](visage::Canvas& canvas, float w, float h) {
              float t = canvas.time();
              for (int i = 0; i < 400; ++i) {
                float x = (i % 20 + 0.5f) * w / 20.0f;
                float y = (i / 20 + 0.5f) * h / 20.0f;
                float r = std::min(w, h) / 45.0f * (1.0f + 0.4f * std::sin(t * 2.0f + i));
                canvas.setColor(visage::Color::fromAHSV(1.0f, std::fmod(i * 9.0f, 360.0f), 0.6f, 1.0f));
                canvas.fill(starPath(x, y, r, r * 0.45f).rotated(t + i));
              }
            },
            true);
        addCell("Path::offset",
                "A star (gray), its outset by 10 (green outline) and inset by 6 (blue "
                "outline).",
                [](visage::Canvas& canvas, float w, float h) {
                  const visage::Path star = starPath(w * 0.5f, h * 0.5f, std::min(w, h) * 0.36f,
                                                     std::min(w, h) * 0.16f);
                  canvas.setColor(0xff555b66);
                  canvas.fill(star);
                  canvas.setColor(kShapeColor);
                  canvas.fill(star.offset(10, visage::Path::Join::Round).stroke(1.5f));
                  canvas.setColor(0xff5ab0ff);
                  canvas.fill(star.offset(-6, visage::Path::Join::Miter).stroke(1.5f));
                });
        addCell("Stroke wider than the shape",
                "A circle of radius 10 stroked 60 wide is a solid disk with no "
                "hole.",
                [](visage::Canvas& canvas, float w, float h) {
                  visage::Path circle;
                  circle.addCircle(w * 0.3f, h * 0.5f, 10);
                  canvas.setColor(kShapeColor);
                  canvas.fill(circle.stroke(60));
                  visage::Path square;
                  square.addRectangle(w * 0.7f - 8, h * 0.5f - 8, 16, 16);
                  canvas.fill(square.stroke(50, visage::Path::Join::Miter));
                });
        addCell("Tiny shapes", "Stars from 4px to 16px. They look like small stars, not blobs or noise.",
                [](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(0xffffffff);
                  float x = 6;
                  for (float size = 4; size <= 16 && x < w; size += 1.5f) {
                    canvas.fill(starPath(x + size, h * 0.35f, size, size * 0.45f));
                    canvas.fill(starPath(x + size + 0.5f, h * 0.7f, size, size * 0.45f));
                    x += size * 2 + 6;
                  }
                });
        addCell("SVG path strings",
                "Parsed from strings with exponents and implicit commands: a zigzag, a "
                "smooth S curve and a closed triangle.",
                [](visage::Canvas& canvas, float w, float h) {
                  visage::Path path;
                  path.loadSvgPath("M1e1,20l1.5e1-10 15 10 15-10 15 10M10 60c20-30 40 30 60 0s40 "
                                   "30 60 0"
                                   "M150 20l30 50h-60z");
                  float scale = std::min(w / 200.0f, h / 90.0f);
                  path.scale(scale);
                  canvas.setColor(kShapeColor);
                  canvas.fill(path.stroke(2.5f));
                });
      }

    private:
      std::vector<visage::Point> dense_points_;
    };

    class GradientsPage : public CellPage {
    public:
      GradientsPage() : CellPage(4) {
        addCell("Horizontal and vertical",
                "Left half red to blue left to right, right half yellow to green top "
                "to bottom. Smooth, no banding steps.",
                [](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(visage::Brush::horizontal(0xffff4040, 0xff4040ff));
                  canvas.fill(0, 0, w * 0.5f, h);
                  canvas.setColor(visage::Brush::vertical(0xffffff40, 0xff40c040));
                  canvas.fill(w * 0.5f, 0, w * 0.5f, h);
                });
        addCell("Diagonal rainbow", "Seven-stop rainbow along the diagonal.",
                [](visage::Canvas& canvas, float w, float h) {
                  visage::Gradient rainbow(0xffff0000, 0xffffff00, 0xff00ff00, 0xff00ffff,
                                           0xff0000ff, 0xffff00ff, 0xffff0000);
                  canvas.setColor(visage::Brush::linear(rainbow, { 0, 0 }, { w, h }));
                  canvas.roundedRectangle(0, 0, w, h, 12);
                });
        addCell("Radial", "White center fading to dark blue at the edge of the circle.",
                [](visage::Canvas& canvas, float w, float h) {
                  float r = std::min(w, h) * 0.5f;
                  canvas.setColor(visage::Brush::radial(0xffffffff, 0xff102060, { w * 0.5f, h * 0.5f }, r));
                  canvas.circle(w * 0.5f - r, h * 0.5f - r, 2 * r);
                });
        addCell("Radial with focal point",
                "The bright spot sits up and to the left of center, like a lit "
                "sphere.",
                [](visage::Canvas& canvas, float w, float h) {
                  float r = std::min(w, h) * 0.5f;
                  visage::Point center(w * 0.5f, h * 0.5f);
                  canvas.setColor(visage::Brush::radial(0xffffffff, 0xff802020, center, r, r,
                                                        center + visage::Point(-r * 0.4f, -r * 0.4f)));
                  canvas.circle(w * 0.5f - r, h * 0.5f - r, 2 * r);
                });
        addCell("Focal circle equals outer circle",
                "Paints the end color (teal) like Chrome does, not an "
                "empty circle.",
                [](visage::Canvas& canvas, float w, float h) {
                  float r = std::min(w, h) * 0.5f;
                  visage::Point center(w * 0.5f, h * 0.5f);
                  canvas.setColor(visage::Brush::radial(0xffffff00, 0xff008080, center, r, r, center, r));
                  canvas.circle(w * 0.5f - r, h * 0.5f - r, 2 * r);
                });
        addCell("Zero-length linear gradient",
                "Start and end points are the same. Paints a solid color, not "
                "blank or noisy.",
                [](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(visage::Brush::linear(0xffff8800, 0xff0088ff, { w * 0.5f, h * 0.5f },
                                                        { w * 0.5f, h * 0.5f }));
                  canvas.roundedRectangle(0, 0, w, h, 12);
                });
        addCell(
            "Solid blending into a gradient",
            "Animates from solid gray to a rainbow and back. The gradient "
            "shows up gradually instead of popping at the end.",
            [](visage::Canvas& canvas, float w, float h) {
              float t = 0.5f + 0.5f * std::sin(canvas.time() * 1.5f);
              visage::Brush solid = visage::Brush::solid(0xff808080);
              visage::Brush rainbow = visage::Brush::horizontal(visage::Gradient(0xffff0000, 0xff00ff00,
                                                                                 0xff0000ff));
              canvas.setColor(visage::Brush::interpolate(solid, rainbow, t));
              canvas.roundedRectangle(0, 0, w, h * 0.8f, 12);
              canvas.setColor(0xffffffff);
              canvas.text("t = " + visage::String(t, 2).toUtf8(), font(13), visage::Font::kCenter,
                          0, h * 0.8f, w, h * 0.2f);
            },
            true);
        addCell("Repeat and reflect",
                "Top: repeating red-blue stripes. Bottom: reflected, so stripes mirror "
                "with no hard jumps.",
                [](visage::Canvas& canvas, float w, float h) {
                  visage::Gradient repeat(0xffff4040, 0xff4040ff);
                  repeat.setRepeat(true);
                  canvas.setColor(visage::Brush::linear(repeat, { 0, 0 }, { w / 5.0f, 0 }));
                  canvas.fill(0, 0, w, h * 0.48f);
                  visage::Gradient reflect(0xffff4040, 0xff4040ff);
                  reflect.setReflect(true);
                  canvas.setColor(visage::Brush::linear(reflect, { 0, 0 }, { w / 5.0f, 0 }));
                  canvas.fill(0, h * 0.52f, w, h * 0.48f);
                });
        addCell("400 different gradients",
                "Every small square has its own gradient. No squares are blank, "
                "wrong or flickering.",
                [](visage::Canvas& canvas, float w, float h) {
                  float cw = w / 20.0f;
                  float ch = h / 20.0f;
                  for (int i = 0; i < 400; ++i) {
                    float x = (i % 20) * cw;
                    float y = (i / 20) * ch;
                    visage::Color from = visage::Color::fromAHSV(1.0f, i * 0.9f, 0.8f, 1.0f);
                    visage::Color to = visage::Color::fromAHSV(1.0f, std::fmod(i * 0.9f + 180.0f, 360.0f),
                                                               0.8f, 0.4f);
                    canvas.setColor(visage::Brush::linear(from, to, { x, y }, { x + cw, y + ch }));
                    canvas.fill(x, y, cw - 1, ch - 1);
                  }
                });
        addCell("Gradient on text and strokes",
                "Text and a stroked rounded rectangle both use a left to right "
                "gradient.",
                [](visage::Canvas& canvas, float w, float h) {
                  visage::Brush brush = visage::Brush::linear(0xffff6060, 0xff60a0ff, { 0, 0 }, { w, 0 });
                  canvas.setColor(brush);
                  canvas.roundedRectangleBorder(4, 4, w - 8, h - 8, 16, 4);
                  canvas.text("Gradient text", font(std::min(28.0f, h * 0.25f)),
                              visage::Font::kCenter, 0, 0, w, h);
                });
        addCell("Color::fromHexString",
                "Shows how hex strings parse. Invalid strings must not turn into a "
                "random color.",
                [](visage::Canvas& canvas, float w, float h) {
                  const char* inputs[] = { "ff8800", "#80ff0000", "FF00FF00", "zz00zz", "12345", "" };
                  float row = h / 6.0f;
                  for (int i = 0; i < 6; ++i) {
                    visage::Color color = visage::Color::fromHexString(inputs[i]);
                    canvas.setColor(0xff555555);
                    canvas.fill(4, i * row + 3, row - 6, row - 6);
                    canvas.setColor(color);
                    canvas.fill(4, i * row + 3, row - 6, row - 6);
                    canvas.setColor(kText);
                    canvas.text(std::string("\"") + inputs[i] + "\" -> " + color.toARGBHexString(),
                                monoFont(11), visage::Font::kLeft, row + 4, i * row, w - row - 4, row);
                  }
                });
        addCell("Alpha gradient over checkerboard",
                "Fades from opaque purple to fully transparent. No dark "
                "fringe in the middle.",
                [](visage::Canvas& canvas, float w, float h) {
                  float size = 10;
                  for (int y = 0; y * size < h; ++y) {
                    for (int x = 0; x * size < w; ++x) {
                      canvas.setColor(((x + y) % 2) ? 0xffffffff : 0xffbbbbbb);
                      canvas.fill(x * size, y * size, size, size);
                    }
                  }
                  canvas.setColor(visage::Brush::horizontal(0xff8844ff, 0x008844ff));
                  canvas.fill(0, 0, w, h);
                });
      }
    };

    class TextPage : public CellPage {
    public:
      TextPage() : CellPage(3) {
        wrapped_.setText(
            "Multi-line text wraps at word boundaries and reflows when the window is resized. "
            "Long words like supercalifragilisticexpialidocious move to their own line.");
        wrapped_.setFont(font(15));
        wrapped_.setMultiLine(true);
        wrapped_.setJustification(visage::Font::kTopLeft);

        addCell("Fractional sizes",
                "Each line is slightly bigger than the one above. No two neighbors are "
                "the same size.",
                [](visage::Canvas& canvas, float w, float h) {
                  float y = 0;
                  canvas.setColor(kText);
                  for (float size = 11.0f; size <= 14.0f && y < h; size += 0.25f) {
                    canvas.text("Size " + visage::String(size, 2).toUtf8() + ": The quick brown fox",
                                font(size), visage::Font::kLeft, 0, y, w, size * 1.3f);
                    y += size * 1.3f;
                  }
                });
        addCell("Justification", "Each word sits in its named corner, edge or center.",
                [](visage::Canvas& canvas, float w, float h) {
                  struct Item {
                    const char* name;
                    visage::Font::Justification justification;
                  };
                  Item items[] = { { "TopLeft", visage::Font::kTopLeft },
                                   { "Top", visage::Font::kTop },
                                   { "TopRight", visage::Font::kTopRight },
                                   { "Left", visage::Font::kLeft },
                                   { "Center", visage::Font::kCenter },
                                   { "Right", visage::Font::kRight },
                                   { "BottomLeft", visage::Font::kBottomLeft },
                                   { "Bottom", visage::Font::kBottom },
                                   { "BottomRight", visage::Font::kBottomRight } };
                  canvas.setColor(kPanelLight);
                  canvas.rectangleBorder(0, 0, w, h, 1);
                  canvas.setColor(kText);
                  for (const Item& item : items)
                    canvas.text(item.name, font(13), item.justification, 4, 4, w - 8, h - 8);
                });
        addCell("Wrapped paragraph", "Wraps cleanly inside the cell with no clipped or overlapping lines.",
                [this](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(kText);
                  canvas.text(&wrapped_, 0, 0, w, h);
                });
        addCell("Light on dark, dark on light",
                "Small text in both themes is crisp and the same weight, not "
                "too thin or too bold.",
                [](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(0xff000000);
                  canvas.fill(0, 0, w * 0.5f, h);
                  canvas.setColor(0xffffffff);
                  canvas.fill(w * 0.5f, 0, w * 0.5f, h);
                  float y = 4;
                  for (float size = 9.0f; size <= 15.0f; size += 2.0f) {
                    canvas.setColor(0xffffffff);
                    canvas.text("Hamburg " + visage::String(size, 0).toUtf8(), font(size),
                                visage::Font::kLeft, 4, y, w * 0.5f - 8, size * 1.4f);
                    canvas.setColor(0xff000000);
                    canvas.text("Hamburg " + visage::String(size, 0).toUtf8(), font(size),
                                visage::Font::kLeft, w * 0.5f + 4, y, w * 0.5f - 8, size * 1.4f);
                    y += size * 1.6f;
                  }
                });
        addCell("Emoji",
                "Color emoji with clean edges. ❤️ and ❤ take the same space. Flags and skin tones "
                "render as one glyph.",
                [](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(0xffffffff);
                  canvas.text("😀 🎉 👍🏽 🇺🇸", font(28), visage::Font::kCenter, 0, 0, w, h * 0.5f);
                  canvas.text("[❤️][❤][★]", font(28), visage::Font::kCenter, 0, h * 0.5f, w, h * 0.5f);
                });
        addCell("Rotated text",
                "Up (normal), Left, Down and Right directions, each readable when you tilt your "
                "head.",
                [](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(kText);
                  canvas.text("Up", font(16), visage::Font::kCenter, w * 0.3f, 0, w * 0.4f,
                              h * 0.2f, visage::Direction::Up);
                  canvas.text("Left", font(16), visage::Font::kCenter, 0, h * 0.2f, w * 0.2f,
                              h * 0.6f, visage::Direction::Left);
                  canvas.text("Down", font(16), visage::Font::kCenter, w * 0.3f, h * 0.8f, w * 0.4f,
                              h * 0.2f, visage::Direction::Down);
                  canvas.text("Right", font(16), visage::Font::kCenter, w * 0.8f, h * 0.2f,
                              w * 0.2f, h * 0.6f, visage::Direction::Right);
                });
        addCell("Font metrics",
                "Cap height is about 70% of the size and the descent (lowerDipHeight) is a "
                "small positive number, about 20% of the size.",
                [](visage::Canvas& canvas, float w, float h) {
                  visage::Font f = font(40).withDpiScale(canvas.dpiScale());
                  canvas.setColor(kText);
                  canvas.text("Hgjpqy", f, visage::Font::kCenter, 0, 0, w, h * 0.55f);
                  std::string info = "size 40\nlineHeight " +
                                     visage::String(f.lineHeight(), 1).toUtf8() + "\ncapitalHeight " +
                                     visage::String(f.capitalHeight(), 1).toUtf8() + "\nlowerDipHeight " +
                                     visage::String(f.lowerDipHeight(), 1).toUtf8();
                  drawLines(canvas, info, monoFont(11), visage::Font::kCenter, 0, h * 0.5f, w, h * 0.5f);
                });
        addCell("Default and missing fonts",
                "Text with a default Font() and with a font file that doesn't exist "
                "draws nothing and doesn't crash. The label below still draws.",
                [](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(kText);
                  canvas.text("default font", visage::Font(), visage::Font::kCenter, 0, 0, w, h * 0.3f);
                  canvas.text("missing font",
                              visage::Font(16, std::string("/nonexistent/missing-font.ttf")),
                              visage::Font::kCenter, 0, h * 0.3f, w, h * 0.3f);
                  canvas.text("This label is drawn after both", font(14), visage::Font::kCenter, 0,
                              h * 0.6f, w, h * 0.4f);
                });
        addCell("Monospace and symbols",
                "Columns line up in the monospace font. Symbols render: ✓ ✗ → ← ↑ ↓ "
                "• ° ± × ÷ € £ ¥",
                [](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(kText);
                  const char* lines[] = { "iiiiiiiiii|", "WWWWWWWWWW|", "0123456789|",
                                          "✓ ✗ → ← ↑ ↓ • ° ± × ÷ € £ ¥" };
                  for (int i = 0; i < 4; ++i)
                    canvas.text(lines[i], i < 3 ? monoFont(15) : font(16), visage::Font::kLeft, 4,
                                i * h / 4.0f, w - 8, h / 4.0f);
                });
      }

    private:
      visage::Text wrapped_;
    };

    class ImagesPage : public CellPage {
    public:
      ImagesPage() : CellPage(3) {
        static const unsigned char kGarbage[] = { 0x89, 'P', 'N', 'G', 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };

        addCell("Sizes over a checkerboard",
                "The test image at 24, 48, 96 and fit size. Edges have no stray "
                "colored pixels or lines.",
                [](visage::Canvas& canvas, float w, float h) {
                  float size = 8;
                  for (int y = 0; y * size < h; ++y) {
                    for (int x = 0; x * size < w; ++x) {
                      canvas.setColor(((x + y) % 2) ? 0xff444444 : 0xff666666);
                      canvas.fill(x * size, y * size, size, size);
                    }
                  }
                  canvas.setColor(0xffffffff);
                  float x = 4;
                  for (float s : { 24.0f, 48.0f, 96.0f }) {
                    canvas.image(resources::images::test_png, x, 4, s, s);
                    x += s + 8;
                  }
                  float fit = std::min(w - x - 4, h - 8);
                  if (fit > 0)
                    canvas.image(resources::images::test_png, x, 4, fit, fit);
                });
        addCell("Stretched", "Non-uniform scaling stretches the image smoothly.",
                [](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(0xffffffff);
                  canvas.image(resources::images::test_png, 0, 0, w * 0.65f, h * 0.4f);
                  canvas.image(resources::images::test_png, w * 0.7f, 0, w * 0.3f, h);
                });
        addCell("Invalid image data",
                "A null image draws nothing. With the toggle on, garbage bytes also draw "
                "nothing. The circle still draws.",
                [this](visage::Canvas& canvas, float w, float h) {
                  canvas.setColor(0xffffffff);
                  canvas.image(nullptr, 0, 0, 0, w * 0.3f, h * 0.5f);
                  if (garbage_)
                    canvas.image(kGarbage, sizeof(kGarbage), w * 0.35f, 0, w * 0.3f, h * 0.5f);
                  canvas.setColor(kShapeColor);
                  float size = std::min(w, h) * 0.3f;
                  canvas.circle(w * 0.5f - size * 0.5f, h * 0.6f, size);
                });
        addCell(
            "100 moving copies", "Many copies moving smoothly with no flicker or swapped images.",
            [](visage::Canvas& canvas, float w, float h) {
              float t = canvas.time();
              canvas.setColor(0xffffffff);
              for (int i = 0; i < 100; ++i) {
                float x = (0.5f + 0.45f * std::sin(t * 0.7f + i * 0.37f)) * (w - 24);
                float y = (0.5f + 0.45f * std::cos(t * 0.9f + i * 0.61f)) * (h - 24);
                canvas.image(resources::images::test_png, x, y, 24, 24);
              }
            },
            true);
        addCell("Tinted image",
                "The image drawn with a semi-transparent red color is tinted and see-through.",
                [](visage::Canvas& canvas, float w, float h) {
                  float size = std::min(w, h) * 0.9f;
                  canvas.setColor(0xff2f5d8a);
                  canvas.fill(0, 0, w, h);
                  canvas.setColor(0x88ff6060);
                  canvas.image(resources::images::test_png, (w - size) * 0.5f, (h - size) * 0.5f,
                               size, size);
                });

        garbage_toggle_ = std::make_unique<visage::ToggleTextButton>(
            kDebugBuild ? "Draw garbage image bytes (Release builds only, asserts in Debug)" :
                          "Draw garbage image bytes");
        garbage_toggle_->setFont(font(13));
        garbage_toggle_->setUndoable(false);
        garbage_toggle_->onToggle() = [this](visage::Button*, bool on) {
          if (kDebugBuild) {
            garbage_toggle_->setToggled(false);
            return;
          }
          garbage_ = on;
          redrawCells();
        };
        addControl(garbage_toggle_.get());
      }

    private:
      bool garbage_ = false;
      std::unique_ptr<visage::ToggleTextButton> garbage_toggle_;
    };
  }

  void addGraphicsTests(std::vector<TestDefinition>& tests) {
    tests.push_back(
        { "shapes",
          "Graphics",
          "Canvas Shapes",
          "Every Canvas shape primitive. Each cell says what it should look like. Use the "
          "Thickness "
          "slider for borders and segments.",
          {
              "Every cell matches its description at the default thickness.",
              "Drag Thickness from 0.5 to 60. Borders and rings stay even, with no gaps, holes or "
              "spikes at any thickness.",
              "The squircles have no dot or blob in their centers at any thickness.",
              "Zero-length segments: the rounded one is a dot and the flat one draws nothing.",
              "The degenerate triangle draws nothing and the circle beside it is fine.",
              "The No setColor cell shows a white circle and an untinted image.",
              "fill() inside clamp bounds only fills the middle of its cell.",
              "Release build: turn on \"Unmatched saveState\". The cell after it stays completely "
              "green, not clipped or shifted. (Debug builds assert here on purpose.)",
              "Shapes look the same after moving the window to a screen with a different scale.",
          },
          [] { return std::make_unique<ShapesPage>(); } });

    tests.push_back(
        { "strokes",
          "Graphics",
          "Path Strokes and Dashes",
          "Cycle the shape, join and cap and drag the sliders. The purple area is the stroke and "
          "the "
          "white line is the path.",
          {
              "Joins: Round is round, Miter is sharp until the miter limit then cut, Bevel is "
              "flat, "
              "Square is squared off. Lower the miter limit on the zigzag to see miters get cut.",
              "Caps on open paths: Round, Square (extends past the end) and Butt (flush). Caps at "
              "curve ends follow the curve's direction, not a chord.",
              "A very wide stroke on the star or circle has no hole in the middle and no stray "
              "spikes.",
              "Dashes on Three subpaths restart at the start of each subpath.",
              "Dashes on the star and circle join smoothly where the path closes, with no extra "
              "short "
              "dash.",
              "Dashes don't wrap around corners: a dash that ends at a corner stops there.",
              "Dashes ending inside a curve are cut square to the curve.",
              "Rounded rect with 0 and 40 corners: the sharp corners have no notch or backward "
              "spike, with dashes on and off.",
              "Collinear vertex with Square join draws without a crash or a bump.",
              "Zero-length segments with Round cap draw dots and with Butt cap draw nothing.",
              "\"Dash length 0.01\" doesn't hang the app.",
              "Animating the dash offset moves dashes smoothly along every shape.",
          },
          [] { return std::make_unique<StrokesPage>(); } });

    tests.push_back(
        { "path_filling",
          "Graphics",
          "Path Filling",
          "Path fills that stress the tile renderer and its anti-aliasing.",
          {
              "Fill rules: NonZero has a solid center, EvenOdd has a hollow center.",
              "Thin wedges and concentric rings have smooth even edges with no seams, gaps or "
              "bright "
              "pixels.",
              "3000 overlapping triangles: no stray dark or bright pixels inside the shape.",
              "Huge circle: the white ring stays on the edge of the blue fill across the whole "
              "cell.",
              "Path larger than the atlas draws its visible part with no crash.",
              "The rotated red arrow and the white outline overlap exactly as they spin.",
              "400 animated stars draw every frame with no flicker or garbage.",
              "Offset outlines are even distances outside and inside the star.",
              "The stroke-wider-than-shape cell shows solid shapes with no holes.",
              "Tiny stars still look like stars. SVG path strings draw a zigzag, an S curve and a "
              "triangle.",
          },
          [] { return std::make_unique<PathFillingPage>(); } });

    tests.push_back({ "gradients",
                      "Graphics",
                      "Gradients and Color",
                      "Linear and radial brushes, interpolation and color parsing.",
                      {
                          "Linear gradients are smooth with no banding.",
                          "The radial gradient is centered and the focal one has its highlight up "
                          "and to the left.",
                          "Focal circle equals outer circle paints solid teal, not nothing.",
                          "The zero-length linear gradient is a solid color.",
                          "Solid to gradient blending shows the rainbow gradually as t rises.",
                          "Repeat shows hard stripes, reflect shows mirrored stripes.",
                          "All 400 small gradients show, each with its own colors.",
                          "Text and the stroked border both show the gradient.",
                          "fromHexString: valid strings give the right colors and invalid ones "
                          "don't produce a "
                          "random color.",
                          "The alpha gradient fades out cleanly with no dark band.",
                      },
                      [] { return std::make_unique<GradientsPage>(); } });

    tests.push_back(
        { "text",
          "Graphics",
          "Text and Fonts",
          "Font sizes, justification, wrapping, emoji and font edge cases.",
          {
              "Fractional sizes grow steadily line by line.",
              "Each justification word is in its named position.",
              "The wrapped paragraph reflows when the window is resized, with no clipped lines.",
              "Small text is crisp in both light and dark, and neither looks bolder.",
              "Emoji render in color with clean edges, and both hearts take the same space.",
              "Rotated text reads in all four directions.",
              "Font metrics show a positive descent about 20% of the size.",
              "Default and missing fonts draw nothing, don't crash, and the label below draws.",
              "Monospace columns line up and the symbols render.",
          },
          [] { return std::make_unique<TextPage>(); } });

    tests.push_back({ "images",
                      "Graphics",
                      "Images",
                      "Image drawing at different sizes, with tints, and with bad data.",
                      {
                          "Images at every size have clean edges over the checkerboard, with no "
                          "stray pixels or "
                          "lines along the borders.",
                          "Stretched images scale smoothly.",
                          "Invalid image data draws nothing, no crash, and the circle still draws.",
                          "Release build: turn on \"Draw garbage image bytes\". Nothing draws for "
                          "them and the app doesn't crash. (Debug builds assert in bimg: known "
                          "bug.)",
                          "100 moving copies stay smooth with no flicker.",
                          "The tinted image is red-tinted and semi-transparent.",
                      },
                      [] { return std::make_unique<ImagesPage>(); } });
  }
}
