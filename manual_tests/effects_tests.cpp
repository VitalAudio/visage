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
#include "embedded/example_shaders.h"
#include "test_helpers.h"

#include <cmath>

namespace manual {
  namespace {
    constexpr float kPi = 3.14159265358979323846f;

    void detachPostEffect(visage::Frame& frame) {
      if (frame.postEffect())
        frame.removePostEffect();
    }

    void drawBusyContent(visage::Canvas& canvas, float w, float h, double time, bool hdr = false) {
      canvas.setColor(0xff1a2030);
      canvas.fill(0, 0, w, h);
      for (int i = 0; i < 8; ++i) {
        float stripe = w / 8.0f;
        canvas.setColor((i % 2) ? 0xff2a3448 : 0xff202838);
        canvas.fill(i * stripe, 0, stripe, h);
      }
      for (int i = 0; i < 6; ++i) {
        float phase = time * 0.6f + i * kPi / 3.0f;
        float r = std::min(w, h) * 0.12f;
        float x = w * 0.5f + std::cos(phase) * w * 0.3f - r;
        float y = h * 0.5f + std::sin(phase * 1.3f) * h * 0.3f - r;
        visage::Color color = visage::Color::fromAHSV(1.0f, i * 60.0f, 0.7f, 1.0f);
        if (hdr)
          color.setHdr(2.0f);
        canvas.setColor(color);
        canvas.circle(x, y, 2 * r);
      }
      canvas.setColor(0xffffffff);
      canvas.text("Sharp text", font(std::min(28.0f, h * 0.12f)), visage::Font::kCenter, 0, 0, w, h);
      canvas.setColor(0xffffffff);
      for (int i = 0; i < 12; ++i)
        canvas.fill(w * 0.05f + i * w * 0.075f, h * 0.88f, 1, h * 0.08f);
    }

    class BusyFrame : public visage::Frame {
    public:
      explicit BusyFrame(bool hdr = false) : hdr_(hdr) { }

      void draw(visage::Canvas& canvas) override {
        if (animate_)
          time_ = canvas.time();
        drawBusyContent(canvas, width(), height(), time_, hdr_);
        if (animate_)
          redraw();
      }

      void setAnimate(bool animate) {
        animate_ = animate;
        redraw();
      }

    private:
      bool hdr_ = false;
      bool animate_ = true;
      double time_ = 0.0;
    };

    std::unique_ptr<visage::ToggleTextButton> makeToggle(const std::string& name, bool on,
                                                         std::function<void(bool)> callback) {
      auto toggle = std::make_unique<visage::ToggleTextButton>(name);
      toggle->setFont(font(13));
      toggle->setToggled(on);
      toggle->setUndoable(false);
      toggle->onToggle() = [callback = std::move(callback)](visage::Button*, bool on) {
        callback(on);
      };
      return toggle;
    }

    class BlurPage : public visage::Frame {
    public:
      BlurPage() :
          blurred_box_("BlurPostEffect on this frame"), reference_box_("Same content, no blur"),
          radius_frame_box_("Frame::setBlurRadius") {
        blur_.setBlurRadius(radius_.value());
        blurred_.setPostEffect(&blur_);
        blurred_box_.addChild(&blurred_);
        reference_box_.addChild(&reference_);
        radius_frame_.setBlurRadius(radius_.value() * 0.25f);
        radius_frame_box_.addChild(&radius_frame_);
        addChild(&blurred_box_);
        addChild(&reference_box_);
        addChild(&radius_frame_box_);

        radius_.onChange() += [this](float value) {
          blur_.setBlurRadius(value);
          blurred_.redraw();
          radius_frame_.setBlurRadius(value * 0.25f);
        };
        addChild(&radius_);
        animate_ = makeToggle("Animate", true, [this](bool on) {
          blurred_.setAnimate(on);
          reference_.setAnimate(on);
          radius_frame_.setAnimate(on);
        });
        addChild(animate_.get());
        remove_ = makeToggle("Blur effect attached", true, [this](bool on) {
          if (on)
            blurred_.setPostEffect(&blur_);
          else
            detachPostEffect(blurred_);
        });
        addChild(remove_.get());
      }

      ~BlurPage() override { detachPostEffect(blurred_); }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        layoutButtonRow(takeTop(bounds, 40), { &radius_, animate_.get(), remove_.get() }, 16);
        auto rows = splitRows(bounds, { 2.0f, 1.0f });
        blurred_box_.setBounds(rows[0]);
        blurred_.setBounds(blurred_box_.contentBounds());
        auto bottom = evenColumns(rows[1], 2);
        reference_box_.setBounds(bottom[0]);
        reference_.setBounds(reference_box_.contentBounds());
        radius_frame_box_.setBounds(bottom[1]);
        radius_frame_.setBounds(radius_frame_box_.contentBounds());
      }

    private:
      Box blurred_box_;
      Box reference_box_;
      Box radius_frame_box_;
      BusyFrame blurred_;
      BusyFrame reference_;
      BusyFrame radius_frame_;
      visage::BlurPostEffect blur_;
      Slider radius_ { "Blur radius", 0.0f, 300.0f, 20.0f, 1 };
      std::unique_ptr<visage::ToggleTextButton> animate_;
      std::unique_ptr<visage::ToggleTextButton> remove_;
    };

    class GlowShapes : public visage::Frame {
    public:
      explicit GlowShapes(float alpha = 1.0f) : alpha_(alpha) { }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(0xff101018);
        canvas.fill(0, 0, width(), height());
        float t = canvas.time();
        float r = std::min(width(), height()) * 0.12f;
        for (int i = 0; i < 3; ++i) {
          visage::Color color = visage::Color::fromAHSV(alpha_, i * 120.0f + 20.0f, 0.8f, 1.0f);
          color.setHdr(hdr_);
          canvas.setColor(color);
          float x = width() * (i + 0.5f) / 3.0f - r;
          canvas.circle(x, height() * 0.35f - r, 2 * r);
        }
        visage::Color line = visage::Color(alpha_, 1.0f, 1.0f, 1.0f, hdr_);
        canvas.setColor(line);
        visage::Path path;
        for (int i = 0; i <= 100; ++i) {
          float x = width() * i / 100.0f;
          float y = height() * (0.75f + 0.12f * std::sin(i * 0.15f + t * 2.0f));
          if (i == 0)
            path.moveTo(x, y);
          else
            path.lineTo(x, y);
        }
        canvas.fill(path.stroke(3.0f));
        redraw();
      }

      void setHdr(float hdr) { hdr_ = hdr; }

    private:
      float alpha_ = 1.0f;
      float hdr_ = 1.5f;
    };

    class BloomPage : public visage::Frame {
    public:
      BloomPage() :
          bloom_box_("Bloom, HDR colors"), reference_box_("No bloom, same colors"),
          translucent_box_("Bloom on translucent shapes"),
          group_box_("Bloom inside a 50% alpha group"), translucent_(0.5f) {
        bloom_.setBloomSize(size_.value());
        bloom_.setBloomIntensity(intensity_.value());
        translucent_bloom_.setBloomSize(size_.value());
        translucent_bloom_.setBloomIntensity(intensity_.value());
        group_bloom_.setBloomSize(size_.value());
        group_bloom_.setBloomIntensity(intensity_.value());

        glowing_.setPostEffect(&bloom_);
        translucent_.setPostEffect(&translucent_bloom_);
        group_inner_.setPostEffect(&group_bloom_);
        group_.setAlphaTransparency(0.5f);
        group_.addChild(&group_inner_);
        group_.setIgnoresMouseEvents(true, true);

        bloom_box_.addChild(&glowing_);
        reference_box_.addChild(&reference_);
        translucent_box_.addChild(&translucent_);
        group_box_.addChild(&group_);
        for (Box* box : { &bloom_box_, &reference_box_, &translucent_box_, &group_box_ })
          addChild(box);

        auto update = [this](float) {
          for (visage::BloomPostEffect* bloom : { &bloom_, &translucent_bloom_, &group_bloom_ }) {
            bloom->setBloomSize(size_.value());
            bloom->setBloomIntensity(intensity_.value());
          }
          for (GlowShapes* shapes : { &glowing_, &reference_, &translucent_, &group_inner_ })
            shapes->setHdr(hdr_.value());
        };
        size_.onChange() += update;
        intensity_.onChange() += update;
        hdr_.onChange() += update;
        addChild(&size_);
        addChild(&intensity_);
        addChild(&hdr_);
        group_bloom_toggle_ = makeToggle("Bloom in alpha group", true, [this](bool on) {
          if (on)
            group_inner_.setPostEffect(&group_bloom_);
          else
            detachPostEffect(group_inner_);
        });
        addChild(group_bloom_toggle_.get());
      }

      ~BloomPage() override {
        detachPostEffect(glowing_);
        detachPostEffect(translucent_);
        detachPostEffect(group_inner_);
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        layoutButtonRow(takeTop(bounds, 40), { &size_, &intensity_, &hdr_, group_bloom_toggle_.get() }, 16);
        auto cells = grid(bounds, 2, 2);
        Box* boxes[] = { &bloom_box_, &reference_box_, &translucent_box_, &group_box_ };
        GlowShapes* contents[] = { &glowing_, &reference_, &translucent_, nullptr };
        for (int i = 0; i < 4; ++i) {
          boxes[i]->setBounds(cells[i]);
          if (contents[i])
            contents[i]->setBounds(boxes[i]->contentBounds());
        }
        group_.setBounds(group_box_.contentBounds());
        group_inner_.setBounds(group_.localBounds());
      }

    private:
      Box bloom_box_;
      Box reference_box_;
      Box translucent_box_;
      Box group_box_;
      GlowShapes glowing_;
      GlowShapes reference_;
      GlowShapes translucent_;
      visage::Frame group_;
      GlowShapes group_inner_;
      visage::BloomPostEffect bloom_;
      visage::BloomPostEffect translucent_bloom_;
      visage::BloomPostEffect group_bloom_;
      Slider size_ { "Bloom size", 1.0f, 120.0f, 30.0f, 0 };
      Slider intensity_ { "Bloom intensity", 0.0f, 6.0f, 1.5f, 2 };
      Slider hdr_ { "HDR multiplier", 1.0f, 4.0f, 1.5f, 2 };
      std::unique_ptr<visage::ToggleTextButton> group_bloom_toggle_;
    };

    class Glass : public visage::Frame {
    public:
      Glass(std::string label, visage::PostEffect* effect) :
          Frame(std::move(label)), effect_(effect) {
        setBackdropEffect(effect_);
        onDrawMask() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xffffffff);
          canvas.roundedRectangle(0, 0, width(), height(), 24);
        };
      }

      ~Glass() override { setBackdropEffect(nullptr); }

      void draw(visage::Canvas& canvas) override {
        canvas.setColor(0x22ffffff);
        canvas.fill(0, 0, width(), height());
        canvas.setColor(visage::Brush::vertical(0x66ffffff, 0x22ffffff));
        canvas.roundedRectangleBorder(0, 0, width(), height(), 24, 2);
        canvas.setColor(0xffffffff);
        canvas.text(name() + " (drag me)", font(14), visage::Font::kCenter, 0, 0, width(), height());
      }

      void mouseDown(const visage::MouseEvent& e) override { grab_ = e.position; }

      void mouseDrag(const visage::MouseEvent& e) override {
        setTopLeft(x() + e.position.x - grab_.x, y() + e.position.y - grab_.y);
      }

      void mouseEnter(const visage::MouseEvent& e) override {
        setCursorStyle(visage::MouseCursor::MultiDirectionalResize);
      }
      void mouseExit(const visage::MouseEvent& e) override {
        setCursorStyle(visage::MouseCursor::Arrow);
      }

    private:
      visage::PostEffect* effect_ = nullptr;
      visage::Point grab_;
    };

    class BackdropPage : public visage::Frame {
    public:
      BackdropPage() :
          invert_(resources::shaders::vs_custom, resources::shaders::fs_backdrop_invert),
          blur_glass_("Blur", &blur_), invert_glass_("Invert shader", &invert_),
          nested_glass_("Blur in 80% parent", &nested_blur_) {
        blur_.setBlurRadius(30.0f);
        nested_blur_.setBlurRadius(15.0f);
        addChild(&background_);
        addChild(&blur_glass_);
        addChild(&invert_glass_);
        nested_parent_.setAlphaTransparency(0.8f);
        nested_parent_.setIgnoresMouseEvents(true, true);
        nested_parent_.addChild(&nested_glass_);
        addChild(&nested_parent_);

        animate_ = makeToggle("Animate background", true,
                              [this](bool on) { background_.setAnimate(on); });
        hide_parent_ = makeToggle("Show nested glass's parent", true,
                                  [this](bool on) { nested_parent_.setVisible(on); });
        radius_.onChange() += [this](float value) {
          blur_.setBlurRadius(value);
          nested_blur_.setBlurRadius(value * 0.5f);
          redrawAll();
        };
        on_top_ = makeButton("Add an on-top child to the blur glass", [this] {
          if (on_top_child_)
            return;
          on_top_child_ = std::make_unique<visage::Frame>();
          visage::Frame* raw = on_top_child_.get();
          on_top_child_->onDraw() = [raw](visage::Canvas& canvas) {
            canvas.setColor(0xffff8040);
            canvas.circle(0, 0, raw->width());
          };
          on_top_child_->setIgnoresMouseEvents(true, false);
          on_top_child_->setOnTop(true);
          blur_glass_.addChild(on_top_child_.get());
          on_top_child_->setBounds(-20, -20, 60, 60);
        });
        for (visage::Frame* frame : { (visage::Frame*)animate_.get(), (visage::Frame*)hide_parent_.get(),
                                      (visage::Frame*)&radius_, (visage::Frame*)on_top_.get() })
          addChild(frame);
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        layoutButtonRow(takeTop(bounds, 40),
                        { animate_.get(), hide_parent_.get(), &radius_, on_top_.get() }, 12);
        background_.setBounds(bounds);
        if (!placed_) {
          placed_ = true;
          float w = std::min(260.0f, bounds.width() * 0.35f);
          float h = w * 0.6f;
          blur_glass_.setBounds(bounds.x() + bounds.width() * 0.1f,
                                bounds.y() + bounds.height() * 0.15f, w, h);
          invert_glass_.setBounds(bounds.x() + bounds.width() * 0.55f,
                                  bounds.y() + bounds.height() * 0.2f, w, h);
          nested_parent_.setBounds(bounds);
          nested_glass_.setBounds(bounds.width() * 0.3f, bounds.height() * 0.6f, w, h);
        }
        nested_parent_.setBounds(bounds);
      }

    private:
      bool placed_ = false;
      BusyFrame background_;
      visage::BlurPostEffect blur_;
      visage::BlurPostEffect nested_blur_;
      visage::ShaderPostEffect invert_;
      Glass blur_glass_;
      Glass invert_glass_;
      visage::Frame nested_parent_;
      Glass nested_glass_;
      std::unique_ptr<visage::Frame> on_top_child_;
      std::unique_ptr<visage::ToggleTextButton> animate_;
      std::unique_ptr<visage::ToggleTextButton> hide_parent_;
      std::unique_ptr<visage::UiButton> on_top_;
      Slider radius_ { "Backdrop blur radius", 0.0f, 120.0f, 30.0f, 0 };
    };

    class MasksPage : public visage::Frame {
    public:
      MasksPage() :
          alpha_box_("Alpha mask: circle"), luminance_box_("Luminance mask: white to black"),
          blur_box_("Mask with blur: blur stays inside the mask"),
          children_box_("Mask covers children added later"), bloom_box_("Mask with bloom"),
          image_box_("Masked image") {
        alpha_.onDrawMask() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xffffffff);
          float size = std::min(alpha_.width(), alpha_.height());
          canvas.circle((alpha_.width() - size) * 0.5f, (alpha_.height() - size) * 0.5f, size);
        };
        luminance_.setMaskType(visage::MaskType::Luminance);
        luminance_.onDrawMask() = [this](visage::Canvas& canvas) {
          canvas.setColor(visage::Brush::horizontal(0xffffffff, 0xff000000));
          canvas.fill(0, 0, luminance_.width(), luminance_.height());
        };
        blurred_.onDrawMask() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xffffffff);
          canvas.roundedRectangle(blurred_.width() * 0.15f, blurred_.height() * 0.15f,
                                  blurred_.width() * 0.7f, blurred_.height() * 0.7f, 20);
        };
        blur_.setBlurRadius(12.0f);
        blurred_.setPostEffect(&blur_);
        parent_.onDraw() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xff2f5d8a);
          canvas.fill(0, 0, parent_.width(), parent_.height());
        };
        parent_.onDrawMask() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xffffffff);
          float size = std::min(parent_.width(), parent_.height());
          canvas.circle((parent_.width() - size) * 0.5f, (parent_.height() - size) * 0.5f, size);
        };
        add_child_ = makeButton("Add on-top child", [this] {
          auto child = std::make_unique<visage::Frame>();
          visage::Frame* raw = child.get();
          child->onDraw() = [raw](visage::Canvas& canvas) {
            canvas.setColor(0xffffcc40);
            canvas.fill(0, 0, raw->width(), raw->height());
          };
          child->setOnTop(true);
          child->setIgnoresMouseEvents(true, false);
          parent_.addChild(child.get());
          float offset = children_.size() * 18.0f;
          child->setBounds(offset, offset, parent_.width() * 0.5f, parent_.height() * 0.3f);
          children_.push_back(std::move(child));
        });
        bloom_.setBloomSize(30);
        bloom_.setBloomIntensity(2);
        glowing_.setPostEffect(&bloom_);
        glowing_.onDrawMask() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xffffffff);
          canvas.roundedRectangle(0, 0, glowing_.width(), glowing_.height() * 0.7f, 20);
        };
        image_.onDraw() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xffffffff);
          float size = std::min(image_.width(), image_.height());
          canvas.image(resources::images::test_png, (image_.width() - size) * 0.5f, 0, size, size);
        };
        image_.onDrawMask() = [this](visage::Canvas& canvas) {
          canvas.setColor(0xffffffff);
          float size = std::min(image_.width(), image_.height());
          canvas.squircle((image_.width() - size) * 0.5f, 0, size);
        };

        alpha_box_.addChild(&alpha_);
        luminance_box_.addChild(&luminance_);
        blur_box_.addChild(&blurred_);
        children_box_.addChild(&parent_);
        children_box_.addChild(add_child_.get());
        bloom_box_.addChild(&glowing_);
        image_box_.addChild(&image_);
        for (Box* box :
             { &alpha_box_, &luminance_box_, &blur_box_, &children_box_, &bloom_box_, &image_box_ })
          addChild(box);
      }

      ~MasksPage() override {
        detachPostEffect(blurred_);
        detachPostEffect(glowing_);
      }

      void resized() override {
        auto cells = grid(localBounds().reduced(12), 3, 2);
        Box* boxes[] = { &alpha_box_,    &luminance_box_, &blur_box_,
                         &children_box_, &bloom_box_,     &image_box_ };
        visage::Frame* contents[] = { &alpha_, &luminance_, &blurred_, &parent_, &glowing_, &image_ };
        for (int i = 0; i < 6; ++i) {
          boxes[i]->setBounds(cells[i]);
          visage::Bounds content = boxes[i]->contentBounds();
          if (contents[i] == &parent_)
            add_child_->setBounds(takeBottom(content, 32, 6));
          contents[i]->setBounds(content);
        }
      }

    private:
      Box alpha_box_;
      Box luminance_box_;
      Box blur_box_;
      Box children_box_;
      Box bloom_box_;
      Box image_box_;
      BusyFrame alpha_;
      BusyFrame luminance_;
      BusyFrame blurred_;
      visage::Frame parent_;
      GlowShapes glowing_;
      visage::Frame image_;
      visage::BlurPostEffect blur_;
      visage::BloomPostEffect bloom_;
      std::unique_ptr<visage::UiButton> add_child_;
      std::vector<std::unique_ptr<visage::Frame>> children_;
    };

    class BlendPage : public visage::Frame {
    public:
      BlendPage() : gray_(resources::shaders::vs_custom, resources::shaders::fs_gray_scale) {
        addChild(&background_);
        background_.setAnimate(false);
        const char* names[] = { "Alpha", "Add", "Sub", "Mult" };
        visage::BlendMode modes[] = { visage::BlendMode::Alpha, visage::BlendMode::Add,
                                      visage::BlendMode::Sub, visage::BlendMode::Mult };
        for (int i = 0; i < 4; ++i) {
          auto frame = std::make_unique<visage::Frame>(names[i]);
          visage::Frame* raw = frame.get();
          frame->onDraw() = [raw](visage::Canvas& canvas) {
            canvas.setColor(0xffffffff);
            canvas.text(raw->name() + " layer", font(14), visage::Font::kTop, 0, 4, raw->width(), 20);
            float r = std::min(raw->width(), raw->height()) * 0.28f;
            unsigned int colors[] = { 0xccff4040, 0xcc40ff40, 0xcc4060ff };
            for (int c = 0; c < 3; ++c) {
              float angle = c * 2.0f * kPi / 3.0f - kPi * 0.5f;
              canvas.setColor(colors[c]);
              canvas.circle(raw->width() * 0.5f + std::cos(angle) * r * 0.6f - r,
                            raw->height() * 0.55f + std::sin(angle) * r * 0.6f - r, 2 * r);
            }
          };
          frame->setLayerBlendMode(modes[i]);
          frame->setIgnoresMouseEvents(true, false);
          addChild(frame.get());
          layers_.push_back(std::move(frame));
        }

        shapes_.onDraw() = [this](visage::Canvas& canvas) {
          float w = shapes_.width() / 4.0f;
          const char* names[] = { "Alpha", "Add", "Sub", "Mult" };
          visage::BlendMode modes[] = { visage::BlendMode::Alpha, visage::BlendMode::Add,
                                        visage::BlendMode::Sub, visage::BlendMode::Mult };
          for (int i = 0; i < 4; ++i) {
            canvas.setBlendMode(visage::BlendMode::Alpha);
            canvas.setColor(0xff808080);
            canvas.fill(i * w + 4, 0, w - 8, shapes_.height());
            canvas.setColor(0xffffffff);
            canvas.text(std::string(names[i]) + " shapes", font(12), visage::Font::kTop, i * w, 2, w, 18);
            canvas.setBlendMode(modes[i]);
            float r = std::min(w, shapes_.height()) * 0.3f;
            canvas.setColor(0x80ff4040);
            canvas.circle(i * w + w * 0.4f - r, shapes_.height() * 0.55f - r, 2 * r);
            canvas.setColor(0x804060ff);
            canvas.circle(i * w + w * 0.6f - r, shapes_.height() * 0.55f - r, 2 * r);
          }
        };
        addChild(&shapes_);

        effect_frame_.onDraw() = [this](visage::Canvas& canvas) {
          float w = effect_frame_.width();
          float h = effect_frame_.height();
          canvas.setColor(visage::Brush::horizontal(visage::Gradient(0xffff0000, 0xff00ff00, 0xff0000ff)));
          canvas.roundedRectangle(0, 0, w, h, 12);
          canvas.setColor(0xffffffff);
          canvas.text("Gray scale shader + frame alpha", font(14), visage::Font::kCenter, 0, 0, w, h);
        };
        effect_frame_.setPostEffect(&gray_);
        effect_frame_.setAlphaTransparency(alpha_.value());
        addChild(&effect_frame_);
        alpha_.onChange() += [this](float value) { effect_frame_.setAlphaTransparency(value); };
        addChild(&alpha_);
      }

      ~BlendPage() override { detachPostEffect(effect_frame_); }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        background_.setBounds(bounds);
        visage::Bounds top = takeTop(bounds, bounds.height() * 0.5f);
        auto cells = evenColumns(top, 4);
        for (int i = 0; i < 4; ++i)
          layers_[i]->setBounds(cells[i]);
        shapes_.setBounds(takeTop(bounds, bounds.height() * 0.5f));
        alpha_.setBounds(takeBottom(bounds, 38));
        effect_frame_.setBounds(bounds.reduced(bounds.width() * 0.15f, bounds.width() * 0.15f, 0, 0));
      }

    private:
      BusyFrame background_;
      std::vector<std::unique_ptr<visage::Frame>> layers_;
      visage::Frame shapes_;
      visage::Frame effect_frame_;
      visage::ShaderPostEffect gray_;
      Slider alpha_ { "Gray scale frame alpha", 0.0f, 1.0f, 0.6f, 2 };
    };

    class StressPage : public visage::Frame {
    public:
      StressPage() {
        container_.setIgnoresMouseEvents(true, true);
        addChild(&container_);
        addMode("Clear", 0, 0);
        addMode("2500 blur frames", 1, 2500);
        addMode("120 bloom frames", 2, 120);
        addMode("4500 shader frames", 3, 4500);
        addMode("3000 redrawing frames", 4, 3000);
        addMode("100k circles in one frame", 5, 100000);
        resize_ = makeToggle("Resize effect frames every frame", false,
                             [this](bool on) { resize_every_frame_ = on; });
        addChild(resize_.get());
        addChild(&info_);
        info_.setMono(true);
        info_.setMultiLine(true);
      }

      ~StressPage() override { clear(); }

      void draw(visage::Canvas& canvas) override {
        double dt = canvas.deltaTime();
        if (dt > 0)
          fps_ = fps_ * 0.95 + 0.05 / dt;
        std::vector<std::string> debug = canvas.debugInfo();
        std::string text = "Mode: " + mode_name_ + "   frames: " + std::to_string(frames_.size()) +
                           "   fps: " + visage::String(static_cast<float>(fps_), 1).toUtf8();
        for (int i = 0; i < std::min<int>(3, debug.size()); ++i)
          text += "\n" + debug[i];
        info_.setText(text);

        if (resize_every_frame_ && !frames_.empty()) {
          float wobble = 0.85f + 0.15f * std::sin(canvas.time() * 5.0);
          layoutFrames(wobble);
        }
        redraw();
      }

      void resized() override {
        visage::Bounds bounds = localBounds().reduced(12);
        std::vector<visage::Frame*> buttons;
        for (auto& button : buttons_)
          buttons.push_back(button.get());
        buttons.push_back(resize_.get());
        auto cells = grid(takeTop(bounds, 76), 4, 2, 6);
        for (int i = 0; i < buttons.size(); ++i)
          buttons[i]->setBounds(cells[i]);
        info_.setBounds(takeTop(bounds, 76));
        container_.setBounds(bounds);
        layoutFrames(1.0f);
      }

    private:
      void addMode(const std::string& name, int mode, int count) {
        buttons_.push_back(makeButton(name, [this, name, mode, count] { setMode(name, mode, count); }));
        addChild(buttons_.back().get());
      }

      void clear() {
        for (auto& frame : frames_) {
          detachPostEffect(*frame);
          container_.removeChild(frame.get());
        }
        frames_.clear();
        blurs_.clear();
        blooms_.clear();
        shaders_.clear();
        container_.onDraw().clear();
        container_.redraw();
      }

      void setMode(const std::string& name, int mode, int count) {
        clear();
        mode_name_ = name;
        if (mode == 5) {
          container_.onDraw() = [this, count](visage::Canvas& canvas) {
            float w = container_.width();
            float h = container_.height();
            float t = canvas.time();
            for (int i = 0; i < count; ++i) {
              float x = std::fmod(i * 7.31f + t * 20.0f, w);
              float y = std::fmod(i * 3.17f, h);
              canvas.setColor(visage::Color::fromAHSV(0.6f, std::fmod(i * 0.37f, 360.0f), 0.7f, 1.0f));
              canvas.circle(x, y, 4);
            }
            container_.redraw();
          };
          container_.redraw();
          return;
        }

        for (int i = 0; i < count; ++i) {
          auto frame = std::make_unique<visage::Frame>();
          visage::Frame* raw = frame.get();
          bool redraws = mode == 4;
          frame->onDraw() = [raw, i, redraws](visage::Canvas& canvas) {
            float t = canvas.time();
            canvas.setColor(visage::Color::fromAHSV(1.0f, std::fmod(i * 13.0f + t * 60.0f, 360.0f),
                                                    0.7f, 1.0f));
            canvas.fill(0, 0, raw->width(), raw->height());
            canvas.setColor(0xffffffff);
            canvas.circle(raw->width() * 0.25f, raw->height() * 0.25f, raw->width() * 0.5f);
            if (redraws)
              raw->redraw();
          };
          if (mode == 1) {
            blurs_.push_back(std::make_unique<visage::BlurPostEffect>());
            blurs_.back()->setBlurRadius(3.0f);
            frame->setPostEffect(blurs_.back().get());
          }
          else if (mode == 2) {
            blooms_.push_back(std::make_unique<visage::BloomPostEffect>());
            blooms_.back()->setBloomSize(8.0f);
            blooms_.back()->setBloomIntensity(1.5f);
            frame->setPostEffect(blooms_.back().get());
          }
          else if (mode == 3) {
            shaders_.push_back(std::make_unique<visage::ShaderPostEffect>(resources::shaders::vs_custom,
                                                                          resources::shaders::fs_sepia));
            frame->setPostEffect(shaders_.back().get());
          }
          container_.addChild(frame.get());
          frames_.push_back(std::move(frame));
        }
        layoutFrames(1.0f);
      }

      void layoutFrames(float scale) {
        if (frames_.empty())
          return;
        int columns = std::max(1,
                               static_cast<int>(std::ceil(std::sqrt(frames_.size() * container_.width() /
                                                                    std::max(1.0f, container_.height())))));
        int rows = (frames_.size() + columns - 1) / columns;
        float cw = container_.width() / columns;
        float ch = container_.height() / std::max(1, rows);
        for (int i = 0; i < frames_.size(); ++i) {
          float x = (i % columns) * cw;
          float y = (i / columns) * ch;
          frames_[i]->setBounds(x, y, std::max(1.0f, (cw - 1) * scale), std::max(1.0f, (ch - 1) * scale));
        }
      }

      visage::Frame container_;
      std::vector<std::unique_ptr<visage::Frame>> frames_;
      std::vector<std::unique_ptr<visage::BlurPostEffect>> blurs_;
      std::vector<std::unique_ptr<visage::BloomPostEffect>> blooms_;
      std::vector<std::unique_ptr<visage::ShaderPostEffect>> shaders_;
      std::vector<std::unique_ptr<visage::UiButton>> buttons_;
      std::unique_ptr<visage::ToggleTextButton> resize_;
      Label info_ { "", 12, kDimText };
      std::string mode_name_ = "Clear";
      bool resize_every_frame_ = false;
      double fps_ = 60.0;
    };
  }

  void addEffectsTests(std::vector<TestDefinition>& tests) {
    tests.push_back(
        { "blur",
          "Effects",
          "Blur",
          "A BlurPostEffect on animated content, a reference with no blur, and a frame using "
          "setBlurRadius.",
          {
              "Drag the radius slowly from 0 to 300. The blur grows smoothly with no sudden jumps "
              "in "
              "strength or brightness.",
              "At radius 0 the content is as sharp as the reference.",
              "Blurred content keeps animating smoothly at every radius, with no stale or frozen "
              "frames.",
              "Blur edges don't pick up colors from outside the frame and have no dark halo.",
              "Turning \"Blur effect attached\" off and on switches between sharp and blurred.",
              "Resizing the window with a big radius has no flicker or crash.",
          },
          [] { return std::make_unique<BlurPage>(); } });

    tests.push_back(
        { "bloom",
          "Effects",
          "Bloom and HDR",
          "Bloom on HDR shapes. The reference frame draws the same shapes with no bloom.",
          {
              "Shapes in the bloom frame glow, and their centers are as bright as the reference, "
              "not "
              "darker.",
              "Bloom size and intensity change the glow smoothly.",
              "Translucent (50% alpha) shapes have smooth alpha with no banding. Raise the HDR "
              "multiplier to 3 or more: they start to glow, weaker than the opaque ones, never "
              "brighter.",
              "Bloom inside the 50% alpha group doesn't black out the frame's rectangle, and the "
              "whole "
              "group (shapes and glow) looks half transparent compared to the top-left frame.",
              "Turning \"Bloom in alpha group\" off leaves the shapes at the same half-transparent "
              "brightness, just without the glow.",
              "Raising the HDR multiplier makes the glow stronger. At 1.0 there is little or no "
              "glow.",
              "No flicker after leaving and coming back to this page.",
          },
          [] { return std::make_unique<BloomPage>(); } });

    tests.push_back({ "backdrop",
                      "Effects",
                      "Backdrop Effects",
                      "Glass panels blur or invert what's behind them. Drag them around.",
                      {
                          "Each glass shows only what's under it, blurred or inverted, with "
                          "rounded corners and "
                          "mirrored edges (no dark or smeared borders).",
                          "Dragging a glass updates immediately with no trail of its old position.",
                          "Content moving just outside a glass shows up in its blur as it enters.",
                          "Overlapping glasses each show the content correctly.",
                          "The glass in the 80% alpha parent only blurs what's inside that parent "
                          "(nothing), so it "
                          "looks like plain frosted gray, the way CSS backdrop roots work.",
                          "\"Show nested glass's parent\" off hides it without a crash, and on "
                          "brings it back.",
                          "\"Add an on-top child\" draws an orange circle on top of the blur "
                          "glass, not under it.",
                          "Changing the backdrop blur radius updates both blur glasses.",
                      },
                      [] { return std::make_unique<BackdropPage>(); } });

    tests.push_back(
        { "masks",
          "Effects",
          "Masks",
          "Frames masked with onDrawMask, alone and combined with other effects.",
          {
              "Alpha mask: the animated content shows only inside the circle, with a smooth edge.",
              "Luminance mask: content fades from fully visible on the left to invisible on the "
              "right.",
              "Mask with blur: the blurred content is cut by the rounded rectangle mask, with no "
              "blur "
              "spilling outside.",
              "\"Add on-top child\" adds yellow rectangles that are also cut by the circle mask.",
              "Mask with bloom: the glow stays inside the mask shape.",
              "The masked image is a squircle with clean edges.",
          },
          [] { return std::make_unique<MasksPage>(); } });

    tests.push_back(
        { "blend",
          "Effects",
          "Blend Modes and Opacity",
          "Layer blend modes on frames, shape blend modes, and a shader effect combined with frame "
          "alpha.",
          {
              "Alpha layer: overlapping translucent circles look normal.",
              "Add layer brightens what's behind it, and translucent circles don't add twice as "
              "much.",
              "Sub layer darkens what's behind it.",
              "Mult layer darkens with the circle colors and never cuts black or transparent "
              "holes.",
              "Shape blend modes: the four pairs of circles over gray match their names and the "
              "alpha "
              "is respected (half-strength effect).",
              "The gray scale frame fades with the alpha slider and stays gray at every alpha.",
          },
          [] { return std::make_unique<BlendPage>(); } });

    tests.push_back(
        { "stress",
          "Effects",
          "Effect Stress",
          "Large numbers of frames and effects. The readout shows the mode, frame count and frame "
          "rate.",
          {
              "2500 blur frames: every small frame stays blurred. None show up sharp.",
              "120 bloom frames: all draw and glow with no crash.",
              "4500 shader frames: all draw in sepia with no crash or missing frames.",
              "3000 redrawing frames: frame rate stays usable and colors cycle smoothly.",
              "100k circles in one frame draw with no missing patches or flicker.",
              "\"Resize effect frames every frame\" with blur or bloom running for 30 seconds "
              "doesn't "
              "slow down over time or crash.",
              "\"Clear\" after each mode returns to a normal frame rate.",
          },
          [] { return std::make_unique<StressPage>(); } });
  }
}
