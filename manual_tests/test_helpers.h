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

#pragma once

#include "embedded/example_fonts.h"

#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <visage/app.h>
#include <visage/graphics.h>
#include <visage/ui.h>
#include <visage/widgets.h>

namespace manual {
  static constexpr unsigned int kBackground = 0xff1b1f24;
  static constexpr unsigned int kPanel = 0xff262b32;
  static constexpr unsigned int kPanelLight = 0xff323843;
  static constexpr unsigned int kBorder = 0xff3d4450;
  static constexpr unsigned int kText = 0xffe8eaed;
  static constexpr unsigned int kDimText = 0xff9aa3ad;
  static constexpr unsigned int kAccent = 0xff5ab0ff;
  static constexpr unsigned int kPass = 0xff4caf50;
  static constexpr unsigned int kFail = 0xffe5534b;
  static constexpr unsigned int kSkip = 0xff8a8f98;
  static constexpr unsigned int kWarning = 0xffe0a030;

  inline visage::Font font(float size) {
    return visage::Font(size, resources::fonts::Lato_Regular_ttf);
  }

  inline visage::Font monoFont(float size) {
    return visage::Font(size, resources::fonts::DroidSansMono_ttf);
  }

  inline std::string pathString(const std::filesystem::path& path) {
    std::u8string utf8 = path.u8string();
    return { utf8.begin(), utf8.end() };
  }

  inline std::filesystem::path pathFromUtf8(const std::string& utf8) {
    return std::filesystem::path(std::u8string(utf8.begin(), utf8.end()));
  }

  inline float wrappedTextHeight(const std::string& text, const visage::Font& font, float width) {
    if (text.empty() || width <= 0.0f)
      return 0.0f;
    visage::String string(text);
    std::vector<int> breaks = font.lineBreaks(string.c_str(), string.length(), width);
    return (breaks.size() + 1) * font.lineHeight();
  }

  inline void drawLines(visage::Canvas& canvas, const std::string& text, const visage::Font& font,
                        visage::Font::Justification justification, float x, float y, float width,
                        float height) {
    std::vector<std::string> lines;
    size_t start = 0;
    while (true) {
      size_t end = text.find('\n', start);
      lines.push_back(text.substr(start, end == std::string::npos ? std::string::npos : end - start));
      if (end == std::string::npos)
        break;
      start = end + 1;
    }

    float line_height = font.size() * 1.35f;
    float total = line_height * lines.size();
    float top = y;
    if (!(justification & visage::Font::kTop))
      top = (justification & visage::Font::kBottom) ? y + height - total : y + (height - total) * 0.5f;

    int horizontal = justification & (visage::Font::kLeft | visage::Font::kRight);
    auto line_justification = static_cast<visage::Font::Justification>(horizontal);
    for (int i = 0; i < lines.size(); ++i)
      canvas.text(lines[i], font, line_justification, x, top + i * line_height, width, line_height);
  }

  struct TestDefinition {
    std::string id;
    std::string category;
    std::string title;
    std::string setup;
    std::vector<std::string> checks;
    std::function<std::unique_ptr<visage::Frame>()> create;
    bool on_web = true;
    std::string web_note;
  };

#if VISAGE_EMSCRIPTEN
  static constexpr bool kWebBuild = true;
#else
  static constexpr bool kWebBuild = false;
#endif

  void addInputTests(std::vector<TestDefinition>& tests);
  void addWindowTests(std::vector<TestDefinition>& tests);
  void addUiTests(std::vector<TestDefinition>& tests);
  void addGraphicsTests(std::vector<TestDefinition>& tests);
  void addSvgTests(std::vector<TestDefinition>& tests);
  void addEffectsTests(std::vector<TestDefinition>& tests);
  void addSystemTests(std::vector<TestDefinition>& tests);

  std::string lastSessionStatus();

  class Label : public visage::Frame {
  public:
    explicit Label(const std::string& text = "", float size = 15.0f, unsigned int color = kText,
                   visage::Font::Justification justification = visage::Font::kLeft) :
        size_(size), color_(color) {
      text_.setText(text);
      text_.setFont(font(size));
      text_.setJustification(justification);
      setIgnoresMouseEvents(true, false);
    }

    void draw(visage::Canvas& canvas) override {
      if (background_) {
        canvas.setColor(background_);
        canvas.roundedRectangle(0, 0, width(), height(), 4);
      }
      canvas.setColor(color_);
      canvas.text(&text_, 0, 0, width(), height());
    }

    void setText(const std::string& text) {
      text_.setText(text);
      redraw();
    }

    std::string text() const { return text_.text().toUtf8(); }

    void setColor(unsigned int color) {
      color_ = color;
      redraw();
    }

    void setBackground(unsigned int background) {
      background_ = background;
      redraw();
    }

    void setMultiLine(bool multi_line) {
      text_.setMultiLine(multi_line);
      text_.setJustification(multi_line ? visage::Font::kTopLeft : text_.justification());
      redraw();
    }

    void setMono(bool mono) {
      text_.setFont(mono ? monoFont(size_) : font(size_));
      redraw();
    }

    float heightForWidth(float w) const {
      return wrappedTextHeight(text_.text().toUtf8(), text_.font().withDpiScale(dpiScale()), w) + 2.0f;
    }

  private:
    visage::Text text_;
    float size_ = 15.0f;
    unsigned int color_ = kText;
    unsigned int background_ = 0;
  };

  class Box : public visage::Frame {
  public:
    static constexpr float kTitleHeight = 26.0f;
    static constexpr float kPadding = 8.0f;

    explicit Box(std::string title = "") : title_(std::move(title)) { }

    void draw(visage::Canvas& canvas) override {
      canvas.setColor(kPanel);
      canvas.roundedRectangle(0, 0, width(), height(), 6);
      canvas.setColor(kBorder);
      canvas.roundedRectangleBorder(0, 0, width(), height(), 6, 1);
      if (!title_.empty()) {
        canvas.setColor(kDimText);
        canvas.text(title_, font(13), visage::Font::kLeft, kPadding, 0, width() - 2 * kPadding, kTitleHeight);
      }
    }

    visage::Bounds contentBounds() const {
      float top = title_.empty() ? kPadding : kTitleHeight;
      return { kPadding, top, width() - 2 * kPadding, height() - top - kPadding };
    }

    void setTitle(const std::string& title) {
      title_ = title;
      redraw();
    }

  private:
    std::string title_;
  };

  class Slider : public visage::Frame {
  public:
    Slider(std::string name, float min, float max, float value, int decimals = 1) :
        name_(std::move(name)), min_(min), max_(max), value_(value), decimals_(decimals) { }

    auto& onChange() { return on_change_; }

    void draw(visage::Canvas& canvas) override {
      float label_height = height() * 0.5f;
      canvas.setColor(kDimText);
      canvas.text(name_ + ": " + visage::String(value_, decimals_).toUtf8(), font(13),
                  visage::Font::kLeft, 0, 0, width(), label_height);

      float track_y = label_height + (height() - label_height) * 0.5f;
      canvas.setColor(kPanelLight);
      canvas.roundedRectangle(0, track_y - 3, width(), 6, 3);
      float t = (value_ - min_) / (max_ - min_);
      canvas.setColor(kAccent);
      canvas.roundedRectangle(0, track_y - 3, width() * t, 6, 3);
      canvas.setColor(hover_ || dragging_ ? 0xffffffff : 0xffd0d4da);
      canvas.circle(width() * t - 7, track_y - 7, 14);
    }

    void mouseEnter(const visage::MouseEvent& e) override {
      hover_ = true;
      redraw();
    }
    void mouseExit(const visage::MouseEvent& e) override {
      hover_ = false;
      redraw();
    }
    void mouseDown(const visage::MouseEvent& e) override {
      dragging_ = true;
      setFromPosition(e.position.x);
    }
    void mouseDrag(const visage::MouseEvent& e) override { setFromPosition(e.position.x); }
    void mouseUp(const visage::MouseEvent& e) override {
      dragging_ = false;
      redraw();
    }

    void setValue(float value, bool notify = true) {
      value_ = std::max(min_, std::min(max_, value));
      if (notify)
        on_change_.callback(value_);
      redraw();
    }

    float value() const { return value_; }
    void setFraction(float t) { setValue(min_ + t * (max_ - min_)); }

  private:
    void setFromPosition(float x) {
      float t = std::max(0.0f, std::min(1.0f, x / std::max(1.0f, width())));
      float value = min_ + t * (max_ - min_);
      if (decimals_ == 0)
        value = std::round(value);
      setValue(value);
    }

    std::string name_;
    float min_ = 0.0f;
    float max_ = 1.0f;
    float value_ = 0.0f;
    int decimals_ = 1;
    bool hover_ = false;
    bool dragging_ = false;
    visage::CallbackList<void(float)> on_change_;
  };

  class LogView : public visage::Frame {
  public:
    explicit LogView(int max_lines = 400) : max_lines_(max_lines) {
      setIgnoresMouseEvents(true, false);
    }

    void add(const std::string& line) {
      lines_.push_back(std::to_string(++count_) + "  " + line);
      while (lines_.size() > max_lines_)
        lines_.pop_front();
      redraw();
    }

    void clear() {
      lines_.clear();
      redraw();
    }

    void draw(visage::Canvas& canvas) override {
      canvas.setColor(0xff111418);
      canvas.roundedRectangle(0, 0, width(), height(), 4);
      visage::Font log_font = monoFont(12);
      float line_height = 16.0f;
      int visible = std::max(0, static_cast<int>((height() - 8) / line_height));
      int start = std::max(0, static_cast<int>(lines_.size()) - visible);
      canvas.setColor(0xffc8d0d8);
      for (int i = start; i < lines_.size(); ++i) {
        float y = 4 + (i - start) * line_height;
        canvas.text(lines_[i], log_font, visage::Font::kLeft, 6, y, width() - 12, line_height);
      }
    }

  private:
    std::deque<std::string> lines_;
    int max_lines_ = 400;
    int count_ = 0;
  };

  inline std::unique_ptr<visage::UiButton> makeButton(const std::string& text,
                                                      std::function<void()> callback) {
    auto button = std::make_unique<visage::UiButton>(text);
    button->setName(text);
    button->setFont(font(14));
    button->onToggle() = [callback = std::move(callback)](visage::Button*, bool) { callback(); };
    return button;
  }

  inline std::vector<visage::Bounds> splitRows(const visage::Bounds& bounds,
                                               const std::vector<float>& weights, float gap = 8.0f) {
    float total = 0.0f;
    for (float w : weights)
      total += w;
    float available = bounds.height() - gap * (weights.size() - 1);
    std::vector<visage::Bounds> result;
    float y = bounds.y();
    for (float w : weights) {
      float h = available * w / total;
      result.emplace_back(bounds.x(), y, bounds.width(), h);
      y += h + gap;
    }
    return result;
  }

  inline std::vector<visage::Bounds> splitColumns(const visage::Bounds& bounds,
                                                  const std::vector<float>& weights, float gap = 8.0f) {
    float total = 0.0f;
    for (float w : weights)
      total += w;
    float available = bounds.width() - gap * (weights.size() - 1);
    std::vector<visage::Bounds> result;
    float x = bounds.x();
    for (float w : weights) {
      float width = available * w / total;
      result.emplace_back(x, bounds.y(), width, bounds.height());
      x += width + gap;
    }
    return result;
  }

  inline std::vector<visage::Bounds> evenRows(const visage::Bounds& bounds, int count, float gap = 8.0f) {
    return splitRows(bounds, std::vector<float>(count, 1.0f), gap);
  }

  inline std::vector<visage::Bounds> evenColumns(const visage::Bounds& bounds, int count, float gap = 8.0f) {
    return splitColumns(bounds, std::vector<float>(count, 1.0f), gap);
  }

  inline std::vector<visage::Bounds> grid(const visage::Bounds& bounds, int columns, int rows,
                                          float gap = 8.0f) {
    std::vector<visage::Bounds> result;
    for (const visage::Bounds& row : evenRows(bounds, rows, gap)) {
      for (const visage::Bounds& cell : evenColumns(row, columns, gap))
        result.push_back(cell);
    }
    return result;
  }

  inline visage::Bounds takeTop(visage::Bounds& bounds, float height, float gap = 8.0f) {
    visage::Bounds top(bounds.x(), bounds.y(), bounds.width(), height);
    bounds = visage::Bounds(bounds.x(), bounds.y() + height + gap, bounds.width(),
                            std::max(0.0f, bounds.height() - height - gap));
    return top;
  }

  inline visage::Bounds takeBottom(visage::Bounds& bounds, float height, float gap = 8.0f) {
    visage::Bounds bottom(bounds.x(), bounds.bottom() - height, bounds.width(), height);
    bounds = visage::Bounds(bounds.x(), bounds.y(), bounds.width(),
                            std::max(0.0f, bounds.height() - height - gap));
    return bottom;
  }

  inline visage::Bounds takeLeft(visage::Bounds& bounds, float width, float gap = 8.0f) {
    visage::Bounds left(bounds.x(), bounds.y(), width, bounds.height());
    bounds = visage::Bounds(bounds.x() + width + gap, bounds.y(),
                            std::max(0.0f, bounds.width() - width - gap), bounds.height());
    return left;
  }

  inline visage::Bounds takeRight(visage::Bounds& bounds, float width, float gap = 8.0f) {
    visage::Bounds right(bounds.right() - width, bounds.y(), width, bounds.height());
    bounds = visage::Bounds(bounds.x(), bounds.y(), std::max(0.0f, bounds.width() - width - gap),
                            bounds.height());
    return right;
  }

  inline void layoutButtonRow(const visage::Bounds& bounds,
                              const std::vector<visage::Frame*>& frames, float gap = 8.0f) {
    auto columns = evenColumns(bounds, frames.size(), gap);
    for (int i = 0; i < frames.size(); ++i)
      frames[i]->setBounds(columns[i]);
  }

  inline std::string modifierString(int modifiers) {
    std::string result;
    if (modifiers & visage::kModifierShift)
      result += "Shift+";
    if (modifiers & visage::kModifierRegCtrl)
      result += "Ctrl+";
    if (modifiers & visage::kModifierMacCtrl)
      result += "MacCtrl+";
    if (modifiers & visage::kModifierAlt)
      result += "Alt/Opt+";
    if (modifiers & visage::kModifierCmd)
      result += "Cmd+";
    if (modifiers & visage::kModifierMeta)
      result += "Meta+";
    return result;
  }

  inline std::string pointString(const visage::Point& point) {
    return "(" + visage::String(point.x, 1).toUtf8() + ", " + visage::String(point.y, 1).toUtf8() + ")";
  }

  inline std::string buttonName(visage::MouseButton button) {
    switch (button) {
    case visage::kMouseButtonLeft: return "left";
    case visage::kMouseButtonMiddle: return "middle";
    case visage::kMouseButtonRight: return "right";
    case visage::kMouseButtonTouch: return "touch";
    default: return "none";
    }
  }

  inline std::string buttonStateString(int state) {
    std::string result;
    if (state & visage::kMouseButtonLeft)
      result += "L";
    if (state & visage::kMouseButtonMiddle)
      result += "M";
    if (state & visage::kMouseButtonRight)
      result += "R";
    if (state & visage::kMouseButtonTouch)
      result += "T";
    return result.empty() ? "-" : result;
  }

  std::string keyCodeName(visage::KeyCode key_code);
}
