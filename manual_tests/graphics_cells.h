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

#include "test_helpers.h"

namespace manual {
  using CellDraw = std::function<void(visage::Canvas&, float, float)>;

  class Cell : public Box {
  public:
    static constexpr float kDescriptionHeight = 46.0f;

    Cell(const std::string& title, const std::string& description, CellDraw draw, bool animated) :
        Box(title), description_(description, 11, kDimText), draw_(std::move(draw)),
        animated_(animated) {
      description_.setMultiLine(true);
      addChild(&description_);
      content_.setIgnoresMouseEvents(true, false);
      content_.onDraw() = [this](visage::Canvas& canvas) {
        draw_(canvas, content_.width(), content_.height());
        if (animated_)
          content_.redraw();
      };
      addChild(&content_);
    }

    void resized() override {
      visage::Bounds content = contentBounds();
      description_.setBounds(takeBottom(content, kDescriptionHeight, 4));
      content_.setBounds(content);
    }

    visage::Frame& content() { return content_; }

  private:
    Label description_;
    visage::Frame content_;
    CellDraw draw_;
    bool animated_ = false;
  };

  class CellPage : public visage::Frame {
  public:
    explicit CellPage(int columns) : columns_(columns) { }

    Cell* addCell(const std::string& title, const std::string& description, CellDraw draw,
                  bool animated = false) {
      cells_.push_back(std::make_unique<Cell>(title, description, std::move(draw), animated));
      addChild(cells_.back().get());
      return cells_.back().get();
    }

    void addControl(visage::Frame* control) {
      controls_.push_back(control);
      addChild(control);
    }

    void redrawCells() {
      for (auto& cell : cells_)
        cell->content().redraw();
    }

    void resized() override {
      visage::Bounds bounds = localBounds().reduced(12);
      if (!controls_.empty())
        layoutButtonRow(takeTop(bounds, 38), controls_, 16);
      int rows = (cells_.size() + columns_ - 1) / columns_;
      auto cells = grid(bounds, columns_, rows, 8);
      for (int i = 0; i < cells_.size(); ++i)
        cells_[i]->setBounds(cells[i]);
    }

  private:
    int columns_ = 3;
    std::vector<std::unique_ptr<Cell>> cells_;
    std::vector<visage::Frame*> controls_;
  };
}
