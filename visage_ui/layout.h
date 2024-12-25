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

#pragma once

#include <functional>
#include <visage_utils/dimension.h>
#include <visage_utils/space.h>

namespace visage {

  class Layout {
  public:
    enum class ItemAlignment {
      NotSet,
      Stretch,
      Start,
      Center,
      End,
    };

    enum class WrapAlignment {
      Start,
      Center,
      End,
      Stretch,
      SpaceBetween,
      SpaceAround,
      SpaceEvenly
    };

    std::vector<Bounds> flexPositions(const std::vector<const Layout*>& children,
                                      const Bounds& bounds, float dpi_scale) {
      int pad_left = padding_before_[0].roundWithDefault(dpi_scale, bounds.width(), bounds.height());
      int pad_right = padding_after_[0].roundWithDefault(dpi_scale, bounds.width(), bounds.height());
      int pad_top = padding_before_[1].roundWithDefault(dpi_scale, bounds.width(), bounds.height());
      int pad_bottom = padding_after_[1].roundWithDefault(dpi_scale, bounds.width(), bounds.height());

      Bounds flex_bounds = { bounds.x() + pad_left, bounds.y() + pad_top,
                             bounds.width() - pad_left - pad_right,
                             bounds.height() - pad_top - pad_bottom };

      if (flex_wrap_)
        return flexChildWrap(children, flex_bounds, dpi_scale);

      return flexChildGroup(children, flex_bounds, dpi_scale);
    }

    void setFlex(bool flex) { flex_ = flex; }
    bool flex() const { return flex_; }

    void setMargin(const Dimension& margin) {
      margin_before_[0] = margin;
      margin_before_[1] = margin;
      margin_after_[0] = margin;
      margin_after_[1] = margin;
    }

    void setMarginLeft(const Dimension& margin) { margin_before_[0] = margin; }
    void setMarginRight(const Dimension& margin) { margin_after_[0] = margin; }
    void setMarginTop(const Dimension& margin) { margin_before_[1] = margin; }
    void setMarginBottom(const Dimension& margin) { margin_after_[1] = margin; }

    void setPadding(const Dimension& padding) {
      padding_before_[0] = padding;
      padding_before_[1] = padding;
      padding_after_[0] = padding;
      padding_after_[1] = padding;
    }

    void setPaddingLeft(const Dimension& padding) { padding_before_[0] = padding; }
    void setPaddingRight(const Dimension& padding) { padding_after_[0] = padding; }
    void setPaddingTop(const Dimension& padding) { padding_before_[1] = padding; }
    void setPaddingBottom(const Dimension& padding) { padding_after_[1] = padding; }

    void setDimensions(const Dimension& width, const Dimension& height) {
      dimensions_[0] = width;
      dimensions_[1] = height;
    }

    void setWidth(const Dimension& width) { dimensions_[0] = width; }
    void setHeight(const Dimension& height) { dimensions_[1] = height; }

    void setFlexGrow(float grow) { flex_grow_ = grow; }
    void setFlexShrink(float shrink) { flex_shrink_ = shrink; }
    void setFlexRows(bool rows) { flex_rows_ = rows; }
    void setFlexReverseDirection(bool reverse) { flex_reverse_direction_ = reverse; }
    void setFlexWrap(bool wrap) { flex_wrap_ = wrap ? 1 : 0; }
    void setFlexItemAlignment(ItemAlignment alignment) { item_alignment_ = alignment; }
    void setFlexSelfAlignment(ItemAlignment alignment) { self_alignment_ = alignment; }
    void setFlexWrapAlignment(WrapAlignment alignment) { wrap_alignment_ = alignment; }
    void setFlexWrapReverse(bool wrap) { flex_wrap_ = wrap ? -1 : 0; }
    void setFlexGap(Dimension gap) { flex_gap_ = std::move(gap); }

  private:
    std::vector<Bounds> flexChildGroup(const std::vector<const Layout*>& children, Bounds bounds,
                                       float dpi_scale);

    std::vector<int> alignCrossPositions(std::vector<int>& cross_sizes, int cross_area, int gap);

    std::vector<Bounds> flexChildWrap(const std::vector<const Layout*>& children, Bounds bounds,
                                      float dpi_scale);

    bool flex_ = false;
    Dimension margin_before_[2];
    Dimension margin_after_[2];
    Dimension padding_before_[2];
    Dimension padding_after_[2];
    Dimension dimensions_[2];

    ItemAlignment item_alignment_ = ItemAlignment::Stretch;
    ItemAlignment self_alignment_ = ItemAlignment::NotSet;
    WrapAlignment wrap_alignment_ = WrapAlignment::Start;
    float flex_grow_ = 0.0f;
    float flex_shrink_ = 0.0f;
    bool flex_rows_ = true;
    bool flex_reverse_direction_ = false;
    int flex_wrap_ = 0;
    Dimension flex_gap_;
  };
}