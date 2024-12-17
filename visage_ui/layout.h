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

namespace visage {

  struct Dimension {
    std::function<float(float dpi_scale, float parent_width, float parent_height)> compute = nullptr;

    float computeWithDefault(float dpi_scale, float parent_width, float parent_height,
                             float default_value = 0.0f) const {
      if (compute)
        return compute(dpi_scale, parent_width, parent_height);
      return default_value;
    }

    Dimension() = default;
    Dimension(float amount) : compute([amount](float, float, float) { return amount; }) { }
    Dimension(std::function<float(float, float, float)> compute) : compute(std::move(compute)) { }

    static Dimension devicePixels(float pixels) {
      return Dimension([pixels](float, float, float) { return pixels; });
    }

    static Dimension logicalPixels(float pixels) {
      return Dimension([pixels](float dpi_scale, float, float) { return dpi_scale * pixels; });
    }

    static Dimension widthPercent(float percent) {
      float ratio = percent * 0.01f;
      return Dimension([ratio](float, float parent_width, float) { return ratio * parent_width; });
    }

    static Dimension heightPercent(float percent) {
      float ratio = percent * 0.01f;
      return Dimension([ratio](float, float, float parent_height) { return ratio * parent_height; });
    }

    static Dimension viewMinPercent(float percent) {
      float ratio = percent * 0.01f;
      return Dimension([ratio](float, float parent_width, float parent_height) {
        return ratio * std::min(parent_width, parent_height);
      });
    }

    static Dimension viewMaxPercent(float percent) {
      float ratio = percent * 0.01f;
      return Dimension([ratio](float, float parent_width, float parent_height) {
        return ratio * std::max(parent_width, parent_height);
      });
    }

    Dimension operator+(const Dimension& other) const {
      auto compute1 = compute;
      auto compute2 = other.compute;
      return Dimension([compute1, compute2](float dpi_scale, float parent_width, float parent_height) {
        return compute1(dpi_scale, parent_width, parent_height) +
               compute2(dpi_scale, parent_width, parent_height);
      });
    }

    Dimension& operator+=(const Dimension& other) {
      auto compute1 = compute;
      auto compute2 = other.compute;
      compute = [compute1, compute2](float dpi_scale, float parent_width, float parent_height) {
        return compute1(dpi_scale, parent_width, parent_height) +
               compute2(dpi_scale, parent_width, parent_height);
      };
      return *this;
    }

    Dimension operator-(const Dimension& other) const {
      auto compute1 = compute;
      auto compute2 = other.compute;
      return Dimension([compute1, compute2](float dpi_scale, float parent_width, float parent_height) {
        return compute1(dpi_scale, parent_width, parent_height) -
               compute2(dpi_scale, parent_width, parent_height);
      });
    }

    Dimension& operator-=(const Dimension& other) {
      auto compute1 = compute;
      auto compute2 = other.compute;
      compute = [compute1, compute2](float dpi_scale, float parent_width, float parent_height) {
        return compute1(dpi_scale, parent_width, parent_height) -
               compute2(dpi_scale, parent_width, parent_height);
      };
      return *this;
    }
    Dimension operator*(float scalar) const {
      auto compute1 = compute;
      return Dimension([compute1, scalar](float dpi_scale, float parent_width, float parent_height) {
        return scalar * compute1(dpi_scale, parent_width, parent_height);
      });
    }

    friend Dimension operator*(float scalar, const Dimension& dimension) {
      return dimension * scalar;
    }

    Dimension operator/(float scalar) const {
      auto compute1 = compute;
      return Dimension([compute1, scalar](float dpi_scale, float parent_width, float parent_height) {
        return compute1(dpi_scale, parent_width, parent_height) / scalar;
      });
    }
  };

  namespace dimension {
    inline Dimension operator""_dpx(long double pixels) {
      return Dimension::devicePixels(pixels);
    }

    inline Dimension operator""_dpx(unsigned long long pixels) {
      return Dimension::devicePixels(pixels);
    }

    inline Dimension operator""_px(long double pixels) {
      return Dimension::logicalPixels(pixels);
    }

    inline Dimension operator""_px(unsigned long long pixels) {
      return Dimension::logicalPixels(pixels);
    }

    inline Dimension operator""_vw(long double percent) {
      return Dimension::widthPercent(percent);
    }

    inline Dimension operator""_vw(unsigned long long percent) {
      return Dimension::widthPercent(percent);
    }

    inline Dimension operator""_vh(long double percent) {
      return Dimension::heightPercent(percent);
    }

    inline Dimension operator""_vh(unsigned long long percent) {
      return Dimension::heightPercent(percent);
    }

    inline Dimension operator""_vmin(long double percent) {
      return Dimension::viewMinPercent(percent);
    }

    inline Dimension operator""_vmin(unsigned long long percent) {
      return Dimension::viewMinPercent(percent);
    }

    inline Dimension operator""_vmax(long double percent) {
      return Dimension::viewMaxPercent(percent);
    }

    inline Dimension operator""_vmax(unsigned long long percent) {
      return Dimension::viewMaxPercent(percent);
    }
  }

  class Layout {
  public:
    enum class ItemAlignment {
      Default,
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
      int pad_left = padding_before_[0].computeWithDefault(dpi_scale, bounds.width(), bounds.height());
      int pad_right = padding_after_[0].computeWithDefault(dpi_scale, bounds.width(), bounds.height());
      int pad_top = padding_before_[1].computeWithDefault(dpi_scale, bounds.width(), bounds.height());
      int pad_bottom = padding_after_[1].computeWithDefault(dpi_scale, bounds.width(), bounds.height());

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

    ItemAlignment item_alignment_ = ItemAlignment::Default;
    ItemAlignment self_alignment_ = ItemAlignment::Default;
    WrapAlignment wrap_alignment_ = WrapAlignment::Start;
    float flex_grow_ = 0.0f;
    float flex_shrink_ = 0.0f;
    bool flex_rows_ = true;
    bool flex_reverse_direction_ = false;
    int flex_wrap_ = 0;
    Dimension flex_gap_ = 0;
  };
}