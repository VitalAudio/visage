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

#include "layout.h"

namespace visage {
  std::vector<Bounds> Layout::flexChildGroup(Bounds bounds, float dpi_scale,
                                             const std::vector<const Layout*>& children) {
    int width = bounds.width();
    int height = bounds.height();
    int dim = flex_rows_ ? 1 : 0;
    int cross_dim = 1 - dim;

    int flex_area = flex_rows_ ? height : width;
    flex_area -= flex_gap_ * (children.size() - 1);
    float total_flex_grow = 0.0f;
    float total_flex_shrink = 0.0f;

    std::vector<int> dimensions, margins_before, margins_after;
    dimensions.reserve(children.size());
    margins_before.reserve(children.size());
    margins_after.reserve(children.size());
    for (const Layout* child : children) {
      int dimension = child->dimensions_[dim].computeWithDefault(dpi_scale, width, height);
      int margin_before = child->margin_before_[dim].computeWithDefault(dpi_scale, width, height);
      int margin_after = child->margin_after_[dim].computeWithDefault(dpi_scale, width, height);
      flex_area -= dimension + margin_before + margin_after;

      dimensions.push_back(dimension);
      margins_before.push_back(margin_before);
      margins_after.push_back(margin_after);
      total_flex_grow += child->flex_grow_;
      total_flex_shrink += child->flex_shrink_;
    }

    if (flex_area > 0) {
      for (int i = 0; i < children.size(); ++i) {
        if (children[i]->flex_grow_) {
          int delta = flex_area * children[i]->flex_grow_ / total_flex_grow;
          dimensions[i] += delta;
          flex_area -= delta;
          total_flex_grow -= children[i]->flex_grow_;
        }
      }
    }

    if (flex_area < 0) {
      for (int i = 0; i < children.size(); ++i) {
        if (children[i]->flex_shrink_) {
          int delta = flex_area * children[i]->flex_shrink_ / total_flex_shrink;
          dimensions[i] += delta;
          flex_area -= delta;
          total_flex_shrink -= children[i]->flex_shrink_;
        }
      }
    }

    std::vector<Bounds> results;
    results.reserve(children.size());

    int position = 0;
    for (int i = 0; i < children.size(); ++i) {
      int cross_pos = children[i]->margin_before_[cross_dim].computeWithDefault(dpi_scale, width, height);
      int cross_size = children[i]->dimensions_[cross_dim].computeWithDefault(dpi_scale, width, height);
      position += margins_before[i];
      results.push_back({ position, cross_pos, dimensions[i], cross_size });
      position += dimensions[i] + margins_after[i] + flex_gap_;
    }

    if (flex_reverse_direction_) {
      int flex_area = flex_rows_ ? height : width;
      for (Bounds& result : results)
        result.setX(flex_area - result.right());
    }

    if (flex_rows_) {
      for (Bounds& result : results)
        result.flipDimensions();
    }

    for (Bounds& result : results)
      result = result + Point(bounds.x(), bounds.y());

    return results;
  }

  std::vector<Bounds> Layout::flexChildWrap(Bounds bounds, float dpi_scale,
                                            const std::vector<const Layout*>& children) {
    std::vector<Bounds> results;
    results.reserve(children.size());

    int width = bounds.width();
    int height = bounds.height();
    int dim = flex_rows_ ? 1 : 0;
    int cross_dim = 1 - dim;

    int total_flex_area = flex_rows_ ? height : width;
    int cross_area = flex_rows_ ? width : height;
    int flex_area = total_flex_area;
    int cross_max = 0;

    std::vector<int> breaks;
    std::vector<int> cross_sizes;

    for (int i = 0; i < children.size(); ++i) {
      const Layout* child = children[i];
      int dimension = child->dimensions_[dim].computeWithDefault(dpi_scale, width, height);
      int margin_before = child->margin_before_[dim].computeWithDefault(dpi_scale, width, height);
      int margin_after = child->margin_after_[dim].computeWithDefault(dpi_scale, width, height);
      int total = dimension + margin_before + margin_after;
      flex_area -= total;

      if (flex_area < 0) {
        cross_sizes.push_back(cross_max);
        breaks.push_back(i);
        flex_area = total_flex_area - total;
        cross_max = 0;
      }

      int cross_amount = child->dimensions_[cross_dim].computeWithDefault(dpi_scale, width, height) +
                         child->margin_before_[cross_dim].computeWithDefault(dpi_scale, width, height) +
                         child->margin_after_[cross_dim].computeWithDefault(dpi_scale, width, height);
      cross_max = std::max(cross_amount, cross_max);

      flex_area -= flex_gap_;
    }

    int cross_pos = 0;
    int group_index = 0;
    breaks.push_back(children.size());
    cross_sizes.push_back(cross_max);
    for (int i = 0; i < breaks.size(); ++i) {
      if (breaks[i] == group_index)
        continue;

      Bounds group_bounds;
      if (flex_wrap_ < 0) {
        int offset = cross_pos + cross_sizes[i];
        if (flex_rows_)
          group_bounds = { bounds.right() - offset, bounds.y(), cross_sizes[i], bounds.height() };
        else
          group_bounds = { bounds.x(), bounds.bottom() - offset, bounds.width(), cross_sizes[i] };
      }
      else {
        if (flex_rows_)
          group_bounds = { bounds.x() + cross_pos, bounds.y(), cross_sizes[i], bounds.height() };
        else
          group_bounds = { bounds.x(), bounds.y() + cross_pos, bounds.width(), cross_sizes[i] };
      }

      auto group = std::vector<const Layout*>(children.begin() + group_index,
                                              children.begin() + breaks[i]);
      group_index = breaks[i];
      std::vector<Bounds> bounds = flexChildGroup(group_bounds, dpi_scale, group);
      results.insert(results.end(), bounds.begin(), bounds.end());
      cross_pos += cross_sizes[i] + flex_gap_;
    }

    return results;
  }
}