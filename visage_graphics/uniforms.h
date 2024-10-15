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

namespace visage {
  struct Uniforms {
    static constexpr char kTime[] = "u_time";
    static constexpr char kMult[] = "u_mult";
    static constexpr char kBounds[] = "u_bounds";
    static constexpr char kColorMult[] = "u_color_mult";
    static constexpr char kLimitMult[] = "u_limit_mult";
    static constexpr char kOriginFlip[] = "u_origin_flip";
    static constexpr char kAtlasScale[] = "u_atlas_scale";
    static constexpr char kCenterPosition[] = "u_center_position";
    static constexpr char kScale[] = "u_scale";
    static constexpr char kDimensions[] = "u_dimensions";
    static constexpr char kTopLeftColor[] = "u_top_left_color";
    static constexpr char kTopRightColor[] = "u_top_right_color";
    static constexpr char kBottomLeftColor[] = "u_bottom_left_color";
    static constexpr char kBottomRightColor[] = "u_bottom_right_color";
    static constexpr char kLineWidth[] = "u_line_width";
    static constexpr char kResampleValues[] = "u_resample_values";
    static constexpr char kThreshold[] = "u_threshold";
    static constexpr char kPixelSize[] = "u_pixel_size";
    static constexpr char kTexture[] = "s_texture";
  };
}
