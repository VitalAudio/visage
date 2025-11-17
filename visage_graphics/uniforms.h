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

namespace visage {
  struct Uniforms {
    static constexpr char kTime[] = "u_time";
    static constexpr char kMult[] = "u_mult";
    static constexpr char kTextureClamp[] = "u_texture_clamp";
    static constexpr char kBounds[] = "u_bounds";
    static constexpr char kColor[] = "u_color";
    static constexpr char kColorMult[] = "u_color_mult";
    static constexpr char kLimitMult[] = "u_limit_mult";
    static constexpr char kOriginFlip[] = "u_origin_flip";
    static constexpr char kAtlasScale[] = "u_atlas_scale";
    static constexpr char kAtlasScale2[] = "u_atlas_scale2";
    static constexpr char kCenterPosition[] = "u_center_position";
    static constexpr char kDimensions[] = "u_dimensions";
    static constexpr char kLineWidth[] = "u_line_width";
    static constexpr char kResampleValues[] = "u_resample_values";
    static constexpr char kResampleValues2[] = "u_resample_values2";
    static constexpr char kThreshold[] = "u_threshold";
    static constexpr char kPixelSize[] = "u_pixel_size";
    static constexpr char kGradient[] = "s_gradient";
    static constexpr char kTexture[] = "s_texture";
    static constexpr char kTexture2[] = "s_texture2";
    static constexpr char kGradientTexturePosition[] = "u_gradient_texture_position";
    static constexpr char kGradientPosition[] = "u_gradient_position";
    static constexpr char kGradientPosition2[] = "u_gradient_position2";
    static constexpr char kRadialGradient[] = "u_radial_gradient";
  };
}
