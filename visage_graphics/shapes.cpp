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

#include "shapes.h"

#include "embedded/shaders.h"

#define VISAGE_SET_PROGRAM(shape, vertex, fragment) \
  const EmbeddedFile& shape::vertexShader() {       \
    return vertex;                                  \
  }                                                 \
  const EmbeddedFile& shape::fragmentShader() {     \
    return fragment;                                \
  }

namespace visage {
  VISAGE_SET_PROGRAM(Clear, shaders::vs_color, shaders::fs_color)
  VISAGE_SET_PROGRAM(Fill, shaders::vs_color, shaders::fs_color)
  VISAGE_SET_PROGRAM(RectangleShadow, shaders::vs_shape_shadow, shaders::fs_rectangle_shadow)
  VISAGE_SET_PROGRAM(RoundedRectangleShadow, shaders::vs_shape_shadow, shaders::fs_rounded_rectangle_shadow)
  VISAGE_SET_PROGRAM(Rectangle, shaders::vs_shape, shaders::fs_rectangle)
  VISAGE_SET_PROGRAM(RoundedRectangle, shaders::vs_shape, shaders::fs_rounded_rectangle)
  VISAGE_SET_PROGRAM(RectangleBorder, shaders::vs_shape, shaders::fs_rectangle_border)
  VISAGE_SET_PROGRAM(RoundedRectangleBorder, shaders::vs_shape, shaders::fs_rounded_rectangle_border)
  VISAGE_SET_PROGRAM(Circle, shaders::vs_shape, shaders::fs_circle)
  VISAGE_SET_PROGRAM(Triangle, shaders::vs_shape, shaders::fs_triangle)
  VISAGE_SET_PROGRAM(RoundedCorner, shaders::vs_rounded_corner, shaders::fs_rounded_corner)
  VISAGE_SET_PROGRAM(Ring, shaders::vs_shape, shaders::fs_ring)
  VISAGE_SET_PROGRAM(FlatArc, shaders::vs_arc, shaders::fs_flat_arc)
  VISAGE_SET_PROGRAM(RoundedArc, shaders::vs_arc, shaders::fs_rounded_arc)
  VISAGE_SET_PROGRAM(FlatSegment, shaders::vs_complex_shape, shaders::fs_flat_segment)
  VISAGE_SET_PROGRAM(RoundedSegment, shaders::vs_complex_shape, shaders::fs_rounded_segment)
  VISAGE_SET_PROGRAM(Rotary, shaders::vs_rotary, shaders::fs_rolly_ball_rotary)
  VISAGE_SET_PROGRAM(Diamond, shaders::vs_shape, shaders::fs_diamond)
  VISAGE_SET_PROGRAM(IconWrapper, shaders::vs_tinted_texture, shaders::fs_tinted_texture)
  VISAGE_SET_PROGRAM(ImageWrapper, shaders::vs_image_sample, shaders::fs_image_sample)
  VISAGE_SET_PROGRAM(LineWrapper, shaders::vs_line, shaders::fs_line)
  VISAGE_SET_PROGRAM(LineFillWrapper, shaders::vs_line_fill, shaders::fs_line_fill)
  VISAGE_SET_PROGRAM(CanvasWrapper, shaders::vs_tinted_texture, shaders::fs_tinted_texture)
}