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

#include "shapes.h"

#include "embedded/shaders.h"
#include "layer.h"
#include "region.h"

#define VISAGE_SET_PROGRAM(shape, vertex, fragment) \
  const EmbeddedFile& shape::vertexShader() {       \
    return vertex;                                  \
  }                                                 \
  const EmbeddedFile& shape::fragmentShader() {     \
    return fragment;                                \
  }

namespace visage {
  VISAGE_SET_PROGRAM(Fill, shaders::vs_color, shaders::fs_color)
  VISAGE_SET_PROGRAM(Rectangle, shaders::vs_shape, shaders::fs_rectangle)
  VISAGE_SET_PROGRAM(RoundedRectangle, shaders::vs_shape, shaders::fs_rounded_rectangle)
  VISAGE_SET_PROGRAM(Circle, shaders::vs_shape, shaders::fs_circle)
  VISAGE_SET_PROGRAM(Squircle, shaders::vs_shape, shaders::fs_squircle)
  VISAGE_SET_PROGRAM(FlatArc, shaders::vs_arc, shaders::fs_flat_arc)
  VISAGE_SET_PROGRAM(RoundedArc, shaders::vs_arc, shaders::fs_rounded_arc)
  VISAGE_SET_PROGRAM(FlatSegment, shaders::vs_complex_shape, shaders::fs_flat_segment)
  VISAGE_SET_PROGRAM(RoundedSegment, shaders::vs_complex_shape, shaders::fs_rounded_segment)
  VISAGE_SET_PROGRAM(Triangle, shaders::vs_complex_shape, shaders::fs_triangle)
  VISAGE_SET_PROGRAM(QuadraticBezier, shaders::vs_complex_shape, shaders::fs_quadratic_bezier)
  VISAGE_SET_PROGRAM(Diamond, shaders::vs_shape, shaders::fs_diamond)
  VISAGE_SET_PROGRAM(ImageWrapper, shaders::vs_tinted_texture, shaders::fs_tinted_texture)
  VISAGE_SET_PROGRAM(PathFillWrapper, shaders::vs_sample_path, shaders::fs_sample_path)
  VISAGE_SET_PROGRAM(GraphLineWrapper, shaders::vs_shape, shaders::fs_graph_line)
  VISAGE_SET_PROGRAM(GraphFillWrapper, shaders::vs_shape, shaders::fs_graph_fill)
  VISAGE_SET_PROGRAM(HeatMapWrapper, shaders::vs_shape, shaders::fs_heat_map)
  VISAGE_SET_PROGRAM(SampleRegion, shaders::vs_post_effect, shaders::fs_post_effect)

  SampleRegion::SampleRegion(const ClampBounds& clamp, const PackedBrush* brush, float x, float y,
                             float width, float height, const Region* region, PostEffect* post_effect) :
      Shape(region->layer(), clamp, brush, x, y, width, height), region(region),
      post_effect(post_effect) {
    if (post_effect)
      batch_id = post_effect;
  }

  void SampleRegion::setVertexData(Vertex* vertices) const {
    region->layer()->setTexturePositionsForRegion(region, vertices);
  }
}
