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

#include "shader_quad.h"

#include "visage_graphics/theme.h"

#include <utility>

namespace visage {
  THEME_COLOR(ShaderQuadColor, 0xffffffff);

  ShaderQuad::ShaderQuad(const EmbeddedFile& vertex_shader, const EmbeddedFile& fragment_shader,
                         BlendState state) : shader_(vertex_shader, fragment_shader, state) { }

  void ShaderQuad::draw(Canvas& canvas) {
    canvas.setPaletteColor(kShaderQuadColor);
    canvas.shader(&shader_, 0.0f, 0.0f, width(), height());
    redraw();
  }
}