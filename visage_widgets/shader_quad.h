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

#include "visage_graphics/shader.h"
#include "visage_ui/frame.h"

namespace visage {
  class ShaderQuad : public Frame {
  public:
    ShaderQuad(const EmbeddedFile& vertex_shader, const EmbeddedFile& fragment_shader, BlendState state);
    ~ShaderQuad() override = default;

    void draw(Canvas& canvas) override;

  private:
    Shader shader_;

    VISAGE_LEAK_CHECKER(ShaderQuad)
  };
}