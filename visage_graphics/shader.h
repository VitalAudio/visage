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

#include "graphics_utils.h"
#include "visage_file_embed/embedded_file.h"

namespace visage {
  class Canvas;

  class Shader {
  public:
    Shader() = delete;
    Shader(const EmbeddedFile& vertex_shader, const EmbeddedFile& fragment_shader, BlendMode state) :
        vertex_shader_(vertex_shader), fragment_shader_(fragment_shader), state_(state) { }
    virtual ~Shader() = default;

    const EmbeddedFile& vertexShader() const { return vertex_shader_; }
    const EmbeddedFile& fragmentShader() const { return fragment_shader_; }
    BlendMode state() const { return state_; }

  private:
    EmbeddedFile vertex_shader_;
    EmbeddedFile fragment_shader_;
    BlendMode state_ = BlendMode::Alpha;
  };
}
