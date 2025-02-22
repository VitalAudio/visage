$input v_texture_uv

#include <shader_include.sh>

uniform vec4 u_color_mult;

SAMPLER2D(s_texture, 0);

void main() {
  gl_FragColor = u_color_mult * texture2D(s_texture, v_texture_uv);
}
