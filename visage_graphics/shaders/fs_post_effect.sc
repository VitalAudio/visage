$input v_texture_uv, v_color0

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);

void main() {
  gl_FragColor = v_color0 * texture2D(s_texture, v_texture_uv);
}
