$input v_coordinates, v_color0

#include <shader_utils.sh>

SAMPLER2D(s_texture, 0);

void main() {
  gl_FragColor = v_color0 * texture2D(s_texture, v_coordinates);
}
