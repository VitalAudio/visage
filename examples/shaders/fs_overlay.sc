$input v_coordinates, v_dimensions, v_color0, v_shader_values

#include <shader_utils.sh>

SAMPLER2D(s_texture, 0);

uniform vec4 u_alpha;

void main() {
  gl_FragColor = texture2D(s_texture, v_coordinates) * v_color0 * u_alpha;
}
