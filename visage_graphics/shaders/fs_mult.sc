$input v_coordinates

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);

uniform vec4 u_mult;

void main() {
  gl_FragColor = texture2D(s_texture, v_coordinates) * u_mult;
}
