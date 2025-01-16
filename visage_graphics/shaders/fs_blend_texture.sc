$input v_coordinates, v_dimensions

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);
SAMPLER2D(s_texture2, 1);

uniform vec4 u_mult;

void main() {
  gl_FragColor = texture2D(s_texture, v_coordinates) * u_mult;
  gl_FragColor = gl_FragColor + texture2D(s_texture2, v_dimensions) * (vec4(1.0, 1.0, 1.0, 1.0) - u_mult);
}
