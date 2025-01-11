$input v_coordinates

#include <shader_include.sh>

uniform vec4 u_threshold;

SAMPLER2D(s_texture, 0);

void main() {
  gl_FragColor = max(vec4(0.0, 0.0, 0.0, 0.0), texture2D(s_texture, v_coordinates) - u_threshold);
}

