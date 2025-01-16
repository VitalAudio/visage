$input v_coordinates

#include <shader_include.sh>

uniform vec4 u_threshold;
uniform vec4 u_mult;

SAMPLER2D(s_texture, 0);

void main() {
  vec4 color = texture2D(s_texture, v_coordinates);
  float value = max(max(color.r, color.b), color.g) + 0.0001;
  float mult = max(0.0, 1.0 - u_threshold.x / value);
  vec4 bleed = color * mult;
  gl_FragColor = bleed * u_mult;
  gl_FragColor.a = 1.0;
}

