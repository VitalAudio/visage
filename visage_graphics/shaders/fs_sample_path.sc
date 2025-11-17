$input v_coordinates, v_position, v_gradient_pos, v_gradient_pos2, v_gradient_texture_pos

#include <shader_include.sh>

SAMPLER2D(s_gradient, 0);
SAMPLER2D(s_texture, 1);

void main() {
  vec4 color = gradient(s_gradient, v_gradient_texture_pos, v_gradient_pos, v_gradient_pos2, v_position);
  gl_FragColor = color;
  float coverage = texture2D(s_texture, v_coordinates).r;
  float t = mod(abs(coverage), 2.0);
  gl_FragColor.a = gl_FragColor.a * (1.0 - abs(t - 1.0));
}
