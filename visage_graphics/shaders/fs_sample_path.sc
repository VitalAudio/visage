$input v_coordinates, v_position, v_dimensions, v_gradient_pos, v_gradient_pos2, v_gradient_texture_pos

#include <shader_include.sh>

SAMPLER2D(s_gradient, 0);
SAMPLER2D(s_texture, 1);

void main() {
  vec4 color = gradient(s_gradient, v_gradient_texture_pos, v_gradient_pos, v_gradient_pos2, v_position);
  gl_FragColor = color;
  float coverage = abs(texture2D(s_texture, v_coordinates).r);
  float t = mod(coverage, 2.0);
  float alpha = v_dimensions.x * (1.0 - abs(t - 1.0)) + (1.0 - v_dimensions.x) * clamp(coverage, 0.0, 1.0);
  gl_FragColor.a = gl_FragColor.a * alpha;
}
