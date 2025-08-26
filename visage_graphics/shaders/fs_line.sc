$input v_shader_values, v_position

#include <shader_include.sh>

uniform vec4 u_gradient_texture_position;
uniform vec4 u_gradient_position;
uniform vec4 u_gradient_position2;
uniform vec4 u_line_width;
uniform vec4 u_time;

SAMPLER2D(s_gradient, 0);

void main() {
  vec4 color = gradient(s_gradient, u_gradient_texture_position, u_gradient_position, u_gradient_position2, v_position);
  float depth_out = v_shader_values.x;
  float dist_from_edge = min(depth_out, 1.0 - depth_out);
  float mult = 1.0 + max(dist_from_edge - 2.0 / u_line_width.x, 0.0);
  vec4 result = min(1.0, mult) * color;
  float scale = u_line_width.x * dist_from_edge;
  result.a = min(result.a * scale * 0.5, 1.0);
  gl_FragColor = result;
}
