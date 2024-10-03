$input v_shader_values

#include <shader_utils.sh>

uniform vec4 u_top_left_color;
uniform vec4 u_top_right_color;
uniform vec4 u_bottom_left_color;
uniform vec4 u_bottom_right_color;
uniform vec4 u_line_width;
uniform vec4 u_time;

void main() {
  float x_t = v_shader_values.z;
  float y_t = v_shader_values.w;
  vec4 top_color = u_top_left_color + (u_top_right_color - u_top_left_color) * x_t;
  vec4 bottom_color = u_bottom_left_color + (u_bottom_right_color - u_bottom_left_color) * x_t;
  vec4 color = bottom_color + (top_color - bottom_color) * y_t;

  float depth_out = v_shader_values.x;
  float dist_from_edge = min(depth_out, 1.0 - depth_out);
  float mult = 1.0 + max(dist_from_edge - 2.0 / u_line_width.x, 0.0);
  vec4 result = min(1.0, mult) * color;
  float scale = u_line_width.x * dist_from_edge;
  result.a = min(result.a * scale * 0.5, 1.0);
  result.rgb = result.rgb * v_shader_values.y;
  gl_FragColor = result;
}
