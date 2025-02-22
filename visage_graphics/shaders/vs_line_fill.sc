$input a_position, a_texcoord0
$output v_shader_values, v_position

#include <shader_include.sh>

uniform vec4 u_bounds;
uniform vec4 u_center_position;
uniform vec4 u_dimensions;

void main() {
  v_position = a_position.xy;
  v_shader_values.x = (a_position.y - u_center_position.y) / (1.0 - u_center_position.y);
  v_shader_values.y = a_texcoord0.y;
  vec2 adjusted_position = a_position.xy * u_bounds.xy + u_bounds.zw;
  v_shader_values.zw = abs(a_position.xy - u_center_position.xy) / u_dimensions.xy;
  gl_Position = vec4(adjusted_position, 0.5, 1.0);
}
