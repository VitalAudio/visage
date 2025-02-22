$input a_position, a_texcoord0, a_texcoord1, a_texcoord2, a_texcoord3, a_texcoord4, a_texcoord5
$output v_coordinates, v_dimensions, v_shader_values, v_shader_values1, v_position, v_gradient_color_pos, v_gradient_pos

#include <shader_include.sh>

uniform vec4 u_bounds;

void main() {
  vec2 minimum = a_texcoord3.xy;
  vec2 maximum = a_texcoord3.zw;
  vec2 clamped = clamp(a_position.xy + a_texcoord2.xy * 0.5, minimum, maximum);
  vec2 delta = clamped - (a_position.xy + a_texcoord2.xy * 0.5);

  v_position = clamped;
  v_gradient_color_pos = a_texcoord0;
  v_gradient_pos = a_texcoord1;
  v_dimensions = a_texcoord2.zw + vec2(1.0, 1.0);
  v_coordinates = a_texcoord2.xy + (2.0 * delta) / v_dimensions;
  vec2 adjusted_position = clamped * u_bounds.xy + u_bounds.zw;
  gl_Position = vec4(adjusted_position, 0.5, 1.0);
  v_shader_values = a_texcoord4;
  v_shader_values1 = a_texcoord5;
}
