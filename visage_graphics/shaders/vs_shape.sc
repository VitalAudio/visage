$input a_position, a_texcoord0, a_texcoord1, a_texcoord2, a_texcoord3, a_color0, a_color1, a_color2
$output v_coordinates, v_dimensions, v_color0, v_color1, v_shader_values, v_position, v_gradient_pos

#include <shader_include.sh>

uniform vec4 u_color_mult;
uniform vec4 u_bounds;

void main() {
  vec2 minimum = a_texcoord2.xy;
  vec2 maximum = a_texcoord2.zw;
  vec2 clamped = clamp(a_position.xy + a_texcoord1.xy * 0.5, minimum, maximum);
  vec2 delta = clamped - (a_position.xy + a_texcoord1.xy * 0.5);

  v_position = a_position.xy;
  v_gradient_pos = a_texcoord0;
  v_dimensions = a_texcoord1.zw + vec2(1.0, 1.0);
  v_coordinates = a_texcoord1.xy + (2.0 * delta) / v_dimensions;
  vec2 adjusted_position = clamped * u_bounds.xy + u_bounds.zw;
  gl_Position = vec4(adjusted_position, 0.5, 1.0);
  v_shader_values = a_texcoord3;
  v_color0 = (a_color0 * a_color2.x) * u_color_mult.x;
  v_color0.a = a_color0.a;
  v_color1 = (a_color1 * a_color2.x) * u_color_mult.x;
  v_color1.a = a_color1.a;
}
