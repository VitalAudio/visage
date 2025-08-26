$input a_position, a_color0, a_color1, a_color2, a_texcoord0, a_texcoord1, a_texcoord2
$output v_coordinates, v_dimensions, v_shader_values, v_position, v_gradient_pos, v_gradient_pos2, v_gradient_texture_pos

#include <shader_include.sh>

uniform vec4 u_bounds;

void main() {
  vec2 min = a_texcoord1.xy;
  vec2 max = a_texcoord1.zw;
  vec2 clamped = clamp(a_position, min, max);
  vec2 delta = clamped - a_position;

  v_position = a_position.xy;
  v_gradient_texture_pos = a_color0;
  v_gradient_pos = a_color1;
  v_gradient_pos2 = a_color2;
  v_dimensions = a_texcoord0.zw;
  v_coordinates = a_texcoord0.xy + 2.0 * delta / v_dimensions;
  vec2 adjusted_position = clamped * u_bounds.xy + u_bounds.zw;
  gl_Position = vec4(adjusted_position, 0.5, 1.0);
  v_shader_values = a_texcoord2;
}
