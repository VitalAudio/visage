$input a_position, a_texcoord0
$output v_shader_values

#include <bgfx_shader.sh>

uniform vec4 u_color_mult;

uniform vec4 u_bounds;
uniform vec4 u_scale;
uniform vec4 u_dimensions;

void main() {
  vec2 adjusted_position = a_position * u_bounds.xy + u_bounds.zw;
  v_shader_values.x = a_texcoord0.x;
  v_shader_values.y = (a_texcoord0.y + 1.0) * u_color_mult.x;
  v_shader_values.zw = abs(a_position.xy) / u_dimensions.xy;
  gl_Position = vec4(adjusted_position * u_scale.xy, 0.5, 1.0);
}
