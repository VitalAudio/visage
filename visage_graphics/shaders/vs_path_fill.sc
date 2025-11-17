$input a_position, a_texcoord0
$output v_shader_values, v_shader_values1

#include <shader_include.sh>

uniform vec4 u_bounds;

void main() {
  v_shader_values = a_position;
  v_shader_values1 = a_texcoord0;
  gl_Position = vec4(a_position.xy * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
