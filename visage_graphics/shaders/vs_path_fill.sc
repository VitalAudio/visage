$input a_position
$output v_shader_values, v_position

#include <shader_include.sh>

uniform vec4 u_bounds;
uniform vec4 u_dimensions;

void main() {
  v_position = a_position.xy;
  v_shader_values.x = a_position.z;
  gl_Position = vec4(v_position * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
