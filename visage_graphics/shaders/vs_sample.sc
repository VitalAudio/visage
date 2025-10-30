$input a_position
$output v_coordinates

#include <shader_include.sh>

uniform vec4 u_resample_values;

void main() {
  gl_Position = vec4(a_position.xy, 0.5, 1.0);
  v_coordinates = u_resample_values.xy * a_position.zw;
}
