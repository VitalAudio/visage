$input a_position, a_texcoord0
$output v_coordinates, v_dimensions

#include <shader_include.sh>

uniform vec4 u_resample_values;
uniform vec4 u_resample_values2;

void main() {
  gl_Position = vec4(a_position.xy, 0.5, 1.0);
  v_coordinates = a_texcoord0.xy * u_resample_values;
  v_dimensions = a_texcoord0.xy * u_resample_values2;
}
